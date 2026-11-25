/* ============================================================
 * USB FS 虚拟串口（CDC-ACM）—— 自包含实现，不依赖 ST 的 USB 库
 *
 * 片子：STM32F407，OTG_FS 核 + 内置全速 PHY
 * 引脚：PA11 = D-，PA12 = D+（AF10）
 * 时钟：PLL_Q = 7 → 336/7 = 48 MHz，精确
 *
 * 为什么不用 ST 的 USB 库：
 *   那套库要额外引入 10 个文件；这里只实现 CDC 必需的部分，
 *   代码量小、依赖少，出问题容易定位。
 *
 * 端点分配：
 *   EP0  控制（双向，64 B）
 *   EP1  IN  中断（CDC 通知，64 B，本工程不发数据，只 NAK）
 *   EP2  OUT 批量（CDC 数据下行，64 B，本工程只丢弃）
 *   EP3  IN  批量（CDC 数据上行，64 B）← 原始数据从这里出去
 *
 * FIFO 分配（OTG_FS 总共 320 字 = 1280 B）：
 *   RX      128 字（512 B）
 *   EP0 TX   64 字（256 B）
 *   EP1 TX   16 字（ 64 B）
 *   EP3 TX   96 字（384 B）
 *   合计    304 字，够用
 *
 * 关键：PA9 是 USB 的 VBUS 检测脚，正好是 USART1_TX，所以
 *       初始化时置 GCCFG.NOVBUSSENS，并且强制设备模式（GUSBCFG.FORCEDEVMODE），
 *       这样 USB 只用 PA11/PA12，串口完全不受影响。
 * ============================================================ */

#include "usb_cdc.h"

/* ------------------------------------------------------------------
 * 寄存器（OTG_FS，基址 0x50000000）
 * 注意：F4 的 GCCFG 在 0x038，IN 端点 TX FIFO 大小寄存器在 0x100+4n
 * ------------------------------------------------------------------ */
#define USB_BASE        0x50000000UL

#define GOTGCTL         (*(volatile uint32_t *)(USB_BASE + 0x000))
#define GOTGINT         (*(volatile uint32_t *)(USB_BASE + 0x004))
#define GAHBCFG         (*(volatile uint32_t *)(USB_BASE + 0x008))
#define GUSBCFG         (*(volatile uint32_t *)(USB_BASE + 0x00C))
#define GRSTCTL         (*(volatile uint32_t *)(USB_BASE + 0x010))
#define GINTSTS         (*(volatile uint32_t *)(USB_BASE + 0x014))
#define GINTMSK         (*(volatile uint32_t *)(USB_BASE + 0x018))
#define GRXSTSR         (*(volatile uint32_t *)(USB_BASE + 0x01C))
#define GRXSTSP         (*(volatile uint32_t *)(USB_BASE + 0x020))
#define GRXFSIZ         (*(volatile uint32_t *)(USB_BASE + 0x024))
#define GNPTXFSIZ       (*(volatile uint32_t *)(USB_BASE + 0x028))
#define GCCFG           (*(volatile uint32_t *)(USB_BASE + 0x038))
#define DIEPTXF(n)      (*(volatile uint32_t *)(USB_BASE + 0x100 + 4u * (n)))
#define FIFO(n)         (*(volatile uint32_t *)(USB_BASE + 0x1000 + 0x1000u * (n)))

#define DCFG            (*(volatile uint32_t *)(USB_BASE + 0x800))
#define DCTL            (*(volatile uint32_t *)(USB_BASE + 0x804))
#define DSTS            (*(volatile uint32_t *)(USB_BASE + 0x808))
#define DIEPMSK         (*(volatile uint32_t *)(USB_BASE + 0x810))
#define DOEPMSK         (*(volatile uint32_t *)(USB_BASE + 0x814))
#define DAINT           (*(volatile uint32_t *)(USB_BASE + 0x818))
#define DAINTMSK        (*(volatile uint32_t *)(USB_BASE + 0x81C))
#define PCGCCTL         (*(volatile uint32_t *)(USB_BASE + 0xE00))

#define DIEPCTL(n)      (*(volatile uint32_t *)(USB_BASE + 0x900 + 0x20u * (n)))
#define DIEPINT(n)      (*(volatile uint32_t *)(USB_BASE + 0x908 + 0x20u * (n)))
#define DIEPTSIZ(n)     (*(volatile uint32_t *)(USB_BASE + 0x910 + 0x20u * (n)))
#define DTXFSTS(n)      (*(volatile uint32_t *)(USB_BASE + 0x918 + 0x20u * (n)))

#define DOEPCTL(n)      (*(volatile uint32_t *)(USB_BASE + 0xB00 + 0x20u * (n)))
#define DOEPINT(n)      (*(volatile uint32_t *)(USB_BASE + 0xB08 + 0x20u * (n)))
#define DOEPTSIZ(n)     (*(volatile uint32_t *)(USB_BASE + 0xB10 + 0x20u * (n)))

/* 位定义 */
#define GINT_USBRST     (1u << 12)
#define GINT_ENUMDNE    (1u << 13)
#define GINT_RXFLVL     (1u << 4)
#define GINT_IEPINT     (1u << 18)
#define GINT_OEPINT     (1u << 19)

#define DCTL_RWUSIG     (1u << 0)
#define DCTL_SDIS       (1u << 1)

#define DIEPINT_XFRC    (1u << 0)
#define DIEPINT_EPDISD  (1u << 1)
#define DIEPINT_TIMEOUT (1u << 3)
#define DOEPINT_XFRC    (1u << 0)
#define DOEPINT_EPDISD  (1u << 1)
#define DOEPINT_SETUP   (1u << 3)

#define EPCTL_USBAEP    (1u << 15)
#define EPCTL_EPTYP(n)  ((uint32_t)(n) << 18)
#define EPCTL_STALL     (1u << 21)
#define EPCTL_TXFNUM(n) ((uint32_t)(n) << 22)
#define EPCTL_CNAK      (1u << 26)
#define EPCTL_SNAK      (1u << 27)
#define EPCTL_EPDIS     (1u << 30)
#define EPCTL_EPENA     (1u << 31)

/* ------------------------------------------------------------------
 * 描述符
 * ------------------------------------------------------------------ */
static const uint8_t dev_desc[18] =
{
    0x12, 0x01,             /* bLength, DEVICE */
    0x00, 0x02,             /* bcdUSB 2.00 */
    0x02, 0x00, 0x00,       /* class=CDC, subclass, protocol */
    0x40,                   /* bMaxPacketSize0 = 64 */
    0x83, 0x04,             /* idVendor  0x0483 */
    0x40, 0x57,             /* idProduct 0x5740 */
    0x00, 0x02,             /* bcdDevice 2.00 */
    0x01, 0x02, 0x03,       /* iManufacturer/iProduct/iSerial */
    0x01                    /* bNumConfigurations */
};

/* 配置描述符：9 + 9 + 5 + 5 + 4 + 5 + 7 + 9 + 7 + 7 = 67 (0x43) */
static const uint8_t cfg_desc[67] =
{
    /* Configuration */
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,
    /* Interface 0: CDC communication */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* Header functional */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* Call management */
    0x05, 0x24, 0x01, 0x00, 0x01,
    /* ACM */
    0x04, 0x24, 0x02, 0x02,
    /* Union */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* EP1 IN, interrupt */
    0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x10,
    /* Interface 1: CDC data */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* EP2 OUT, bulk */
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
    /* EP3 IN, bulk */
    0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00
};

/* 设备限定符（高速设备才需要；这里给一个合法的，避免某些主机报错） */
static const uint8_t qual_desc[10] =
{
    0x0A, 0x06, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40, 0x01, 0x00
};

/* ------------------------------------------------------------------
 * 状态
 * ------------------------------------------------------------------ */
#define EP0_IDLE        0
#define EP0_IN_DATA     1
#define EP0_OUT_DATA    2
#define EP0_STATUS_IN   3
#define EP0_STATUS_OUT  4

typedef struct
{
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_t;

static volatile uint8_t  ep0_state;
static volatile uint8_t  usb_ready;          /* SET_CONFIGURATION 完成 */
static volatile uint8_t  ep3_busy;           /* EP3 IN 正在传 */
static uint8_t           setup_buf[8];

/* 命令下行缓冲：PC 往 COM5 写 'c'/'x'/'u' 等，从这里取
 * （CDC 的 EP2 OUT，之前是丢弃的，现在用来收命令） */
#define USB_CMD_SIZE    64u
static uint8_t           rx_cmd[USB_CMD_SIZE];
static volatile uint16_t rx_wr;
static volatile uint16_t rx_rd;

/* 调试计数：枚举卡在哪一步，看 'u' 命令的回显就知道 */
static volatile uint32_t ev_reset;           /* 总线复位次数 */
static volatile uint32_t ev_enum;            /* ENUMDNE 次数（主机完成速度协商） */
static volatile uint32_t ev_setup;           /* 收到的 SETUP 包数 */
static volatile uint32_t ev_ep0in;           /* EP0 IN 完成次数 */
static volatile uint32_t ev_ep0out;          /* EP0 OUT 完成次数 */
static volatile uint32_t ev_ep0arm;          /* ep0_in() 被调用次数 */
static volatile uint32_t dbg_diepctl0;       /* 武装后 DIEPCTL0 的读回值 */
static volatile uint32_t dbg_diepint0;       /* 处理完时 DIEPINT0 的值 */
static uint8_t           line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 };

/* TX 环形队列 */
static uint8_t           tx_ring[USB_CDC_TX_SIZE];
static volatile uint16_t tx_wr;
static volatile uint16_t tx_rd;
static volatile uint32_t tx_drop;

/* 一次武装多少个 64 字节包。
 * EP3 的 TX FIFO 是 96 字 = 384 B，正好放 6 个包。
 * 一次装 6 包比一次装 1 包快很多：中断次数降 6 倍，
 * 每 1 ms 帧能塞进更多包（实测 12 包/帧 → 20 包/帧）。
 * 注意：这个值不能超过 FIFO 容量 / 64，否则数据会被丢。 */
#define EP3_PKTS        6u
#define EP3_BATCH       (EP3_PKTS * 64u)

static uint8_t           tx_pkt[EP3_BATCH];

/* ------------------------------------------------------------------
 * 小工具
 * ------------------------------------------------------------------ */
static void usb_delay(volatile uint32_t n)
{
    while (n--)
    {
    }
}

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void usb_write_fifo(uint8_t ep, const uint8_t *p, uint16_t n)
{
    uint16_t i;

    for (i = 0; i < n; i += 4u)
    {
        uint32_t w = (uint32_t)p[i];

        if ((uint16_t)(i + 1u) < n) { w |= (uint32_t)p[i + 1u] << 8; }
        if ((uint16_t)(i + 2u) < n) { w |= (uint32_t)p[i + 2u] << 16; }
        if ((uint16_t)(i + 3u) < n) { w |= (uint32_t)p[i + 3u] << 24; }

        FIFO(ep) = w;
    }
}

static void usb_read_fifo(uint8_t ep, uint8_t *p, uint16_t n)
{
    uint16_t i;

    for (i = 0; i < n; i += 4u)
    {
        uint32_t w = FIFO(ep);

        p[i] = (uint8_t)(w);
        if ((uint16_t)(i + 1u) < n) { p[i + 1u] = (uint8_t)(w >> 8); }
        if ((uint16_t)(i + 2u) < n) { p[i + 2u] = (uint8_t)(w >> 16); }
        if ((uint16_t)(i + 3u) < n) { p[i + 3u] = (uint8_t)(w >> 24); }
    }
}

/* ------------------------------------------------------------------
 * EP0
 * ------------------------------------------------------------------ */
static void ep0_in(const uint8_t *p, uint16_t n)
{
    uint16_t pkts = (uint16_t)((n + 63u) / 64u);
    uint32_t guard;

    ev_ep0arm++;

    /* 如果上一次还没传完（EPENA 仍为 1），先关掉再重新装，
       否则 DIEPTSIZ 的写入会被硬件忽略 */
    if ((DIEPCTL(0) & EPCTL_EPENA) != 0u)
    {
        DIEPCTL(0) |= EPCTL_EPDIS;
        guard = 100000UL;
        while (((DIEPCTL(0) & EPCTL_EPENA) != 0u) && (--guard != 0u)) { }
    }

    /* 顺序很重要：先装长度、再使能、最后灌 FIFO（和 ST 的库一致） */
    DIEPTSIZ(0) = ((uint32_t)pkts << 19) | (uint32_t)n;
    DIEPCTL(0) |= EPCTL_EPENA | EPCTL_CNAK;

    if (n != 0u)
    {
        usb_write_fifo(0, p, n);
    }

    dbg_diepctl0 = DIEPCTL(0);

    ep0_state = EP0_IN_DATA;
}

static void ep0_status_in(void)
{
    DIEPTSIZ(0) = 0;
    DIEPCTL(0) |= EPCTL_EPENA | EPCTL_CNAK;

    ep0_state = EP0_STATUS_IN;
}

static void ep0_out(uint16_t n)
{
    uint16_t pkts = (uint16_t)((n + 63u) / 64u);

    if (pkts == 0u)
    {
        pkts = 1u;
    }

    DOEPTSIZ(0) = (1uL << 29) | ((uint32_t)pkts << 19) | (uint32_t)n;
    DOEPCTL(0) |= EPCTL_EPENA | EPCTL_CNAK;

    ep0_state = EP0_OUT_DATA;
}

static void ep0_status_out(void)
{
    DOEPTSIZ(0) = (1uL << 29) | (1uL << 19);
    DOEPCTL(0) |= EPCTL_EPENA | EPCTL_CNAK;

    ep0_state = EP0_STATUS_OUT;
}

/* ------------------------------------------------------------
 * 重新武装 EP0 OUT，等下一个 SETUP
 *
 * 这一步绝对不能漏：每次控制传输（数据阶段 + 状态阶段）结束后，
 * 硬件会把 EP0 OUT 关掉；不重新武装，主机发的下一个 SETUP 就收不到，
 * 表现就是“设备描述符请求失败”——第一次 GET_DESCRIPTOR 之后卡死。
 * ------------------------------------------------------------ */
static void ep0_rx_setup(void)
{
    DOEPTSIZ(0) = (1uL << 29) | (1uL << 19);
    DOEPCTL(0) |= EPCTL_EPENA | EPCTL_CNAK;
}

static void ep0_stall(void)
{
    DIEPCTL(0) |= EPCTL_STALL;
    DOEPCTL(0) |= EPCTL_STALL;
    ep0_state = EP0_IDLE;
}

/* ------------------------------------------------------------------
 * 字符串描述符
 * ------------------------------------------------------------------ */
static const char *str_tab[] =
{
    "",
    "STMicro",
    "MIC16 RAW CDC",
    "0001"
};

static uint16_t usb_string_desc(uint8_t idx, uint8_t *buf)
{
    uint16_t i;
    uint16_t len;

    if ((idx == 0u) || (idx > 3u))
    {
        buf[0] = 4; buf[1] = 0x03;
        buf[2] = 0x09; buf[3] = 0x04;       /* 0x0409 = 英文 */
        return 4;
    }

    len = 0;
    while (str_tab[idx][len] != '\0')
    {
        len++;
    }

    buf[0] = (uint8_t)(2u + 2u * len);
    buf[1] = 0x03;

    for (i = 0; i < len; i++)
    {
        buf[2u + 2u * i] = (uint8_t)str_tab[idx][i];
        buf[3u + 2u * i] = 0;
    }

    return (uint16_t)(2u + 2u * len);
}

/* 把描述符拷进临时缓冲，返回实际长度 */
static uint16_t copy_desc(uint8_t *dst, const uint8_t *src, uint16_t srclen, uint16_t want)
{
    uint16_t n = (want < srclen) ? want : srclen;
    uint16_t i;

    for (i = 0; i < n; i++)
    {
        dst[i] = src[i];
    }

    return n;
}

/* ------------------------------------------------------------------
 * 处理 SETUP
 * ------------------------------------------------------------------ */
static void usb_setup(const uint8_t *s8)
{
    usb_setup_t s;
    static uint8_t tmp[128];
    uint16_t n;
    uint16_t want;

    s.bmRequestType = s8[0];
    s.bRequest      = s8[1];
    s.wValue        = rd16(s8 + 2);
    s.wIndex        = rd16(s8 + 4);
    s.wLength       = rd16(s8 + 6);
    want            = s.wLength;

    switch (s.bRequest)
    {
    case 0x06:                              /* GET_DESCRIPTOR */
        switch ((uint8_t)(s.wValue >> 8))
        {
        case 0x01:                          /* 设备 */
            n = copy_desc(tmp, dev_desc, (uint16_t)sizeof(dev_desc), want);
            ep0_in(tmp, n);
            break;

        case 0x02:                          /* 配置 */
            n = copy_desc(tmp, cfg_desc, (uint16_t)sizeof(cfg_desc), want);
            ep0_in(tmp, n);
            break;

        case 0x03:                          /* 字符串 */
            n = usb_string_desc((uint8_t)(s.wValue & 0xFFu), tmp);
            if (n > want)
            {
                n = want;
            }
            ep0_in(tmp, n);
            break;

        case 0x06:                          /* 设备限定符 */
            n = copy_desc(tmp, qual_desc, (uint16_t)sizeof(qual_desc), want);
            ep0_in(tmp, n);
            break;

        default:
            ep0_stall();
            break;
        }
        break;

    case 0x05:                              /* SET_ADDRESS */
        DCFG = (DCFG & ~0x7F0u) | ((uint32_t)(s.wValue & 0x7Fu) << 4);
        ep0_status_in();
        break;

    case 0x09:                              /* SET_CONFIGURATION */
        if ((s.wValue & 0xFFu) != 0u)
        {
            /* EP1 IN 中断（CDC 通知，本工程不发） */
            DIEPCTL(1) = 64u | EPCTL_USBAEP | EPCTL_EPTYP(3) | EPCTL_TXFNUM(1)
                       | EPCTL_EPENA | EPCTL_CNAK;

            /* EP2 OUT 批量（下行，丢弃） */
            DOEPTSIZ(2) = (1uL << 19) | 64u;
            DOEPCTL(2) = 64u | EPCTL_USBAEP | EPCTL_EPTYP(2)
                       | EPCTL_EPENA | EPCTL_CNAK;

            /* EP3 IN 批量（上行，数据从这里走） */
            DIEPCTL(3) = 64u | EPCTL_USBAEP | EPCTL_EPTYP(2) | EPCTL_TXFNUM(3)
                       | EPCTL_EPENA | EPCTL_CNAK;

            ep3_busy  = 0;
            usb_ready = 1;
        }
        else
        {
            usb_ready = 0;
        }
        ep0_status_in();
        break;

    case 0x08:                              /* GET_CONFIGURATION */
        tmp[0] = usb_ready ? 1u : 0u;
        ep0_in(tmp, 1);
        break;

    case 0x00:                              /* GET_STATUS */
        tmp[0] = 0; tmp[1] = 0;
        ep0_in(tmp, 2);
        break;

    case 0x01:                              /* CLEAR_FEATURE */
    case 0x03:                              /* SET_FEATURE */
        ep0_status_in();
        break;

    case 0x20:                              /* SET_LINE_CODING (OUT 7 B) */
        ep0_out(7);
        break;

    case 0x21:                              /* GET_LINE_CODING */
        ep0_in(line_coding, 7);
        break;

    case 0x22:                              /* SET_CONTROL_LINE_STATE */
        ep0_status_in();
        break;

    default:
        ep0_stall();
        break;
    }
}

/* ------------------------------------------------------------------
 * 复位 / 配置端点
 * ------------------------------------------------------------------ */
static void usb_flush_all_fifo(void)
{
    uint32_t guard;

    guard = 1000000UL;
    GRSTCTL |= (1uL << 4);                          /* RX FIFO 刷新 */
    while (((GRSTCTL & (1uL << 4)) != 0u) && (--guard != 0u)) { }

    guard = 1000000UL;
    GRSTCTL |= (0x10uL << 6) | (1uL << 5);          /* TX FIFO 全部刷新 */
    while (((GRSTCTL & (1uL << 5)) != 0u) && (--guard != 0u)) { }
}

static void usb_bus_reset(void)
{
    ev_reset++;
    DCFG &= ~0x7F0u;                                /* 地址归零 */
    usb_flush_all_fifo();

    usb_ready = 0;
    ep3_busy  = 0;
    ep0_state = EP0_IDLE;

    /* EP0 重新武装 */
    DIEPCTL(0) = 64u;
    DOEPCTL(0) = 64u;
    DIEPTSIZ(0) = 0;
    DOEPTSIZ(0) = (1uL << 29) | (1uL << 19);
    DOEPCTL(0) |= EPCTL_EPENA | EPCTL_CNAK;

    /* 其余端点关掉 */
    DIEPCTL(1) = 0;
    DOEPCTL(2) = 0;
    DIEPCTL(3) = 0;
}

/* ------------------------------------------------------------------
 * 中断处理
 * ------------------------------------------------------------------ */
static void usb_rx_handler(void)
{
    while ((GINTSTS & GINT_RXFLVL) != 0u)
    {
        uint32_t sts = GRXSTSP;
        uint8_t  ep  = (uint8_t)(sts & 0x0Fu);
        uint16_t cnt = (uint16_t)((sts >> 4) & 0x7FFu);
        uint8_t  pkt = (uint8_t)((sts >> 17) & 0x0Fu);

        if (pkt == 0x06u)                       /* SETUP 包 */
        {
            ev_setup++;
            usb_read_fifo(0, setup_buf, 8);
            usb_setup(setup_buf);
        }
        else if (pkt == 0x02u)                  /* OUT 数据 */
        {
            if (ep == 0u)
            {
                usb_read_fifo(0, line_coding, (cnt > 7u) ? 7u : cnt);
            }
            else if (ep == 2u)
            {
                uint8_t  tmp[64];
                uint16_t n = (cnt > 64u) ? 64u : cnt;
                uint16_t i;

                usb_read_fifo(2, tmp, n);

                for (i = 0; i < n; i++)
                {
                    uint16_t nxt = (uint16_t)((rx_wr + 1u) & (USB_CMD_SIZE - 1u));

                    if (nxt != rx_rd)
                    {
                        rx_cmd[rx_wr] = tmp[i];
                        rx_wr = nxt;
                    }
                }
            }
        }
        else
        {
            /* 0x03 OUT 完成 / 0x04 SETUP 完成：不需要动作 */
        }
    }
}

static void usb_iep_handler(void)
{
    uint32_t aint = DAINT & 0xFFFFu;
    uint8_t  n;

    for (n = 0; n < 4u; n++)
    {
        if ((aint & (1uL << n)) == 0u)
        {
            continue;
        }

        if ((DIEPINT(n) & DIEPINT_XFRC) != 0u)
        {
            DIEPINT(n) = DIEPINT_XFRC;

            if (n == 0u)
            {
                ev_ep0in++;

                if (ep0_state == EP0_IN_DATA)
                {
                    ep0_status_out();           /* IN 数据发完，等状态 OUT */
                }
                else if (ep0_state == EP0_STATUS_IN)
                {
                    ep0_state = EP0_IDLE;
                    ep0_rx_setup();             /* 状态阶段结束，等下一个 SETUP */
                }
            }
            else if (n == 3u)
            {
                ep3_busy = 0;
                usb_cdc_poll();                 /* 立刻续包，这是提速关键 */
            }
        }

        if ((DIEPINT(n) & DIEPINT_EPDISD) != 0u)
        {
            DIEPINT(n) = DIEPINT_EPDISD;
        }

        if ((DIEPINT(n) & DIEPINT_TIMEOUT) != 0u)
        {
            DIEPINT(n) = DIEPINT_TIMEOUT;
        }
    }

    dbg_diepint0 = DIEPINT(0);
}

static void usb_oep_handler(void)
{
    uint32_t aint = (DAINT >> 16) & 0xFFFFu;
    uint8_t  n;

    for (n = 0; n < 4u; n++)
    {
        if ((aint & (1uL << n)) == 0u)
        {
            continue;
        }

        if ((DOEPINT(n) & DOEPINT_SETUP) != 0u)
        {
            DOEPINT(n) = DOEPINT_SETUP;
        }

        if ((DOEPINT(n) & DOEPINT_XFRC) != 0u)
        {
            DOEPINT(n) = DOEPINT_XFRC;

            if (n == 0u)
            {
                ev_ep0out++;

                if (ep0_state == EP0_OUT_DATA)
                {
                    ep0_status_in();            /* 收完数据，回状态 IN */
                }
                else if (ep0_state == EP0_STATUS_OUT)
                {
                    ep0_state = EP0_IDLE;
                    ep0_rx_setup();             /* 状态阶段结束，等下一个 SETUP */
                }
            }
            else if (n == 2u)
            {
                /* 下行数据（本工程不用），重新武装以便下次继续收 */
                DOEPTSIZ(2) = (1uL << 19) | 64u;
                DOEPCTL(2) |= EPCTL_EPENA | EPCTL_CNAK;
            }
        }

        if ((DOEPINT(n) & DOEPINT_EPDISD) != 0u)
        {
            DOEPINT(n) = DOEPINT_EPDISD;
        }
    }
}

void OTG_FS_IRQHandler(void)
{
    uint32_t sts = GINTSTS;

    if ((sts & GINT_RXFLVL) != 0u)
    {
        usb_rx_handler();
    }

    if ((sts & GINT_IEPINT) != 0u)
    {
        usb_iep_handler();
    }

    if ((sts & GINT_OEPINT) != 0u)
    {
        usb_oep_handler();
    }

    if ((sts & GINT_ENUMDNE) != 0u)
    {
        GINTSTS = GINT_ENUMDNE;
        ev_enum++;
    }

    if ((sts & GINT_USBRST) != 0u)
    {
        GINTSTS = GINT_USBRST;
        usb_bus_reset();
    }

    /* 清掉没在用的状态位（SOF/NPTXFE/GINAKEFF/GONAKEFF/ESUSP/USBSUSP/EOPF），
       避免它们一直挂着误导判断 */
    GINTSTS = (1uL << 3) | (1uL << 5) | (1uL << 6) | (1uL << 7)
            | (1uL << 10) | (1uL << 11) | (1uL << 15);
}

/* ------------------------------------------------------------------
 * 公共接口
 * ------------------------------------------------------------------ */
uint16_t usb_cdc_free(void)
{
    return (uint16_t)((tx_rd - tx_wr - 1u) & (USB_CDC_TX_SIZE - 1u));
}

uint16_t usb_cdc_write(const uint8_t *p, uint16_t n)
{
    uint16_t i;
    uint16_t room;
    uint16_t wr;
    uint32_t pm;

    pm = __get_PRIMASK();
    __disable_irq();

    room = (uint16_t)((tx_rd - tx_wr - 1u) & (USB_CDC_TX_SIZE - 1u));
    if (room < n)
    {
        tx_drop += (uint32_t)(n - room);
        n = room;
    }

    wr = tx_wr;
    for (i = 0; i < n; i++)
    {
        tx_ring[(uint16_t)((wr + i) & (USB_CDC_TX_SIZE - 1u))] = p[i];
    }
    tx_wr = (uint16_t)((wr + n) & (USB_CDC_TX_SIZE - 1u));

    __set_PRIMASK(pm);

    return n;
}

void usb_cdc_poll(void)
{
    uint16_t n;
    uint16_t i;
    uint16_t rd;
    uint16_t pkts;
    uint32_t pm;

    if ((usb_ready == 0u) || (ep3_busy != 0u))
    {
        return;
    }

    pm = __get_PRIMASK();
    __disable_irq();

    if ((ep3_busy != 0u) || (tx_rd == tx_wr))
    {
        __set_PRIMASK(pm);
        return;
    }

    /* 一次取最多 6 个包（384 B） */
    n = (uint16_t)((tx_wr - tx_rd) & (USB_CDC_TX_SIZE - 1u));
    if (n > EP3_BATCH)
    {
        n = (uint16_t)EP3_BATCH;
    }

    rd = tx_rd;
    for (i = 0; i < n; i++)
    {
        tx_pkt[i] = tx_ring[(uint16_t)((rd + i) & (USB_CDC_TX_SIZE - 1u))];
    }
    tx_rd = (uint16_t)((rd + n) & (USB_CDC_TX_SIZE - 1u));

    ep3_busy = 1;

    __set_PRIMASK(pm);

    pkts = (uint16_t)((n + 63u) / 64u);

    /* 先装长度、再使能、最后灌 FIFO（和 ST 的库一致） */
    DIEPTSIZ(3) = ((uint32_t)pkts << 19) | (uint32_t)n;
    DIEPCTL(3) |= EPCTL_EPENA | EPCTL_CNAK;
    usb_write_fifo(3, tx_pkt, n);
}

uint8_t usb_cdc_configured(void)
{
    return usb_ready;
}

/* 从 USB 数据口取一个命令字节（PC 往 COM5 写的） */
uint8_t usb_cdc_read(uint8_t *ch)
{
    uint8_t got = 0;
    uint32_t pm = __get_PRIMASK();

    __disable_irq();
    if (rx_rd != rx_wr)
    {
        *ch = rx_cmd[rx_rd];
        rx_rd = (uint16_t)((rx_rd + 1u) & (USB_CMD_SIZE - 1u));
        got = 1;
    }
    __set_PRIMASK(pm);

    return got;
}

uint32_t usb_cdc_dropped(void)
{
    return tx_drop;
}

uint8_t usb_cdc_get_line_coding(uint8_t *out7)
{
    uint8_t i;

    for (i = 0; i < 7u; i++)
    {
        out7[i] = line_coding[i];
    }

    return 1;
}

void usb_cdc_stats(uint32_t *rst, uint32_t *enums, uint32_t *setups,
                   uint32_t *in, uint32_t *out)
{
    if (rst    != 0) { *rst    = ev_reset; }
    if (enums  != 0) { *enums  = ev_enum; }
    if (setups != 0) { *setups = ev_setup; }
    if (in     != 0) { *in     = ev_ep0in; }
    if (out    != 0) { *out    = ev_ep0out; }
}

/* 寄存器快照：给 'u' 命令排查用 */
uint32_t usb_cdc_reg(uint8_t which)
{
    switch (which)
    {
    case 0u: return GINTSTS;
    case 1u: return DCTL;
    case 2u: return DCFG;
    case 3u: return DSTS;
    case 4u: return GOTGCTL;
    case 5u: return GCCFG;
    case 6u: return GINTMSK;
    case 7u: return GAHBCFG;
    case 8u: return GUSBCFG;
    default: return 0u;
    }
}

/* ------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------ */
void usb_cdc_init(void)
{
    GPIO_InitTypeDef  g;
    NVIC_InitTypeDef  n;

    /* 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);

    /* PA11 = D-，PA12 = D+ */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG_FS);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG_FS);

    g.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
    g.GPIO_Mode  = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &g);

    /* 先断开，配置期间不让主机枚举 */
    DCTL |= DCTL_SDIS;

    /* 选内部全速 PHY：清 PHYLPCS，强制设备模式 */
    GUSBCFG &= ~(1uL << 23);
    GUSBCFG &= ~(1uL << 29);
    GUSBCFG |=  (1uL << 30);

    /* 软复位 */
    {
        uint32_t guard = 1000000UL;

        GRSTCTL |= 1u;
        while (((GRSTCTL & 1u) != 0u) && (--guard != 0u)) { }
    }
    usb_delay(20000);
    GRSTCTL = 0;
    {
        uint32_t guard = 1000000UL;

        while (((GRSTCTL & (1uL << 31)) == 0u) && (--guard != 0u)) { }
    }

    /* 软复位之后必须再设一次！
       有些情况下 CSRST 会把 GUSBCFG 恢复默认（主机模式），
       那样核根本不响应主机 —— 现象就是“有设备但无法识别”。
       所以这里再写一遍，确保是设备模式 + 内部 FS PHY。 */
    GUSBCFG &= ~(1uL << 23);        /* PHYLPCS = 0：内部全速 PHY */
    GUSBCFG &= ~(1uL << 29);        /* 清 FORCEHOSTMODE */
    GUSBCFG |=  (1uL << 30);        /* FORCEDEVMODE = 1 */

    /* 全速 12M */
    DCFG = (DCFG & ~0x3u) | 0x3u;
    DCFG &= ~0x7F0u;

    /* FIFO 分配 */
    GRXFSIZ    = 128u;
    GNPTXFSIZ  = (64uL << 16) | 128u;
    DIEPTXF(1) = (16uL << 16) | 192u;
    DIEPTXF(2) = 0u;
    DIEPTXF(3) = (96uL << 16) | 208u;

    /* 清所有中断标志 */
    GINTSTS = 0xFFFFFFFFu;

    /* 打开需要的中断 */
    GINTMSK = GINT_USBRST | GINT_ENUMDNE | GINT_RXFLVL | GINT_IEPINT | GINT_OEPINT;

    GAHBCFG |= 1u;                          /* 全局中断使能 */

    /* 端点中断屏蔽 */
    DAINTMSK = 0xFFFFFFFFu;
    DIEPMSK  = DIEPINT_XFRC | DIEPINT_EPDISD | DIEPINT_TIMEOUT;
    DOEPMSK  = DOEPINT_XFRC | DOEPINT_EPDISD | DOEPINT_SETUP;

    /* EP0 */
    DIEPCTL(0) = 64u;
    DOEPCTL(0) = 64u;
    DOEPTSIZ(0) = (1uL << 29) | (1uL << 19);
    DOEPCTL(0) |= EPCTL_EPENA | EPCTL_CNAK;

    /* GCCFG：上电 + 关 VBUS 检测（PA9 留给串口） */
    GCCFG |= (1uL << 16) | (1uL << 21);

    /* 退出电源/时钟门控（ST 的库在 CoreInit 最后也写这一句） */
    PCGCCTL = 0;

    /* NVIC：优先级低于 ADC 的 DMA（那是 0） */
    n.NVIC_IRQChannel                   = OTG_FS_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 2;
    n.NVIC_IRQChannelSubPriority        = 0;
    n.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&n);

    /* 等 PHY 稳定（几 ms），再连到主机 */
    usb_delay(200000);

    /* 上电、连接：清 SDIS 和 RWUSIG */
    DCTL &= ~(DCTL_SDIS | DCTL_RWUSIG);

    tx_wr  = 0;
    tx_rd  = 0;
    tx_drop = 0;
    rx_wr  = 0;
    rx_rd  = 0;
}
