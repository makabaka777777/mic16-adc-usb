#include "mic.h"
#include "uart.h"
#include "usb_cdc.h"

/* ============================================================
 * 十六路麦克风：ADC1 扫 6 通道，ADC2/ADC3 各扫 5 通道，各配一条 DMA
 *
 * DMA 分配（F407 上这三条流不冲突，别再改）：
 *   ADC1 → DMA2_Stream4  Channel0
 *   ADC2 → DMA2_Stream2  Channel1
 *   ADC3 → DMA2_Stream0  Channel2
 *
 * 注意 ADC1 和 ADC3 都能走 Stream0（Ch0 / Ch2），所以 ADC1 必须让到
 * Stream4，否则两者抢同一条流。
 *
 * 每条流搬的是“帧”：每触发一次，ADC 先转 rank1 再转 rank2 ... rankN，
 * DMA 连续搬 [rank1, rank2, ...]，所以缓冲里是交织的。
 * ADC1 的帧 = 6 个半字，ADC2/ADC3 的帧 = 5 个半字。
 * ============================================================ */

/* 每颗 ADC 的通道数（= 每帧的半字数） */
static const uint16_t adc_lanes[MIC_ADC_N] = { 6, 5, 5 };

/* 三块环形缓冲，按各 ADC 的实际帧大小分配，不浪费 RAM：
   6*2048 + 5*2048 + 5*2048 = 32768 个半字 = 64KB */
static volatile uint16_t mic_buf1[MIC_LANES_MAX * MIC_RING];
static volatile uint16_t mic_buf2[5u * MIC_RING];
static volatile uint16_t mic_buf3[5u * MIC_RING];
static volatile uint16_t *const mic_buf[MIC_ADC_N] = { mic_buf1, mic_buf2, mic_buf3 };

static ADC_TypeDef *const adcs[MIC_ADC_N] = { ADC1, ADC2, ADC3 };

typedef struct
{
    DMA_Stream_TypeDef *stream;
    uint32_t            tc_flag;    /* 这条流的传输完成标志 */
    volatile uint64_t   base;       /* 已搬完的整帧数（绝对帧号） */
} mic_hw_t;

static mic_hw_t hw[MIC_ADC_N] =
{
    { DMA2_Stream4, DMA_FLAG_TCIF4, 0 },    /* ADC1 → MIC1..MIC6 */
    { DMA2_Stream2, DMA_FLAG_TCIF2, 0 },    /* ADC2 → MIC7..MIC11 */
    { DMA2_Stream0, DMA_FLAG_TCIF0, 0 },    /* ADC3 → MIC12..MIC16 */
};

/* 通道号 → (哪颗 ADC, 该 ADC 里的第几个 lane)
 * lane 0 是 rank1，lane 5 是 rank6。
 *   1..6   = ADC1 的 rank1..6
 *   7..11  = ADC2 的 rank1..5
 *   12..16 = ADC3 的 rank1..5
 * 三颗 ADC 的 rank k 由同一个 TRGO 同时启动，是同一时刻采样的。
 */
static const uint8_t ch_adc[MIC_CHN]  =
{
    0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1,
    2, 2, 2, 2, 2
};
static const uint8_t ch_lane[MIC_CHN] =
{
    0, 1, 2, 3, 4, 5,
    0, 1, 2, 3, 4,
    0, 1, 2, 3, 4
};

/* 每颗 ADC 的扫描通道号（按 rank 顺序）：
 *   ADC1：IN1..IN6              = PA1..PA6
 *   ADC2：IN7,IN8,IN9,IN14,IN15 = PA7 PB0 PB1 PC4 PC5
 *   ADC3：IN0,IN10,IN11,IN12,IN13 = PA0 PC0 PC1 PC2 PC3
 *
 * 注意：LQFP100 封装的 F407VGT6 没有 PF 引脚（PF6~PF10 只在 144 脚/
 * BGA 封装上），所以 ADC3 用不了 PF，只能从 PA0~PA3、PC0~PC3 里选。
 * LQFP100 的 ADC 可用引脚总共 16 个：PA0~PA7、PB0/PB1、PC0~PC5，
 * 16 路正好一路一个。
 * 换引脚时这里和 mic.h 的 GPIO 掩码必须成对改。 */
static const uint8_t adc_ch[MIC_ADC_N][MIC_LANES_MAX] =
{
    { ADC_Channel_1,  ADC_Channel_2,  ADC_Channel_3,
      ADC_Channel_4,  ADC_Channel_5,  ADC_Channel_6 },
    { ADC_Channel_7,  ADC_Channel_8,  ADC_Channel_9,
      ADC_Channel_14, ADC_Channel_15, 0 },
    { ADC_Channel_0,  ADC_Channel_10, ADC_Channel_11,
      ADC_Channel_12, ADC_Channel_13, 0 }
};

static uint64_t g_rd = 0;               /* 主循环已经处理到的绝对帧号 */
static uint32_t g_fs = MIC_FS_HZ;       /* 实际采样率（取整后的） */
static uint16_t g_arr = 0;
static uint16_t g_smp_cyc = 0;          /* 当前 ADC 采样周期数，打印用 */

volatile uint32_t mic_lost     = 0;
volatile uint32_t mic_blocks   = 0;
volatile uint32_t mic_restarts = 0;
volatile uint32_t mic_ovr      = 0;     /* ADC 溢出次数：触发打在转换中途 */

/* 最近一块算出的值，'A'/'V' 命令直接取 */
static volatile uint32_t last_mean_mv[MIC_CHN];
static volatile uint32_t last_rms_mv[MIC_CHN];
static volatile uint8_t  last_valid = 0;

/* ============================================================
 * 原始数据直通状态（RAW 版）
 *
 * 没有累加器：ADC 采到的点直接进 raw_out[] 就发走，一个字节都不改。
 * raw_out 只当"当前这一帧的 16 个通道值"的暂存。
 * ============================================================ */
static uint8_t  raw_on     = 0;
static uint16_t raw_seq    = 0;
static uint16_t raw_out[MIC_CHN];
static uint32_t raw_frames = 0;
static uint32_t raw_drop   = 0;

static void raw_block(uint32_t f);

/* 带宽自检：1:1 直通的帧率 × 帧长 必须落在 USB CDC 预算内（按 1.0 MB/s 算） */
#if ((MIC_FS_HZ * MIC_RAW_FRAME) > 1000000UL)
#error "1:1 raw stream exceeds USB CDC budget (1 MB/s)"
#endif

/* 1:1 直通的每路最高采样率：按 1 MB/s 预算的 91% 取整（24 kHz） */
#define MIC_RAW_MAX_FS  24000UL

/* 采样时间候选：周期数越小、转换越快、允许的 fs 越高，但采样电容
 * 充电时间越短。顺序从小到大，自动挑“能塞进 90% 触发周期”的最大值。 */
typedef struct
{
    uint16_t cycles;
    uint8_t  code;
} mic_smp_t;

static const mic_smp_t smp_tab[] =
{
    {   3, ADC_SampleTime_3Cycles   },
    {  15, ADC_SampleTime_15Cycles  },
    {  28, ADC_SampleTime_28Cycles  },
    {  56, ADC_SampleTime_56Cycles  },
    {  84, ADC_SampleTime_84Cycles  },
    { 112, ADC_SampleTime_112Cycles },
    { 144, ADC_SampleTime_144Cycles },
    { 480, ADC_SampleTime_480Cycles },
};

static void adc_set_smp(uint8_t smp);

/* ============================================================
 * 小工具
 * ============================================================ */

static void delay_loop(uint32_t n)
{
    volatile uint32_t i;
    for (i = 0; i < n; i++)
    {
    }
}

static uint32_t isqrt32(uint32_t v)
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30;

    while (bit > v)
    {
        bit >>= 2;
    }

    while (bit)
    {
        if (v >= res + bit)
        {
            v   -= res + bit;
            res  = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }

    return res;
}

/* ============================================================
 * 采样率规划
 * ============================================================ */

static uint8_t pick_smp(uint32_t hz, uint16_t *cyc_out)
{
    uint32_t period = MIC_ADC_CLK_HZ / hz;
    uint32_t i;
    uint8_t  code = 0xFF;

    for (i = 0; i < (sizeof(smp_tab) / sizeof(smp_tab[0])); i++)
    {
        /* 最重那颗 ADC（6 通道）的整串转换时间 <= 触发周期的 90%：
           6*(cyc+12) <= period*0.9  →  6*(cyc+12)*10 <= period*9
           这样 ADC 在下一个触发到来前一定已经转完，触发不会打在
           转换中途被丢掉（那才是真正的“丢数”）。 */
        if (((uint32_t)MIC_LANES_MAX * (smp_tab[i].cycles + 12u) * 10u) <=
            (period * 9u))
        {
            code = smp_tab[i].code;
            if (cyc_out)
            {
                *cyc_out = smp_tab[i].cycles;
            }
        }
    }

    return code;
}

static uint8_t fs_plan(uint32_t hz, uint16_t *arr, uint8_t *smp, uint16_t *cyc)
{
    uint32_t a;
    uint8_t  code;
    uint16_t c = 0;

    if ((hz < MIC_MIN_FS_HZ) || (hz > MIC_MAX_FS_HZ))
    {
        return 0;
    }

    a = MIC_TIM_CLK_HZ / hz;            /* 整数分频，实际 fs 略高于请求值 */
    if ((a < 8u) || (a > 65536u))
    {
        return 0;
    }

    code = pick_smp(hz, &c);
    if (code == 0xFF)
    {
        return 0;
    }

    *arr = (uint16_t)(a - 1u);
    *smp = code;
    if (cyc)
    {
        *cyc = c;
    }

    return 1;
}

/* ============================================================
 * 写指针
 *
 * base 只在 TC 中断里更新，NDTR 由 DMA 硬件递减。分开读会打架：
 * 硬件重装 NDTR 和中断更新 base 不是同一时刻。
 *
 * 关中断后按 nd → base → TC 标志 的顺序读一次：
 *
 *   - TC 没置位：base 是当前圈，NDTR 就是圈内偏移，直接算；
 *   - TC 置位：说明整圈已经搬完、中断还没把 base 加上，本地补一圈。
 *     这时再看 NDTR：
 *       · nd >= tot/2 → NDTR 已经落到新圈，补圈后再加圈内偏移；
 *       · nd <  tot/2 → 可能还是旧圈末尾（TC 恰好发生在读 nd 之后），
 *                        只报到整圈边界，圈内偏移留给下一次。
 *
 * 关键是“宁可少报，绝不多报”：少报最多让主循环晚几微秒取数，
 * 多报会去读 DMA 还没写的区域，那才是真的读到脏数据。
 *
 * 缓冲是交织的，NDTR 记的是半字数，除以该 ADC 的通道数才是帧数。
 * ============================================================ */
static uint64_t mic_wr(uint32_t a)
{
    uint32_t pm;
    uint16_t nd;
    uint32_t tot  = (uint32_t)adc_lanes[a] * MIC_RING;   /* 半字总数 */
    uint32_t half = tot >> 1;
    uint64_t b, wr;
    uint8_t  pending;

    pm = __get_PRIMASK();
    __disable_irq();

    nd      = DMA_GetCurrDataCounter(hw[a].stream);
    b       = hw[a].base;
    pending = (DMA_GetFlagStatus(hw[a].stream, hw[a].tc_flag) != RESET) ? 1u : 0u;

    __set_PRIMASK(pm);

    if (pending)
    {
        if (nd >= half)
        {
            b += MIC_RING;              /* 整圈完成，NDTR 已在新圈 */
            wr = b + (uint64_t)((tot - nd) / adc_lanes[a]);
        }
        else
        {
            wr = b + MIC_RING;          /* 只报到整圈边界，绝不往前多算 */
        }
    }
    else
    {
        wr = b + (uint64_t)((tot - nd) / adc_lanes[a]);
    }

    if (wr < g_rd)
    {
        wr += MIC_RING;
    }
    if (wr < g_rd)                  /* 防御：绝不返回比读指针更旧的进度 */
    {
        wr = g_rd;
    }

    return wr;
}

static uint64_t mic_wr_min(void)
{
    uint64_t wr = mic_wr(0);
    uint32_t a;

    for (a = 1; a < MIC_ADC_N; a++)
    {
        uint64_t t = mic_wr(a);
        if (t < wr)
        {
            wr = t;
        }
    }

    return wr;
}

/* ============================================================
 * 中断：只更新“已搬完多少帧”，别的什么都不做
 * ============================================================ */
void DMA2_Stream4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);
        hw[0].base += MIC_RING;
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
        hw[1].base += MIC_RING;
    }
}

void DMA2_Stream0_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        hw[2].base += MIC_RING;
    }
}

/* ============================================================
 * 处理一块：每通道 512 点，算直流均值和交流有效值
 *
 * 交流有效值用单遍公式，避免再做一遍减法：
 *   var = (N*Σx² - (Σx)²) / N²
 * 分子用 64 位；N=512 时 N*Σx² 最大约 4.4e12，32 位装不下。
 * ============================================================ */
static void block_process(uint64_t rd)
{
    uint32_t f = (uint32_t)(rd & (MIC_RING - 1));
    uint32_t c;

    for (c = 0; c < MIC_CHN; c++)
    {
        uint32_t a = ch_adc[c];
        uint32_t l = ch_lane[c];
        const volatile uint16_t *p = &mic_buf[a][adc_lanes[a] * f + l];
        uint32_t i;
        uint32_t sum = 0;
        uint32_t mean, var, rms;
        uint64_t sumsq = 0;
        uint64_t num;

        for (i = 0; i < MIC_BLOCK; i++)
        {
            uint32_t x = p[adc_lanes[a] * i];
            sum   += x;
            sumsq += (uint64_t)x * x;
        }

        mean = (sum + MIC_BLOCK / 2u) / MIC_BLOCK;
        num  = (uint64_t)MIC_BLOCK * sumsq - (uint64_t)sum * sum;
        var  = (uint32_t)(num / ((uint64_t)MIC_BLOCK * MIC_BLOCK));
        rms  = isqrt32(var);

        last_mean_mv[c] = (mean * MIC_VREF_MV + MIC_ADC_FULL / 2u) / MIC_ADC_FULL;
        last_rms_mv[c]  = (rms  * MIC_VREF_MV + MIC_ADC_FULL / 2u) / MIC_ADC_FULL;
    }

    last_valid = 1;

    /* 统计算完再走原始直通：两者用的是同一块数据，不影响原功能 */
    if (raw_on)
    {
        raw_block(f);
    }
}

/* ============================================================
 * 原始数据直通输出（'c' / 'x' / 'w'）
 *
 * 要求就一句话：**ADC 采一个点，串口就发一个点，中间不做任何处理。**
 * 所以这里没有累加、没有平均、没有移位、没有查表 —— raw_out[c] 拿到的
 * 就是 DMA 搬进环形缓冲的原始 uint16（0..4095，12 位右对齐），
 * 直接按小端拆成两个字节塞进帧里。
 *
 * 串口帧数严格等于 ADC 帧数（1:1）：
 *   2000 Hz 采样 → 2000 帧/s × 38 字节 = 76 000 B/s（921600 的 82%）
 * 所以 fs 必须 <= 2425 Hz，否则串口跟不上，帧会被丢（raw_drop）。
 *
 * 整帧入队：队列剩余空间不足 38 字节就整帧丢掉（raw_drop++），
 * 绝不发半帧。PC 端靠 AA55 帧头 + CRC 重新同步，丢帧只影响
 * 帧号连续性，不会造成数据错位。
 * ============================================================ */

static uint16_t crc16_ccitt(const uint8_t *p, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;
    uint8_t  b;

    for (i = 0; i < n; i++)
    {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);

        for (b = 0; b < 8u; b++)
        {
            if ((crc & 0x8000u) != 0u)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

static void raw_emit(void)
{
    uint8_t  f[MIC_RAW_FRAME];
    uint16_t crc;
    uint32_t c;

    /* USB 队列放不下一整帧就整帧丢，绝不发半帧 */
    if (usb_cdc_free() < MIC_RAW_FRAME)
    {
        raw_drop++;
        return;
    }

    f[0] = 0xAAu;
    f[1] = 0x55u;
    f[2] = (uint8_t)(raw_seq & 0xFFu);
    f[3] = (uint8_t)(raw_seq >> 8);

    for (c = 0; c < MIC_CHN; c++)
    {
        f[4u + 2u * c] = (uint8_t)(raw_out[c] & 0xFFu);
        f[5u + 2u * c] = (uint8_t)(raw_out[c] >> 8);
    }

    crc = crc16_ccitt(&f[2], 34u);  /* 覆盖 seq + 32 字节数据 */
    f[36] = (uint8_t)(crc & 0xFFu);
    f[37] = (uint8_t)(crc >> 8);

    if (usb_cdc_write(f, MIC_RAW_FRAME) != MIC_RAW_FRAME)
    {
        raw_drop++;
        return;
    }

    raw_seq++;
    raw_frames++;
}

/* 处理一整块（MIC_BLOCK 帧）：一帧 ADC → 一帧串口，原样转发 */
static void raw_block(uint32_t f)
{
    const volatile uint16_t *base[MIC_CHN];
    uint32_t stride[MIC_CHN];
    uint32_t k, c;

    /* 先算好 16 路在本块里的首地址和步长（步长 = 该 ADC 的 lane 数） */
    for (c = 0; c < MIC_CHN; c++)
    {
        uint32_t a = ch_adc[c];

        base[c]   = &mic_buf[a][adc_lanes[a] * f + ch_lane[c]];
        stride[c] = adc_lanes[a];
    }

    for (k = 0; k < MIC_BLOCK; k++)
    {
        /* 直接取原始转换值：不平均、不缩放、不截断、不做任何运算 */
        for (c = 0; c < MIC_CHN; c++)
        {
            raw_out[c] = base[c][k * stride[c]];
        }

        raw_emit();                 /* 一帧 ADC = 一帧串口，1:1 */
    }
}

uint8_t mic_raw_is_on(void)
{
    return raw_on;
}

/* 供 USB 吞吐测试构造合法帧用 */
uint16_t mic_crc16(const uint8_t *p, uint32_t n)
{
    return crc16_ccitt(p, n);
}

void mic_raw_start(void)
{
    uint32_t c;

    raw_on     = 0;
    raw_seq    = 0;
    raw_frames = 0;
    raw_drop   = 0;

    for (c = 0; c < MIC_CHN; c++)
    {
        raw_out[c] = 0;
    }

    /* 1:1 直通：串口帧率必须等于采样率，fs 太高就装不下，先拦住 */
    if (g_fs > MIC_RAW_MAX_FS)
    {
        uart_puts("fs too high for 1:1 raw stream: ");
        uart_put_u32(g_fs);
        uart_puts(" Hz > max ");
        uart_put_u32(MIC_RAW_MAX_FS);
        uart_puts(" Hz\r\n");
        return;
    }

    raw_on = 1;

    /* 这行文字在流开始之前发，不会混进二进制帧 */
    uart_puts("raw on: ");
    uart_put_u32(g_fs);
    uart_puts(" Hz x ");
    uart_put_u32(MIC_CHN);
    uart_puts(" ch  1:1  frame=");
    uart_put_u32(MIC_RAW_FRAME);
    uart_puts("B\r\n");
}

void mic_raw_stop(void)
{
    if (raw_on == 0u)
    {
        uart_puts("raw already off\r\n");
        return;
    }

    raw_on = 0;                      /* 先停，再打印，避免文字混进流里 */

    uart_puts("\r\nraw off  sent=");
    uart_put_u32(raw_frames);
    uart_puts(" drop=");
    uart_put_u32(raw_drop);
    uart_puts("\r\n");
}

void mic_raw_status(void)
{
    uart_puts("raw=");
    uart_puts(raw_on ? "ON" : "off");
    uart_puts(" mode=1:1 fs=");  uart_put_u32(g_fs);
    uart_puts(" max=");         uart_put_u32(MIC_RAW_MAX_FS);
    uart_puts(" frame=");       uart_put_u32(MIC_RAW_FRAME);
    uart_puts("B sent=");       uart_put_u32(raw_frames);
    uart_puts(" drop=");        uart_put_u32(raw_drop);
    uart_puts(" txdrop=");      uart_put_u32(uart_tx_dropped());
    uart_puts("\r\nblocks=");   uart_put_u32(mic_blocks);
    uart_puts(" lost=");        uart_put_u32(mic_lost);
    uart_puts(" ovr=");         uart_put_u32(mic_ovr);
    uart_puts("\r\n");
}


/* ============================================================
 * 主循环调用：把已经写满的整块全部消费掉
 *
 * 每次最多处理一整圈的块数（MIC_RING/MIC_BLOCK），所以这个函数
 * 永远在有限步内返回，不会把主循环挂住。
 *
 * 顺带检查三颗 ADC 的 OVR 溢出标志：OVR 置位说明触发到来时上一串
 * 转换还没结束（配置错误或时钟异常），数据已经不可信。正常运行时
 * 由 90% 占空规则保证永远不置位，mic_ovr 恒为 0。
 * ============================================================ */
void mic_poll(void)
{
    uint64_t wr = mic_wr_min();

    /* 落后 >= 一整圈：g_rd 那一帧正在被 DMA 覆盖，不能读。
       直接跳到最新的块边界，把跳过的帧数记进 mic_lost。 */
    if ((wr - g_rd) >= MIC_RING)
    {
        uint64_t nr = wr & ~((uint64_t)MIC_BLOCK - 1u);

        if (nr > g_rd)
        {
            mic_lost += (uint32_t)(nr - g_rd);
            g_rd = nr;
        }
    }

    while ((wr - g_rd) >= MIC_BLOCK)
    {
        block_process(g_rd);
        g_rd += MIC_BLOCK;
        mic_blocks++;
    }

    if (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET)
    {
        ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
        mic_ovr++;
    }
    if (ADC_GetFlagStatus(ADC2, ADC_FLAG_OVR) != RESET)
    {
        ADC_ClearFlag(ADC2, ADC_FLAG_OVR);
        mic_ovr++;
    }
    if (ADC_GetFlagStatus(ADC3, ADC_FLAG_OVR) != RESET)
    {
        ADC_ClearFlag(ADC3, ADC_FLAG_OVR);
        mic_ovr++;
    }
}

void mic_stats_reset(void)
{
    mic_lost   = 0;
    mic_blocks = 0;
    mic_ovr    = 0;

    /* g_rd 必须对齐到块边界，否则 base = rd & (RING-1) 会落在
       缓冲中间，顺序读 512 帧时会越过本块、读到还没写的数据。 */
    g_rd = mic_wr_min() & ~((uint64_t)MIC_BLOCK - 1u);
}

void mic_rms(void)
{
    uint32_t c;

    if (!last_valid)
    {
        uart_puts("no data\r\n");
        return;
    }

    for (c = 0; c < MIC_CHN; c++)
    {
        uart_put_u32_pad(c + 1u, 2);
        uart_putc('=');
        uart_put_u32_pad(last_rms_mv[c], 4);
        uart_putc(' ');
    }
    uart_puts("mVrms L=");
    uart_put_u32(mic_lost);
    uart_puts(" ovr=");
    uart_put_u32(mic_ovr);
    uart_puts("\r\n");
}

void mic_mean(void)
{
    uint32_t c;

    if (!last_valid)
    {
        uart_puts("no data\r\n");
        return;
    }

    for (c = 0; c < MIC_CHN; c++)
    {
        uart_put_u32_pad(c + 1u, 2);
        uart_putc('=');
        uart_put_u32_pad(last_mean_mv[c], 4);
        uart_putc(' ');
    }
    uart_puts("mVdc\r\n");
}

uint32_t mic_get_fs(void)
{
    return g_fs;
}

/* ============================================================
 * 等 DMA 停稳
 *
 * TIM2 停掉以后最多还有一整个扫描序列在 ADC 里跑
 * （200kHz 下最长约 4.3us）。只连续读两次 NDTR 是不够的，
 * 这里要求 2000 次读数都不变，约 0.3ms，比最长转换时间大一个
 * 数量级。有界循环，硬件真坏了也不会卡在这里。
 * ============================================================ */
static void dma_settle(void)
{
    uint32_t n;
    uint32_t stable = 0;
    uint16_t a0, b0, c0, a1, b1, c1;

    for (n = 0; n < 200000UL; n++)
    {
        a0 = DMA2_Stream0->NDTR;
        b0 = DMA2_Stream2->NDTR;
        c0 = DMA2_Stream4->NDTR;
        a1 = DMA2_Stream0->NDTR;
        b1 = DMA2_Stream2->NDTR;
        c1 = DMA2_Stream4->NDTR;

        if ((a0 == a1) && (b0 == b1) && (c0 == c1))
        {
            stable++;
            if (stable >= 2000u)
            {
                return;
            }
        }
        else
        {
            stable = 0;
        }
    }
}

static void dma_clear_all(void)
{
    /* LIFCR/HIFCR 是写 1 清零：每个 stream 占 6 位
       (FEIF/DMEIF/TEIF/HTIF/TCIF，bit0/2/3/4/5) → 0x3D。
       Stream0 在 bit0~5，Stream2 在 bit12~17，Stream4 在 bit0~5(高寄存器)。 */
    DMA2->LIFCR = 0x0003D03DUL;     /* Stream0 + Stream2 */
    DMA2->HIFCR = 0x0000003DUL;     /* Stream4 */
}

static void adc_clear_all(void)
{
    ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
    ADC_ClearFlag(ADC2, ADC_FLAG_OVR);
    ADC_ClearFlag(ADC3, ADC_FLAG_OVR);
}

static void hw_stop(void)
{
    uint32_t a;

    TIM_Cmd(TIM2, DISABLE);
    dma_settle();

    for (a = 0; a < MIC_ADC_N; a++)
    {
        DMA_Cmd(hw[a].stream, DISABLE);
    }

    ADC_Cmd(ADC1, DISABLE);
    ADC_Cmd(ADC2, DISABLE);
    ADC_Cmd(ADC3, DISABLE);
}

static void hw_reset(void)
{
    uint32_t a;

    for (a = 0; a < MIC_ADC_N; a++)
    {
        hw[a].base = 0;
        DMA_SetCurrDataCounter(hw[a].stream,
                               (uint16_t)(adc_lanes[a] * MIC_RING));
    }

    g_rd       = 0;
    last_valid = 0;
    mic_lost   = 0;
    mic_blocks = 0;
    mic_ovr    = 0;
}

static void hw_start(void)
{
    uint32_t a;

    /* 先让 DMA 就位，再开 ADC，最后开定时器——顺序反了会漏掉头几帧 */
    for (a = 0; a < MIC_ADC_N; a++)
    {
        DMA_Cmd(hw[a].stream, ENABLE);
    }

    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    ADC_Cmd(ADC3, ENABLE);

    delay_loop(3000);                   /* ADC 上电稳定，约 70us */

    TIM_Cmd(TIM2, ENABLE);
}

/* ============================================================
 * 抓一块原始数据吐出来（给上位机看波形）
 *
 * 停 TIM2 → 等最后几帧搬完 → 拷 512 帧到快照 → 立刻恢复采集。
 * 16 通道的快照有 16KB，一次塞不进 8KB 的串口发送队列，所以
 * 发送改成分块状态机：mic_dump() 只做快照 + 发头，剩下的 16384
 * 字节由主循环里的 mic_dump_poll() 每次搬一小块（<=64 字节）进
 * 发送队列。发送期间 DMA 照常写环形缓冲，主循环照常消费，
 * 串口再慢也不会丢采样、不会卡死。
 * ============================================================ */
#define DUMP_TOTAL      ((uint32_t)MIC_BLOCK * MIC_CHN * 2u)  /* 16384 */
#define DUMP_CHUNK      64u

static uint16_t snap1[MIC_LANES_MAX * MIC_BLOCK];   /* ADC1：6*512 */
static uint16_t snap2[5u * MIC_BLOCK];              /* ADC2：5*512 */
static uint16_t snap3[5u * MIC_BLOCK];              /* ADC3：5*512 */
static volatile uint16_t *const snaps[MIC_ADC_N] = { snap1, snap2, snap3 };

static volatile uint8_t  dump_active = 0;
static volatile uint16_t dump_pos    = 0;

void mic_dump(void)
{
    uint64_t wr;
    uint32_t base, a, k;

    /* 原始直通在跑的时候不做 dump：两者往同一个发送队列灌，
       而且 16KB 的 dump 会彻底冲掉二进制流，PC 端只能当垃圾丢掉 */
    if (raw_on)
    {
        uart_puts("busy: raw stream on, send x first\r\n");
        return;
    }

    if (dump_active)
    {
        uart_puts("busy\r\n");
        return;
    }

    if (uart_tx_free() < 64u)
    {
        uart_puts("busy\r\n");
        return;
    }

    TIM_Cmd(TIM2, DISABLE);
    dma_settle();

    wr = mic_wr_min();
    if (wr < MIC_BLOCK)
    {
        TIM_Cmd(TIM2, ENABLE);
        uart_puts("no data\r\n");
        return;
    }

    base = (uint32_t)((wr - MIC_BLOCK) & (MIC_RING - 1));

    for (a = 0; a < MIC_ADC_N; a++)
    {
        for (k = 0; k < MIC_BLOCK; k++)
        {
            uint32_t f = (base + k) & (MIC_RING - 1);
            uint32_t l;

            for (l = 0; l < adc_lanes[a]; l++)
            {
                snaps[a][adc_lanes[a] * k + l] = mic_buf[a][adc_lanes[a] * f + l];
            }
        }
    }

    TIM_Cmd(TIM2, ENABLE);

    /* 帧优先：每帧 16 个 uint16，顺序是 MIC1..MIC16，小端 */
    uart_puts("#DUMP ");
    uart_put_u32(MIC_BLOCK);
    uart_putc(' ');
    uart_put_u32(MIC_CHN);
    uart_puts("\r\n");

    dump_pos    = 0;
    dump_active = 1;
}

void mic_dump_poll(void)
{
    while (dump_active)
    {
        uint16_t room = uart_tx_free();
        uint16_t left = (uint16_t)(DUMP_TOTAL - dump_pos);
        uint16_t n;

        if (room == 0u)
        {
            return;                     /* 队列满，下圈再搬，绝不阻塞 */
        }
        if (room > DUMP_CHUNK)
        {
            room = DUMP_CHUNK;
        }
        if (left > room)
        {
            left = room;
        }

        for (n = 0; n < left; n++)
        {
            uint32_t idx = (uint32_t)dump_pos + n;
            uint32_t f   = idx / (uint32_t)(MIC_CHN * 2u);   /* 帧号 */
            uint32_t off = idx % (uint32_t)(MIC_CHN * 2u);   /* 帧内字节偏移 */
            uint32_t c   = off >> 1;                         /* 通道 0..15 */
            uint32_t lo  = off & 1u;                         /* 低/高字节 */
            uint32_t a   = ch_adc[c];
            uint16_t v   = snaps[a][adc_lanes[a] * f + ch_lane[c]];

            uart_putc((uint8_t)(lo ? (v >> 8) : (v & 0xFFu)));
        }

        dump_pos = (uint16_t)(dump_pos + left);

        if (dump_pos >= DUMP_TOTAL)
        {
            dump_active = 0;
            return;
        }
    }
}

/* ============================================================
 * 重新武装：不改采样率，只把 DMA / ADC / 指针全部复位重开
 * ============================================================ */
void mic_rearm(void)
{
    raw_on = 0;                 /* 重新武装会清空指针，原始直通跟着停 */
    hw_stop();
    dma_clear_all();
    adc_clear_all();
    hw_reset();
    hw_start();
    mic_restarts++;
}

/* ============================================================
 * 运行时改采样率
 *
 * 先做完全部计算和合法性检查，再动机器；任何一步不合法都直接返回 0，
 * 原来的采集不受影响。改完 fs 会清空缓冲、计数器归零（旧数据作废）。
 * ============================================================ */
uint8_t mic_set_fs(uint32_t hz)
{
    uint16_t arr, cyc;
    uint8_t  smp;

    if (!fs_plan(hz, &arr, &smp, &cyc))
    {
        return 0;
    }

    /* 改采样率等于换了时间基准：先停直通（不打印，避免和下面的 fs=
       回显混在一起）。停之前不会有半帧，因为整帧入队。 */
    raw_on = 0;

    hw_stop();
    dma_clear_all();
    adc_clear_all();

    TIM2->ARR  = (uint32_t)arr;
    TIM2->CCR1 = ((uint32_t)arr + 1u) / 2u;
    TIM2->EGR  = TIM_EGR_UG;            /* 把新 ARR/PSC 立刻加载 */
    TIM2->SR   = 0;

    adc_set_smp(smp);

    hw_reset();
    hw_start();

    g_arr     = arr;
    g_smp_cyc = cyc;
    g_fs      = MIC_TIM_CLK_HZ / ((uint32_t)arr + 1u);
    mic_restarts++;

    return 1;
}

/* ============================================================
 * 诊断：把 TIM2 / ADC / DMA / 写指针的状态全部打出来（'s' 命令）
 * ============================================================ */
void mic_diag(void)
{
    uint32_t a;

    uart_puts("== MIC16 diag ==\r\n");

    uart_puts("fs=");       uart_put_u32(g_fs);
    uart_puts(" arr=");     uart_put_u32(g_arr);
    uart_puts(" smp=");     uart_put_u32(g_smp_cyc);
    uart_puts("cyc ring="); uart_put_u32(MIC_RING);
    uart_puts(" blk=");     uart_put_u32(MIC_BLOCK);
    uart_puts("\r\n");

    uart_puts("RCC CFGR=");  uart_put_hex32(RCC->CFGR);
    uart_puts(" APB1ENR=");  uart_put_hex32(RCC->APB1ENR);
    uart_puts(" APB2ENR=");  uart_put_hex32(RCC->APB2ENR);
    uart_puts("\r\n");

    uart_puts("TIM2 CR1=");  uart_put_hex32(TIM2->CR1);
    uart_puts(" CNT=");      uart_put_u32(TIM2->CNT);
    uart_puts(" PSC=");      uart_put_u32(TIM2->PSC);
    uart_puts(" ARR=");      uart_put_u32(TIM2->ARR);
    uart_puts(" CCR1=");     uart_put_u32(TIM2->CCR1);
    uart_puts(" CCMR1=");    uart_put_hex32(TIM2->CCMR1);
    uart_puts(" CCER=");     uart_put_hex32(TIM2->CCER);
    uart_puts(" CR2=");      uart_put_hex32(TIM2->CR2);
    uart_puts(" SR=");       uart_put_hex32(TIM2->SR);
    uart_puts("\r\n");

    uart_puts("ADC CCR=");   uart_put_hex32(ADC->CCR);
    uart_puts(" SR1/2/3=");
    uart_put_hex32(ADC1->SR); uart_putc('/');
    uart_put_hex32(ADC2->SR); uart_putc('/');
    uart_put_hex32(ADC3->SR);
    uart_puts("\r\n");

    for (a = 0; a < MIC_ADC_N; a++)
    {
        uart_puts("ADC");    uart_put_u32(a + 1u);
        uart_puts(" CR1=");  uart_put_hex32(adcs[a]->CR1);
        uart_puts(" CR2=");  uart_put_hex32(adcs[a]->CR2);
        uart_puts(" SQR1="); uart_put_hex32(adcs[a]->SQR1);
        uart_puts(" SMPR2="); uart_put_hex32(adcs[a]->SMPR2);
        uart_puts(" SMPR1="); uart_put_hex32(adcs[a]->SMPR1);
        uart_puts("\r\n");
    }

    uart_puts("LISR=");  uart_put_hex32(DMA2->LISR);
    uart_puts(" HISR="); uart_put_hex32(DMA2->HISR);
    uart_puts("\r\n");

    for (a = 0; a < MIC_ADC_N; a++)
    {
        uart_puts("S");  uart_put_u32(a);
        uart_puts(" CR=");  uart_put_hex32(hw[a].stream->CR);
        uart_puts(" NDTR="); uart_put_u32(hw[a].stream->NDTR);
        uart_puts(" FCR=");  uart_put_hex32(hw[a].stream->FCR);
        uart_puts("\r\n");
    }

    uart_puts("NVIC ISER1="); uart_put_hex32(NVIC->ISER[1]);
    uart_puts("\r\n");

    for (a = 0; a < MIC_ADC_N; a++)
    {
        uart_putc((uint8_t)('0' + a));
        uart_puts(": base="); uart_put_u32((uint32_t)hw[a].base);
        uart_puts(" wr=");    uart_put_u32((uint32_t)mic_wr(a));
        uart_puts(" ndtr=");  uart_put_u32(hw[a].stream->NDTR);
        uart_puts("\r\n");
    }
    uart_puts("g_rd=");      uart_put_u32((uint32_t)g_rd);
    uart_puts(" blocks=");   uart_put_u32(mic_blocks);
    uart_puts(" lost=");     uart_put_u32(mic_lost);
    uart_puts(" ovr=");      uart_put_u32(mic_ovr);
    uart_puts(" restarts="); uart_put_u32(mic_restarts);
    uart_puts("\r\n");

    uart_puts("CNT A="); uart_put_u32(TIM2->CNT);
    delay_loop(4200);                   /* 约 0.1ms，看到 CNT 在走即可，别停太久 */
    uart_puts(" CNT B="); uart_put_u32(TIM2->CNT);
    uart_puts("\r\n");

    uart_puts("buf0[0..11]=");
    for (a = 0; a < 12u; a++)
    {
        uart_put_u32(mic_buf1[a]);
        uart_putc(' ');
    }
    uart_puts("\r\n== end ==\r\n");
}

/* ============================================================
 * 初始化
 * ============================================================ */

static void tim_init(uint16_t arr)
{
    GPIO_InitTypeDef        g;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

#if MIC_REF_OUT
    /* PA0 现在是 ADC3_IN0（MIC12 的模拟输入），不能再做方波输出。
       示波器触发参考方波改从 PB10 输出（TIM2_CH3，AF1）。 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
    GPIO_StructInit(&g);
    g.GPIO_Pin   = GPIO_Pin_10;
    g.GPIO_Mode  = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &g);
#endif

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler     = 2 - 1;               /* 84MHz / 2 = 42MHz */
    tb.TIM_Period        = arr;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* OC1 只作为 TRGO 的内部参考（不输出到引脚，CC1E 保持关闭）。
       OC1Ref 是内部信号，即使引脚不输出也照样每周期翻转，ADC 触发不受影响。 */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Disable;
    oc.TIM_Pulse       = ((uint32_t)arr + 1u) / 2u;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);

#if MIC_REF_OUT
    oc.TIM_OutputState = TIM_OutputState_Enable;   /* CH3 真正输出到 PB10 */
    TIM_OC3Init(TIM2, &oc);
#endif

    /* 必须是 OC1Ref（MMS=0b100）：TRGO 跟着 50% 方波走。
       TIM_TRGOSource_OC1 是 1 个时钟宽的比较脉冲，ADC 异步采样会丢沿。 */
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_OC1Ref);
}

static void adc_set_smp(uint8_t smp)
{
    uint32_t a, r;

    /* ADC 必须先 DISABLE；调用者保证这一点。
       三颗 ADC 的采样时间必须一样，否则 rank k 的采样时刻会对不齐。 */
    for (a = 0; a < MIC_ADC_N; a++)
    {
        for (r = 0; r < adc_lanes[a]; r++)
        {
            ADC_RegularChannelConfig(adcs[a], adc_ch[a][r],
                                     (uint8_t)(r + 1u), smp);
        }
    }
}

static void adc_init(uint8_t smp)
{
    GPIO_InitTypeDef      g;
    ADC_InitTypeDef       ai;
    ADC_CommonInitTypeDef ac;
    uint32_t              a, r;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                           RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 |
                           RCC_APB2Periph_ADC2 |
                           RCC_APB2Periph_ADC3, ENABLE);

    GPIO_StructInit(&g);
    g.GPIO_Pin  = MIC_GPIOA_MASK;
    g.GPIO_Mode = GPIO_Mode_AN;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &g);

    g.GPIO_Pin  = MIC_GPIOB_MASK;
    GPIO_Init(GPIOB, &g);

    g.GPIO_Pin  = MIC_GPIOC_MASK;
    GPIO_Init(GPIOC, &g);

    ADC_DeInit();

    ADC_CommonStructInit(&ac);
    ac.ADC_Mode             = ADC_Mode_Independent;
    ac.ADC_Prescaler        = ADC_Prescaler_Div4;      /* 84MHz / 4 = 21MHz */
    ac.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ac.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ac);

    ADC_StructInit(&ai);
    ai.ADC_Resolution           = ADC_Resolution_12b;
    ai.ADC_ScanConvMode         = ENABLE;              /* 每颗扫多个通道 */
    ai.ADC_ContinuousConvMode   = DISABLE;             /* 等 TIM2 触发 */
    ai.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T2_TRGO;
    ai.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ai.ADC_DataAlign            = ADC_DataAlign_Right;

    for (a = 0; a < MIC_ADC_N; a++)
    {
        ai.ADC_NbrOfConversion = adc_lanes[a];

        ADC_Init(adcs[a], &ai);
        for (r = 0; r < adc_lanes[a]; r++)
        {
            ADC_RegularChannelConfig(adcs[a], adc_ch[a][r],
                                     (uint8_t)(r + 1u), smp);
        }
        ADC_DMACmd(adcs[a], ENABLE);
        /* 关键！不开这个，DMA 缓冲绕回（最后一次传输）之后 ADC 就再也
           不发请求了 —— 表现为采满一圈就永久停住。 */
        ADC_DMARequestAfterLastTransferCmd(adcs[a], ENABLE);
        ADC_Cmd(adcs[a], ENABLE);
    }
}

static void dma_one(DMA_Stream_TypeDef *s, uint32_t ch,
                    uint32_t par, volatile uint16_t *buf, uint16_t n)
{
    DMA_InitTypeDef d;

    DMA_DeInit(s);

    d.DMA_Channel            = ch;
    d.DMA_PeripheralBaseAddr = par;
    d.DMA_Memory0BaseAddr    = (uint32_t)buf;
    d.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    d.DMA_BufferSize         = n;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    d.DMA_Mode               = DMA_Mode_Circular;
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    d.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
    d.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    d.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(s, &d);

    DMA_ITConfig(s, DMA_IT_TC, ENABLE);
}

static void dma_init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    dma_one(DMA2_Stream4, DMA_Channel_0, (uint32_t)&ADC1->DR,
            mic_buf[0], (uint16_t)(adc_lanes[0] * MIC_RING));
    dma_one(DMA2_Stream2, DMA_Channel_1, (uint32_t)&ADC2->DR,
            mic_buf[1], (uint16_t)(adc_lanes[1] * MIC_RING));
    dma_one(DMA2_Stream0, DMA_Channel_2, (uint32_t)&ADC3->DR,
            mic_buf[2], (uint16_t)(adc_lanes[2] * MIC_RING));
}

static void nvic_init(void)
{
    NVIC_InitTypeDef n;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    n.NVIC_IRQChannel                   = DMA2_Stream4_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 0;
    n.NVIC_IRQChannelSubPriority        = 0;
    n.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&n);

    n.NVIC_IRQChannel = DMA2_Stream2_IRQn;
    NVIC_Init(&n);

    n.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_Init(&n);
}

void mic_init(void)
{
    uint16_t arr = 0, cyc = 0;
    uint8_t  smp = 0;
    uint32_t a, i;

    if (!fs_plan(MIC_FS_HZ, &arr, &smp, &cyc))
    {
        /* 编译期已经检查过，正常到不了这里；留一个安全兜底 */
        (void)fs_plan(200000UL, &arr, &smp, &cyc);
    }

    for (a = 0; a < MIC_ADC_N; a++)
    {
        for (i = 0; i < ((uint32_t)adc_lanes[a] * MIC_RING); i++)
        {
            mic_buf[a][i] = 0;
        }
        hw[a].base = 0;
    }

    g_rd       = 0;
    last_valid = 0;

    g_arr     = arr;
    g_smp_cyc = cyc;
    g_fs      = MIC_TIM_CLK_HZ / ((uint32_t)arr + 1u);

    tim_init(arr);
    adc_init(smp);
    dma_init();
    nvic_init();

    hw_reset();
    hw_start();
}
