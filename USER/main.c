/* ============================================================
 * 十六路麦克风 —— ADC1 扫 6 通道，ADC2/ADC3 各扫 5 通道
 *
 * 接线（模块 VCC 一定要用 3.3V，输出别超过 VDDA）：
 *   MIC1  OUT → PA1   (ADC1_IN1)   rank1  与 MIC7/MIC12 同时
 *   MIC2  OUT → PA2   (ADC1_IN2)   rank2  与 MIC8/MIC13 同时
 *   MIC3  OUT → PA3   (ADC1_IN3)   rank3  与 MIC9/MIC14 同时
 *   MIC4  OUT → PA4   (ADC1_IN4)   rank4  与 MIC10/MIC15 同时
 *   MIC5  OUT → PA5   (ADC1_IN5)   rank5  与 MIC11/MIC16 同时
 *   MIC6  OUT → PA6   (ADC1_IN6)   rank6（ADC1 独有）
 *   MIC7  OUT → PA7   (ADC2_IN7)   rank1
 *   MIC8  OUT → PB0   (ADC2_IN8)   rank2
 *   MIC9  OUT → PB1   (ADC2_IN9)   rank3
 *   MIC10 OUT → PC4   (ADC2_IN14)  rank4
 *   MIC11 OUT → PC5   (ADC2_IN15)  rank5
 *   MIC12 OUT → PA0   (ADC3_IN0)   rank1
 *   MIC13 OUT → PC0   (ADC3_IN10)  rank2
 *   MIC14 OUT → PC1   (ADC3_IN11)  rank3
 *   MIC15 OUT → PC2   (ADC3_IN12)  rank4
 *   MIC16 OUT → PC3   (ADC3_IN13)  rank5
 *
 *   PA9  → USB-TTL 的 RXD
 *   PA10 ← USB-TTL 的 TXD
 *   GND  → USB-TTL 的 GND
 *   PB10 → 可选，fs 方波，示波器触发参考
 *
 * 注意：F407VGT6 是 LQFP100 封装，没有 PF 引脚（PF6~PF10 只在 144 脚/
 * BGA 封装上）。LQFP100 的 ADC 可用引脚正好 16 个：PA0~PA7、PB0/PB1、
 * PC0~PC5，本工程 16 路一路一个。PA0 现在是 MIC12 的模拟输入。
 *
 * 串口 921600 8N1 只做**命令通道**（A/V/d/c/x/w/u/s/t/r/挡位）；
 * 原始数据走 **USB CDC 虚拟串口**（PA11/PA12），PC 上多出一个 COM 口。
 *
 *   A       16 路交流有效值（mVrms），一行       ← 串口
 *   V       16 路直流均值（mV），一行             ← 串口
 *   d       抓 MIC_BLOCK 帧 × 16 通道原始数据     ← 串口
 *   c       开始原始直通：ADC 采一个点发一个点    ← 数据走 USB
 *   x       停止直通
 *   w       打印直通状态（采样率/帧数/丢帧）
 *   u       打印 USB 状态（是否已配置/队列空间/丢弃）
 *   8k…200k 改采样率挡位（>24 kHz 时 c 会拒绝启动，USB 带宽不够）
 *   r       清零计数
 *   s       打印寄存器/指针诊断
 *   t       重新武装 DMA/ADC
 *   ?       帮助
 *
 * 本工程（USB 版）与其它工程的关系：
 *   - MIC16_ADC123        原始工程：200 kHz 采集，A/V/d 按需查询，115200
 *   - MIC16_ADC123_STREAM 200 kHz 抽成 2 kHz 上传（100 点平均，带低通）
 *   - MIC16_ADC123_RAW    2 kHz 采样 + 1:1 直通，走串口（921600）
 *   - MIC16_ADC123_USB    **本工程**：20 kHz 采样 + 1:1 直通，走 USB CDC
 *   - 帧格式见 mic.h；上位机脚本在 tools/raw_recv.py
 * 其它工程都不做任何修改。
 *
 * 注意：USB 全速的规范上限是 1.22 MB/s，所以 16 路 1:1 直通的每路上限
 * 约 26 kHz；本工程默认 20 kHz（760 kB/s），留余量给主机调度抖动。
 * 200 kHz × 16 路（7.6 MB/s）这块板子做不到，那是换硬件的事。
 *
 * 防跑飞：IWDG 独立看门狗（约 2s）由主循环喂狗；HardFault 等
 * 异常直接 NVIC_SystemReset()。任何死循环/跑飞都会被硬件拉回。
 * ============================================================ */

#include "stm32f4xx.h"
#include "mic.h"
#include "uart.h"
#include "usb_cdc.h"

/* ------------------------------------------------------------
 * 独立看门狗
 *
 * LSI 标称 32kHz（实际 17~47kHz），64 分频后约 500Hz，
 * reload 1000 → 超时约 2 秒。主循环每圈都喂狗，任何死循环、
 * 跑飞、异常中断风暴超过 2 秒都会被 IWDG 硬件复位，重新拉起采集。
 * IWDG 由 LSI 驱动，和主时钟无关，主晶振/PLL 死了也能复位。
 * ------------------------------------------------------------ */
static void wdg_init(void)
{
    uint32_t guard = 1000000UL;         /* 有界等待：LSI 真起不来也不死等 */

    RCC_LSICmd(ENABLE);
    while ((RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) && (--guard != 0u))
    {
    }

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(1000);
    IWDG_ReloadCounter();
    IWDG_Enable();                      /* 一旦使能，只能复位才能关 */

    /* 调试器挂起 CPU 时冻结看门狗，断点调试不会被喂狗超时打断 */
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);
}

static void wdg_kick(void)
{
    IWDG_ReloadCounter();
}

/* ------------------------------------------------------------
 * 采样率挡位
 *
 * 只开放这些值，避免随手输入一个奇怪的 fs 把 ADC 采样时间
 * 逼到没有余量。16 路架构下最重的 ADC1 要扫 6 个通道，
 * 200k 以上（400k/500k）转换时间超过触发周期，物理上装不下，
 * 所以挡位表比 6 路版本少了最后两挡。
 * 48k 正好是 42MHz/875，无误差；其它值除不尽的会略高一点，
 * 回显的是实际值（mic_get_fs）。
 * ------------------------------------------------------------ */
static const uint32_t rate_gears[] =
{
    8000UL, 16000UL, 18000UL, 20000UL, 32000UL, 40000UL, 48000UL,
    64000UL, 80000UL, 96000UL, 128000UL, 200000UL
};

static void rate_show(void)
{
    uart_puts(" fs: 8k 16k 18k 20k 32k 40k 48k\r\n");
    uart_puts("     64k 80k 96k 128k 200k\r\n");
    uart_puts(" (1:1 raw stream: >24k will be refused by 'c')\r\n");
}

static uint8_t rate_is_gear(uint32_t hz)
{
    uint32_t i;

    for (i = 0; i < (sizeof(rate_gears) / sizeof(rate_gears[0])); i++)
    {
        if (rate_gears[i] == hz)
        {
            return 1;
        }
    }

    return 0;
}

static void rate_apply(uint32_t hz)
{
    if (!rate_is_gear(hz))
    {
        uart_puts("bad rate\r\n");
        rate_show();
        return;
    }

    if (mic_set_fs(hz))
    {
        uart_puts("fs=");
        uart_put_u32(mic_get_fs());
        uart_puts(" Hz\r\n");
    }
    else
    {
        /* 理论上挡位都算过，不会走到这里；真失败了原配置不动 */
        uart_puts("set fs failed\r\n");
    }
}

static void help(void)
{
    uart_puts(" A = RMS of MIC01..MIC16 (mVrms)\r\n");
    uart_puts(" V = DC mean of MIC01..MIC16 (mV)\r\n");
    uart_puts(" d = dump MIC_BLOCK frames x 16ch (binary)\r\n");
    uart_puts(" c = start RAW 1:1 stream over USB CDC\r\n");
    uart_puts(" p = USB throughput test (no ADC, x to stop)\r\n");
    uart_puts(" x = stop stream / test\r\n");
    uart_puts(" w = stream status\r\n");
    uart_puts(" u = USB status\r\n");
    rate_show();
    uart_puts("     e.g. 40k / 80k / 200k (Enter optional)\r\n");
    uart_puts(" r = reset counters\r\n");
    uart_puts(" s = print registers (diagnostic)\r\n");
    uart_puts(" t = rearm DMA/ADC\r\n");
    uart_puts(" ? = help\r\n");
}

/* ------------------------------------------------------------
 * 采样率命令的输入状态
 *
 * 数字和 'k' 不一定同时进 RX 缓冲（一个字节的间隔 = 10/波特率：
 * 921600 下 10.9us，115200 下 87us），
 * 所以不能在收到数字的当场死等——那样会停住 mic_poll，
 * 采样率越高越容易把环形缓冲等穿。这里拆成状态机，一个字一个字喂，
 * 主循环始终照常消费采集数据。
 *
 * 支持：
 *   40k / 80k / 200k      ← 推荐，收到 k 立即生效
 *   40000 + 回车          ← 也认，但必须在挡位表里
 *   f 40000 / f40k        ← 兼容旧写法
 * ------------------------------------------------------------ */
static uint8_t  rate_active = 0;
static uint32_t rate_value  = 0;
static uint8_t  rate_digits = 0;
static uint8_t  rate_k      = 0;

static void rate_finish(void)
{
    if (rate_k)
    {
        /* 先挡住乘法溢出：最大挡位 200k，所以数字部分不能超过 200 */
        if (rate_value > 200UL)
        {
            uart_puts("bad rate\r\n");
            rate_show();
            rate_active = 0;
            return;
        }
        rate_apply(rate_value * 1000UL);
    }
    else
    {
        rate_apply(rate_value);
    }

    rate_active = 0;
}

static void rate_feed(uint8_t ch)
{
    if ((ch >= '0') && (ch <= '9'))
    {
        if (rate_value < 1000000UL)
        {
            rate_value = rate_value * 10u + (uint32_t)(ch - '0');
        }
        rate_digits++;
        return;
    }

    if ((ch == 'k') || (ch == 'K'))
    {
        if (rate_digits)
        {
            rate_k = 1;
            rate_finish();
        }
        else
        {
            uart_puts("usage: 40k\r\n");
            rate_active = 0;
        }
        return;
    }

    if ((ch == ' ') || (ch == '\t'))
    {
        if (rate_digits)
        {
            rate_finish();
        }
        /* 还没收到数字时，前导空格忽略，继续等 */
        return;
    }

    if ((ch == '\r') || (ch == '\n'))
    {
        if (rate_digits)
        {
            rate_finish();
        }
        else
        {
            rate_active = 0;    /* 只有回车、没有数字：取消，不吞下一条命令 */
        }
        return;
    }

    uart_puts("usage: 40k\r\n");
    rate_active = 0;
}

/* ------------------------------------------------------------
 * USB 吞吐测试（'p' 开始，'x' 停止）
 *
 * 不经过 ADC：构造一个合法的 38 字节帧，能塞多少塞多少，
 * 用来单独测"USB CDC 这条路本身"能跑多少 kB/s。
 *
 * 帧内容是固定的（同一份数据反复发），所以 CRC 只算一次，
 * CPU 开销可以忽略 —— 测出来的就是 USB 总线+协议栈的上限，
 * 不含采样、组帧、CRC 的成本。
 *
 * 用法：命令口发 p → 数据口跑 raw_recv.py 看 fps
 *      实际字节率 = fps × 38
 * ------------------------------------------------------------ */
static uint8_t  perf_on = 0;
static uint32_t perf_frames = 0;
static uint8_t  perf_frame[MIC_RAW_FRAME];

static void perf_pump(void)
{
    if (perf_on == 0u)
    {
        return;
    }

    while (usb_cdc_free() >= MIC_RAW_FRAME)
    {
        usb_cdc_write(perf_frame, MIC_RAW_FRAME);
        perf_frames++;
    }
}

static void perf_start(void)
{
    uint16_t crc;
    uint32_t c;

    if (usb_cdc_configured() == 0u)
    {
        uart_puts("usb not configured yet\r\n");
        return;
    }

    if (mic_raw_is_on())
    {
        mic_raw_stop();             /* 两种模式不能同时跑 */
    }

    perf_frame[0] = 0xAAu;
    perf_frame[1] = 0x55u;
    perf_frame[2] = 0x00u;
    perf_frame[3] = 0x00u;

    for (c = 0; c < MIC_CHN; c++)
    {
        perf_frame[4u + 2u * c] = 0x34u;
        perf_frame[5u + 2u * c] = 0x12u;
    }

    crc = mic_crc16(&perf_frame[2], 34u);
    perf_frame[36] = (uint8_t)(crc & 0xFFu);
    perf_frame[37] = (uint8_t)(crc >> 8);

    perf_frames = 0;
    perf_on = 1;

    uart_puts("perf on: 38B frames, as fast as USB allows\r\n");
}

static void perf_stop(void)
{
    if (perf_on == 0u)
    {
        return;
    }

    perf_on = 0;

    uart_puts("perf off: frames=");
    uart_put_u32(perf_frames);
    uart_puts("  bytes=");
    uart_put_u32(perf_frames * MIC_RAW_FRAME);
    uart_puts("\r\n");
}

int main(void)
{
    uint8_t  ch;
    uint32_t budget;

    /* 优先级分组一定要在第一次 NVIC_Init 之前设好。
       uart_init() 里给 USART1 设的是“抢占 1 / 子 0”，DMA 是“抢占 0”，
       分组不对的话 USART1 的优先级字节会变成 0x00，和 DMA 一样，
       “串口比 DMA 低一级”就不成立了。 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 看门狗最先开：后面任何一步初始化卡住都能被拉回 */
    wdg_init();

    uart_init();

#if UART_BEACON
    uart_beacon('A');       /* A = uart_init 成功 */
#endif

    /* USB 虚拟串口：放在 ADC 之前，让主机先枚举。
       注意它关掉了 VBUS 检测，所以 PA9/PA10 的串口照常可用。 */
    usb_cdc_init();

#if UART_BEACON
    uart_beacon('U');       /* U = usb_cdc_init 也成功 */
#endif

    mic_init();

#if UART_BEACON
    uart_beacon('B');       /* B = mic_init 也成功 */
#endif

    uart_puts("\r\nMIC16_ADC123_USB ready  fs=");
    uart_put_u32(mic_get_fs());
    uart_puts(" Hz  ring=");
    uart_put_u32(MIC_RING);
    uart_puts(" blk=");
    uart_put_u32(MIC_BLOCK);
    uart_puts(" ch=");
    uart_put_u32(MIC_CHN);
    uart_puts("\r\nraw stream: ");
    uart_put_u32(mic_get_fs());
    uart_puts(" Hz x ");
    uart_put_u32(MIC_CHN);
    uart_puts(" ch  1:1  frame=");
    uart_put_u32(MIC_RAW_FRAME);
    uart_puts("B  over USB CDC (PA11/PA12)\r\n");
    uart_puts("cmd uart stays at ");
    uart_put_u32(UART_BAUD);
    uart_puts("\r\n");
    help();

    while (1)
    {
        wdg_kick();                 /* 喂狗：这圈活着，看门狗就不复位 */

        /* 消费所有写满的整块。函数内部有上限，不会挂住主循环 */
        mic_poll();

        /* 把串口发送队列里的字节搬出去（每次最多一个，不阻塞） */
        uart_tx_poll();

        /* 把 USB CDC 队列里的字节搬进端点（端点空闲才搬，不阻塞） */
        usb_cdc_poll();

        /* 吞吐测试模式：尽量往队列里塞帧 */
        perf_pump();

        /* dump 的分块发送：串口队列有空就搬一小块，没空下圈再搬 */
        mic_dump_poll();

        /* 命令处理，一个字符一条。
           每次最多处理 8 个字符就回去 mic_poll 一次：就算上位机灌命令，
           也不会让采集消费停太久（8 条最坏也就几毫秒）。 */
        budget = 8;
        while (budget != 0u)
        {
            /* 命令可以从两个地方来：串口（PA9/PA10）或 USB 数据口（COM5） */
            if (uart_getc(&ch) == 0u)
            {
                if (usb_cdc_read(&ch) == 0u)
                {
                    break;
                }
            }

            budget--;

            if (rate_active)
            {
                rate_feed(ch);
                continue;
            }

            /* 数字开头直接进采样率输入：40k、80k、40000 都能用 */
            if ((ch >= '0') && (ch <= '9'))
            {
                rate_active = 1;
                rate_value  = 0;
                rate_digits = 0;
                rate_k      = 0;
                rate_feed(ch);
                continue;
            }

            switch (ch)
            {
            case 'A':
            case 'a':
                mic_rms();          /* 01=... 02=... ... 16=... mVrms L=0 ovr=0 */
                break;

            case 'V':
            case 'v':
                mic_mean();         /* 01=... 02=... ... 16=... mVdc */
                break;

            case 'd':
            case 'D':
                mic_dump();
                break;

            case 'c':
            case 'C':
                mic_raw_start();
                break;

            case 'x':
            case 'X':
                perf_stop();
                mic_raw_stop();
                break;

            case 'p':
            case 'P':
                perf_start();
                break;

            case 'w':
            case 'W':
                mic_raw_status();
                break;

            case 'u':
            case 'U':
            {
                uint32_t rst, en, su, epin, epout;

                usb_cdc_stats(&rst, &en, &su, &epin, &epout);

                uart_puts("usb=");
                uart_puts(usb_cdc_configured() ? "configured" : "not configured");
                uart_puts(" free=");   uart_put_u32(usb_cdc_free());
                uart_puts(" drop=");   uart_put_u32(usb_cdc_dropped());
                uart_puts("\r\nrst=");     uart_put_u32(rst);
                uart_puts(" enum=");       uart_put_u32(en);
                uart_puts(" setup=");      uart_put_u32(su);
                uart_puts(" ep0in=");      uart_put_u32(epin);
                uart_puts(" ep0out=");     uart_put_u32(epout);
                uart_puts("\r\ngint=");    uart_put_hex32(usb_cdc_reg(0));
                uart_puts(" dsts=");       uart_put_hex32(usb_cdc_reg(3));
                uart_puts(" dctl=");       uart_put_hex32(usb_cdc_reg(1));
                uart_puts("\r\ndcfg=");    uart_put_hex32(usb_cdc_reg(2));
                uart_puts(" gotg=");       uart_put_hex32(usb_cdc_reg(4));
                uart_puts(" gccfg=");      uart_put_hex32(usb_cdc_reg(5));
                uart_puts("\r\n");
                break;
            }

            case 'f':
            case 'F':
                rate_active = 1;
                rate_value  = 0;
                rate_digits = 0;
                rate_k      = 0;
                break;

            case 'r':
            case 'R':
                mic_stats_reset();
                uart_puts("reset\r\n");
                break;

            case 's':
            case 'S':
                mic_diag();
                break;

            case 't':
            case 'T':
                mic_rearm();
                uart_puts("rearmed\r\n");
                break;

            case '?':
                help();
                break;

            default:
                break;
            }
        }
    }
}
