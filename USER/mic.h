#ifndef __MIC_H
#define __MIC_H

#include "stm32f4xx.h"

/* ============================================================
 * 十六路麦克风：三颗 ADC —— ADC1 扫 6 个通道，ADC2/ADC3 各扫 5 个
 *
 * 为什么是 6+5+5：
 *   F407 只有 3 颗 ADC，16 路只能拆成 3 组，最均衡的拆法是
 *   6 / 5 / 5（最大相差 1 路）。每颗 ADC 内部按 rank1..rankN 顺序扫描。
 *   三颗 ADC 由同一个 TIM2 TRGO 边沿同时启动、用同样的 ADCCLK 和
 *   同样的采样时间，所以三颗 ADC 的 rank k 是严格同一时刻采样的。
 *
 *   rank1：MIC1 / MIC7  / MIC12   （同一时刻）
 *   rank2：MIC2 / MIC8  / MIC13   （同一时刻，晚一个转换时间）
 *   rank3：MIC3 / MIC9  / MIC14
 *   rank4：MIC4 / MIC10 / MIC15
 *   rank5：MIC5 / MIC11 / MIC16
 *   rank6：MIC6（ADC1 独有）
 *
 *   相邻 rank 之间的固定延时 = (SMP+12) 个 ADC 周期（一次转换的时间），
 *   200kHz 默认挡位下 SMP=3，约 714ns。这个延时是固定的、可标定的。
 *
 * 引脚分配（F407VGT6，LQFP100。注意：LQFP100 封装没有 PF 引脚，
 *   PF6~PF10 只存在于 144 脚/BGA 封装；ADC 可用脚总共 16 个：
 *   PA0~PA7、PB0/PB1、PC0~PC5，正好一路一个）：
 *   ADC1  rank1..6：PA1 PA2 PA3 PA4 PA5 PA6       = IN1..IN6
 *   ADC2  rank1..5：PA7 PB0 PB1 PC4 PC5           = IN7,IN8,IN9,IN14,IN15
 *   ADC3  rank1..5：PA0 PC0 PC1 PC2 PC3           = IN0,IN10,IN11,IN12,IN13
 *
 *   注意：PA4~PA7、PB0/PB1、PC4/PC5 只能给 ADC1/ADC2，不能给 ADC3；
 *   ADC3 只能用 PA0~PA3、PC0~PC3。
 *   PA0 现在是 MIC12 的模拟输入，原来的 fs 方波改从 PB10（TIM2_CH3）输出，
 *   不需要就把 MIC_REF_OUT 设成 0。
 *
 * 数据通路：
 *   TIM2(TRGO) → ADC1/2/3 各自扫描 6/5/5 个通道
 *              → 三条 DMA 把交织数据搬进三块环形缓冲
 *              → TC 中断只做 base += MIC_RING（单位：帧）
 *              → 主循环按帧进度取整块，每块 MIC_BLOCK 帧 × 16 通道
 *              → 统计出 RMS/均值（A/V）
 *              → RAW 版：每帧原封不动打成一串口帧发出去（c/x/w），
 *                ADC 采一个点就发一个点，没有任何抽取/平均/滤波
 *
 * 每帧 = 该 ADC 的 lanes 个半字：ADC1 帧 = 6 个半字，ADC2/3 帧 = 5 个半字。
 * 三颗 ADC 每触发一次各产生一帧，所以帧号可以跨 ADC 对齐。
 *
 * 不丢数 / 不跑飞的关键：
 *   1. DMA 中断只有一句加法，没有循环、没有等待；
 *   2. 写指针关中断读 NDTR/base/TC 标志；TC 竞态时宁可少报一圈，
 *      绝不多报，重装不会算错；
 *   3. 主循环每次最多消费一整圈（MIC_RING/MIC_BLOCK 块），必然返回；
 *   4. 环形缓冲 2048 帧 = 1.024s@2kHz，主循环最坏约 3ms，
 *      中间有几百倍余量；
 *   5. 真落后一整圈时跳着追上最新块，把跳过帧数记进 mic_lost，
 *      不原地死等、不读正在被覆盖的数据；
 *   6. 串口发送是环形队列 + 主循环轮询，putc 永不阻塞；
 *      dump 改成 64 字节一块的分块发送状态机，发送期间采集照常；
 *      RAW 直通同样只往队列里追加，且“整帧入队”：队列放不下
 *      38 字节就整帧丢弃并计数，绝不发半帧；
 *   7. 6 通道扫描的整串转换时间限制在触发周期的 90% 以内（由程序
 *      按 fs 自动选采样时间），ADC 在下一个触发到来前一定已经转完，
 *      不会因为触发打在转换中途而漏沿；OVR 溢出标志在主循环里
 *      实时检查并计数（mic_ovr），正常恒为 0；
 *   8. 独立看门狗 IWDG（约 2s）由主循环喂狗，任何死循环/跑飞都会
 *      硬件复位重新拉起采集；HardFault 等异常直接 NVIC_SystemReset；
 *   9. 停机改采样率/抓波形都只用有界等待，不会无限等硬件标志。
 * ============================================================ */

/* ---------------- 模拟输入引脚 ----------------
 * 换引脚时 PIN 和 CH 必须成对改（通道号表在 mic.c 的 adc_ch[][]），
 * 而且要看清楚这个通道属于哪颗 ADC：
 *   ADC1/ADC2 可用 PA0~PA7、PB0/PB1、PC0~PC5；
 *   ADC3 可用 PA0~PA3、PC0~PC3（LQFP100 没有 PF 引脚，用不了 PF6~PF10）。
 * 本工程用到的脚：
 *   PA0~PA7 → GPIOA；PB0/PB1 → GPIOB；PC0~PC5 → GPIOC。
 */
#define MIC_GPIOA_MASK  (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
                         GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)
#define MIC_GPIOB_MASK  (GPIO_Pin_0 | GPIO_Pin_1)
#define MIC_GPIOC_MASK  (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
                         GPIO_Pin_4 | GPIO_Pin_5)

/* 示波器触发参考方波：PA0 被 MIC12 占用后，改从 PB10（TIM2_CH3，AF1）输出。
   板子上不需要这个参考信号时改成 0。 */
#define MIC_REF_OUT     1

/* ---------------- 采样率（改这里就能改默认采样率） ----------------
 * USB 版：原始数据走 USB CDC（虚拟串口）。
 *
 * 默认取 **18000 Hz**，是实测出来的"稳定上限"：
 *   16 kHz → PC 收到 100.3%（零丢帧）
 *   18 kHz → PC 收到 100.2%（零丢帧）  ← 默认
 *   20 kHz → PC 收到  96.9%（丢 3%）
 *
 * 数据率 18000 × 38 B = 684 kB/s，占 USB 实测能力（约 840 kB/s）的 81%。
 *
 * 注意：CDC 的波特率是虚拟的，改它不会改变速度；
 * 真正决定速度的是 USB 总线的帧调度（全速 12 Mbps）。
 *
 * 运行中可发挡位改 fs（8k/16k/18k/20k/32k/.../200k），
 * 但 fs 超过 MIC_RAW_MAX_FS（24 kHz）时 `c` 会拒绝启动。
 * `A`/`V` 不受影响，仍是全速采样下的统计值。
 *
 * TIM2 计数 42MHz，ARR = 42MHz/fs - 1，TRGO = fs。
 * ADC 采样时间由程序按 fs 自动挑（见 mic.c 的 pick_smp）。
 * 18 kHz 下自动选 144 个采样周期，整串约 44.6us，占周期 80%。
 */
#define MIC_FS_HZ       18000UL
#define MIC_MIN_FS_HZ   1000UL
#define MIC_MAX_FS_HZ   200000UL

#define MIC_TIM_CLK_HZ  42000000UL      /* APB1 定时器时钟 84MHz / 2 */
#define MIC_ADC_CLK_HZ  21000000UL      /* 84MHz / 4 */

#define MIC_TIM_ARR     (MIC_TIM_CLK_HZ / MIC_FS_HZ - 1UL)  /* 仅用于文档/自检 */

/* ---------------- 缓冲参数 ----------------
 * MIC_RING 是每颗 ADC 的帧数（每触发一次 = 1 帧），必须是 2 的幂。
 * 每颗 ADC 的 DMA 缓冲是 lanes*MIC_RING 个半字，必须能放进 16 位 NDTR。
 * 最重的是 ADC1：6*MIC_RING。
 * MIC_BLOCK 是每次处理的帧数，必须整除 MIC_RING。
 *
 * RAM 核算（F407VG，0x20000000 上 128KB 常规 SRAM）：
 *   环形缓冲 3 块 = (6+5+5)*2048*2 = 64KB
 *   dump 快照 (6+5+5)*64*2 = 2KB（RAW 版 MIC_BLOCK 改成 64，快照跟着变小）
 *   串口发送 8KB，接收 64B，栈 2KB，其余统计量 < 1KB，合计约 77KB。
 *   注意：RING=4096 时环形缓冲就要 128KB，整个 RAM 都不够，所以
 *   16 路只能用 2048。2 kHz 下 2048 帧 = 1.024 秒的缓冲深度。
 */
#define MIC_RING        2048
#define MIC_BLOCK       64
#define MIC_CHN         16
#define MIC_ADC_N       3
#define MIC_LANES_MAX   6           /* 最重的那颗 ADC（ADC1）扫的通道数 */

/* ---------------- 换算 ---------------- */
#define MIC_VREF_MV     3300UL
#define MIC_ADC_FULL    4096UL

/* ---------------- 原始数据直通输出（RAW 版） ----------------
 * 没有任何抽取、平均、滤波：ADC 每转换完一个点，主循环就把它原封不动
 * 塞进串口帧发出去 —— 串口上每帧的 16 个 u16 就是那一次 ADC 扫描的
 * 原始转换结果，一位都不改。
 *
 * 因此 ADC 采样率必须自己降到串口能 1:1 承接的水平：
 *   每帧 38 字节（含帧头/帧号/CRC），2000 帧/s × 38 B = 76 000 B/s
 *   = 760 kbaud，占 921600 的 82%。
 *   1:1 的物理上限 = 921600 / (38 × 10) = 2425 Hz/路。
 *
 * 帧格式（MIC_RAW_FRAME = 38 字节，小端，CRC16/CCITT-FALSE）：
 *   [0]      0xAA               帧头
 *   [1]      0x55               帧头
 *   [2..3]   帧号 seq（u16）    溢出回绕，PC 端用来算丢帧
 *   [4..35]  16 × u16 原始码值  0..4095，右对齐 12 bit（未做任何处理）
 *   [36..37] CRC16              poly 0x1021 init 0xFFFF，覆盖 [2..35]
 *
 * 代价（接受它，不再用滤波补偿）：
 *   1. fs 从 200 kHz 降到 2 kHz，可测带宽只有 0~1 kHz；
 *   2. 板上没有模拟抗混叠滤波器，1 kHz 以上的噪声会折叠进来；
 *   3. 2000 Hz 下 SMP 自动选到 480 周期，相邻 rank 的固定延时变成
 *      23.4 us（见本文件顶部说明），跨 rank 比相位要先补这个延时。
 * ============================================================ */
#define MIC_RAW_FRAME    38u

/* ---------------- 编译期自检 ---------------- */
#if (MIC_RING & (MIC_RING - 1)) != 0
#error "MIC_RING must be a power of two"
#endif
#if (MIC_RING % MIC_BLOCK) != 0
#error "MIC_BLOCK must divide MIC_RING"
#endif
#if MIC_BLOCK > MIC_RING
#error "MIC_BLOCK must be <= MIC_RING"
#endif
#if (MIC_LANES_MAX * MIC_RING) > 65535
#error "LANES_MAX*MIC_RING must fit in the 16-bit DMA NDTR"
#endif
#if MIC_FS_HZ < MIC_MIN_FS_HZ || MIC_FS_HZ > MIC_MAX_FS_HZ
#error "MIC_FS_HZ out of range 1000..200000"
#endif
/* 6 通道最短也要 6*(3+12)=90 个 ADC 周期，还必须 <= 触发周期的 90%：
 *   90*fs*10 <= 21MHz*9   →   fs <= 210kHz                       */
#if (10UL * MIC_LANES_MAX * 15UL * MIC_FS_HZ) > (9UL * MIC_ADC_CLK_HZ)
#error "MIC_FS_HZ too high for 6-channel scan at 21MHz ADCCLK"
#endif

extern volatile uint32_t mic_lost;      /* 被覆盖/跳过的帧数，正常恒为 0 */
extern volatile uint32_t mic_blocks;    /* 已处理的块数，判断采集是否在跑 */
extern volatile uint32_t mic_restarts;  /* 改采样率/重新武装的次数 */
extern volatile uint32_t mic_ovr;       /* ADC 溢出(OVR)次数，正常恒为 0 */

void mic_init(void);
void mic_poll(void);
void mic_diag(void);
void mic_rearm(void);
void mic_rms(void);          /* 回一行：16 路交流有效值 mVrms */
void mic_mean(void);         /* 回一行：16 路直流均值 mV */
void mic_dump(void);         /* 抓 512 帧 × 16 通道原始数据（二进制） */
void mic_dump_poll(void);    /* 主循环里调：把 dump 数据分块搬进发送队列 */
void mic_stats_reset(void);

/* 原始数据直通：'c' 开始，'x' 停止，'w' 打印状态（帧数/丢帧/队列丢弃） */
void mic_raw_start(void);
void mic_raw_stop(void);
void mic_raw_status(void);
uint8_t mic_raw_is_on(void);

/* 给 USB 吞吐测试用：算一帧的 CRC16（和直通帧同一个多项式） */
uint16_t mic_crc16(const uint8_t *p, uint32_t n);

/* 运行时改采样率：成功返回 1；失败返回 0，原配置不受影响。
   切换期间采集会停约 1ms，旧数据清空，计数器归零。 */
uint8_t mic_set_fs(uint32_t hz);
uint32_t mic_get_fs(void);   /* 实际采样率（42MHz/(ARR+1)） */

#endif
