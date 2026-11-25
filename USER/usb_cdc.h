#ifndef __USB_CDC_H
#define __USB_CDC_H

#include "stm32f4xx.h"

/* ============================================================
 * USB FS 虚拟串口（CDC-ACM）
 *
 * 硬件：OTG_FS 内置全速 PHY，PA11(D-) / PA12(D+)
 * 时钟：PLL_Q=7 → 336/7 = 48 MHz（本工程已经满足，不用改 PLL）
 * 冲突：USB 的 VBUS 检测脚是 PA9、ID 脚是 PA10，正好是 USART1，
 *       所以初始化里关掉 VBUS 检测、强制设备模式，串口保持可用。
 *
 * 速率：USB 2.0 全速规范上限 1.22 MB/s（19 包 × 64 B / 1 ms 帧），
 *       实测 0.7~1.0 MB/s。
 * ============================================================ */

/* TX 环形缓冲：8 KB @1 MB/s ≈ 8 ms 缓冲 */
#define USB_CDC_TX_SIZE     8192

void     usb_cdc_init(void);                       /* 初始化并连接主机 */
uint16_t usb_cdc_write(const uint8_t *p, uint16_t n);  /* 返回实际入队字节 */
uint16_t usb_cdc_free(void);                       /* 队列剩余空间 */
void     usb_cdc_poll(void);                       /* 主循环里调：把队列搬进端点 */
uint8_t  usb_cdc_configured(void);                 /* 主机已完成配置（可以发数据） */
uint8_t  usb_cdc_read(uint8_t *ch);                /* 从 USB 数据口取一个命令字节 */
uint32_t usb_cdc_dropped(void);                    /* 因队列满丢掉的字节 */
uint8_t  usb_cdc_get_line_coding(uint8_t *out7);   /* 主机设的波特率等（仅参考） */

/* 调试计数：枚举卡在哪一步。任一指针传 0 表示不取。
 *   rst=总线复位次数  enum=速度协商次数  setup=收到的 SETUP 数
 *   in=EP0 IN 完成数  out=EP0 OUT 完成数
 * 典型判断：
 *   setup=0            → EP0 OUT 没武装，SETUP 收不到
 *   setup>0 但 in=0    → EP0 IN 发不出去（FIFO/端点配置问题）
 *   setup 涨、in 也涨、但还是不认识 → 描述符内容问题
 */
void     usb_cdc_stats(uint32_t *rst, uint32_t *enums, uint32_t *setups,
                       uint32_t *in, uint32_t *out);

/* 寄存器快照（排查用）：
 *   0=GINTSTS 1=DCTL 2=DCFG 3=DSTS 4=GOTGCTL 5=GCCFG 6=GINTMSK 7=GAHBCFG 8=GUSBCFG
 */
uint32_t usb_cdc_reg(uint8_t which);

#endif
