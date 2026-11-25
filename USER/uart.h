#ifndef __UART_H
#define __UART_H

#include "stm32f4xx.h"

/* 串口：USART1，PA9 发 / PA10 收，115200 8N1
 *
 * 注意：本工程里串口**只做命令通道**（A/V/d/c/x/w/u/?），原始数据走 USB CDC。
 * 命令都是几十字节的文本，115200 完全够用，而且兼容性最好
 * （CH340/CP2102/PL2303 都支持；921600 有些模块/驱动上不去）。
 * 如果以后想吃高波特率，改这个宏就行，数据通道不受影响。
 *
 * 发送走环形缓冲 + 主循环轮询，uart_putc / uart_puts 永远不阻塞。
 * 队列满了就丢字符并记数，绝不原地等。
 */

#define UART_BAUD       115200

/* 调试标记：上电多发 'A' 'B' 两个字符，用来定位程序卡在哪一步。
   调通之后把这里改成 0 就干净了。 */
#define UART_BEACON     0

void uart_init(void);

void uart_tx_poll(void);        /* 主循环里调，把发送队列搬出去 */
void uart_beacon(char c);       /* 调试用：轮询直发一个字符 */

void uart_putc(uint8_t b);
void uart_puts(const char *s);
void uart_put_u32(uint32_t v);
void uart_put_hex32(uint32_t v);
void uart_put_u32_pad(uint32_t v, uint32_t width);
void uart_write(const uint8_t *p, uint32_t n);

uint16_t uart_tx_free(void);
uint32_t uart_tx_dropped(void);

/* 取一个收到的字节，没有就返回 0 */
uint8_t uart_getc(uint8_t *ch);

#endif
