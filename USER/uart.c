#include "uart.h"

/* ------------------------------------------------------------
 * 收发都用环形缓冲
 *
 *   发送：主循环往 tx_buf 里塞，然后由 uart_tx_poll() 在主循环里搬出去。
 *         不用 TXE 中断 —— TXE 是电平标志，队列一空它就恒为 1，
 *         中断方式只要漏关一次就会无限重入、把主循环活活饿死，
 *         表现就是“串口一个字都发不出来”。轮询方式简单、确定、不会卡。
 *   接收：RXNE 中断塞，主循环取。
 * ------------------------------------------------------------ */

#define TX_RING     8192
#define RX_RING     64

static volatile uint8_t  tx_buf[TX_RING];
static volatile uint16_t tx_wr = 0;
static volatile uint16_t tx_rd = 0;
static volatile uint32_t tx_drop = 0;

static volatile uint8_t  rx_buf[RX_RING];
static volatile uint16_t rx_wr = 0;
static volatile uint16_t rx_rd = 0;

static uint8_t uart_up = 0;     /* uart_init() 跑完才置 1，beacon 靠它兜底 */

void uart_init(void)
{
    GPIO_InitTypeDef  g;
    USART_InitTypeDef u;
    NVIC_InitTypeDef  n;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    g.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    g.GPIO_Mode  = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd  = GPIO_PuPd_UP;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &g);

    USART_StructInit(&u);
    u.USART_BaudRate   = UART_BAUD;
    u.USART_WordLength = USART_WordLength_8b;
    u.USART_StopBits   = USART_StopBits_1;
    u.USART_Parity     = USART_Parity_No;
    u.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &u);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* TXE 中断从头到尾都不开。TXE 是“发送寄存器空”，空闲时恒为 1，
       一旦开着就会无限重入中断，主循环一个指令都跑不到。
       发送完全交给主循环里的 uart_tx_poll()，确定、不会卡。 */
    USART_ITConfig(USART1, USART_IT_TXE,  DISABLE);

    n.NVIC_IRQChannel                   = USART1_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;   /* 比 DMA 低一级 */
    n.NVIC_IRQChannelSubPriority        = 0;
    n.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&n);

    USART_Cmd(USART1, ENABLE);

    uart_up = 1;
}

/* ------------------------------------------------------------
 * 中断：只收字节
 *
 * 发送不用中断（见上面 uart_init 的说明），这里只管 RXNE。
 * ------------------------------------------------------------ */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t  ch   = (uint8_t)USART_ReceiveData(USART1);
        uint16_t next = (uint16_t)((rx_wr + 1) & (RX_RING - 1));

        if (next != rx_rd)
        {
            rx_buf[rx_wr] = ch;
            rx_wr = next;
        }
    }
    else if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART1);        /* 读一下清溢出 */
    }

    /* 发送不用中断，这里只管收。字节由 uart_tx_poll() 在主循环里搬走。 */
}

/* ------------------------------------------------------------ */

void uart_putc(uint8_t b)
{
    uint16_t next = (uint16_t)((tx_wr + 1) & (TX_RING - 1));

    if (next != tx_rd)
    {
        tx_buf[tx_wr] = b;
        tx_wr = next;
        /* 只入队，不发送，绝不阻塞。搬运交给 uart_tx_poll()。 */
    }
    else
    {
        tx_drop++;                              /* 满了就丢，绝不阻塞 */
    }
}

/* ------------------------------------------------------------
 * 主循环里调一次：串口空闲就把队列里的下一个字节搬出去。
 * 不阻塞（TXE 没空就直接返回），也不用任何中断。
 * ------------------------------------------------------------ */
void uart_tx_poll(void)
{
    if (tx_rd == tx_wr)                             /* 队列空，没东西可发 */
    {
        return;
    }

    if ((USART1->SR & USART_SR_TXE) == 0U)          /* 上一个字节还在移位 */
    {
        return;
    }

    USART1->DR = (uint16_t)(tx_buf[tx_rd]);
    tx_rd = (uint16_t)((tx_rd + 1) & (TX_RING - 1));
}

/* ------------------------------------------------------------
 * 调试专用：绕过缓冲和中断，直接用轮询把一个字符发出去。
 * 带 guard 计数，就算串口没配好也不会死等。
 * ------------------------------------------------------------ */
void uart_beacon(char c)
{
    uint32_t guard = 500000UL;

    if (!uart_up)
    {
        return;
    }

    while (((USART1->SR & USART_SR_TXE) == 0U) && (--guard != 0U))
    {
    }

    USART1->DR = (uint16_t)(uint8_t)c;
}

void uart_puts(const char *s)
{
    while (*s)
    {
        uart_putc((uint8_t)*s++);
    }
}

void uart_put_hex32(uint32_t v)
{
    uint8_t i;

    uart_puts("0x");
    for (i = 0; i < 8; i++)
    {
        uint8_t d = (uint8_t)(v >> ((7u - i) * 4u)) & 0xFu;
        uart_putc((uint8_t)((d < 10) ? ('0' + d) : ('A' + d - 10)));
    }
}

void uart_put_u32(uint32_t v)
{
    char     b[11];
    uint32_t i = 0;

    if (v == 0)
    {
        uart_putc('0');
        return;
    }

    while (v)
    {
        b[i++] = (char)('0' + (v % 10));
        v /= 10;
    }

    while (i)
    {
        uart_putc((uint8_t)b[--i]);
    }
}

void uart_put_u32_pad(uint32_t v, uint32_t width)
{
    char     b[11];
    uint32_t i = 0;
    uint32_t n;

    if (v == 0)
    {
        b[i++] = '0';
    }
    while (v)
    {
        b[i++] = (char)('0' + (v % 10));
        v /= 10;
    }

    n = i;
    while (n < width)
    {
        uart_putc(' ');
        n++;
    }
    while (i)
    {
        uart_putc((uint8_t)b[--i]);
    }
}

void uart_write(const uint8_t *p, uint32_t n)
{
    while (n--)
    {
        uart_putc(*p++);
    }
}

uint16_t uart_tx_free(void)
{
    return (uint16_t)((tx_rd - tx_wr - 1) & (TX_RING - 1));
}

uint32_t uart_tx_dropped(void)
{
    return tx_drop;
}

uint8_t uart_getc(uint8_t *ch)
{
    if (rx_rd == rx_wr)
    {
        return 0;
    }

    *ch   = rx_buf[rx_rd];
    rx_rd = (uint16_t)((rx_rd + 1) & (RX_RING - 1));
    return 1;
}
