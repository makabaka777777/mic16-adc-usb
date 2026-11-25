/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   异常处理。串口和 DMA 的中断分别写在 uart.c / mic.c 里，
  *          这里不要重复定义，否则链接时符号冲突。
  *
  *          防跑飞策略：所有硬件异常都直接复位重来，而不是死循环
  *          停在那里。配合主循环的 IWDG（约 2s 超时），任何跑飞、
  *          死锁、异常中断风暴都会被硬件拉回，重新拉起采集。
  *          开机后串口会打印 "MIC16_ADC123 ready"，连续打印多次
  *          说明在反复复位，需要查具体异常原因。
  ******************************************************************************
  */

#include "stm32f4xx_it.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    NVIC_SystemReset();
}

void MemManage_Handler(void)
{
    NVIC_SystemReset();
}

void BusFault_Handler(void)
{
    NVIC_SystemReset();
}

void UsageFault_Handler(void)
{
    NVIC_SystemReset();
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/* SysTick 没开，不需要实现 */
