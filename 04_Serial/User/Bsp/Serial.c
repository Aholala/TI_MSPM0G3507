/**
 * @file Serial.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief
 * @version 1.0
 * @date 2026-05-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "User/Bsp/Serial.h"

void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str)
    {
        UART_send_char(uart, (uint8_t)*str);
        str++;
    }
}

void PRINT_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(PRINT_INST))
    {
    case DL_UART_IIDX_RX:
    {
        uint8_t rec = DL_UART_receiveData(PRINT_INST);
        UART_send_char(PRINT_INST, rec);
        break;
    }

    default:
        break;
    }
}
