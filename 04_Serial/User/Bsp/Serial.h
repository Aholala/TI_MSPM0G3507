/**
 * @file serial.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief
 * @version 1.0
 * @date 2026-05-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef SERIAL_H
#define SERIAL_H

#include "ti_msp_dl_config.h"

void UART_send_string(UART_Regs *uart, const char *str);
void UART_send_char(UART_Regs *uart, const uint8_t chr);

#endif
