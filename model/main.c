/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "User/App/app_freertos.h"
#include "User/Bsp/board_config.h"
#include "User/Bsp/bsp_oled.h"

static void boot_buzzer_test(void)
{
    /* Active-high buzzer through an NPN low-side switch. */
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_OUT_PIN);
    delay_cycles(CPUCLK_FREQ / 10U);
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_OUT_PIN);
}

int main(void)
{
    SYSCFG_DL_init();

    boot_buzzer_test();

#if BOARD_OLED_STANDALONE_TEST
    OLED_Init();
    OLED_WriteCommand(0xA5U); /* Force every OLED pixel on. */
    delay_cycles(CPUCLK_FREQ);
    OLED_WriteCommand(0xA4U);
    OLED_Clear();
    OLED_ShowString(0U, 0U, "OLED SOFT I2C", OLED_8X16);
    OLED_ShowString(0U, 24U, "PA0 SDA PA1 SCL", OLED_6X8);
    OLED_ShowString(0U, 40U, "ADDR 0x3C", OLED_6X8);
    OLED_Update();
    for (;;)
    {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_OUT_PIN);
        delay_cycles(CPUCLK_FREQ);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_OUT_PIN);
        delay_cycles(CPUCLK_FREQ);
    }
#else

    App_FreeRTOS_CreateTasks();
    vTaskStartScheduler();

    for (;;) {
    }
#endif
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *taskName)
{
    (void) task;
    (void) taskName;

    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}
