#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>
#include "ti_msp_dl_config.h"   /* 提供 CPUCLK_FREQ 和 delay_cycles */

void Delay_us(uint32_t xus);
void Delay_ms(uint32_t xms);
void Delay_s(uint32_t xs);

#endif /* __DELAY_H */
