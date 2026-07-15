#ifndef BSP_OLED_SOFT_I2C_H
#define BSP_OLED_SOFT_I2C_H

#include <stdint.h>

extern volatile uint32_t g_oled_soft_i2c_nack_count;
extern volatile uint8_t g_oled_soft_i2c_detected_address;
extern volatile uint8_t g_oled_soft_i2c_pins_swapped;
extern volatile uint8_t g_oled_soft_i2c_sda_idle_high;
extern volatile uint8_t g_oled_soft_i2c_scl_idle_high;

void BspOledSoftI2c_Init(void);
uint8_t BspOledSoftI2c_Write(uint8_t address_7bit,
                             const uint8_t *data,
                             uint16_t count);

#endif
