#include "main.h"

#include "FreeRTOS.h"
#include "board_config.h"
#include "task.h"

TIM_HandleTypeDef htim3 = {
    .autoreload = BOARD_TB6612_PWM_MAX_DUTY,
};
TIM_HandleTypeDef htim4 = {
    .autoreload = 0xffffu,
};
TIM_HandleTypeDef htim8 = {
    .autoreload = 0xffffu,
};
SPI_HandleTypeDef hspi2 = {
    .Instance = SPI2,
};
I2C_HandleTypeDef hi2c1;

uint32_t HAL_GetTick(void)
{
    return (uint32_t)xTaskGetTickCount();
}

void HAL_Delay(uint32_t delay_ms)
{
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    (void)channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t channel)
{
    (void)htim;
    (void)channel;
    return HAL_OK;
}

HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    return HAL_SPI_STATE_READY;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                           uint8_t *tx_data,
                                           uint8_t *rx_data,
                                           uint16_t size,
                                           uint32_t timeout_ms)
{
    uint16_t index;

    (void)hspi;
    (void)timeout_ms;

    for (index = 0; index < size; ++index) {
        rx_data[index] = (tx_data != NULL) ? tx_data[index] : 0u;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi,
                                               uint8_t *tx_data,
                                               uint8_t *rx_data,
                                               uint16_t size)
{
    return HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, size, 0u);
}

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    return HAL_OK;
}

HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    return HAL_I2C_STATE_READY;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
                                           uint16_t dev_addr,
                                           uint8_t *data,
                                           uint16_t size,
                                           uint32_t timeout_ms)
{
    (void)hi2c;
    (void)dev_addr;
    (void)data;
    (void)size;
    (void)timeout_ms;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c,
                                               uint16_t dev_addr,
                                               uint8_t *data,
                                               uint16_t size)
{
    return HAL_I2C_Master_Transmit(hi2c, dev_addr, data, size, 0u);
}
