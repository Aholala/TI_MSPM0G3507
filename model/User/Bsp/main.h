#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#include "ti_msp_dl_config.h"
#include <ti/driverlib/dl_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef GPIO_Regs GPIO_TypeDef;
typedef uint32_t GPIO_PinState;

#ifndef GPIOC
#define GPIOC GPIOA
#endif

#define GPIO_PIN_RESET 0u
#define GPIO_PIN_SET   1u

#define GPIO_PIN_0  DL_GPIO_PIN_0
#define GPIO_PIN_1  DL_GPIO_PIN_1
#define GPIO_PIN_2  DL_GPIO_PIN_2
#define GPIO_PIN_3  DL_GPIO_PIN_3
#define GPIO_PIN_4  DL_GPIO_PIN_4
#define GPIO_PIN_5  DL_GPIO_PIN_5
#define GPIO_PIN_6  DL_GPIO_PIN_6
#define GPIO_PIN_7  DL_GPIO_PIN_7
#define GPIO_PIN_8  DL_GPIO_PIN_8
#define GPIO_PIN_9  DL_GPIO_PIN_9
#define GPIO_PIN_10 DL_GPIO_PIN_10
#define GPIO_PIN_11 DL_GPIO_PIN_11
#define GPIO_PIN_12 DL_GPIO_PIN_12
#define GPIO_PIN_13 DL_GPIO_PIN_13
#define GPIO_PIN_14 DL_GPIO_PIN_14
#define GPIO_PIN_15 DL_GPIO_PIN_15
#define GPIO_PIN_16 DL_GPIO_PIN_16
#define GPIO_PIN_17 DL_GPIO_PIN_17
#define GPIO_PIN_18 DL_GPIO_PIN_18
#define GPIO_PIN_19 DL_GPIO_PIN_19
#define GPIO_PIN_20 DL_GPIO_PIN_20
#define GPIO_PIN_21 DL_GPIO_PIN_21
#define GPIO_PIN_22 DL_GPIO_PIN_22
#define GPIO_PIN_23 DL_GPIO_PIN_23
#define GPIO_PIN_24 DL_GPIO_PIN_24
#define GPIO_PIN_25 DL_GPIO_PIN_25
#define GPIO_PIN_26 DL_GPIO_PIN_26
#define GPIO_PIN_27 DL_GPIO_PIN_27
#define GPIO_PIN_28 DL_GPIO_PIN_28
#define GPIO_PIN_29 DL_GPIO_PIN_29
#define GPIO_PIN_30 DL_GPIO_PIN_30
#define GPIO_PIN_31 DL_GPIO_PIN_31

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3,
} HAL_StatusTypeDef;

typedef enum {
    HAL_SPI_STATE_RESET = 0,
    HAL_SPI_STATE_READY = 1,
    HAL_SPI_STATE_BUSY = 2,
} HAL_SPI_StateTypeDef;

typedef enum {
    HAL_I2C_STATE_RESET = 0,
    HAL_I2C_STATE_READY = 1,
    HAL_I2C_STATE_BUSY = 2,
} HAL_I2C_StateTypeDef;

typedef struct {
    void *Instance;
    uint32_t counter;
    uint32_t autoreload;
} TIM_HandleTypeDef;

typedef struct {
    void *Instance;
} SPI_HandleTypeDef;

typedef struct {
    void *Instance;
} I2C_HandleTypeDef;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;

#define SPI2 ((void *)0x40003800u)

#define TIM_CHANNEL_1   0x00000000u
#define TIM_CHANNEL_2   0x00000004u
#define TIM_CHANNEL_ALL 0x0000003Cu

#define SPI2_CS_GPIO_Port GPIOA
#define SPI2_CS_Pin       GPIO_PIN_0
#define ICM45686_INT1_Pin GPIO_PIN_1

static inline void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                           uint8_t *tx_data,
                                           uint8_t *rx_data,
                                           uint16_t size,
                                           uint32_t timeout_ms);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi,
                                               uint8_t *tx_data,
                                               uint8_t *rx_data,
                                               uint16_t size);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
                                           uint16_t dev_addr,
                                           uint8_t *data,
                                           uint16_t size,
                                           uint32_t timeout_ms);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c,
                                               uint16_t dev_addr,
                                               uint8_t *data,
                                               uint16_t size);

#define __HAL_TIM_SET_COUNTER(htim, value) \
    do {                                   \
        (htim)->counter = (uint32_t)(value); \
    } while (0)

#define __HAL_TIM_GET_COUNTER(htim) ((htim)->counter)
#define __HAL_TIM_GET_AUTORELOAD(htim) ((htim)->autoreload)
#define __HAL_TIM_SET_COMPARE(htim, channel, compare) \
    do {                                              \
        (void)(htim);                                 \
        (void)(channel);                              \
        (void)(compare);                              \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
