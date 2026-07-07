#include "bsp_icm45686.h"

#include "main.h"
#include "spi.h"
#include "stm32f1xx_hal.h"

#include <string.h>

#define ICM45686_SPI_TIMEOUT_MS 10U
#define ICM45686_SPI_DMA_TIMEOUT_MS 10U
#define ICM45686_SPI_DMA_SCRATCH_LEN 256U

static icm45686_bsp_int1_cb_t s_int1_cb;
static void *s_int1_cb_arg;
static volatile uint8_t s_spi_dma_done;
static volatile uint8_t s_spi_dma_error;

static int wait_spi_ready(uint32_t timeout_ms)
{
	uint32_t start_tick = HAL_GetTick();

	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
		if ((HAL_GetTick() - start_tick) >= timeout_ms)
			return -1;
	}

	return 0;
}

static void sleep_us_busy(uint32_t us)
{
	uint32_t ticks;
	uint32_t start_tick;

	if (us == 0U)
		return;

	ticks = (SystemCoreClock / 1000000U) * us;
	start_tick = DWT->CYCCNT;
	if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	}
	if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
		DWT->CYCCNT = 0U;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
		start_tick = DWT->CYCCNT;
	}

	while ((uint32_t)(DWT->CYCCNT - start_tick) < ticks) {
	}
}

int icm45686_bsp_init(void *ctx)
{
	(void)ctx;

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	return 0;
}

void icm45686_bsp_cs_low(void *ctx)
{
	(void)ctx;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

void icm45686_bsp_cs_high(void *ctx)
{
	(void)ctx;
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

int icm45686_bsp_spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, uint32_t len)
{
	uint8_t tx_byte;
	uint8_t rx_byte;

	(void)ctx;

	if (wait_spi_ready(ICM45686_SPI_TIMEOUT_MS) != 0)
		return -1;

	for (uint32_t i = 0; i < len; i++) {
		tx_byte = tx ? tx[i] : 0x00U;
		if (HAL_SPI_TransmitReceive(&hspi2, &tx_byte, &rx_byte, 1U,
		                            ICM45686_SPI_TIMEOUT_MS) != HAL_OK)
			return -1;
		if (rx)
			rx[i] = rx_byte;
	}

	return 0;
}

int icm45686_bsp_spi_transfer_dma(void *ctx, const uint8_t *tx, uint8_t *rx, uint32_t len,
                                  icm45686_bsp_dma_cb_t cb, void *cb_arg)
{
	static uint8_t tx_scratch[ICM45686_SPI_DMA_SCRATCH_LEN];
	static uint8_t rx_scratch[ICM45686_SPI_DMA_SCRATCH_LEN];
	uint8_t *tx_buf;
	uint8_t *rx_buf;
	uint32_t start_tick;

	(void)ctx;

	if (len == 0U) {
		if (cb)
			cb(cb_arg);
		return 0;
	}

	if (len > ICM45686_SPI_DMA_SCRATCH_LEN)
		return icm45686_bsp_spi_transfer(ctx, tx, rx, len);

	tx_buf = (uint8_t *)tx;
	rx_buf = rx;
	if (!tx_buf) {
		memset(tx_scratch, 0, len);
		tx_buf = tx_scratch;
	}
	if (!rx_buf)
		rx_buf = rx_scratch;

	if (wait_spi_ready(ICM45686_SPI_TIMEOUT_MS) != 0)
		return -1;

	s_spi_dma_done = 0U;
	s_spi_dma_error = 0U;
	if (HAL_SPI_TransmitReceive_DMA(&hspi2, tx_buf, rx_buf, (uint16_t)len) != HAL_OK)
		return -1;

	start_tick = HAL_GetTick();
	while (!s_spi_dma_done) {
		if ((HAL_GetTick() - start_tick) >= ICM45686_SPI_DMA_TIMEOUT_MS) {
			(void)HAL_SPI_Abort(&hspi2);
			return -1;
		}
	}

	if (s_spi_dma_error)
		return -1;

	if (cb)
		cb(cb_arg);

	return 0;
}

void icm45686_bsp_sleep_us(void *ctx, uint32_t us)
{
	(void)ctx;
	if (us >= 1000U)
		HAL_Delay(us / 1000U);
	sleep_us_busy(us % 1000U);
}

int icm45686_bsp_int1_init(void *ctx, icm45686_bsp_int1_cb_t cb, void *cb_arg)
{
	(void)ctx;
	s_int1_cb     = cb;
	s_int1_cb_arg = cb_arg;

	return 0;
}

void icm45686_bsp_int1_irq_handler(void)
{
	if (s_int1_cb)
		s_int1_cb(s_int1_cb_arg);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ICM45686_INT1_Pin)
		icm45686_bsp_int1_irq_handler();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2)
		s_spi_dma_done = 1U;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2) {
		s_spi_dma_error = 1U;
		s_spi_dma_done = 1U;
	}
}

const icm45686_bsp_t icm45686_bsp_default = {
	.ctx              = 0,
	.init             = icm45686_bsp_init,
	.cs_low           = icm45686_bsp_cs_low,
	.cs_high          = icm45686_bsp_cs_high,
	.spi_transfer     = icm45686_bsp_spi_transfer,
	.spi_transfer_dma = icm45686_bsp_spi_transfer_dma,
	.sleep_us         = icm45686_bsp_sleep_us,
	.int1_init        = icm45686_bsp_int1_init,
};
