#include "icm45686_bsp.h"

#include "delay.h"
#include "spi.h"

#define ICM45686_SPI_CS PAout(4)

static icm45686_bsp_int1_cb_t s_int1_cb;
static void *s_int1_cb_arg;

int icm45686_bsp_init(void *ctx)
{
	GPIO_InitTypeDef gpio;

	(void)ctx;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio.Pin   = GPIO_PIN_4;
	gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio.Pull  = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio);

	ICM45686_SPI_CS = 1;
	SPI1_Init();
	SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_8);

	return 0;
}

void icm45686_bsp_cs_low(void *ctx)
{
	(void)ctx;
	ICM45686_SPI_CS = 0;
}

void icm45686_bsp_cs_high(void *ctx)
{
	(void)ctx;
	ICM45686_SPI_CS = 1;
}

int icm45686_bsp_spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, uint32_t len)
{
	uint8_t data;

	(void)ctx;

	for (uint32_t i = 0; i < len; i++) {
		data = tx ? tx[i] : 0x00;
		data = SPI1_ReadWriteByte(data);
		if (rx)
			rx[i] = data;
	}

	return 0;
}

int icm45686_bsp_spi_transfer_dma(void *ctx, const uint8_t *tx, uint8_t *rx, uint32_t len,
                                  icm45686_bsp_dma_cb_t cb, void *cb_arg)
{
	(void)ctx;
	(void)tx;
	(void)rx;
	(void)len;
	(void)cb;
	(void)cb_arg;

	return -1;
}

void icm45686_bsp_sleep_us(void *ctx, uint32_t us)
{
	(void)ctx;
	delay_us(us);
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
