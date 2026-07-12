/**
 * @file bsp_icm45686.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __ICM45686_BSP_H
#define __ICM45686_BSP_H

#include <stdint.h>

typedef void (*icm45686_bsp_int1_cb_t)(void *arg);
typedef void (*icm45686_bsp_dma_cb_t)(void *arg, int status);

typedef struct {
	void *ctx;
	int (*init)(void *ctx);
	void (*cs_low)(void *ctx);
	void (*cs_high)(void *ctx);
	int (*spi_transfer)(void *ctx, const uint8_t *tx, uint8_t *rx, uint32_t len);
	int (*spi_transfer_dma)(void *ctx, const uint8_t *tx, uint8_t *rx, uint32_t len,
	                        icm45686_bsp_dma_cb_t cb, void *cb_arg);
	void (*sleep_us)(void *ctx, uint32_t us);
	int (*int1_init)(void *ctx, icm45686_bsp_int1_cb_t cb, void *cb_arg);
} icm45686_bsp_t;

int icm45686_bsp_init(void *ctx);
void icm45686_bsp_cs_low(void *ctx);
void icm45686_bsp_cs_high(void *ctx);
int icm45686_bsp_spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, uint32_t len);
int icm45686_bsp_spi_transfer_dma(void *ctx, const uint8_t *tx, uint8_t *rx, uint32_t len,
                                  icm45686_bsp_dma_cb_t cb, void *cb_arg);
void icm45686_bsp_sleep_us(void *ctx, uint32_t us);
int icm45686_bsp_int1_init(void *ctx, icm45686_bsp_int1_cb_t cb, void *cb_arg);
void icm45686_bsp_int1_irq_handler(void);

extern const icm45686_bsp_t icm45686_bsp_default;

#endif
