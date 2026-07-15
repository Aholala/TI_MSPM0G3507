#include "bsp_i2c.h"

#include <stdbool.h>
#include <stddef.h>

#define BSP_I2C_TIMEOUT_LOOPS 1000000u
/*
 * MSPM0 I2C_ERR_13 requires a delay after starting a controller transfer.
 * Use the slowest configured bus (OLED at 100 kHz) for a conservative delay.
 * This also safely covers the MPU6050 bus at 400 kHz.
 */
#define BSP_I2C_BUS_HZ 100000u
#define BSP_I2C_START_DELAY_CYCLES ((CPUCLK_FREQ / BSP_I2C_BUS_HZ) * 3u)

static BspI2c_Status wait_status(I2C_Regs *i2c,
                                uint32_t mask,
                                uint8_t wait_while_set)
{
    uint32_t timeout = BSP_I2C_TIMEOUT_LOOPS;

    while (timeout-- != 0u)
    {
        uint32_t status = DL_I2C_getControllerStatus(i2c);

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0u)
        {
            return BSP_I2C_ERROR;
        }
        if (wait_while_set != 0u)
        {
            if ((status & mask) == 0u)
            {
                return BSP_I2C_OK;
            }
        }
        else if ((status & mask) != 0u)
        {
            return BSP_I2C_OK;
        }
    }

    return BSP_I2C_TIMEOUT;
}

static BspI2c_Status write_transfer(I2C_Regs *i2c,
                                   uint8_t address_7bit,
                                   const uint8_t *data,
                                   uint16_t size,
                                   DL_I2C_CONTROLLER_STOP stop)
{
    uint16_t sent;
    uint32_t timeout = BSP_I2C_TIMEOUT_LOOPS;
    BspI2c_Status status;

    status = wait_status(i2c, DL_I2C_CONTROLLER_STATUS_IDLE, 0u);
    if (status != BSP_I2C_OK)
    {
        return status;
    }

    DL_I2C_flushControllerTXFIFO(i2c);
    sent = DL_I2C_fillControllerTXFIFO(i2c, data, size);
    DL_I2C_startControllerTransferAdvanced(i2c,
                                           address_7bit,
                                           DL_I2C_CONTROLLER_DIRECTION_TX,
                                           size,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           stop,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(BSP_I2C_START_DELAY_CYCLES);

    while (sent < size)
    {
        uint32_t controller_status = DL_I2C_getControllerStatus(i2c);

        if ((controller_status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0u)
        {
            return BSP_I2C_ERROR;
        }
        if (!DL_I2C_isControllerTXFIFOFull(i2c))
        {
            DL_I2C_transmitControllerData(i2c, data[sent++]);
            timeout = BSP_I2C_TIMEOUT_LOOPS;
        }
        else if (timeout-- == 0u)
        {
            return BSP_I2C_TIMEOUT;
        }
    }

    return wait_status(i2c, DL_I2C_CONTROLLER_STATUS_BUSY, 1u);
}

BspI2c_Status BspI2c_Write(I2C_Regs *i2c,
                           uint8_t address_7bit,
                           const uint8_t *data,
                           uint16_t size)
{
    if ((i2c == NULL) || (data == NULL) || (size == 0u))
    {
        return BSP_I2C_INVALID_ARG;
    }

    return write_transfer(i2c,
                          address_7bit,
                          data,
                          size,
                          DL_I2C_CONTROLLER_STOP_ENABLE);
}

BspI2c_Status BspI2c_WriteRead(I2C_Regs *i2c,
                               uint8_t address_7bit,
                               const uint8_t *write_data,
                               uint16_t write_size,
                               uint8_t *read_data,
                               uint16_t read_size)
{
    uint16_t received = 0u;
    uint32_t timeout = BSP_I2C_TIMEOUT_LOOPS;
    BspI2c_Status status;

    if ((i2c == NULL) || (write_data == NULL) || (write_size == 0u) ||
        (read_data == NULL) || (read_size == 0u))
    {
        return BSP_I2C_INVALID_ARG;
    }

    /*
     * The MPU6050 retains its register pointer across STOP.  Using a complete
     * write transaction here also guarantees that BUSY can clear before the
     * receive transaction, instead of waiting forever while holding the bus
     * for a repeated START.
     */
    status = write_transfer(i2c,
                            address_7bit,
                            write_data,
                            write_size,
                            DL_I2C_CONTROLLER_STOP_ENABLE);
    if (status != BSP_I2C_OK)
    {
        return status;
    }

    DL_I2C_flushControllerRXFIFO(i2c);
    DL_I2C_startControllerTransferAdvanced(i2c,
                                           address_7bit,
                                           DL_I2C_CONTROLLER_DIRECTION_RX,
                                           read_size,
                                           DL_I2C_CONTROLLER_START_ENABLE,
                                           DL_I2C_CONTROLLER_STOP_ENABLE,
                                           DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(BSP_I2C_START_DELAY_CYCLES);

    while (received < read_size)
    {
        uint32_t controller_status = DL_I2C_getControllerStatus(i2c);

        if ((controller_status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0u)
        {
            return BSP_I2C_ERROR;
        }
        if (!DL_I2C_isControllerRXFIFOEmpty(i2c))
        {
            read_data[received++] = DL_I2C_receiveControllerData(i2c);
            timeout = BSP_I2C_TIMEOUT_LOOPS;
        }
        else if (timeout-- == 0u)
        {
            return BSP_I2C_TIMEOUT;
        }
    }

    return wait_status(i2c, DL_I2C_CONTROLLER_STATUS_BUSY, 1u);
}
