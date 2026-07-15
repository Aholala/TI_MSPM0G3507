#include "bsp_oled_soft_i2c.h"

#include "ti_msp_dl_config.h"

#define OLED_SOFT_I2C_PORT GPIOA
#define OLED_SOFT_I2C_PIN_0 DL_GPIO_PIN_0
#define OLED_SOFT_I2C_PIN_1 DL_GPIO_PIN_1
#define OLED_SOFT_I2C_SDA_IOMUX IOMUX_PINCM1
#define OLED_SOFT_I2C_SCL_IOMUX IOMUX_PINCM2
#define OLED_SOFT_I2C_HALF_PERIOD_US 2U

volatile uint32_t g_oled_soft_i2c_nack_count;
volatile uint8_t g_oled_soft_i2c_detected_address;
volatile uint8_t g_oled_soft_i2c_pins_swapped;
volatile uint8_t g_oled_soft_i2c_sda_idle_high;
volatile uint8_t g_oled_soft_i2c_scl_idle_high;

static uint32_t g_sda_pin = OLED_SOFT_I2C_PIN_1;
static uint32_t g_scl_pin = OLED_SOFT_I2C_PIN_0;

static void start_condition(void);
static void stop_condition(void);
static uint8_t write_byte(uint8_t value);

static void bus_delay(void)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * OLED_SOFT_I2C_HALF_PERIOD_US);
}

static void drive_low(uint32_t pin)
{
    DL_GPIO_clearPins(OLED_SOFT_I2C_PORT, pin);
    DL_GPIO_enableOutput(OLED_SOFT_I2C_PORT, pin);
}

static void release_line(uint32_t pin)
{
    DL_GPIO_disableOutput(OLED_SOFT_I2C_PORT, pin);
}

void BspOledSoftI2c_Init(void)
{
    uint8_t swapped;
    uint8_t address;

    DL_GPIO_initDigitalInputFeatures(OLED_SOFT_I2C_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(OLED_SOFT_I2C_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(OLED_SOFT_I2C_PORT,
        OLED_SOFT_I2C_PIN_0 | OLED_SOFT_I2C_PIN_1);
    release_line(OLED_SOFT_I2C_PIN_0 | OLED_SOFT_I2C_PIN_1);
    g_oled_soft_i2c_nack_count = 0U;
    g_oled_soft_i2c_detected_address = 0U;
    g_oled_soft_i2c_pins_swapped = 0U;
    bus_delay();

    /* Try both possible PA0/PA1 assignments and scan every legal address. */
    for (swapped = 1U; swapped < 3U; ++swapped)
    {
        uint8_t actual_swapped = (swapped == 1U) ? 1U : 0U;

        g_sda_pin = (actual_swapped == 0U) ? OLED_SOFT_I2C_PIN_0 : OLED_SOFT_I2C_PIN_1;
        g_scl_pin = (actual_swapped == 0U) ? OLED_SOFT_I2C_PIN_1 : OLED_SOFT_I2C_PIN_0;
        release_line(OLED_SOFT_I2C_PIN_0 | OLED_SOFT_I2C_PIN_1);
        bus_delay();
        g_oled_soft_i2c_sda_idle_high =
            ((DL_GPIO_readPins(OLED_SOFT_I2C_PORT, g_sda_pin) & g_sda_pin) != 0U) ? 1U : 0U;
        g_oled_soft_i2c_scl_idle_high =
            ((DL_GPIO_readPins(OLED_SOFT_I2C_PORT, g_scl_pin) & g_scl_pin) != 0U) ? 1U : 0U;

        /* Probe the two normal OLED addresses first. */
        address = 0x3CU;
        start_condition();
        if (write_byte((uint8_t)(address << 1U)) != 0U)
        {
            stop_condition();
            g_oled_soft_i2c_detected_address = address;
            g_oled_soft_i2c_pins_swapped = actual_swapped;
            break;
        }
        stop_condition();
        address = 0x3DU;
        start_condition();
        if (write_byte((uint8_t)(address << 1U)) != 0U)
        {
            stop_condition();
            g_oled_soft_i2c_detected_address = address;
            g_oled_soft_i2c_pins_swapped = actual_swapped;
            break;
        }
        stop_condition();

        for (address = 0x08U; address <= 0x77U; ++address)
        {
            if ((address == 0x3CU) || (address == 0x3DU))
            {
                continue;
            }
            start_condition();
            if (write_byte((uint8_t)(address << 1U)) != 0U)
            {
                stop_condition();
                g_oled_soft_i2c_detected_address = address;
                g_oled_soft_i2c_pins_swapped = actual_swapped;
                break;
            }
            stop_condition();
        }
        if (g_oled_soft_i2c_detected_address != 0U)
        {
            break;
        }
    }

    if (g_oled_soft_i2c_detected_address == 0U)
    {
        g_sda_pin = OLED_SOFT_I2C_PIN_1;
        g_scl_pin = OLED_SOFT_I2C_PIN_0;
    }
    g_oled_soft_i2c_nack_count = 0U;
}

static void start_condition(void)
{
    release_line(g_sda_pin);
    release_line(g_scl_pin);
    bus_delay();
    drive_low(g_sda_pin);
    bus_delay();
    drive_low(g_scl_pin);
}

static void stop_condition(void)
{
    drive_low(g_sda_pin);
    bus_delay();
    release_line(g_scl_pin);
    bus_delay();
    release_line(g_sda_pin);
    bus_delay();
}

static uint8_t write_byte(uint8_t value)
{
    uint8_t bit;
    uint8_t acknowledged;

    for (bit = 0U; bit < 8U; ++bit)
    {
        if ((value & 0x80U) != 0U)
        {
            release_line(g_sda_pin);
        }
        else
        {
            drive_low(g_sda_pin);
        }
        bus_delay();
        release_line(g_scl_pin);
        bus_delay();
        drive_low(g_scl_pin);
        value <<= 1U;
    }

    release_line(g_sda_pin);
    bus_delay();
    release_line(g_scl_pin);
    bus_delay();
    acknowledged = ((DL_GPIO_readPins(OLED_SOFT_I2C_PORT,
        g_sda_pin) & g_sda_pin) == 0U) ? 1U : 0U;
    drive_low(g_scl_pin);
    return acknowledged;
}

uint8_t BspOledSoftI2c_Write(uint8_t address_7bit,
                             const uint8_t *data,
                             uint16_t count)
{
    uint8_t ok = 1U;

    if ((data == 0) || (count == 0U))
    {
        return 0U;
    }

    start_condition();
    ok &= write_byte((uint8_t)(address_7bit << 1U));
    while (count-- != 0U)
    {
        ok &= write_byte(*data++);
    }
    stop_condition();
    if (ok == 0U)
    {
        ++g_oled_soft_i2c_nack_count;
    }
    return ok;
}
