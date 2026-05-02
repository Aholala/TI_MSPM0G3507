/// @file       Delay.c
/// @brief      提供微秒、毫秒和秒级的延时函数
/// @author     Ahola
/// @date       2026-05-02
/// @version    1.0

#include "Delay.h"
/**
 * @brief  微秒级延时
 * @param  xus 延时时长，范围：0~4294967295
 * @retval 无
 */
void Delay_us(uint32_t xus)
{
  /* MSPM0G3507 默认主频 32MHz，1us = 32个周期 */
  delay_cycles((CPUCLK_FREQ / 1000000) * xus);
}

/**
 * @brief  毫秒级延时
 * @param  xms 延时时长，范围：0~4294967295
 * @retval 无
 */
void Delay_ms(uint32_t xms)
{
  delay_cycles((CPUCLK_FREQ / 1000) * xms);
}

/**
 * @brief  秒级延时
 * @param  xs 延时时长，范围：0~4294967295
 * @retval 无
 */
void Delay_s(uint32_t xs)
{
  while (xs--)
  {
    Delay_ms(1000);
  }
}
