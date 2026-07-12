#ifndef LIB_RATE_FILTER_H
#define LIB_RATE_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t units_per_cycle;
    uint16_t sample_period_ms;
    uint8_t window_samples;
    uint8_t filter_div;
} RateFilter_Config;

typedef struct
{
    RateFilter_Config cfg;
    int32_t window_delta;
    uint8_t window_count;
    int16_t instant_rate;
    int16_t windowed_rate;
    int32_t filtered_rate;
} RateFilter;

typedef struct
{
    int32_t output;
    uint8_t filter_div;
    uint8_t initialized;
} LowPassFilter;

void RateFilter_Init(RateFilter *filter, const RateFilter_Config *cfg);
void RateFilter_Reset(RateFilter *filter);
void RateFilter_Update(RateFilter *filter, int16_t delta);
int16_t RateFilter_DeltaToRate(const RateFilter *filter, int16_t delta);
int16_t RateFilter_GetInstantRate(const RateFilter *filter);
int16_t RateFilter_GetFilteredRate(const RateFilter *filter);

void LowPassFilter_Init(LowPassFilter *filter, uint8_t filter_div);
void LowPassFilter_Reset(LowPassFilter *filter);
int32_t LowPassFilter_Update(LowPassFilter *filter, int32_t input);

#ifdef __cplusplus
}
#endif

#endif
