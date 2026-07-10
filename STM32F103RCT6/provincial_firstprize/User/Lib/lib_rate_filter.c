#include "lib_rate_filter.h"

#include <stddef.h>

static int16_t delta_to_rate(uint16_t units_per_cycle,
                             uint16_t sample_period_ms,
                             int32_t delta,
                             uint8_t samples)
{
    int32_t denominator;

    if ((units_per_cycle == 0u) || (sample_period_ms == 0u) || (samples == 0u))
    {
        return 0;
    }

    denominator = (int32_t)units_per_cycle *
                  (int32_t)sample_period_ms *
                  (int32_t)samples;
    return (int16_t)((delta * 60000) / denominator);
}

void RateFilter_Init(RateFilter *filter, const RateFilter_Config *cfg)
{
    if (filter == NULL)
    {
        return;
    }

    filter->cfg.units_per_cycle = 1u;
    filter->cfg.sample_period_ms = 1u;
    filter->cfg.window_samples = 1u;
    filter->cfg.filter_div = 1u;
    if (cfg != NULL)
    {
        filter->cfg = *cfg;
    }

    if (filter->cfg.units_per_cycle == 0u)
    {
        filter->cfg.units_per_cycle = 1u;
    }

    if (filter->cfg.sample_period_ms == 0u)
    {
        filter->cfg.sample_period_ms = 1u;
    }

    if (filter->cfg.window_samples == 0u)
    {
        filter->cfg.window_samples = 1u;
    }

    if (filter->cfg.filter_div == 0u)
    {
        filter->cfg.filter_div = 1u;
    }

    RateFilter_Reset(filter);
}

void RateFilter_Reset(RateFilter *filter)
{
    if (filter == NULL)
    {
        return;
    }

    filter->window_delta = 0;
    filter->window_count = 0u;
    filter->instant_rate = 0;
    filter->windowed_rate = 0;
    filter->filtered_rate = 0;
}

void RateFilter_Update(RateFilter *filter, int16_t delta)
{
    if (filter == NULL)
    {
        return;
    }

    filter->instant_rate = RateFilter_DeltaToRate(filter, delta);
    filter->window_delta += delta;
    ++filter->window_count;

    if (filter->window_count >= filter->cfg.window_samples)
    {
        filter->windowed_rate = delta_to_rate(filter->cfg.units_per_cycle,
                                              filter->cfg.sample_period_ms,
                                              filter->window_delta,
                                              filter->window_count);
        filter->window_delta = 0;
        filter->window_count = 0u;
    }

    filter->filtered_rate +=
        (((int32_t)filter->windowed_rate - filter->filtered_rate) /
         (int32_t)filter->cfg.filter_div);
}

int16_t RateFilter_DeltaToRate(const RateFilter *filter, int16_t delta)
{
    if (filter == NULL)
    {
        return 0;
    }

    return delta_to_rate(filter->cfg.units_per_cycle,
                         filter->cfg.sample_period_ms,
                         delta,
                         1u);
}

int16_t RateFilter_GetInstantRate(const RateFilter *filter)
{
    return (filter != NULL) ? filter->instant_rate : 0;
}

int16_t RateFilter_GetFilteredRate(const RateFilter *filter)
{
    return (filter != NULL) ? (int16_t)filter->filtered_rate : 0;
}
