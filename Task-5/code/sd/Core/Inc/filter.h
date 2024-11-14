#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>  // For expf(), isnan(), and PI definition
#include <stddef.h> // For NULL

#define PI 3.14159265358979323846

/* Min/Max/Average Statistics Block */
typedef struct {
    float min;
    float max;
    float sum;
    int count;
} MinMaxAvgStatistic;

/* Initialize the Statistic block */
void MinMaxAvgStatistic_Init(MinMaxAvgStatistic* stat) {
    stat->min = NAN;
    stat->max = NAN;
    stat->sum = 0;
    stat->count = 0;
}

/* Add value to the statistic */
void MinMaxAvgStatistic_Process(MinMaxAvgStatistic* stat, float value) {
    stat->min = isnan(stat->min) ? value : fminf(stat->min, value);
    stat->max = isnan(stat->max) ? value : fmaxf(stat->max, value);
    stat->sum += value;
    stat->count++;
}

/* Reset the statistic block */
void MinMaxAvgStatistic_Reset(MinMaxAvgStatistic* stat) {
    stat->min = NAN;
    stat->max = NAN;
    stat->sum = 0;
    stat->count = 0;
}

/* Get the minimum value */
float MinMaxAvgStatistic_Minimum(const MinMaxAvgStatistic* stat) {
    return stat->min;
}

/* Get the maximum value */
float MinMaxAvgStatistic_Maximum(const MinMaxAvgStatistic* stat) {
    return stat->max;
}

/* Get the average value */
float MinMaxAvgStatistic_Average(const MinMaxAvgStatistic* stat) {
    return stat->sum / stat->count;
}

/* High Pass Filter */
typedef struct {
    float kX;
    float kA0;
    float kA1;
    float kB1;
    float last_filter_value;
    float last_raw_value;
} HighPassFilter;

/* Initialize High Pass Filter with samples */
void HighPassFilter_Init(HighPassFilter* filter, float samples) {
    filter->kX = expf(-1 / samples);
    filter->kA0 = (1 + filter->kX) / 2;
    filter->kA1 = -(filter->kA0);
    filter->kB1 = filter->kX;
    filter->last_filter_value = NAN;
    filter->last_raw_value = NAN;
}

/* Initialize High Pass Filter with cutoff and sampling frequency */
void HighPassFilter_InitWithCutoff(HighPassFilter* filter, float cutoff, float sampling_frequency) {
    HighPassFilter_Init(filter, sampling_frequency / (cutoff * 2 * PI));
}

/* Apply High Pass Filter */
float HighPassFilter_Process(HighPassFilter* filter, float value) {
    if (isnan(filter->last_filter_value) || isnan(filter->last_raw_value)) {
        filter->last_filter_value = 0.0;
    } else {
        filter->last_filter_value = filter->kA0 * value + filter->kA1 * filter->last_raw_value + filter->kB1 * filter->last_filter_value;
    }
    filter->last_raw_value = value;
    return filter->last_filter_value;
}

/* Reset High Pass Filter */
void HighPassFilter_Reset(HighPassFilter* filter) {
    filter->last_raw_value = NAN;
    filter->last_filter_value = NAN;
}

/* Low Pass Filter */
typedef struct {
    float kX;
    float kA0;
    float kB1;
    float last_value;
} LowPassFilter;

/* Initialize Low Pass Filter with samples */
void LowPassFilter_Init(LowPassFilter* filter, float samples) {
    filter->kX = expf(-1 / samples);
    filter->kA0 = 1 - filter->kX;
    filter->kB1 = filter->kX;
    filter->last_value = NAN;
}

/* Initialize Low Pass Filter with cutoff and sampling frequency */
void LowPassFilter_InitWithCutoff(LowPassFilter* filter, float cutoff, float sampling_frequency) {
    LowPassFilter_Init(filter, sampling_frequency / (cutoff * 2 * PI));
}

/* Apply Low Pass Filter */
float LowPassFilter_Process(LowPassFilter* filter, float value) {
  if (isnan(filter->last_value)) {
       filter->last_value = value;
   } else {
        filter->last_value = filter->kA0 * value + filter->kB1 * filter->last_value;
   }
    return filter->last_value;
}

/* Reset Low Pass Filter */
void LowPassFilter_Reset(LowPassFilter* filter) {
    filter->last_value = NAN;
}

/* Differentiator */
typedef struct {
    float sampling_frequency;
    float last_value;
} Differentiator;

/* Initialize Differentiator */
void Differentiator_Init(Differentiator* diff, float sampling_frequency) {
    diff->sampling_frequency = sampling_frequency;
    diff->last_value = NAN;
}

/* Apply Differentiator */
float Differentiator_Process(Differentiator* diff, float value) {
    if (isnan(diff->last_value)) {
        diff->last_value = value;
        return 0.0;  // No change in the first iteration
    }
    float result = (value - diff->last_value) * diff->sampling_frequency;
    diff->last_value = value;
    return result;
}

/* Reset Differentiator */
void Differentiator_Reset(Differentiator* diff) {
    diff->last_value = NAN;
}

/* Moving Average Filter */
typedef struct {
    int index;
    int count;
    int buffer_size;
    float* values;
} MovingAverageFilter;

/* Initialize Moving Average Filter */
void MovingAverageFilter_Init(MovingAverageFilter* filter, float* buffer, int buffer_size) {
    filter->index = 0;
    filter->count = 0;
    filter->buffer_size = buffer_size;
    filter->values = buffer;
}

/* Apply Moving Average Filter */
float MovingAverageFilter_Process(MovingAverageFilter* filter, float value) {
    filter->values[filter->index] = value;
    filter->index = (filter->index + 1) % filter->buffer_size;
    if (filter->count < filter->buffer_size) {
        filter->count++;
    }

    float sum = 0.0;
    for (int i = 0; i < filter->count; i++) {
        sum += filter->values[i];
    }
    return sum / filter->count;
}

/* Reset Moving Average Filter */
void MovingAverageFilter_Reset(MovingAverageFilter* filter) {
    filter->index = 0;
    filter->count = 0;
}

#endif // FILTERS_H
