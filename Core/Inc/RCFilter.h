#ifndef RC_FILTER_H
#define RC_FILTER_H

#define BUFF_LENGTH 50
typedef struct {
	float coeff[2];
	float out[2];
}RCFilter;

typedef struct {
	float val[BUFF_LENGTH];
	float out;
}MovAvgFilter;

void RCFilter_Init(RCFilter * filter, float cutoffFreqHz, float sampleFreq_Hz);
float RCFilter_Update(RCFilter *filter, float input);

void MovAvgFilter_init(MovAvgFilter *filter);
float MovAvgFilter_Update(MovAvgFilter *filter, float input);

#endif
