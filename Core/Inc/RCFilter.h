#ifndef RC_FILTER_H
#define RC_FILTER_H

typedef struct {
	float coeff[2];
	float out[2];
}RCFilter;

void RCFilter_Init(RCFilter * filter, float cutoffFreqHz, float sampleFreq_Hz);
float RCFilter_Update(RCFilter *filter, float input);

#endif
