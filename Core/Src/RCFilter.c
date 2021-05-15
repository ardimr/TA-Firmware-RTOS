#include "RCFilter.h"
#include <math.h>

void RCFilter_Init(RCFilter * filter, float cutoffFreqHz, float sampleFreq_Hz){
	/*Clear output buffer*/
	filter->out[0] = 0.0f;
	filter->out[1] = 0.0f;
	float sampleTime_s = 1.0f/ sampleFreq_Hz;
	/*Compute equivalent RC Constant from cutoff Frequency*/
	float RC = 1.0f/(2* M_PI * cutoffFreqHz);
	/*Pre-compute filter coefficient for first order low-pass filter*/
	filter->coeff[0] = sampleTime_s/(sampleTime_s+ RC);
	filter->coeff[1] = RC/(sampleTime_s+RC);

}

float RCFilter_Update(RCFilter *filter, float input){
	/* Shift output samples */
	filter->out[1] = filter->out[0];

	/* Compute new ouput sample */
	filter->out[0] = filter->coeff[0] * input+filter->coeff[1] * filter->out[1];

	/* Return Filtered sample */
	return filter->out[0];
}
