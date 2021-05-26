#include "KalmanFilter.h"
#include <math.h>

void MatrixTransformInit(MatrixTransform * Matrix){
	Matrix->x[0] = 1;
	Matrix->x[1] = 0;
	Matrix->x[2] = 0;
	Matrix->y[0] = 0;
	Matrix->y[1] = 1;
	Matrix->y[2] = 0;
	Matrix->z[0] = 0;
	Matrix->z[1] = 0;
	Matrix->z[2] = 1;

	Matrix->acc = 0;
}
