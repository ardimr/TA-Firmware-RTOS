#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct MatrixTranform_t{
	double x[3];
	double y[3];
	double z[3];

	double acc;
}MatrixTransform;

void MatrixTransformInit(MatrixTransform * Matrix);

#endif
