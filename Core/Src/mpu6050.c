/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2019
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */


#include <math.h>
#include "mpu6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x00
#define ADXL345_POWER_CTL_REG 0x2D
#define SMPLRT_DIV_REG 0x2C
#define ACCEL_CONFIG_REG 0x31
#define ACCEL_XOUT_H_REG 0x32
//#define TEMP_OUT_H_REG 0x41
//#define GYRO_CONFIG_REG 0x1B
//#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR (0x53 << 1)

const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;
float scale_factor  = 0.0039;
uint32_t timer;

Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx) {
    uint8_t check, check_gyro;
    uint8_t Data;
    uint8_t registerContents;


    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ADXL345_DEVID_REG, 1, &check, 1, i2c_timeout);
    if (check == 0xE5)  // 0xE5 will be returned by the sensor if everything goes well
    {
        // power management register 0X2D we should write all 0's to wake the sensor up
        Data = 0x0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ADXL345_POWER_CTL_REG, 1, &Data, 1, i2c_timeout);
        HAL_Delay(100);

        // Set DATA FORMAT
        Data = 0x08;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ADXL345_DATA_FORMAT_REG, 1, &Data, 1, i2c_timeout);
        HAL_Delay(100);

        // Get the current register contents, so we don't clobber the power bit
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ADXL345_BW_RATE_REG, 1, &registerContents, 1, i2c_timeout);
        registerContents &= 0x10;
        registerContents |= ADXL345_100HZ;

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ADXL345_BW_RATE_REG, 1, &registerContents, 1, i2c_timeout);
        HAL_Delay(100);

        // Set Power Control to Measurement mode
        Data = 0x08;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ADXL345_POWER_CTL_REG, 1, &Data, 1, i2c_timeout);
        HAL_Delay(100);

//        return check;
    }
    HAL_I2C_Mem_Read(I2Cx, ITG3205_ADDR, ITG3205_WHO_AM_I, 1, &check_gyro, 1, i2c_timeout);
    if(check_gyro == 0x68){
    	Data = 0x00;
    	HAL_I2C_Mem_Write(I2Cx, ITG3205_ADDR, ITG3205_PWRM, 1, &Data, 1, i2c_timeout);
    	HAL_Delay(100);

    	Data = 0x07;
    	HAL_I2C_Mem_Write(I2Cx, ITG3205_ADDR, ITG3205_SMPLRT, 1, &Data, 1, i2c_timeout);
    	HAL_Delay(100);

    	Data = 0x1E;
    	HAL_I2C_Mem_Write(I2Cx, ITG3205_ADDR, ITG3205_DLPFS, 1, &Data, 1, i2c_timeout);
    	HAL_Delay(100);

    	Data = 0x00;
    	HAL_I2C_Mem_Write(I2Cx, ITG3205_ADDR, ITG3205_INT_CFG, 1, &Data, 1, i2c_timeout);
    	HAL_Delay(100);
    }
    return check;
}

int16_t MPU6060_Read_Offset (I2C_HandleTypeDef *I2Cx, char axis){
	uint8_t address = 0;
	uint8_t offset;
	if (axis == ADXL345_X) {
		address = ADXL345_OFSX_REG;
	} else if (axis == ADXL345_Y) {
		address = ADXL345_OFSY_REG;
	} else if (axis == ADXL345_Z) {
		address = ADXL345_OFSZ_REG;
	}
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, address, 1, &offset, 1, i2c_timeout);

	return offset;
}
void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[1] << 8 | Rec_Data[0]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[3] << 8 | Rec_Data[2]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[5] << 8 | Rec_Data[4]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW * scale_factor ;
    DataStruct->Ay = DataStruct->Accel_Y_RAW * scale_factor ;
    DataStruct->Az = DataStruct->Accel_Z_RAW * scale_factor ;
}


float CalSpeed(MPU6050_t DataStruct, uint16_t IMU_TS){
	//Speed calculation
	float speed[3] = {0,0,0};
	float speed_result;
	speed[0] = speed[0] + DataStruct.Ax * IMU_TS * 0.001; //Vx TS in MS
	speed[1] = speed[1] + DataStruct.Ay * IMU_TS * 0.001; //Vy
	speed[2] = speed[2] + (DataStruct.Az - 5.52) * IMU_TS * 0.001; //Vz
	speed_result = sqrt(pow(speed[0],2) + pow(speed[1],2) + pow(speed[2],2));

	return speed_result;

}
void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, ITG3205_ADDR, ITG3205_XMSB, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 14.375;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 14.375;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 14.375;
}

//void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
//    uint8_t Rec_Data[2];
//    int16_t temp;
//
//    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register
//
//    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);
//
//    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
//    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
//}
//
//void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
//    uint8_t Rec_Data[14];
//    int16_t temp;
//
//    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
//
//    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);
//
//    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
//    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
//    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
//    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
//    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
//    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
//    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);
//
//    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
//    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
//    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
//    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
//    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
//    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
//    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
//
//    // Kalman angle solve
//    double dt = (double) (HAL_GetTick() - timer) / 1000;
//    timer = HAL_GetTick();
//    double roll;
//    double roll_sqrt = sqrt(
//            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
//    if (roll_sqrt != 0.0) {
//        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
//    } else {
//        roll = 0.0;
//    }
//    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
//    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
//        KalmanY.angle = pitch;
//        DataStruct->KalmanAngleY = pitch;
//    } else {
//        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
//    }
//    if (fabs(DataStruct->KalmanAngleY) > 90)
//        DataStruct->Gx = -DataStruct->Gx;
//    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gy, dt);
//
//}
//
//double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
//    double rate = newRate - Kalman->bias;
//    Kalman->angle += dt * rate;
//
//    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
//    Kalman->P[0][1] -= dt * Kalman->P[1][1];
//    Kalman->P[1][0] -= dt * Kalman->P[1][1];
//    Kalman->P[1][1] += Kalman->Q_bias * dt;
//
//    double S = Kalman->P[0][0] + Kalman->R_measure;
//    double K[2];
//    K[0] = Kalman->P[0][0] / S;
//    K[1] = Kalman->P[1][0] / S;
//
//    double y = newAngle - Kalman->angle;
//    Kalman->angle += K[0] * y;
//    Kalman->bias += K[1] * y;
//
//    double P00_temp = Kalman->P[0][0];
//    double P01_temp = Kalman->P[0][1];
//
//    Kalman->P[0][0] -= K[0] * P00_temp;
//    Kalman->P[0][1] -= K[0] * P01_temp;
//    Kalman->P[1][0] -= K[1] * P00_temp;
//    Kalman->P[1][1] -= K[1] * P01_temp;
//
//    return Kalman->angle;
//};
