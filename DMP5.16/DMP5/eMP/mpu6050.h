#ifndef __MPU6050_H__
#define __MPU6050_H__


#include "main.h"

int MPU6050_DMP_Init();
int MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw);

#endif
