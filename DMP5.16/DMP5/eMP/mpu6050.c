#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#define DEFAULT_MPU_HZ (100)
#define Q30   (1073741824.0f)
static signed char gyro_orientation[9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };

enum packet_type_e {
	PACKET_TYPE_ACCEL,
	PACKET_TYPE_GYRO,
	PACKET_TYPE_QUAT,
	PACKET_TYPE_TAP,
	PACKET_TYPE_ANDROID_ORIENT,
	PACKET_TYPE_PEDO,
	PACKET_TYPE_MISC
};

//static int run_self_test(void) {
//	int result;
//	long gyro[3], accel[3];
//	unsigned char i = 0;
//	result = mpu_run_self_test(gyro, accel);
//	if (result == 0x3) {
//		/* Test passed. We can trust the gyro data here, so let's push it down
//		 * to the DMP.
//		 */
//		for (i = 0; i < 3; i++) {
//			gyro[i] = (long) (gyro[i] * 32.8f); //convert to +-1000dps
//			accel[i] *= 2048.f; //convert to +-16G
//			accel[i] = accel[i] >> 16;
//			gyro[i] = (long) (gyro[i] >> 16);
//		}
//
//		mpu_set_gyro_bias_reg(gyro);
//		mpu_set_accel_bias_6050_reg(accel);
//	} else {
//		return -1;
//	}
//	return 0;
//}
static int run_self_test(void) {
	int result;
	long gyro[3], accel[3];
	unsigned char i = 0;
	result = mpu_run_self_test(gyro, accel);
	if (result == 0x7) {
		/* Test passed. We can trust the gyro data here, so let's push it down
		 * to the DMP.
		 */
		for (i = 0; i < 3; i++) {
			gyro[i] = (long) (gyro[i] * 32.8f); //convert to +-1000dps
			accel[i] *= 2048.f; //convert to +-16G
			accel[i] = accel[i] >> 16;
			gyro[i] = (long) (gyro[i] >> 16);
		}

		mpu_set_gyro_bias_reg(gyro);
		mpu_set_accel_bias_6050_reg(accel);
	} else {
		return -1;
	}
	return 0;
}

static unsigned short inv_row_2_scale(const signed char *row) {
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;      // error
	return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
	unsigned short scalar;

	/*
	 XYZ  010_001_000 Identity Matrix
	 XZY  001_010_000
	 YXZ  010_000_001
	 YZX  000_010_001
	 ZXY  001_000_010
	 ZYX  000_001_010
	 */

	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;

	return scalar;
}

int MPU6050_DMP_Init() {
	int result;
	struct int_param_s int_param;

	result = mpu_init(&int_param);      //MPU初始化
	if (result != 0) {
		return -1;
	}
	result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);      //设置传感器
	if (result != 0) {
		return -2;
	}
	result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);      //设置fifo
	if (result != 0) {
		return -3;
	}
	result = mpu_set_sample_rate(DEFAULT_MPU_HZ);      //设置采样频率
	if (result != 0) {
		return -4;
	}
	result = dmp_load_motion_driver_firmware();      //加载固件
	if (result != 0) {
		return -5;
	}
	result = dmp_set_orientation(
			inv_orientation_matrix_to_scalar(gyro_orientation));      //设置方向
	if (result != 0) {
		return -6;
	}
	result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
	DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);      //设置DMP功能
	if (result != 0) {
		return -7;
	}
	result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);      //设置输出速率
	if (result != 0) {
		return -8;
	}
//	result = run_self_test();      //自检
//	if (result != 0) {
//		return -9;
//	}
	result = mpu_set_dmp_state(1);      //使能dmp
	if (result != 0) {
		return -10;
	}
	return 0;
}



int MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw) {
	float q0 = 0.0f;
	float q1 = 0.0f;
	float q2 = 0.0f;
	float q3 = 0.0f;
	short gyro[3];
	short accel[3];
	short sensors;
	unsigned char more;
	long quat[4];
	unsigned long timestamp;
	if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)) {
		return -1;
	}
	if (sensors & INV_XYZ_ACCEL) {

		q0 = quat[0] / Q30;
		q1 = quat[1] / Q30;
		q2 = quat[2] / Q30;
		q3 = quat[3] / Q30;

		*pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;
		*roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)
				* 57.3;
		*yaw = atan2(2 * (q1 * q2 + q0 * q3),
				q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;
	}
	return 0;
}

