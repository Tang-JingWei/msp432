#include "mpu6050.h"
#include <math.h>

Fly_Angle Angle = {0};
mpu6050 Mpu6050_Dat = {0};
mpu6050 Mpu6050_Offset = {0};
float High_Offset = 0;

#define factor 0.15f
static float tBuff[3] = {0};
static float Norm_az;

#define PI 3.1415926f
#define squa(Sq) (((float)Sq) * ((float)Sq))
#define absu16(Math_X) ((Math_X) < 0 ? -(Math_X) : (Math_X))
#define absFloat(Math_X) ((Math_X) < 0 ? -(Math_X) : (Math_X))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

const float M_PI = 3.1415926535;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;
const float Gyro_G = 0.03051756f; //�����ǳ�ʼ������+-2000��ÿ����1 / (65536 / 4000) = 0.03051756*2
const float Gyro_Gr = 0.0005326f; //������ÿ��,ת������ÿ���� 2*0.03051756	 * 0.0174533f = 0.0005326*2

/* дһ���ֽ� */
uint8_t MPU6050_WriteReg(uint8_t reg_add, uint8_t reg_dat)
{
	IIC_Start();
	IIC_Send_Byte(MPU_WRITE); //����������ַ+д����
	if (IIC_Wait_Ack())				//�ȴ�Ӧ��
	{
		IIC_Stop();
		char str[50];
		sprintf((char *)str, "error\r\n");
		printf(str);
		delay_ms(100);
		return 1;
	}
	IIC_Send_Byte(reg_add); //д�Ĵ�����ַ
	IIC_Wait_Ack();					//�ȴ�Ӧ��
	IIC_Send_Byte(reg_dat); //��������
	if (IIC_Wait_Ack())			//�ȴ�ACK
	{
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}

/* ��ȡһ���ֽ� */
uint8_t MPU6050_ReadData(uint8_t reg_add, unsigned char *Read, uint8_t num)
{
	IIC_Start();
	IIC_Send_Byte(MPU_WRITE); //����������ַ+д����
	if (IIC_Wait_Ack())				//�ȴ�Ӧ��
	{
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg_add); //д�Ĵ�����ַ
	IIC_Wait_Ack();					//�ȴ�Ӧ��
	IIC_Start();
	IIC_Send_Byte(MPU_READ); //����������ַ+������
	IIC_Wait_Ack();					 //�ȴ�Ӧ��
	while (num)
	{
		if (num == 1)
			*Read = IIC_Read_Byte(0); //������,����nACK
		else
			*Read = IIC_Read_Byte(1); //������,����ACK
		num--;
		Read++;
	}
	IIC_Stop(); //����һ��ֹͣ����
	return 0;
}

/* ��ȡ��ʼ״̬ */
void MpuGetOffset(void)
{
	int32_t buffer[6] = {0};
	uint8_t k = 50;
	const int8_t MAX_GYRO_QUIET = 10;
	const int8_t MIN_GYRO_QUIET = -10;
	int16_t LastGyro[3] = {0};
	int16_t ErrorGyro[3];

	while (k--) //�жϷɿ��Ƿ��ھ�ֹ״̬, �ȴ������ȶ�
	{
		do
		{
			// HAL_Delay(10);
			delay_ms(10);
			while (MPU6050_ReadAll(&Mpu6050_Dat))
				;
			ErrorGyro[0] = Mpu6050_Dat.gx - LastGyro[0];
			LastGyro[0] = Mpu6050_Dat.gx;
			ErrorGyro[1] = Mpu6050_Dat.gy - LastGyro[1];
			LastGyro[1] = Mpu6050_Dat.gy;
			ErrorGyro[2] = Mpu6050_Dat.gz - LastGyro[2];
			LastGyro[2] = Mpu6050_Dat.gz;
		} while ((ErrorGyro[0] > MAX_GYRO_QUIET) || (ErrorGyro[0] < MIN_GYRO_QUIET) || (ErrorGyro[1] > MAX_GYRO_QUIET) || (ErrorGyro[1] < MIN_GYRO_QUIET) || (ErrorGyro[2] > MAX_GYRO_QUIET) || (ErrorGyro[2] < MIN_GYRO_QUIET));
	}
	for (int i = 0; i < 356; i++)
	{
		while (MPU6050_ReadAll(&Mpu6050_Dat))
			;
		if (100 <= i)
		{
			buffer[0] += Mpu6050_Dat.ax;
			buffer[1] += Mpu6050_Dat.ay;
			buffer[2] += Mpu6050_Dat.az;
			buffer[3] += Mpu6050_Dat.gx;
			buffer[4] += Mpu6050_Dat.gy;
			buffer[5] += Mpu6050_Dat.gz;
		}
	}
	Mpu6050_Offset.ax = buffer[0] >> 8;
	Mpu6050_Offset.ay = buffer[1] >> 8;
	Mpu6050_Offset.az = buffer[2] >> 8;
	Mpu6050_Offset.gx = buffer[3] >> 8;
	Mpu6050_Offset.gy = buffer[4] >> 8;
	Mpu6050_Offset.gz = buffer[5] >> 8;
	GetAngle(&Mpu6050_Dat, &Angle, 0.00626f);
	High_Offset = GetAz();
}

/* ��ʼ�� */
void MPU6050_Init(void)
{
	IIC_Init();
	// HAL_Delay(50);
	delay_ms(50);
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x03);

	//����Ƶ�ʷ�Ƶ�����üĴ���ָ��������������ʵķ�Ƶ��������ΪMPU-60X0���ɲ������ʡ�
	//---> ����Ƶ��=���������Ƶ��/��1+SMPLRT_DIV��
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV, 0x01);

	MPU6050_WriteReg(MPU6050_RA_CONFIG, 0x03);			 //���üĴ���
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG, 0x09); //���ٶȼ����üĴ���
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x10);	 //���������üĴ���
	MpuGetOffset();
}

/* ��ȡID 0x68 */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
	MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &Re, 1);
	return Re;
}

/* ��ȡԭʼ������ٶ� */
uint8_t MPU6050ReadAcc(mpu6050 *data)
{
	uint8_t buf[6];
	if (!MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6))
	{
		data->ax = (short)(buf[0] << 8) | buf[1];
		data->ay = (short)(buf[2] << 8) | buf[3];
		data->az = (short)(buf[4] << 8) | buf[5];
		return SUCCESS;
	}
	return ERROR;
}

/* ��ȡԭʼ������ٶ� */
uint8_t MPU6050ReadGyro(mpu6050 *data)
{
	uint8_t buf[6];
	if (!MPU6050_ReadData(MPU6050_GYRO_OUT, buf, 6))
	{
		data->gx = (short)(buf[0] << 8) | buf[1];
		data->gy = (short)(buf[2] << 8) | buf[3];
		data->gz = (short)(buf[4] << 8) | buf[5];
		return SUCCESS;
	}
	return ERROR;
}

/* ��ȡ�¶� */
uint8_t MPU6050_ReturnTemp(mpu6050 *data)
{
	short temp3;
	uint8_t buf[2];
	if (!MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H, buf, 2))
	{
		temp3 = (buf[0] << 8) | buf[1];
		data->Temperature = ((double)temp3 / 340.0) + 36.53;
		return SUCCESS;
	}
	return ERROR;
}

/* ��ȡ�������� */
uint8_t MPU6050_ReadAll(mpu6050 *data)
{
	mpu6050 *buf = data;
	if (!(MPU6050_ReturnTemp(buf) | MPU6050ReadGyro(buf) | MPU6050ReadAcc(buf)))
	{
		static ekf_filter ekfx = {0.02, 0, 0, 0, 0.001, 0.543};
		kalman_1(&ekfx, data->ax);
		data->ax = ekfx.out;

		static ekf_filter ekfy = {0.02, 0, 0, 0, 0.001, 0.543};
		kalman_1(&ekfy, data->ay);
		data->ay = ekfy.out;

		static ekf_filter ekfz = {0.02, 0, 0, 0, 0.001, 0.543};
		kalman_1(&ekfz, data->az);
		data->az = ekfz.out;

		data->gx = tBuff[0] = tBuff[0] * (1 - factor) + data->gx * factor;
		data->gy = tBuff[1] = tBuff[1] * (1 - factor) + data->gy * factor;
		data->gz = tBuff[2] = tBuff[2] * (1 - factor) + data->gz * factor;

		return SUCCESS;
	}
	return ERROR;
}

float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y = number;
	i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (threehalfs - (x2 * y * y)); // 1st iteration ����һ��ţ�ٵ�����
	return y;
}

/* ������Ԫ�� */
void GetAngle(mpu6050 *data, Fly_Angle *angle, float dat)
{
	volatile struct V
	{
		float x;
		float y;
		float z;
	} Gravity, Acc, Gyro, AccGravity;

	static struct V GyroIntegError = {0};
	static float KpDef = 0.8f;
	static float KiDef = 0.0003f;
	static Quaternion NumQ = {1, 0, 0, 0};
	float q0_t, q1_t, q2_t, q3_t;
	// float NormAcc;
	float NormQuat;
	float HalfTime = dat * 0.5f;

	// ��ȡ��Ч��ת�����е���������
	Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
	Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
	Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
	// ���ٶȹ�һ��
	NormQuat = Q_rsqrt(squa(Mpu6050_Dat.ax) + squa(Mpu6050_Dat.ay) + squa(Mpu6050_Dat.az));

	Acc.x = data->ax * NormQuat;
	Acc.y = data->ay * NormQuat;
	Acc.z = data->az * NormQuat;
	//������˵ó���ֵ
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	//�������ٶȻ��ֲ������ٶȵĲ���ֵ
	GyroIntegError.x += AccGravity.x * KiDef;
	GyroIntegError.y += AccGravity.y * KiDef;
	GyroIntegError.z += AccGravity.z * KiDef;
	//���ٶ��ںϼ��ٶȻ��ֲ���ֵ
	Gyro.x = data->gx * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x; //������
	Gyro.y = data->gy * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
	Gyro.z = data->gz * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;
	// һ�����������, ������Ԫ��

	q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
	q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
	q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// ��Ԫ����һ��
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;

	// ��Ԫ��תŷ����
	{
		/*��������ϵ�µ�Z��������*/
		float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;		 /*����(3,1)��*/
		float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;		 /*����(3,2)��*/
		float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2; /*����(3,3)��*/
#ifdef YAW_GYRO
		*(float *)pAngE = atan2f(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA; // yaw
#else
		float yaw_G = data->gz * Gyro_G;
		if ((yaw_G > 3.0f) || (yaw_G < -3.0f)) //����̫С������Ϊ�Ǹ��ţ�����ƫ������
		{
			angle->yaw += yaw_G * dat;
		}
#endif
		angle->pitch = asin(vecxZ) * RtA;

		angle->roll = atan2f(vecyZ, veczZ) * RtA; // PITCH

		Norm_az = data->ax * vecxZ + data->ay * vecyZ + data->az * veczZ; /*Z����ٶ�*/
	}
}

/* һά������ */
void kalman_1(ekf_filter *ekf, float input)
{
	ekf->Now_P = ekf->LastP + ekf->Q;
	ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
	ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
	ekf->LastP = (1 - ekf->Kg) * ekf->Now_P;
}

/* ��ȡZ����ٶ� */
float GetAz(void)
{
	return Norm_az;
}

/* ��װ��ȡyaw�� */
float Mpu_Read_Yaw()
{
	MPU6050_ReadAll(&Mpu6050_Dat);
	GetAngle(&Mpu6050_Dat, &Angle, Normal);
	return Angle.yaw;
}

/* ��װ��ȡroll�� */
float Mpu_Read_Roll()
{
	MPU6050_ReadAll(&Mpu6050_Dat);
	GetAngle(&Mpu6050_Dat, &Angle, Normal);
	return Angle.roll;
}

/* ��װ��ȡpitch�� */
float Mpu_Read_Pitch()
{
	MPU6050_ReadAll(&Mpu6050_Dat);
	GetAngle(&Mpu6050_Dat, &Angle, Normal);
	return Angle.pitch;
}

