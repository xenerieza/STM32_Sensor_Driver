/*
 * BMP180.c
 *
 *  Created on: May 21, 2023
 *      Author: xenerieza
 */

#include "BMP180.h"
#include "math.h"
/*	------------------------------------------------------------  */
/*	Calibration Values	*/
short AC1;
short AC2;
short AC3;
unsigned short AC4;
unsigned short AC5;
unsigned short AC6;
short B1;
short B2;
short MB;
short MC;
short MD;
/*	------------------------------------------------------------  */

/*	------------------------------------------------------------  */
/*	Calculate Parameters	*/
long X1;
long X2;
long X3;
long B3;
unsigned long B4;
long B5;
long B6;
unsigned long B7;
/*	------------------------------------------------------------  */







struct BMP180_Parameters
{
	BMP180_Pressure_Parameters_Structure Pressure_Params;
	BMP180_Temperature_Pressure_Structure Temperature_Params;
}BMP180_Parameters;


uint8_t BMP180_Initialize()
{
	//Check Device Control.
	if(HAL_I2C_IsDeviceReady(&hi2c1, BMP180_DEVICE_WRITE_ADDRESS, 1, 10000) != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	}

	BMP180_Get_Calibration_Values();

	return 0;
}

uint8_t BMP180_Read_All(BMP180_OverSampling_Structures OSS)
{
	BMP180_Get_Uncompansated_Temperature_Values();
	BMP180_Get_Uncompansated_Pressure_Values(OSS);
	BMP180_Calculate_True_Temperature();
	BMP180_Calculate_True_Pressure(OSS);
	BMP180_Set_Users_Settings(OSS);
}

short BMP180_Set_Users_Settings(BMP180_OverSampling_Structures OSS)
{
	uint8_t ctrl_meas_value = 0;

	switch(OSS)
	{
	case Ultra_Low_Power:
		ctrl_meas_value |= 0;
		break;
	case Standard:
		ctrl_meas_value |= 1;
		break;
	case High_Resolution:
		ctrl_meas_value |= 2;
		break;
	case Ultra_High_Resolution:
		ctrl_meas_value |= 3;
	default:
		ctrl_meas_value |= 3;
		break;
	}

	HAL_I2C_Mem_Write(&hi2c1, BMP180_DEVICE_WRITE_ADDRESS, ctrl_meas, 1, (uint8_t*)&ctrl_meas_value, 1, 100);
	return OSS;
}

void BMP180_Get_Calibration_Values(void)
{
	uint8_t calibration_Buffer[22] = {0};

	HAL_I2C_Mem_Write(&hi2c1, BMP180_DEVICE_WRITE_ADDRESS, Soft_Reset, 1, (uint8_t*)&calibration_Buffer[0], 1, 100);

	HAL_I2C_Mem_Read(&hi2c1, BMP180_DEVICE_READ_ADDRESS, BMP180_Calibration_Start_Address, 1, (uint8_t*)&calibration_Buffer, 22, 100);

	AC1 = (calibration_Buffer[0]  << 8) | (calibration_Buffer[1]);
	AC2 = (calibration_Buffer[2]  << 8) | (calibration_Buffer[3]);
	AC3 = (calibration_Buffer[4]  << 8) | (calibration_Buffer[5]);
	AC4 = (calibration_Buffer[6]  << 8) | (calibration_Buffer[7]);
	AC5 = (calibration_Buffer[8] << 8) | (calibration_Buffer[9]);
	AC6 = (calibration_Buffer[10] << 8) | (calibration_Buffer[11]);
	B1 =  (calibration_Buffer[12] << 8) | (calibration_Buffer[13]);
	B2 =  (calibration_Buffer[14] << 8) | (calibration_Buffer[15]);
	MB =  (calibration_Buffer[16] << 8) | (calibration_Buffer[17]);
	MC =  (calibration_Buffer[18] << 8) | (calibration_Buffer[19]);
	MD =  (calibration_Buffer[20] << 8) | (calibration_Buffer[21]);

}

short BMP180_Get_Uncompansated_Temperature_Values(void)
{
	uint8_t read_Data[2] = {0};
	uint8_t write_Data[1];
	write_Data[0] = 0x2E;
	HAL_I2C_Mem_Write(&hi2c1, BMP180_DEVICE_WRITE_ADDRESS, ctrl_meas, 1, write_Data, 1, 100);
	//Wait 5ms
	HAL_Delay(5);

	HAL_I2C_Mem_Read(&hi2c1, BMP180_DEVICE_READ_ADDRESS, out_msb, 1, read_Data, 2, 100000);
	BMP180_Parameters.Temperature_Params.UnCompansated_Temperature = ((read_Data[0] << 8) | read_Data[1]);
	return BMP180_Parameters.Temperature_Params.UnCompansated_Temperature;
}

short BMP180_Get_Uncompansated_Pressure_Values(BMP180_OverSampling_Structures OSS)
{
	uint8_t oss_time = 0;
	uint8_t read_Data[3] = {0};
	uint8_t write_Data[1];
	write_Data[0] = 0x34 | (OSS << 6);
	HAL_I2C_Mem_Write(&hi2c1, BMP180_DEVICE_WRITE_ADDRESS, ctrl_meas, 1, write_Data, 1, 100);

	switch(OSS)
	{
	case Ultra_Low_Power:
		oss_time = 5;
		break;
	case Standard:
		oss_time = 8;
		break;
	case High_Resolution:
		oss_time = 14;
		break;
	case Ultra_High_Resolution:
		oss_time = 26;
		break;
	default:
		oss_time = 26;
		break;
	}

	HAL_Delay(oss_time);

	HAL_I2C_Mem_Read(&hi2c1, BMP180_DEVICE_READ_ADDRESS, out_msb,1, read_Data, 3, 100);
	BMP180_Parameters.Pressure_Params.UnCompansated_Pressure = ((read_Data[0] << 16) + (read_Data[1] << 8) + (read_Data[2])) >> (8 - OSS);
	return BMP180_Parameters.Pressure_Params.UnCompansated_Pressure;

}

short BMP180_Calculate_True_Temperature(void)
{
	X1 = ((BMP180_Parameters.Temperature_Params.UnCompansated_Temperature - AC6) * AC5) / pow(2,15);
	X2 = (MC * pow(2,11)) / (X1 + MD);
	B5 = X1+X2;

	BMP180_Parameters.Temperature_Params.Temperature = (B5 + 8) / 16;
	return BMP180_Parameters.Temperature_Params.Temperature;
}

short BMP180_Calculate_True_Pressure(BMP180_OverSampling_Structures OSS)
{
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = AC2*B6/(pow(2,11));
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<OSS)+2)/4;
	X1 = AC3*B6/pow(2,13);
	X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
	B7 = ((unsigned long)BMP180_Parameters.Pressure_Params.UnCompansated_Pressure-B3)*(50000>>OSS);
	if (B7<0x80000000) BMP180_Parameters.Pressure_Params.Pressure = (B7*2)/B4;
	else BMP180_Parameters.Pressure_Params.Pressure = (B7/B4)*2;
	X1 = (BMP180_Parameters.Pressure_Params.Pressure/(pow(2,8)))*(BMP180_Parameters.Pressure_Params.Pressure/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*BMP180_Parameters.Pressure_Params.Pressure)/(pow(2,16));
	BMP180_Parameters.Pressure_Params.Pressure = BMP180_Parameters.Pressure_Params.Pressure + (X1+X2+3791)/(pow(2,4));

	return BMP180_Parameters.Pressure_Params.Pressure;



}


float BMP180_Temperature()
{
	return BMP180_Parameters.Temperature_Params.Temperature;
}
float BMP180_Pressure()
{
	return BMP180_Parameters.Pressure_Params.Pressure;
}


float BMP180_Get_Temperature_In_Celsius()
{
	BMP180_Parameters.Temperature_Params.Celsius = BMP180_Parameters.Temperature_Params.Temperature / 10.0;
	return BMP180_Parameters.Temperature_Params.Celsius;
}
float BMP180_Get_Temperature_In_Kelvin()
{
	BMP180_Parameters.Temperature_Params.Kelvin = BMP180_Parameters.Temperature_Params.Temperature + 273.15;
	return BMP180_Parameters.Temperature_Params.Kelvin;
}

float BMP180_Get_Temperature_In_Fahrenheit()
{
	BMP180_Parameters.Temperature_Params.Fahrenheit = ((BMP180_Parameters.Temperature_Params.Celsius * 1.8) + 32.0);
	return BMP180_Parameters.Temperature_Params.Fahrenheit;
}

float BMP180_Get_Pressure_In_Pa()
{
	BMP180_Parameters.Pressure_Params.Pa = BMP180_Parameters.Pressure_Params.Pressure;
	return BMP180_Parameters.Pressure_Params.Pa;
}

float BMP180_Get_Pressure_In_hPa()
{
	BMP180_Parameters.Pressure_Params.hPa = BMP180_Parameters.Pressure_Params.Pa * 0.01;
	return BMP180_Parameters.Pressure_Params.hPa;
}

float BMP180_C_Get_Altitude(BMP180_OverSampling_Structures OSS)
{
	BMP180_Calculate_True_Temperature();
	BMP180_Calculate_True_Pressure(OSS);
	return 44330*(1-(pow(((float)BMP180_Parameters.Pressure_Params.hPa / (float)BMP180_Parameters.Pressure_Params.hPa_in_sea_level), 0.19029495718)));

}

float BMP180_Set_Pressure_In_Sea_Level(float hPa_in_sea_level)
{
	return BMP180_Parameters.Pressure_Params.hPa_in_sea_level = hPa_in_sea_level;
}
