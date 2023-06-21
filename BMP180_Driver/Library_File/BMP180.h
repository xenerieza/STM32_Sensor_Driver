/*
 * BMP180.h
 *
 *  Created on: May 21, 2023
 *      Author: xenerieza
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

/*	Values	*/
/* UP = Uncompansated Pressure Value --> 16 to 19 bit --> uint16_t UnCompansatedPressure.
 * UT = Uncompansated Temperature Value --> 16 bit --> uint16_t UnCompansatedTemperature.
 *
 * Measurement Flow BMP180
 *
 * Start
 * Start Temperature Measurement. Wait 4.5ms
 * Read UnCompansated Temperature
 * Start Pressure Measurement.
 * Wait ( It depends on mode )
 * Read UnCompansated Pressure
 * Calculate Pressure and Temperature in pyhsical units
 */

/* Calibration Coefficients Values	*/
/*
 * Parameters	MSB		LSB
 * 	AC1			0xAA	0xAB
 * 	AC2			0xAC	0xAD
 * 	AC3			0xAE	0xAF
 * 	AC4			0xB0	0xB1
 * 	AC5			0xB2	0xB3
 * 	AC6			0xB4	0xB5
 * 	B1			0xB6	0xB7
 * 	B2			0xB8	0xB9
 * 	MB			0xBA	0xBB
 * 	MC			0xBC	0xBD
 * 	MD			0xBE	0xBF
 */

/* The mode (Ultra Low Power, Standard, High, Ultra High Resolution) can be selected by the variable
   oversampling_setting (0,1,2,3) in the C code.*/
/*
 * Calculation of true temperature and pressure in steps of 1Pa (=0.01hPa = 0.01mbar) and
 * Temperature in steps of 0.1°C.
 */


/*	Device and Register Address		*/
#define BMP180_DEVICE_READ_ADDRESS	(0xEF)
#define	BMP180_DEVICE_WRITE_ADDRESS	(0xEE)

/*	Calibration Start Address  */
#define BMP180_Calibration_Start_Address	(0xAA)

/*	Control and Other Register Adress	*/
#define out_xlsb	(0xF8)
#define out_lsb		(0xF7)
#define out_msb		(0xF6)
#define ctrl_meas	(0xF4)
#define Soft_Reset	(0xE0)

/*	Temperature & Pressure Structures	*/
typedef struct BMP180_Temperature_Parameters_Structure
{
	long UnCompansated_Temperature;
	float Temperature;
	double Kelvin;
	double Celsius;
	double Fahrenheit;

}BMP180_Temperature_Pressure_Structure;

typedef struct BMP180_Pressure_Parameters_Structure
{
	long UnCompansated_Pressure;
	long Pressure;
	float Altitude;
	float Pa;
	float hPa;
	float hPa_in_sea_level;
}BMP180_Pressure_Parameters_Structure;

/*	------------------------------------------------------------  */
/**
  * @brief Oversampling Setting Tables
  * @note Table Parameters:
  * Mode				|	Parameter		|	Conversion Time Pressure max.(ms)		|
  * Ultra_Low_Power			|		0		|			4.5				|
  * Standart				|		1		|			7.5				|
  * High_Resolution			|		2		|			13.5				|
  * Ultra_High_Resolution  	        |		3		|			25.5				|
  */

typedef enum BMP180_OverSampling_Structures
{
	Ultra_Low_Power,		//0
	Standard,				//1
	High_Resolution,		//2
	Ultra_High_Resolution	//3
}BMP180_OverSampling_Structures;


/*	Main Functions	*/
uint8_t BMP180_Initialize(BMP180_OverSampling_Structures OSS);
uint8_t BMP180_Read_All(BMP180_OverSampling_Structures OSS);
short BMP180_Set_Users_Settings(BMP180_OverSampling_Structures OSS);
void BMP180_Get_Calibration_Values(void);
short BMP180_Get_Uncompansated_Temperature_Values(void);
short BMP180_Calculate_True_Temperature(void);
short BMP180_Get_Uncompansated_Pressure_Values(BMP180_OverSampling_Structures OSS);
short BMP180_Calculate_True_Pressure(BMP180_OverSampling_Structures OSS);

float BMP180_Temperature();
float BMP180_Pressure();

float BMP180_Get_Temperature_In_Celsius();
float BMP180_Get_Temperature_In_Kelvin();
float BMP180_Get_Temperature_In_Fahrenheit();

float BMP180_Get_Pressure_In_Pa();
float BMP180_Get_Pressure_In_hPa();
float BMP180_C_Get_Altitude(BMP180_OverSampling_Structures OSS);
float BMP180_Set_Pressure_In_Sea_Level(float hPa_in_sea_level);

#endif /* INC_BMP180_H_ */
