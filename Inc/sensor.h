/*
	 HUYNH NGOC THUONG
	 DA NANG UNIVERSITY OF SCIENCE AND TECHNOLOGY
	 Last modified: 16/08/2018 - Version 2
	 **Describe: 
 *Facebook: ngocthuong0208 - huynhngocthuong0208@gmail.com - 01216911759
*/
#ifndef SENSOR_H_
#define SENSOR_H_
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#define Timeout 100
typedef __packed struct {
	uint16_t ADC_Raw[4];
	
	uint16_t ADC_Process[4];
	
	float ppmCO;
	float ppmNO2;
	float ppmSO2;
	
	float vCO;
	float vNO2;
	float vSO2;
	
	float vAcquy;
	
	uint8_t pAcquy;
	
	uint8_t cADC;
	bool fAdc;
	
}Sensor_t;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void Sensor_Convertdata(Sensor_t* handle);
#endif /* SENSOR_H_ */
