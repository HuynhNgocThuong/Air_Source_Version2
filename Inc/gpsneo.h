/*
	 HUYNH NGOC THUONG
	 DA NANG UNIVERSITY OF SCIENCE AND TECHNOLOGY
	 Last modified: 1/05/2018 - Version 2
	 **Describe: 
	 +SDcard by module SD and STM32F103C8T6
	 +In this project i had used: 
	 +Thirt party: FreeRTOS
	 +Thirt party: FATFS
	 +Comunicate protocol: UART1 - Rx for receive data from GPS Neo-Ulbox
												 UART2 - Tx for transfer data from MCU to PC for observation
 *Facebook: ngocthuong0208 - huynhngocthuong0208@gmail.com - 01216911759
*/
#ifndef _GPSNEO_H_
#define _GPSNEO_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//------------------------------------Define-----------------------------------//
//-----------------------------------------------------------------------------//
#define GPS_BUFFER_SIZE 200
//------------------------------------Struct-----------------------------------//
//-----------------------------------------------------------------------------//
struct nmeaMessage_t{
	//Raw Data GPS
	char GPS_RX_Buffer[GPS_BUFFER_SIZE];
	uint8_t GPS_Counter;
	uint8_t GPS_Counter_Tmp;
	uint8_t GPS_Flag;
};
typedef struct {
		//Data GPS
	bool Status;
	char Latitude[11];
	char S_N[2];
	char Longtitude[12];
	char E_W[2];
	char Speed[20];
	char Location[50];
	float Velocity;
}dataGps_t;
/*________________________________GPS_Prototype_________________________________*/
 /*____________________________________________________________________________*/
bool GPS_Data(dataGps_t* handle);
void GPS_ClearData(void);
void GPS_ClearRxBuffer(void);
int  GPS_SearchChar(unsigned char Char, char *Str, unsigned char Time, int Len);
void GPS_DeleteChar(char s[], int pos);
bool GPS_RawData(void);
void GPS_Knot2Kmh(char * knot, float * km_h);
#endif /* _GPSNEO_H_ */
