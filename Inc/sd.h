/*
	 HUYNH NGOC THUONG
	 DA NANG UNIVERSITY OF SCIENCE AND TECHNOLOGY
	 Last modified: 1/05/2018 - Version 2
	 **Describe: 
	 +SDcard by module SD and STM32F103C8T6
	 +In this project i had used: 
	 +Thirt party: FreeRTOS
	 +Thirt party: FATFS
	 +Comunicate protocol: SPI1 or you can change other SPI if you want
	 +Timer: TIM2 for turn on or turn off
	 +UART: UART1 for observation data from SPI
 *Facebook: ngocthuong0208 - huynhngocthuong0208@gmail.com - 01216911759
*/

#ifndef SD_H_
#define SD_H_
//--------------------------------------------------
#include "stm32f1xx_hal.h"
#include "fatfs.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
//--------------------------------------------------
#define LD_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //RED
#define LD_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //RED
//--------------------------------------------------
/* Card type flags (CardType) */
#define CT_MMC 0x01 /* MMC ver 3 */
#define CT_SD1 0x02 /* SD ver 1 */
#define CT_SD2 0x04 /* SD ver 2 */
#define CT_SDC (CT_SD1|CT_SD2) /* SD */
#define CT_BLOCK 0x08 /* Block addressing */
//--------------------------------------------------
typedef struct sd_info {
  volatile uint8_t type;//Loai the chung ta su dungs
} sd_info_ptr;
struct sd_data{
	uint8_t size;
	uint8_t rdata[512];
	char data[512];
  char wdata[300];
	char pm1p0[15];
	char pm2p5[15];
	char pm10[15];
	char ppmSO2[15];
	char ppmNO2[15];
	char ppmCO[15];
	char pAcquy[15];
	
	char date[5];
	char month[5];
	char year[5];
	char hour[5];
	char minute[5];
	char second[5];
	char filename[50];

};
//--------------------------------------------------
//FATFS SDFatFs; //File system object structure (FATFS)
//FATFS *fs;		 //File system object structure (FATFS)
extern uint8_t sect[512];
extern uint32_t bytesread;; //byte doc va ghi
//--------------------------------------------------
void SD_PowerOn(void);
uint8_t sd_ini(void);
void SPI_Release(void);
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba);
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba);
uint8_t SPI_wait_ready(void);
//--------------------------------------------------
FRESULT ReadLongFile(void);

void SD_Write_File(const char* filename, const char* buffer, uint8_t size);
uint8_t* SD_Read_File(const char* filename);
void SD_List_File(void);
unsigned long SD_Amount_Space(void);
void SD_Push_Data(uint8_t date, uint8_t month, uint8_t year, uint8_t hour, uint8_t minute, uint8_t second, char* latitude, char* s_n, char* longtitude, char* e_w, 
		uint16_t pm10, uint16_t pm1p0, uint16_t pm2p5, float ppmco, float ppmno2, float ppmso2, float acquy);
void SD_Filename(uint8_t date, uint8_t month, uint8_t year, uint8_t hour);
void SD_Frame2(char* buffer, uint16_t count);
void SD_Frame3(char* buffer, uint16_t count);
void SD_Frame33(char* buffer, float count);
#endif /* SD_H_ */
