/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define EXT_GPS_Pin GPIO_PIN_4
#define EXT_GPS_GPIO_Port GPIOA
#define EXT_GPS_EXTI_IRQn EXTI4_IRQn
#define C3_3V_Pin GPIO_PIN_5
#define C3_3V_GPIO_Port GPIOA
#define C4V_Pin GPIO_PIN_12
#define C4V_GPIO_Port GPIOB
#define C5V_Pin GPIO_PIN_8
#define C5V_GPIO_Port GPIOA
#define DTR_SIM_Pin GPIO_PIN_11
#define DTR_SIM_GPIO_Port GPIOA
#define RI_SIM_Pin GPIO_PIN_12
#define RI_SIM_GPIO_Port GPIOA
#define RI_SIM_EXTI_IRQn EXTI15_10_IRQn
#define SQW_DS3231_Pin GPIO_PIN_8
#define SQW_DS3231_GPIO_Port GPIOB
#define SQW_DS3231_EXTI_IRQn EXTI9_5_IRQn
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

#define SIM_Sleep HAL_GPIO_WritePin(DTR_SIM_GPIO_Port, DTR_SIM_Pin, GPIO_PIN_SET)
#define SIM_Wakeup HAL_GPIO_WritePin(DTR_SIM_GPIO_Port, DTR_SIM_Pin, GPIO_PIN_RESET)

#define C3V_Off HAL_GPIO_WritePin(C3_3V_GPIO_Port, C3_3V_Pin, GPIO_PIN_SET)
#define C3V_On HAL_GPIO_WritePin(C3_3V_GPIO_Port, C3_3V_Pin, GPIO_PIN_RESET)
#define C4V_Off HAL_GPIO_WritePin(C4V_GPIO_Port, C4V_Pin, GPIO_PIN_SET)
#define C4V_On HAL_GPIO_WritePin(C4V_GPIO_Port, C4V_Pin, GPIO_PIN_RESET)
#define C5V_Off HAL_GPIO_WritePin(C5V_GPIO_Port, C5V_Pin, GPIO_PIN_SET)
#define C5V_On HAL_GPIO_WritePin(C5V_GPIO_Port, C5V_Pin, GPIO_PIN_RESET)

#define LED1_On HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_Off HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED2_On HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_Off HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
