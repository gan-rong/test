/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stdint.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ADC_REST_Pin GPIO_PIN_13
#define ADC_REST_GPIO_Port GPIOC
#define WKUP_Pin GPIO_PIN_0
#define WKUP_GPIO_Port GPIOA
#define WKUP_EXTI_IRQn EXTI0_IRQn
#define ADC_VOLTAGE_Pin GPIO_PIN_1
#define ADC_VOLTAGE_GPIO_Port GPIOA
#define CH0_G2_Pin GPIO_PIN_3
#define CH0_G2_GPIO_Port GPIOA
#define CH0_G1_Pin GPIO_PIN_4
#define CH0_G1_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define CHG_MCU_Pin GPIO_PIN_8
#define CHG_MCU_GPIO_Port GPIOE
#define CHG_PG_Pin GPIO_PIN_9
#define CHG_PG_GPIO_Port GPIOE
#define TRIGGER_OUT_Pin GPIO_PIN_14
#define TRIGGER_OUT_GPIO_Port GPIOB
#define TRIGGER_IN_Pin GPIO_PIN_13
#define TRIGGER_IN_GPIO_Port GPIOB
#define TRIGGER_IN_EXTI_IRQn EXTI15_10_IRQn
#define FULL_CHANRGE_Pin GPIO_PIN_14
#define FULL_CHANRGE_GPIO_Port GPIOD
#define POWER_EN_Pin GPIO_PIN_5
#define POWER_EN_GPIO_Port GPIOC
#define nReady_Pin GPIO_PIN_7
#define nReady_GPIO_Port GPIOC
#define WIFI_RESET_Pin GPIO_PIN_0
#define WIFI_RESET_GPIO_Port GPIOD
#define WIFI_RELOAD_Pin GPIO_PIN_1
#define WIFI_RELOAD_GPIO_Port GPIOD
#define CH1_G1_Pin GPIO_PIN_5
#define CH1_G1_GPIO_Port GPIOB
#define CH1_G2_Pin GPIO_PIN_6
#define CH1_G2_GPIO_Port GPIOB
#define ADC_M0_Pin GPIO_PIN_8
#define ADC_M0_GPIO_Port GPIOB
#define ADC_M1_Pin GPIO_PIN_9
#define ADC_M1_GPIO_Port GPIOB
#define ADC_LJ_Pin GPIO_PIN_0
#define ADC_LJ_GPIO_Port GPIOE
#define ADC_HPF_Pin GPIO_PIN_1
#define ADC_HPF_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define countof(a)    (sizeof(a)/sizeof(*(a)))
#define RXSIZE 64
#define TXSIZE 64
// sai
//#define SAI_RX_SIZE 2*1 // ch * datapoint
#define I2S_RX_SIZE 2*1 // ch * datapoint
// data package size
#define DATA_PACKAGE_SIZE  10

// flash congih length, serial number(12) + Ch0 gain( 4 * 3) + Ch1 gain( 4*3)
#define CONFIG_LENGTH 12 + 3 * 4 + 3 * 4

// use debug message, on=1, off=0
#define DEBUG(format,...)	do{ if(0) {printf("%10d LINE: %4d: "format"\r\n", HAL_GetTick(), __LINE__, ##__VA_ARGS__);}} while(0U)


//#define SLEEP_TIME 60 * 60 * 1000

uint8_t Set_Sample(uint32_t value);
void Set_Gain(uint32_t value);
void Soft_Reset(void);
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
