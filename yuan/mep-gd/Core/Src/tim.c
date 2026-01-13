/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "util.h"
#include "protocol.h"
#include "main.h"
#include <string.h>  // 用于memset函数
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;

extern uint8_t trigger_waveform_mode;
extern volatile uint8_t real_raw_data;
extern uint8_t raw_data[];
extern uint16_t GAP_Value;
extern uint16_t read_count;
extern uint16_t write_count;  
extern uint8_t write_complete_flag;
extern uint8_t read_complete_flag;
extern volatile uint8_t raw_index;
extern uint8_t real_data[];
extern uint8_t MEP_real_Send_Data(void);
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  // 时钟计算：HSE=8MHz, PLLM=5, PLLN=200, PLLP=2
  // SYSCLK = (8MHz/5)*200/2 = 160MHz
  // APB1 = 160MHz/4 = 40MHz
  // TIM2时钟 = 40MHz*2 = 80MHz (因为APB1预分频器是4，定时器时钟×2)
  // 0.1ms定时器：80MHz / 80 / 1000 = 1kHz = 1ms，改为80MHz / 80 / 100 = 10kHz = 0.1ms
  htim2.Init.Prescaler = 80-1;        // 预分频器：80-1 = 79
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;          // 周期值：100-1 = 99，实现0.1ms中断（10kHz）
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.RepetitionCounter = 0;  // 重复计数器值（仅TIM1和TIM8有效，TIM2设为0）
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  
  // 启动定时器中断模式
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        if(trigger_waveform_mode==1 && write_complete_flag == 1 && read_complete_flag != 1)
        {
            if(raw_index == 0 )
            {
                uint32_t src_base = read_count * 65;  // 使用uint32_t避免溢出
								if(src_base < TEM_LENGTH)
								{
										// 判断是否是第一个数据包（触发点在数据包中间的情况）
										if(read_count == mep.trigger.write_packet && mep.trigger.real_offset > 0)
										{                           
												memset(&raw_data[3], 0, mep.trigger.real_offset * 6);
												
												uint8_t valid_data_count = DATA_PACKAGE_SIZE - mep.trigger.real_offset;
												uint8_t src_data_offset = 3 + mep.trigger.real_offset * 6;  // 数据在real_data中的偏移
												uint8_t dst_data_offset = 3 + mep.trigger.real_offset * 6;  // 数据在raw_data中的偏移
												
												memcpy(&raw_data[dst_data_offset], &real_data[src_base + src_data_offset], valid_data_count * 6);
 
												read_count++;
												if((read_count * 65) >= TEM_LENGTH)
												{
														read_count = 0;  // 循环使用
												}
												raw_index = 1;  // 标记raw_data已满，准备发送
												
										}
										else
										{
												// 后续数据包正常处理：直接memcpy整个65字节
												memcpy(raw_data, &real_data[src_base], 65);                                                     
												// 数据包完整，可以发送
												read_count++;
												if((read_count * 65) >= TEM_LENGTH)
												{
														read_count = 0;  // 循环使用
												}
												if(read_count == mep.trigger.write_packet - 1)
												{
														read_complete_flag = 1;  // 标记读取完成
												}														
												raw_index = 1;  // 标记raw_data已满，准备发送
												GAP_Value--;    // 减少待发送的数据包计数
												
										}
								}
                
            }
            
            if(raw_index >= 1)
            {
                if(MEP_real_Send_Data() != HAL_BUSY)
                {
                    raw_index = 0; 
								
                }
            }
            
            // 退出触发模式的条件（写满后再按包发送）：
            if(write_complete_flag == 1 && read_complete_flag == 1)
            {
                read_complete_flag = 0;
                write_complete_flag = 0;
                trigger_waveform_mode = 0;  // 所有数据发送完成，退出触发模式，回到普通数据采集模式

            }
        }
    }
}
/* USER CODE END 1 */
