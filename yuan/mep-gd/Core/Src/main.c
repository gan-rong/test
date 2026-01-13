
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "sai.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "wifi.h"
#include "protocol.h"
#include "util.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t UART1_Rx_Buffer[RXSIZE];
uint8_t UART1_Tx_Buffer[TXSIZE];
uint8_t UART2_Rx_Buffer[RXSIZE];
uint8_t UART2_Tx_Buffer[TXSIZE];

uint16_t I2S_Rx_Buffer[I2S_RX_SIZE*2];

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

uint8_t loaded = 0; 
int32_t sleep = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HandleWeakUp(uint8_t type);
void Init_ADC(void);
void Enter_Standby_Mode(void);
void Read_Status(void);
static void MX_I2S2_Init(void);
/* USER CODE END PFP */
extern volatile uint8_t real_raw_data;  // 声明外部变量（触发模式使用）
extern volatile uint8_t real_index;     // 声明外部变量（正常模式使用）
extern uint16_t write_count;  // 声明外部变量
extern uint8_t trigger_waveform_mode;  // 声明触发模式标志
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f) 
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    init_model(); // init data model
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    __HAL_DMA_DISABLE(&hdma_spi2_rx);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2S2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_SET);// Enabling power
    HAL_Delay(100);
    //HAL_GPIO_WritePin(CHG_EN_GPIO_Port, CHG_EN_Pin, GPIO_PIN_SET); // 浣胯兘鍏呯數
    
    HandleWeakUp(0);
    loaded = 1;
    HAL_ADC_Start_IT(&hadc1);    // start adc interrupt collection
    HAL_UART_Receive_DMA(&huart1, UART1_Rx_Buffer, RXSIZE); // start uart1
    HAL_UART_Receive_DMA(&huart2, UART2_Rx_Buffer, RXSIZE);        // start uart2
    
    
    Init_ADC();
    HAL_Delay(100);
    WIFI_Init();

    // set the gain and sample
    Set_Sample(mep.sample);
    Set_Gain(mep.gain);
    // Read the charge status and wifi status
    Read_Status(); 
    // read device config
    //WriteConfig(mep.msg.config, CONFIG_LENGTH);
    Read_Config(mep.msg.config, CONFIG_LENGTH);
  /* USER CODE END 2 */
    

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        // set the sample
        if(mep.flags.sample)
        {
            mep.flags.sample = 0;
            Set_Sample(mep.sample);
        }
        // set the gain
        if(mep.flags.gain)
        {
            mep.flags.gain = 0;
            Set_Gain(mep.gain);
        }
        if(mep.flags.upload || mep.msg.upload_time >= 10000)
        {
            if (upload_message() == HAL_OK) mep.flags.upload = 0;
        }
        // trigger outputs
        if(mep.trigger.flag && HAL_GPIO_ReadPin(TRIGGER_OUT_GPIO_Port, TRIGGER_OUT_Pin) == GPIO_PIN_SET)
        {
            sleep = mep.sleep;
            HAL_GPIO_WritePin(TRIGGER_OUT_GPIO_Port, TRIGGER_OUT_Pin, GPIO_PIN_RESET);
            HAL_Delay(2);
            HAL_GPIO_WritePin(TRIGGER_OUT_GPIO_Port, TRIGGER_OUT_Pin, GPIO_PIN_SET);
            mep.trigger.flag = 0;
        }
        if (mep.flags.wifi_reset && mep.msg.wifi_ready == 1)
        {
            WIFI_Reset();
            debug("WIFI_RESTED");
            mep.flags.wifi_reset = 0;
        }
        if (mep.flags.wifi_cmd == 1 && mep.msg.wifi_ready == 1)
        {
            if (WIFI_Model.status == WIFI_STATUS_NORMAL)
            {
                WIFI_SetUart(1);    // 鍒囨崲鎴怉T鎸囦护
            }
            else if (WIFI_Model.status == WIFI_STATUS_UART)
            {
                WIFI_Init_Cmd();    // 鍙戦?丄T璧峰?嬬??
            }
            else if(WIFI_Model.status == WIFI_STATUS_COMPLETE)
            {
                WIFI_Exit_Cmd();    // 閫?鍑篈T鎸囦护妯″紡
                mep.flags.wifi_cmd = 0;
            }
        }
        if(sleep < 0)
        {
            HandleWeakUp(1);
        }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Macro to configure SAI1BlockA clock source selection 
    */
  __HAL_RCC_SAI_BLOCKACLKSOURCE_CONFIG(I2S_CLOCK_PLL);

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
////  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
////  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
////  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
////  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
////  RCC_OscInitStruct.PLL.PLLM = 4;
////  RCC_OscInitStruct.PLL.PLLN = 180;
////  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
////  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 256;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_10K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
//    hi2s2.Instance.

  /* USER CODE BEGIN I2S2_Init 2 */
//    __HAL_I2S_ENABLE(&hi2s2);
//    __HAL_DMA_DISABLE(&hdma_spi2_rx);
//    HAL_I2S_Receive_DMA(&hi2s2, I2S_Rx_Buffer, I2S_RX_SIZE);
//    __HAL_DMA_ENABLE(&hdma_spi2_rx);
//    __HAL_DMA_ENABLE_IT(&hdma_spi2_rx,DMA_IT_TC|DMA_IT_DME);
  /* USER CODE END I2S2_Init 2 */

}
/* USER CODE BEGIN 4 */
void Init_ADC()
{

    
    HAL_GPIO_WritePin(ADC_REST_GPIO_Port, ADC_REST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ADC_LJ_GPIO_Port, ADC_LJ_Pin, GPIO_PIN_SET); // 1=i2c, 0=left-justfied
    HAL_GPIO_WritePin(ADC_HPF_GPIO_Port, ADC_HPF_Pin, GPIO_PIN_RESET);//1=disable, 0=enable. disabled high pass filter
    
}

// Coefficient of different sampling rate of the clock
uint32_t SAMPLE[][4]={
    { 5000,  64,  5,  0},
    {10000,  128, 5,  0},
    {15000,  192, 5,  0},
    {20000,  256, 5,  0},
    {50000,  256, 2,  0},
    {120000, 192, 2,  0},       //432  2~7
    {200000, 320, 2,  0}
};

uint8_t Set_Sample(uint32_t value)
{
    uint32_t sample = 20000;
    uint16_t i = 0, n = 256, r = 5;//, vq = 0;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
    HAL_I2S_DeInit(&hi2s2);
    for(i = 0; i < countof(SAMPLE); i++)
    {
        if(SAMPLE[i][0] == value)
        {
            sample =  SAMPLE[i][0];
            n = SAMPLE[i][1];
            r = SAMPLE[i][2];
    //        vq = SAMPLE[i][3];
            switch(sample)
        {
                case 5000 :
                    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_5K;
                    break;
                case 10000 :
                    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_10K;
                    break;
                case 15000 :
                    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_15K;
                    break;
                case 20000 :
                    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_20K;
                    break;
                case 50000 :
                    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_50K;
                    break;
                case 120000 :
                    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_120K;
                    break;
        }
            
            break;
        }
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = n;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = r;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
    
//    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
//  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
//  PeriphClkInitStruct.PLLI2S.PLLI2SR = 5;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
    
    
//    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
//    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI_PLLI2S;
//  PeriphClkInitStruct.PLLI2S.PLLI2SN = n;
//  PeriphClkInitStruct.PLLI2S.PLLI2SQ = q;
//  PeriphClkInitStruct.PLLI2SDivQ = vq;
//    PeriphClkInitStruct.PeriphClockSelection = I2S_CLOCK_PLL;
//  PeriphClkInitStruct.PLLSAI.PLLSAIN = n;
//  PeriphClkInitStruct.PLLSAI.PLLSAIQ = q;
//  PeriphClkInitStruct.PLLSAIDivQ = vq;
//    hi2s2.Init.AudioFreq = sample;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }



    HAL_Delay(50);
    if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
    
    HAL_I2S_Receive_DMA(&hi2s2, I2S_Rx_Buffer, I2S_RX_SIZE);    // start sai
    return HAL_OK;
}

// set the gain
void Set_Gain(uint32_t value)
{
    if(value == 100)
    {
        HAL_GPIO_WritePin(CH0_G1_GPIO_Port, CH0_G1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CH0_G2_GPIO_Port, CH0_G2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CH1_G1_GPIO_Port, CH1_G1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CH1_G2_GPIO_Port, CH1_G2_Pin, GPIO_PIN_RESET);
    }
    else if(value == 500)
    {
        HAL_GPIO_WritePin(CH0_G1_GPIO_Port, CH0_G1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CH0_G2_GPIO_Port, CH0_G2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CH1_G1_GPIO_Port, CH1_G1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CH1_G2_GPIO_Port, CH1_G2_Pin, GPIO_PIN_SET);
    }
    else if(value == 2500)
    {
        HAL_GPIO_WritePin(CH0_G1_GPIO_Port, CH0_G1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CH0_G2_GPIO_Port, CH0_G2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CH1_G1_GPIO_Port, CH1_G1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CH1_G2_GPIO_Port, CH1_G2_Pin, GPIO_PIN_RESET);
    }
}

void HandleWeakUp(uint8_t type)
{
    int32_t cnt = 0;
    if(type == 0)
    {
        /* 寰呮満妯″紡鍚?鍔? */ 
        if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
        {
            while(HAL_GPIO_ReadPin(WKUP_GPIO_Port, WKUP_Pin) == GPIO_PIN_SET)
            {
                HAL_Delay(100);
                if(++cnt >= 30)
                {
                    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);/* 娓呴櫎寰呮満鏍囧織 */
                    return;
                }
            }
        }
        else
        {
            DEBUG("Power On");
            return;
        }
    }
    mep.msg.standby = 1;
    for( uint8_t i = 0; i < 50; i++)
    {
        if(upload_message() == HAL_OK) break;
    }
    mep.msg.standby = 0;
    HAL_Delay(50);
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_EnterSTANDBYMode();
}

void Read_Status()
{
    mep.msg.charge = !HAL_GPIO_ReadPin(CHG_PG_GPIO_Port, CHG_PG_Pin);
    mep.msg.fully_charged =  HAL_GPIO_ReadPin(CHG_MCU_GPIO_Port, CHG_MCU_Pin); // 楂樼數骞冲厖婊?
    mep.msg.wifi_ready = !HAL_GPIO_ReadPin(nReady_GPIO_Port, nReady_Pin);
    if( mep.msg.charge && mep.flags.wifi_reset == 0 && mep.flags.wifi_cmd == 0)
    {
        mep.msg.interval = mep.msg.fully_charged ? 0 : 500;
        HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port, WIFI_RESET_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(WIFI_RESET_GPIO_Port, WIFI_RESET_Pin, GPIO_PIN_SET);
    }
}

void Enter_Standby_Mode()
{    
    HAL_PWR_EnableWakeUpPin(PWR_CSR_EWUP); 
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTANDBYMode();
}

// soft reset
void SoftReset()
{
    __set_FAULTMASK(1); // 鍏抽棴
  NVIC_SystemReset(); // 澶嶄綅
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
         parse(UART1_Rx_Buffer, RXSIZE);
    }
    else if(huart == &huart2)
    {
        parse(UART2_Rx_Buffer, RXSIZE);
    }
}
// SAI send callback
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hsai)
{
    UART_Send((uint8_t *)"sai tx\r\n", 8);
}


// SAI received callback
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hsai)
{
    Process_Send_Data();
}

// SAI error callback
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hsai)
{
    UART_Send((uint8_t *)"sai error\r\n", 11);
}

// 1 millisecond timer callback
void HAL_SYSTICK_Callback()
{
    uint32_t tick = HAL_GetTick();
    if(!loaded || sleep < 0) 
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        return;
    }
    if ((mep.sleep != 0 && sleep >= 0) || mep.flags.ready_standby == 1)
    {
//        if (sleep <= 1000 && sleep % 500 == 0)
//        {
//            mep.start = 0; // 蹇?浼戠湢鏃跺叧闂?浼犺緭,鐢ㄤ簬浼犺緭鏈?鍚庝竴鍖卪essage
//            mep.msg.standby = 1;
//            mep.flags.upload = 1;
//        }
        sleep -= 1;
    }
    if(mep.msg.interval > 0 && tick % mep.msg.interval == 0)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
    if(mep.msg.interval== 0)
    {
        mep.msg.interval = -1;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
    if(tick % 2000 == 0)
    {
        Read_Status();
    }
    if(mep.start == 0)
    {
        mep.msg.upload_time += 1;
    }

}

 // peripheral interrupt callback
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
     
     if(GPIO_Pin == TRIGGER_IN_Pin) // external trigger
     {
        //reset the sleep time
        sleep = mep.sleep;
        if(!mep.trigger.triggered)
        {         
            mep.trigger.offset = raw_index;
            mep.trigger.real_offset = real_index;     // 正常模式：使用real_index
            mep.trigger.triggered = 1;
            mep.trigger.count = mep.length + 4 * DATA_PACKAGE_SIZE;
            mep.trigger.flag = 1;
        //printf("set trigger cmd. trigger=%d, length=%d\r\n", mep.trigger, mep.XferCount);
        }
     }
     else if(GPIO_Pin == WKUP_Pin) // weak up
     {
         if(HAL_GPIO_ReadPin(WKUP_GPIO_Port, WKUP_Pin) == GPIO_PIN_SET) // rising edge
         {
                sleep = 3000;
                mep.flags.ready_standby = 1;
         }
         else if(sleep > 0) // falling edge
         {
                sleep = mep.sleep == 0;
              mep.flags.ready_standby = 0;
                mep.msg.standby = 0;
         }
     }

 }
// ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
    {
        mep.msg.voltage =  average_voltage(HAL_ADC_GetValue(&hadc1));
    }
}

 
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

