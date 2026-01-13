#include "wifi.h"
#include "usart.h"
#include "protocol.h"
#include <string.h>

extern uint8_t UART2_Rx_Buffer[RXSIZE];
struct WIFI_MODEL WIFI_Model;
/**
  * @brief	初始化WIFI相关数据
  */
void WIFI_Init()
{
	WIFI_Model.index = 0;
	WIFI_Model.status =  WIFI_STATUS_NORMAL;
	memset(WIFI_Model.result, 0, sizeof(WIFI_Model.result));
}

/**
* @brief	WIFI恢复出厂设置
  */
void WIFI_Reset()
{
	HAL_GPIO_WritePin(WIFI_RELOAD_GPIO_Port, WIFI_RELOAD_Pin, GPIO_PIN_RESET);
	HAL_Delay(4000);
	HAL_GPIO_WritePin(WIFI_RELOAD_GPIO_Port, WIFI_RELOAD_Pin, GPIO_PIN_SET);
}

/**
  * @brief	设置WIFI对应串口
  * @param  cmd_mode，WIFI工作模式，cmd_mode=1，115200，AT指令模式；cmd_mode=0，MEP通信模式
  * @retval 无
  */
void WIFI_SetUart(uint8_t cmd_mode)
{
	HAL_UART_AbortReceive(&huart2);
//	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	if(HAL_UART_DeInit(&huart2) != HAL_OK)
	{
		return;
	}
	huart2.Instance = USART2;
  huart2.Init.BaudRate = cmd_mode == 1 ? 115200 : 1500000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_UART_Init(&huart2) != HAL_OK)
	{
		return;
	}
	if(cmd_mode == 1)
	{
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
		WIFI_Model.status = WIFI_STATUS_UART;
	}
	
	if(HAL_UART_Receive_DMA(&huart2, UART2_Rx_Buffer, RXSIZE) != HAL_OK)
	{
		return;
	}
}
	

/**
  * @brief	初始化WIFI指令，发送起始字符
  */
void WIFI_Init_Cmd()
{
	WIFI_Model.index = 0;
	memset(UART2_Rx_Buffer, 0, sizeof(UART2_Rx_Buffer));
	memset(WIFI_Model.result, 0, sizeof(WIFI_Model.result));
	HAL_Delay(100);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"+++", 3);
}

/**
  * @brief	退出AT模式，并切换串口中断
  */
void WIFI_Exit_Cmd()
{
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"AT+ENTM\r", 8);
	memset(WIFI_Model.result, 0, sizeof(WIFI_Model.result));
	WIFI_Model.status = WIFI_STATUS_NORMAL;
	HAL_Delay(100);
	WIFI_SetUart(0);
}
	

/**
  * @brief	发送AT指令
  * @param  无
  * @retval 无
  */
void WIFI_Parse()
{
	if(strcmp((char *)WIFI_Model.result, (char *)"a") == 0)
	{
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"a", 1);
	}				
	else if(strcmp((char *)WIFI_Model.result, (char *)"+OK") == 0)
	{
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)mep.msg.wifi_cmd, strlen((char *)mep.msg.wifi_cmd));
	}
	else if(strlen((char*)WIFI_Model.result) > 2)
	{
		send_at_result((char*)WIFI_Model.result);
		WIFI_Model.status = WIFI_STATUS_COMPLETE;
	}
}
