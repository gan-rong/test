#ifndef __WIFI_H
#define __WIFI_H

#include "stm32f4xx_hal.h"

enum WIFI_STATUS {WIFI_STATUS_NORMAL, WIFI_STATUS_UART, WIFI_STATUS_COMPLETE};

struct WIFI_MODEL
{
	enum WIFI_STATUS status;	// 0=normal, 1=send +++, 2=receive a, 3=receive +OK, 4=over
	char result[64];
	uint16_t max_size;
	uint16_t index;
};

extern struct WIFI_MODEL WIFI_Model;
//extern enum WIFI_STATUS WIFI_Status;

void WIFI_Init(void);

void WIFI_Reset(void);

void WIFI_SetUart(uint8_t cmd_mode);

void WIFI_Init_Cmd(void);

void WIFI_Exit_Cmd(void);

void WIFI_Parse(void);
#endif
