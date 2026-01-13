#ifndef __UTIL_H
#define __UTIL_H
#include "stdint.h"

#define TEM_LENGTH 130000

union uFloat
{
	float value;
	char bytes[sizeof(float)];
};
union uInt
{
	int value;
	char bytes[sizeof(int)];
};

uint8_t Check(uint8_t * bytes, int startIndex, int endIndex);
int bytes2int(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
float bytes2float(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
void int2bytes(uint8_t *bytes, int value);
void float2bytes(uint8_t *bytes, float value);
float average_voltage(uint32_t value);
void ConvertDataToBytes(const uint16_t *Input_BUFF, uint8_t *Output_BUFF);
void Process_Send_Data(void);
#endif
