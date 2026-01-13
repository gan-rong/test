#include "util.h"
#include "protocol.h"
#include "main.h"
#include <string.h>  
// TEM_LENGTH is now defined in util.h
extern uint16_t I2S_Rx_Buffer[];
extern volatile uint8_t real_raw_data;  // 定义在protocol.c中
extern volatile uint8_t raw_index; // raw_data索引
extern struct MEP mep;  // 定义在protocol.c中
uint8_t real_count;
uint8_t real_data[TEM_LENGTH];  // 必须是uint8_t，因为代码按字节访问  
volatile uint8_t real_index=0; 
uint16_t write_count=0;  // 当前写入的数据包编号
uint16_t read_count=0;   // 当前读取的数据包编号
uint16_t GAP_Value;
uint32_t Data_Count;
uint8_t MEP_Last_Offset=0;
uint8_t write_complete_flag=0;  // 写入完成标志：当real_data_send填满TEM_LENGTH时置1
uint8_t read_complete_flag=0;   // 读取完成标志：当(read_count * 65) >= TEM_LENGTH时置1
uint8_t trigger_waveform_mode = 0;  // 触发波形模式标志：1-触发波形采集中，0-正常采集模式
uint8_t Temp_Bytes[6];
uint16_t trigger_start_packet = 0;  // 触发模式开始写入的包号

uint8_t Check(uint8_t *bytes, int startIndex, int endIndex)
{
    int result = 0;
    uint8_t i = 0;
    for (i = startIndex; i <= endIndex; i++)
    {
        result += bytes[i];
    }
    result = (result >> 16) + (result & 0xffff);
    return result & 0xff;
}

int bytes2int(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    union uInt temp = {.bytes={a, b, c, d}};
    return temp.value;
}

float bytes2float(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    union uFloat temp = {.bytes={a, b, c, d}};
    return temp.value;
}

void int2bytes(uint8_t *bytes, int value)
{
    union uInt temp = {.value=value};
    bytes[0] = temp.bytes[0];
    bytes[1] = temp.bytes[1];
    bytes[2] = temp.bytes[2];
    bytes[3] = temp.bytes[3];
}

void float2bytes(uint8_t *bytes, float value)
{
    union uFloat temp = {.value=value};
    bytes[0] = temp.bytes[0];
    bytes[1] = temp.bytes[1];
    bytes[2] = temp.bytes[2];
    bytes[3] = temp.bytes[3];
}

float average_voltage(uint32_t value)
{
    static uint8_t v_index = 0;
    static float v_array[10] = {0};
    float average = 0;
    v_array[v_index++] = value * 0.001611328125; // 4096.0 * 3.3
    v_index = v_index % 10;
    for(uint8_t i = 0; i < 10; i++)
    {
        average += v_array[i];
    }
    return average / 10.0f;
}
//int a=0,b=0;
void Data_Processing()
{
//		if(trigger_waveform_mode == 1)
//		{
//			return;
//		}
		Data_Count++;
		
		if(Data_Count<=2000)
		{
				ConvertDataToBytes(I2S_Rx_Buffer, Temp_Bytes);
				
				// 计算当前数据包在real_data中的基址
				uint32_t base_addr = write_count * 65;  
				
				real_data[base_addr + 3 + real_index * 6 + 0] = Temp_Bytes[0];
				real_data[base_addr + 3 + real_index * 6 + 1] = Temp_Bytes[1];
				real_data[base_addr + 3 + real_index * 6 + 2] = Temp_Bytes[2];
				real_data[base_addr + 3 + real_index * 6 + 3] = Temp_Bytes[3];
				real_data[base_addr + 3 + real_index * 6 + 4] = Temp_Bytes[4];
				real_data[base_addr + 3 + real_index * 6 + 5] = Temp_Bytes[5];
				
				real_index++;
				
				if(real_index >= DATA_PACKAGE_SIZE)
				{
						write_count++;
						real_index = 0;
						//a++;
						if((write_count * 65) >= TEM_LENGTH)
						{
							write_count = 0;  
						}        
						GAP_Value++;
				}
		}
		
		if(raw_index == 0 && GAP_Value > 0)
		{
			// 计算要读取的数据包基址（从最早完成的包开始读取）
			uint32_t src_base = read_count * 65;  // 使用uint32_t避免溢出

			memcpy(raw_data, &real_data[src_base], 65);
			read_count++;
			if((read_count * 65) >= TEM_LENGTH)
			{
				read_count = 0; 
			}
			
			raw_index = 1;  // 标记raw_data已满（1表示有完整数据包）
			GAP_Value--;
		}
		
		if(raw_index >= 1)
		{
			if(MEP_Send_Data() != HAL_BUSY)
			{
				//b++;
				raw_index = 0; 
			}
		}
}
//int c=0,d=0;
void Data_Read()
{
			ConvertDataToBytes(I2S_Rx_Buffer, Temp_Bytes);

			// 计算当前数据包在real_data中的基址
			uint32_t base_addr = write_count * 65;  // 使用uint32_t避免溢出
			
			// 写入6字节有效数据到位置3-62（real_index: 0-9，对应数据位置3-62）
			real_data[base_addr + 3 + real_index * 6 + 0] = Temp_Bytes[0];
			real_data[base_addr + 3 + real_index * 6 + 1] = Temp_Bytes[1];
			real_data[base_addr + 3 + real_index * 6 + 2] = Temp_Bytes[2];
			real_data[base_addr + 3 + real_index * 6 + 3] = Temp_Bytes[3];
			real_data[base_addr + 3 + real_index * 6 + 4] = Temp_Bytes[4];
			real_data[base_addr + 3 + real_index * 6 + 5] = Temp_Bytes[5];

			real_index++;

			if(real_index >= DATA_PACKAGE_SIZE)
			{
				//c++;
					write_count++;
					real_index = 0;
					
					if((write_count * 65) >= TEM_LENGTH)
					{
							write_count = 0;  
					}        
					GAP_Value++; 
			}
			if(raw_index == 0 && GAP_Value > 0)
			{
					// 计算要读取的数据包基址（从最早完成的包开始读取）
					uint32_t src_base = (uint32_t)read_count * 65;  // 使用uint32_t避免溢出

					memcpy(raw_data, &real_data[src_base], 65);
					read_count++;
					if((read_count * 65) >= TEM_LENGTH)
					{
							read_count = 0; 
					}
					
					raw_index = 1;  // 标记raw_data已满（1表示有完整数据包）
					GAP_Value--;
        }
        
        if(raw_index >= 1)
        {
            if(MEP_Send_Data() != HAL_BUSY)
            {
                raw_index = 0; 
							//  d++;
            }
        }
}
void Data_Real_Processing()
{

	if(! write_complete_flag)
	{
		uint8_t Temp_Bytes[6];
		ConvertDataToBytes(I2S_Rx_Buffer, Temp_Bytes);
		
		uint32_t base_addr = write_count * 65;  // 使用uint32_t避免溢出

		real_data[base_addr + 3 + real_raw_data * 6 + 0] = Temp_Bytes[0];
		real_data[base_addr + 3 + real_raw_data * 6 + 1] = Temp_Bytes[1];
		real_data[base_addr + 3 + real_raw_data * 6 + 2] = Temp_Bytes[2];
		real_data[base_addr + 3 + real_raw_data * 6 + 3] = Temp_Bytes[3];
		real_data[base_addr + 3 + real_raw_data * 6 + 4] = Temp_Bytes[4];
		real_data[base_addr + 3 + real_raw_data * 6 + 5] = Temp_Bytes[5];
		
		real_raw_data++;
		
		if(real_raw_data >= DATA_PACKAGE_SIZE)
		{
				write_count++;
				real_raw_data = 0;
				
				if((write_count * 65) >= TEM_LENGTH)
				{
						write_count = 0;  
				} 
				if(write_count == mep.trigger.write_packet - 1)
				{
					write_complete_flag = 1;  // 标记写入完成，停止转换触发数据
				}
				// 写满之前不增加GAP_Value，改为写满后统一设置
		}
	}
	        
	
}

void ConvertDataToBytes(const uint16_t *Input_BUFF, uint8_t *Output_BUFF)
{
    Output_BUFF[0] = (Input_BUFF[1] >> 8) & 0x00ff;
    Output_BUFF[1] = Input_BUFF[0] & 0x00ff;
    Output_BUFF[2] = (Input_BUFF[0] >> 8) & 0x00ff;
    Output_BUFF[3] = (Input_BUFF[3] >> 8) & 0x00ff;
    Output_BUFF[4] = Input_BUFF[2] & 0x00ff;
    Output_BUFF[5] = (Input_BUFF[2] >> 8) & 0x00ff;
}

// Process I2S received data
void Process_Send_Data(void)
{
    static uint32_t last_sample = 0;
    
    if((mep.sample == 120000 || mep.sample == 50000) && (last_sample != 120000 && last_sample != 50000))
    {
       // real_index = 0;
        if(!mep.trigger.triggered)
        {
            real_raw_data = 0;
            MEP_Last_Offset = 0;
        }
    }
    last_sample = mep.sample;

    if(mep.sample == 120000||mep.sample == 50000)
    {        
        if((mep.trigger.real_offset != MEP_Last_Offset) && mep.trigger.triggered)
        {
            MEP_Last_Offset = mep.trigger.real_offset;
						trigger_waveform_mode = 1;  
						write_complete_flag = 0;
						read_complete_flag = 0;
						raw_index = 0 ;
						real_raw_data = mep.trigger.real_offset;
						read_count = mep.trigger.write_packet; 
        }
        if(trigger_waveform_mode == 1)
        {
            Data_Real_Processing();
            return;  // 触发波形模式下，直接返回，不执行普通采样逻辑
        }
        Data_Processing();
				if((mep.sample == 120000 && Data_Count > 2000) || (mep.sample == 50000 && Data_Count > 2000))
				{
					Data_Read();
					// 重置数据计数
					if((mep.sample == 120000 && Data_Count >= 12000) || (mep.sample == 50000 && Data_Count >= 5000))
				//	if((mep.sample == 120000 && Data_Count <= 12000) || (mep.sample == 50000 && Data_Count <= 5000))
					{
						Data_Count = 0;
						GAP_Value = 0;
					}
				}
    }
    else
    {        
        uint8_t index = raw_index * 6;
        ConvertDataToBytes(I2S_Rx_Buffer, &raw_data[index + 3]);
        
        raw_index++;
        
        if(raw_index >= DATA_PACKAGE_SIZE)
        {
            raw_index = 0;
            MEP_Send_Data();
        }
    }
}

