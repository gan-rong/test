#include "protocol.h"
#include "string.h"
#include <stdio.h>
#include "util.h"
#include "usart.h"
#include "wifi.h"


uint8_t raw_data[65]; // 用于接收SAI数据

volatile uint8_t raw_index = 0; // raw_data索引
volatile uint8_t real_raw_data = 0; // real_raw_data索引

struct MEP mep;

extern int32_t sleep;
extern uint8_t Write_Config(uint8_t *data, uint32_t len);
 

/**
  * @brief  初始化数据模型
  * @param  无
  * @retval 无
  */  
void init_model()
{
	mep.channel[0] = 0;
	mep.channel[1] = 0;
	mep.live = 0;
	mep.length = 0;
	mep.sample = 5000;
	mep.gain = 100;
	mep.sleep = 0;
	mep.trigger.triggered = 0;
	mep.trigger.flag = 0;
	// 初始化配置
	memcpy(mep.msg.config, "220708100000", 12);
	float2bytes(&mep.msg.config[12],1);
	float2bytes(&mep.msg.config[16],1);
	float2bytes(&mep.msg.config[20],1);
	float2bytes(&mep.msg.config[24],1);
	float2bytes(&mep.msg.config[28],1);
	float2bytes(&mep.msg.config[32],1);
	
	//***************************************************************
	mep.msg.version = 221; // 201->2.0.1; 221->2.2.1; 1234->12.3.4
	//***************************************************************
	
	mep.msg.charge = 0;
	mep.msg.interval = 0;
	mep.msg.fully_charged = 0;
	mep.msg.standby = 0;
	mep.msg.wifi_ready = 0;
	mep.msg.upload_time = 0;

	mep.flags.sample = 0;
	mep.flags.gain = 0;
	mep.flags.upload = 0;
	mep.flags.config = 0;
	mep.flags.upgrade = 0;
	mep.flags.wifi_reset = 0;
	mep.flags.wifi_cmd = 0;
	mep.flags.ready_standby = 0;
}

/**
  * @brief  串口数据解析部分
  * @param  bytes		接收到的数据
  * @retval 无
  */  
void parse(uint8_t * bytes, uint16_t len)
{
	if(bytes[0] != '<' || bytes[63] != '>' || Check(bytes, 1, 61) != bytes[62])
	{
		//crc error
		//printf("CRC error header=%x\ttrail=%x\tchecksum=%x\r\n", bytes[0], bytes[63], Check(bytes, 1, 61));
		return;
	}

	if(bytes[1] == 0x01) // set parameter
	{
		// debug("set parameter cmd");
		set_parameter(bytes);
	}
	else if(bytes[1] == 0x02) // upload message
	{
		if(!mep.msg.standby)	
		{
			mep.flags.upload = 1;
		}
	}
	else if(bytes[1] == 0x03) // cmd
	{
		if (bytes[2] == 0x00) //开始运行指令
		{
			mep.trigger.triggered = 0;
			mep.trigger.count = 0;
			mep.start = bytes[3];
			//printf("set start cmd. start=%d\r\n", mep.start);
		}
		else if (bytes[2] == 0x01) // 触发指令
		{
			sleep = mep.sleep;
			if(!mep.trigger.triggered)
			{
				mep.trigger.offset = raw_index;
				mep.trigger.real_offset = real_index;
				mep.trigger.write_packet = write_count;  // 记录触发时的数据包编号
				mep.trigger.triggered = 1;
				mep.trigger.count = mep.length + 4 * DATA_PACKAGE_SIZE; // 采样延时（1） + 触发延时（1） + 实际长度 + 滤波延时（2）
				mep.trigger.flag = 1;
			//printf("set trigger cmd. trigger=%d, length=%d\r\n", mep.trigger, mep.XferCount);
			}
		}	
	}
	else if(bytes[1] == 0x04) // 写flash配置
	{
		memcpy(mep.msg.config, &bytes[2], CONFIG_LENGTH);
		Write_Config(mep.msg.config, CONFIG_LENGTH);
		mep.flags.config = 1;
	}
	else if (bytes[1] == 0x05) // WIFI配置
	{
		if (bytes[2] == 0x00) // 恢复出厂设置
		{
			mep.flags.wifi_reset = 1;  // wifi恢复出厂设置标志位，reload拉低3秒, 恢复出厂设置后将该位置零
		}
		else if (bytes[2] == 0x01 && mep.flags.wifi_cmd == 0) // AT指令
		{	
			memset(mep.msg.wifi_cmd, '\x0', 56);
			memcpy(mep.msg.wifi_cmd, &bytes[3], 56);
			WIFI_Model.status = WIFI_STATUS_NORMAL;
			mep.flags.wifi_cmd = 1;
		}
	}
}

/**
  * @brief  设置参数
  * @param  bytes		接收到的数据
  * @retval 无
  */  
void set_parameter(uint8_t* bytes)
{
	mep.channel[0] = bytes[2];
	mep.channel[1] = bytes[3];
	mep.live = bytes[4];
	mep.length = bytes2int(bytes[5],bytes[6],bytes[7],bytes[8]);
	uint32_t temp = bytes[9] * 1000;
	if(mep.sample != temp)
	{
		mep.sample = temp;
		mep.flags.sample = 1;
	}
	temp = bytes[10] * 100;
	if(mep.gain != temp)
	{
		mep.gain = temp;
		mep.flags.gain = 1;
	}
	mep.sleep = bytes[11] * 60 * 1000; // 由分钟转换成毫秒
	sleep = mep.sleep;
	//printf("ch0=%d ch1=%d live=%d length=%d sample=%d gain=%d\r\n", mep.channel[0], mep.channel[1], mep.live, mep.length, mep.sample, mep.gain);
}

/**
	* @brief  上传设备信息
  * @param  
  * @retval HAL status
  */  
uint8_t upload_message()
{
	static uint8_t byte[65]={0};
	byte[0] = '<';
	byte[1] = 0x01;
	memcpy(&byte[2], mep.msg.config, CONFIG_LENGTH);
	int2bytes(&byte[38], mep.msg.version);
	float2bytes(&byte[42], mep.msg.voltage);
	byte[46] = mep.msg.charge;
	byte[47] = mep.msg.fully_charged;
	byte[48] = mep.msg.standby;
	byte[49] = mep.sleep / 60 / 1000;	// 由毫秒转分钟
	byte[63] = Check(byte,1,62);
	byte[64] = '>';
	mep.msg.upload_time = 0;
	return UART_Send(byte, 65);
}

/**
  * @brief  发送SAI原始数据
  * @param  
  * @retval HAL status
  */  
uint8_t send_raw_data()
{
	raw_data[0] = '<';
	raw_data[1] = 0x02;
	raw_data[2] = (mep.trigger.count > 0) << 4 | (mep.trigger.offset + 5); // AD延时5个点, 延时时间为 5/sample
//	for(i = 0; i < SAI_RX_SIZE; i++)
//	{
//		raw_data[i * 3 + 3] = data[i* 4];
//		raw_data[i * 3 + 4] = data[i* 4 + 1];
//		raw_data[i * 3 + 5] = data[i* 4 + 2];
//	}
	raw_data[63] = Check(raw_data,1,62);
	raw_data[64] = '>';
	return UART_Send(raw_data, 65);
}
uint8_t send_real_raw_data()
{
	raw_data[0] = '<';
	raw_data[1] = 0x02;
	raw_data[2] = (mep.trigger.count > 0) << 4 | (mep.trigger.real_offset + 5); // AD延时5个点, 延时时间为 5/sample
	raw_data[63] = Check(raw_data,1,62);
	raw_data[64] = '>';
	return UART_Send(raw_data, 65);
}


/**
  * @brief  发送调试信息
  * @param  msg	信息
  * @retval 无
  */  
void debug(char *msg)
{
	static uint8_t byte[65]={0};
	byte[0] = '<';
	byte[1] = 0x03;
	strcpy((char*)&byte[2], msg);
	byte[63] = Check(byte,1,62);
	byte[64] = '>';
	UART_Send(byte, 65);
}

/**
  * @brief	发送AT指令结果给PC
  * @param  AT结果，最大长度为56
  * @retval 无
  */
void send_at_result(char *msg)
{
	static uint8_t byte[65] = {0};
	memset(byte, 0, sizeof(byte));
	byte[0] = '<';
	byte[1] = 0x04;
	strcpy((char *)&byte[2], msg);
	byte[63] = Check(byte, 1, 62);
	byte[64] = '>';
	HAL_UART_Transmit_DMA(&huart1, byte, 65);
}


/**
  * @brief	串口发送部分
  * @param  data	需要发送的数据
  * @param  len		发送的长度
  * @retval	发送结果, 成功返回HAL_OK
  */  
uint8_t UART_Send(uint8_t *data, uint8_t len)
{
	UART_HandleTypeDef *uart_handle = mep.msg.charge ? &huart1 : &huart2;
	
	// 检查串口状态，如果忙则直接返回，避免阻塞
	if(HAL_UART_GetState(uart_handle) == HAL_BUSY)
	{
		mep.msg.interval = 100; // 增加发送间隔
		return HAL_BUSY;
	}
	
	// 使用DMA发送数据
	uint8_t flag = HAL_UART_Transmit_DMA(uart_handle, data, len);

	if(flag == HAL_BUSY)
	{
		mep.msg.interval = 100;
	}
	return flag;
}

/**
  * @brief	mep发送数据逻辑部分
  * @param  无
  * @retval 无
  */  
uint8_t MEP_Send_Data()
{
	if(mep.start)
	{
		// 此处mep.trigger >= 0中使用=0,是为了最后发送一包常规数据, 用于分割波形
		if((mep.live || (mep.trigger.triggered) ) && (mep.channel[0] || mep.channel[1]))
		{
			// 检查串口状态，如果忙则直接返回，避免阻塞
			UART_HandleTypeDef *uart_handle = mep.msg.charge ? &huart1 : &huart2;
			if(HAL_UART_GetState(uart_handle) != HAL_BUSY)
			{
				if(send_raw_data() == HAL_OK)
				{
					if(mep.trigger.count > -DATA_PACKAGE_SIZE)
					{
						mep.trigger.count -= DATA_PACKAGE_SIZE;
					}
					else{
						mep.trigger.triggered = 0;
					}
					return HAL_OK;
				}
			}
		}
	}
	return HAL_BUSY;
}
uint8_t MEP_real_Send_Data()
{
	if(mep.start)
	{
		// 此处mep.trigger >= 0中使用=0,是为了最后发送一包常规数据, 用于分割波形
		if((mep.live || (mep.trigger.triggered) ) && (mep.channel[0] || mep.channel[1]))
		{
			// 检查串口状态，如果忙则直接返回，避免阻塞
			UART_HandleTypeDef *uart_handle = mep.msg.charge ? &huart1 : &huart2;
			if(HAL_UART_GetState(uart_handle) != HAL_BUSY)
			{
				if(send_real_raw_data() == HAL_OK)
				{
					if(mep.trigger.count > -DATA_PACKAGE_SIZE)
					{
						mep.trigger.count -= DATA_PACKAGE_SIZE;
					}
					else{
						mep.trigger.triggered = 0;
					}
					return HAL_OK;
				}
			}
		}
	}
	return HAL_BUSY;
}
