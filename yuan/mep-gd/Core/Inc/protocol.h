#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f4xx_hal.h"

#define CH_LEN 2

struct FLAGS
{
	uint8_t sample;			// 设置采样率
	uint8_t gain;				// 设置增益
	uint8_t upload; 		// 上传
	uint8_t config; 		// 写flash配置
	uint8_t upgrade; 		// 升级固件
	uint8_t wifi_reset;	// WIFI恢复出厂设置标志位
	uint8_t wifi_cmd;		// WIFI处理AT指令标志位
	uint8_t ready_standby;	// 准备待机
};
struct MESSAGE
{
	float voltage;													// 电压
	uint8_t charge;													// 充电
	uint8_t wifi_ready; 										// wifi就绪
	uint8_t fully_charged; 									// 充满电
	uint8_t standby; 												// 待机 0-run, 1-start, 2-close
	uint32_t version;												// 协议版本
	uint8_t config[CONFIG_LENGTH + 1];			// 序列号+增益校正
	int32_t interval; 											// LED 间歇
	uint8_t wifi_cmd[56];			   						// AT指令
	uint32_t upload_time;										// 最后一次上传messesg 时间戳
};

typedef struct {
	uint8_t triggered;	     // 正在触发
	uint8_t flag;				     // 收到触发标志
	int32_t count;			     // 触发计数
	uint8_t offset;			     // 触发偏移
	uint8_t real_offset;		 // 真触发偏移
	uint16_t write_packet;	 // 触发时的数据包编号（write_count的值）
} Trigger;

struct MEP
{
	uint8_t channel[CH_LEN];
	uint8_t live;						// 实时模式
	uint32_t length;				// 波形长度
	uint32_t sample;				// 采样率
	uint32_t gain;					// 增益
	uint8_t start;					// 开始
	uint32_t sleep;
	Trigger trigger;
	struct FLAGS flags;
	struct MESSAGE msg;
};


extern uint8_t raw_data[];

extern volatile uint8_t raw_index;

extern volatile uint8_t real_raw_data; // real_raw_data索引（保留用于触发模式）
extern volatile uint8_t real_index;    // real_index索引（用于高采样率模式）
extern uint16_t write_count; 

extern struct MEP mep;

extern enum LED_STATUS led_status;

void init_model(void);

void parse(uint8_t * bytes, uint16_t len);

void set_parameter(uint8_t* bytes);

void send_data(uint8_t ch, uint8_t trigger);

uint8_t send_raw_data(void);

uint8_t send_real_raw_data(void);

uint8_t upload_message(void);

uint8_t MEP_Send_Data(void);

uint8_t MEP_real_Send_Data(void);

void debug(char *msg);

void send_at_result(char *msg);

#endif
