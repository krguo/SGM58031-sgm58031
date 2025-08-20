/*
 * @Descripttion:
 * @Author: 过期的小朋友
 * @Date: 2025-08-11 20:31:24
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2025-08-18 16:39:50
 * @FilePath: \Leak_detection\Hardware\SGM58031\sgm58031.c
 * @版权所有:  Copyright(C) 2025 by 过期的小朋友, All Rights Reserved
 */

#include "sgm58031.h"
#include "gd32f10x_i2c.h"
#include <math.h>

// int ads1115_array[30];
// uint16_t ads1115_array_num = 0;
union SGM58301ConfigRegister SGM58301Config;
uint16_t timeout = 0;
/**
 * @brief
 *
 * @param addr 从机读写地址
 * @param trandirection 发射机或接收机
 * \arg     I2C_TRANSMITTER: 发射机
 *	\arg     I2C_RECEIVER: 接收机
 */
static void IIC_Send_Addr(uint8_t addr, uint32_t trandirection)
{
	/*将从站地址发送到I2C总线*/
	i2c_master_addressing(I2C0, addr, trandirection);
	/* 等待从机应答*/
	while (!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) && (timeout < I2C_TIME_OUT))
	{

		timeout++;
	}
	if (timeout < I2C_TIME_OUT)
	{
		timeout = 0;
	}
	else
	{
		timeout = 0;
		debug_println("i2c master Addr timeout!\n");
	}
	/* 清除ADDSEND位 */
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
}
static void IIC_Start(void)
{
	/* 等待I2C总线空闲 */
	while (i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) & (timeout < I2C_TIME_OUT))
	{
		timeout++;
	}
	if (timeout < I2C_TIME_OUT)
	{
		timeout = 0;
	}
	else
	{
		timeout = 0;
		debug_println("i2c master I2CBSY timeout!\n");
	}
	/* 向I2C总线发送启动条件 */
	i2c_start_on_bus(I2C0);

	/* wait until SBSEND bit is set */
	while (!i2c_flag_get(I2C0, I2C_FLAG_SBSEND) && (timeout < I2C_TIME_OUT))
	{
		timeout++;
	}
	if (timeout < I2C_TIME_OUT)
	{
		timeout = 0;
	}
	else
	{
		timeout = 0;
		debug_println("i2c master Start timeout!\n");
	}
}

/**
 * @brief 发送一个数据字节
 *
 * @param data 发送的数据
 */
static void IIC_Send_Byte(uint8_t data)
{
	/* 发送一个数据字节 */

	// I2C_DATA(I2C0) = data;

	/* 等待，直到传输数据寄存器为空*/
	while (!i2c_flag_get(I2C0, I2C_FLAG_TBE) && (timeout < I2C_TIME_OUT))
	{
		timeout++;
	}
	if (timeout < I2C_TIME_OUT)
	{
		i2c_data_transmit(I2C0, data);
		timeout = 0;
	}
	else
	{
		timeout = 0;
		debug_println("i2c master send Byte timeout!\n");
	}
}

static void IIC_Stop(void)
{
	while (!i2c_flag_get(I2C0, I2C_FLAG_BTC) && (timeout < I2C_TIME_OUT))
	{
		timeout++;
	}
	if (timeout < I2C_TIME_OUT)
	{
		timeout = 0;
	}
	else
	{
		timeout = 0;
		debug_println("i2c master BTC timeout!\n");
	}

	/* 向I2C总线发送停止条件 */
	i2c_stop_on_bus(I2C0);
	/* 等待i2c主控发送STOP信号成功 */
	while ((I2C_CTL0(I2C0) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT))
	{
		timeout++;
	}
	if (timeout < I2C_TIME_OUT)
	{
		timeout = 0;
	}
	else
	{
		timeout = 0;
		debug_println("i2c master sends stop signal timeout in WRITE!\n");
	}
}

/**
 * @brief 读多个字节
 *
 * @param read_address 寄存器的地址
 * @param number_of_byte 读的个数
 * @param data 数据存放位置
 */
static void SGM58301_ReadMulByte(uint8_t read_address, uint8_t number_of_byte, uint8_t *data)
{

	IIC_Start();
	IIC_Send_Addr(SGM58031_addr_R, I2C_RECEIVER);
	i2c_ack_config(I2C0, I2C_ACK_DISABLE);
	while (number_of_byte)
	{
		timeout++;
		// if (3 == number_of_byte)
		// {
		// 	/* wait until BTC bit is set */
		// 	while (!i2c_flag_get(I2C0, I2C_FLAG_BTC))
		// 		;
		// 	// /* disable acknowledge */
		// 	// i2c_ack_config(I2C0, I2C_ACK_DISABLE);
		// }
		if (2 == number_of_byte)
		{
			/* wait until BTC bit is set */
			while (!i2c_flag_get(I2C0, I2C_FLAG_BTC))
				;
			/* send a stop condition to I2C bus */
			// i2c_stop_on_bus(I2C0);
		}
		/* wait until RBNE bit is set */
		if (i2c_flag_get(I2C0, I2C_FLAG_RBNE))
		{
			/* read a byte from the EEPROM */
			*data = i2c_data_receive(I2C0);

			/* point to the next location where the byte read will be saved */
			data++;

			/* decrement the read bytes counter */
			number_of_byte--;
			timeout = 0;
		}
		if (timeout > I2C_TIME_OUT)
		{
			timeout = 0;

			printf("i2c master sends data timeout in READ!\n");
		}
	}
	i2c_stop_on_bus(I2C0);
	timeout = 0;
	/* i2c master sends STOP signal successfully */
	while ((I2C_CTL0(I2C0) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT))
	{
		timeout++;
	}
	if (timeout < I2C_TIME_OUT)
	{
		timeout = 0;
	}
	else
	{
		timeout = 0;

		printf("i2c master sends stop signal timeout in READ!\n");
	}
}
/**
 * @brief 写多个字节
 *
 * @param WriteAddr 写入的字节地址
 * @param size 写入的个数
 * @param data 写入的数据
 */
static void SGM58301_WriteMulByte(uint8_t WriteAddr, uint8_t size, uint8_t *data)
{
	i2c_ack_config(I2C0, I2C_ACK_ENABLE);
	// 发送起始信号
	IIC_Start();
	// 发送从机写地址
	IIC_Send_Addr(SGM58031_addr_W, I2C_TRANSMITTER);
	// 发送寄存器地址
	IIC_Send_Byte(WriteAddr);

	for (uint8_t i = 0; i < size; i++)
	{
		// 发送一个字节
		IIC_Send_Byte(data[i]);

		// II2_ACK();
	}
	IIC_Stop();
}

/**
 * @brief 输入多路复用器配置
 *
 * @param mux 输入多路复用器配置
 *
 *   AIN0_AIN1
 *   AIN0_AIN3,
 *   AIN1_AIN3,
 *   AIN2_AIN3,
 *   AIN0_GND,
 *   AIN1_GND,
 *   AIN2_GND,
 *   AIN3_GND
 */
void SGM58301_SET_MUX(uint8_t mux)
{
	SGM58301Config.SG858301Config.MUX = mux;
	uint8_t data[2] = {0};
	data[0] = SGM58301Config.value >> 8;
	data[1] = SGM58301Config.value;
	// debug_println("Config %#X", SGM58301Config.value);
	SGM58301_WriteMulByte(ConfigRegister, 2, data);
	// 内部指针寄存器的指向地址，指向A/D转换结果寄存器。
	SGM58301_PointRegister(ConversionRegister);
}

/**
 * @brief 配置SGM58301内部寄存器 来完成来完成通道选择，抓换速率，PGA增益倍数等信息配置
 *
 * 如果需要更改设置，更改下面的联合体的结构体内容
 *
 */
void SGM58301_ConfigRegister(void)
{

	SGM58301Config.SG858301Config.COMP_QUE = Four;
	SGM58301Config.SG858301Config.COMP_LAT = Nonlatching;			  // 比较器锁存
	SGM58301Config.SG858301Config.COMP_POL = Active_low;			  // 比较器极性
	SGM58301Config.SG858301Config.COMP_MODE = Traditional_comparator; // 比较器模式
	SGM58301Config.SG858301Config.DR = Rate_860;					  // 数据速率
	SGM58301Config.SG858301Config.MODE = Continuous_conversion_mode;
	SGM58301Config.SG858301Config.PGA = Gain_1_4096V;
	SGM58301Config.SG858301Config.MUX = AIN0_GND;
	SGM58301Config.SG858301Config.OS = Start_single_conversion;
	debug_println(" SGM58301Config = %#X", SGM58301Config.value);

	uint8_t data[2] = {0};
	data[0] = SGM58301Config.value >> 8;
	data[1] = SGM58301Config.value;
	SGM58301_WriteMulByte(ConfigRegister, 2, data);
	data[0] = 0;
	data[1] = 0;
	SGM58301_ReadMulByte(ConfigRegister, 2, data);
	uint16_t temp = 0;
	temp = ((data[0] << 8) | data[1]);
	debug_println("ConfigRegister = %#X", temp);
}

/**
 * @brief 读取SGM58301的16位ADC转换数据，注意类型 是int16 有符号
 *
 * @return int16_t
 */
int16_t SGM58301_ReadConversionRegister(void)
{
	uint8_t adcTemp[2];
	SGM58301_ReadMulByte(ConversionRegister, 2, adcTemp);
	return ((int16_t)adcTemp[0] << 8) | adcTemp[1];
}

/**
 * @brief 获取SGM58301的输入端口电压，根据寄存器的配置，得到实际的电压值
 *
 * 	Table 3. Full-Scale Range and Corresponding LSB Size	ti官网53页手册的17页
 *	FSR 			LSB SIZE
 *	±6.144 V(1) 	187.5 μV
 *	±4.096 V(1) 	125 μV
 *	±2.048 V 		62.5 μV
 *	±1.024 V 		31.25 μV
 *	±0.512 V 		15.625 μV
 *	±0.256 V 		7.8125 μV

 * @return float 电压单位V
 */
float SGM58301_GetAinVoltage(void)
{
	int16_t adcValue;
	float AinVoltage; // Ain 端口输入的电压
	adcValue = SGM58301_ReadConversionRegister();

	double LSB = 0.0;

	switch (SGM58301Config.SG858301Config.PGA)
	{
	case Gain_2_3_6144V:
		LSB = 0.0001875000;
		break;
	case Gain_1_4096V:
		LSB = 0.0001250000;
		break;
	case Gain_2_2048V:
		LSB = 0.0000625000;
		break;
	case Gain_4_1024V:
		LSB = 0.0000312500;
		break;
	case Gain_8_0512V:
		LSB = 0.0000156250;
		break;
	case Gain_16_0256V:
		LSB = 0.0000078125;
		break;
	case Gain_16_0256V1:
		LSB = 0.0000078125;
		break;
	case Gain_16_0256V2:
		LSB = 0.0000078125;
		break;
	default:
		debug_println("voltage error");
		break;
	}
	AinVoltage = (float)adcValue * LSB;
	return AinVoltage;
}

/**
 * @brief 设置内部指针寄存器的指向地址
 *
 * @param point 内部指针寄存器的指向地址
 */
void SGM58301_PointRegister(uint8_t point)
{
	// 发送起始信号
	IIC_Start();
	// 发送从机写地址
	IIC_Send_Addr(SGM58031_addr_W, I2C_TRANSMITTER);
	// 发送内部寄存器指向的地址
	IIC_Send_Byte(point);
	IIC_Stop();
}
void SGM58301_Init(void)
{
	/* 启用GPIOB时钟 */
	rcu_periph_clock_enable(RCU_GPIOB);
	/* 启用I2C0时钟 */
	rcu_periph_clock_enable(RCU_I2C0);
	// 启用AFIO时钟因为需要重映射
	gpio_pin_remap_config(GPIO_I2C0_REMAP, ENABLE);
	gpio_init(SGM58301_SDA_GPIO_Port, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, SGM58301_SDA_Pin | SGM58301_SCL_Pin);

	/* 配置I2C0时钟 */
	i2c_clock_config(I2C0, 400000, I2C_DTCY_2);
	/* 配置I2C0地址 */
	i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0X90);
	/* 使能I2C0 */
	i2c_enable(I2C0);
	/* 允许应答 */
	i2c_ack_config(I2C0, I2C_ACK_ENABLE);
	i2c_ackpos_config(I2C0, I2C_ACKPOS_NEXT);

	// 初始化ALERT 引脚
	gpio_init(SGM58301_ALERT_GPIO_Port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, SGM58301_ALERT_PIN);

	// 配置内部通道寄存器
	SGM58301_ConfigRegister();
	// 内部指针寄存器的指向地址，指向A/D转换结果寄存器。
	SGM58301_PointRegister(ConversionRegister);
}

/**
 * @brief 测试通讯函数 
 * 
 * 正常读出的ID为0X80
 * 
 */
void SGM58301_Test(void)
{
	SGM58301_PointRegister(ChipID);
	uint8_t data[2] = {0};
	uint16_t ID = 0;
	SGM58301_ReadMulByte(ChipID, 2, data);
	ID = ((data[0] << 8) | data[1]);
	debug_println("ID = %#X", ID);
}
