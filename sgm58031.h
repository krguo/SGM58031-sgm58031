/*
 * @Descripttion:
 * @Author: 过期的小朋友
 * @Date: 2025-08-11 20:31:24
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2025-08-18 16:40:38
 * @FilePath: \Leak_detection\Hardware\SGM58031\sgm58031.h
 * @版权所有:  Copyright(C) 2025 by 过期的小朋友, All Rights Reserved
 */
#ifndef __SGM58031_H
#define __SGM58031_H

#include "gd32f10x.h"
#include "gd32f10x_gpio.h"
#include "stdio.h"
#include "string.h"
#include "systick.h"
#include "Debug.h"

#define I2C_TIME_OUT (uint16_t)(10000)
#define I2C_OK                 1
#define I2C_FAIL               0
#define I2C_END                1


#define SGM58301_SDA_GPIO_Port GPIOB
#define SGM58301_SDA_Pin GPIO_PIN_9
#define SGM58301_SCL_GPIO_Port GPIOB
#define SGM58301_SCL_Pin GPIO_PIN_8

#define SGM58301_ALERT_GPIO_Port GPIOA
#define SGM58301_ALERT_PIN GPIO_PIN_3

/* ADDR引脚连接不同电平时，设备显现出不同的I2C地址 */
#define ads1115_GND_iic_addr 0x90
#define ads1115_VDD_iic_addr 0x92
#define ads1115_SDA_iic_addr 0x94
#define ads1115_SCL_iic_addr 0x96

#define SGM58031_addr_W ads1115_GND_iic_addr
#define SGM58031_addr_R ads1115_GND_iic_addr 

/* SGM58031寄存器地址 */
#define ConversionRegister 0x00
#define ConfigRegister 0x01
#define LoThreshRegister 0x02
#define HiThreshRegister 0x03
#define Config1Register 0X04
#define ChipID 0X05

/* 读取Alert引脚电平，判断是否超出数字比较器量程 */
#define GetADS1115Alert gpio_input_bit_get(SGM58301_ALERT_GPIO_Port, SGM58301_ALERT_PIN)

/***************SGM5831内部寄存器功能定义 **********************************/

// 运行状态或单次转换位
// 运行状态或单次转换启动此位确定设备的运行状态。 OS只能在掉电状态下写入，并且在转换正在进行时无效。
enum OS
{
    No_effect = 0,          // 对于写入状态此值无效 对于对于读取状态：0=芯片正在执行转换
    Start_single_conversion // 开启一次单次转换  对于读取状态 芯片没有进行转换
};

// 输入多路复用器配置
// 这些位配置输入多路复用器
enum MUX
{
    AIN0_AIN1 = 0, // MUX选通的连接到PGA的引脚为AIN0和AIN1
    AIN0_AIN3,
    AIN1_AIN3,
    AIN2_AIN3,
    AIN0_GND,
    AIN1_GND,
    AIN2_GND,
    AIN3_GND
};

// 可编程增益放大器配置
// 这些位设置可编程增益放大器的FSR
// 此参数表示ADC缩放的满量程范围。 请勿对器件的模拟输入施加超过VDD + 0.3 V的电压
enum PGA
{
    Gain_2_3_6144V = 0, // 增益为2/3，引脚最大输入电压为±6.144 +0.3V
    Gain_1_4096V,       // 增益为1，引脚最大输入电压为±4.096 +0.3V
    Gain_2_2048V,       // 增益为2，引脚最大输入电压为±2.048 +0.3V
    Gain_4_1024V,       // 增益为4，引脚最大输入电压为±1.024 +0.3V
    Gain_8_0512V,       // 增益为8，引脚最大输入电压为±0.512 +0.3V
    Gain_16_0256V,      // 增益为16，引脚最大输入电压为±0.256 +0.3V
    Gain_16_0256V1,     // 增益为16，引脚最大输入电压为±0.256 +0.3V
    Gain_16_0256V2      // 增益为16，引脚最大输入电压为±0.256 +0.3V
};

// 设备运行模式
// 该位控制操作模式。
enum MODE
{
    Continuous_conversion_mode = 0, // 连续转换模式
    Single_shot_mode                // 单次模式或掉电状态
};

// 数据速率
// 这些位控制数据速率设置。
enum DR
{
    Rate_8 = 0, // 8 SPS
    Rate_16,    // 16 SPS
    Rate_32,    // 32 SPS
    Rate_64,    // 64 SPS
    Rate_128,   // 128 SPS
    Rate_250,   // 250 SPS
    Rate_475,   // 475 SPS
    Rate_860    // 860 SPS
};

// 比较器模式
// 该位配置比较器工作模式
enum COMP_MODE
{
    Traditional_comparator = 0, // 传统比较器
    Window_comparator           // 窗口比较器
};

// 比较器极性
// 该位控制ALERT / RDY引脚的极性
enum COMP_POL
{
    Active_low = 0, // 低电平有效
    Active_high     // 高电平有效
};

// 比较器锁存
// 该位控制ALERT / RDY引脚在置位后是锁存还是在转换后的阈值上限和下限范围内清零
enum COMP_LAT
{
    Nonlatching = 0, // 非锁存比较器
    Latching         // 锁存比较器
};

// 比较器队列和失能
// 这些位执行两个功能。 设置为11时，比较器禁用，ALERT / RDY引脚设置为高阻态。
// 当设置为任何其他值时，ALERT / RDY引脚和比较器功能被使能，
//并且设置值确定在断言ALERT / RDY引脚之前超过所需的上限或下限的连续转换次数
enum COMP_QUE
{
    One = 0, // 一次转换后断言
    Two,     // 两次转换后断言
    Four,    // 四次转换后断言
    Disable  // 禁用比较器并将ALERT / RDY引脚设置为高阻态
};

union SGM58301ConfigRegister
{
    uint16_t value;
    struct
    {
        uint8_t COMP_QUE : 2;
        uint8_t COMP_LAT : 1;
        uint8_t COMP_POL : 1;
        uint8_t COMP_MODE : 1;
        uint8_t DR : 3;
        uint8_t MODE : 1;
        uint8_t PGA : 3;
        uint8_t MUX : 3;
        uint8_t OS : 1;
    } SG858301Config;
};

void SGM58301_Init(void);
void SGM58301_Test(void);
void SGM58301_ConfigRegister(void);
void SGM58301_PointRegister(uint8_t point);

int16_t SGM58301_ReadConversionRegister(void);
float SGM58301_GetAinVoltage(void);
void SGM58301_SET_MUX(uint8_t mux);
#endif /* __SGM58031_H */
