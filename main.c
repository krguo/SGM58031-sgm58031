#include "sgm58031.h"


/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
  //初始化SGM58301 并配置内部寄存器
   SGM58301_Init();
  //调用测试函数判断是不是0X80如果是那么I2C通讯无误
  SGM58301_Test();
  //设置 输入多路复用器配置
  SGM58301_SET_MUX(AIN0_GND);
  //使用前需要先调整SGM58301内部指针指向的位置 比如指向A/D转换结果寄存器
  SGM58301_PointRegister(ConversionRegister);
  //读取ADC转换后的的电压值
  float V1 = SGM58301_GetAinVoltage();
  //读取ADC转换的数据
  int16_t adcValue = SGM58301_ReadConversionRegister();

}
