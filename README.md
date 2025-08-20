这是一个SGM58301的驱动程序
使用说明：
1、配置SGM58301内的端口 I2C外设 底层通讯函数
2、配置SGM58301_Init();内的联合体参数为需要的参数
3、调用初始化SGM58301_Init();
4、调用SGM58301_Test();检查值是否与手册的ID寄存器所对应
5、调用SGM58301_GetAinVoltage(); 即可获得测得的电压值
6、调用SGM58301_ReadConversionRegister();即可获得16位ADC转换数据 寄存器原始数据
