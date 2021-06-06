/**********************************************************************
版权所有：	喵呜创新科技，2017.
官		网：	http://www.miaowlabs.com
淘		宝：	https://shop275516297.taobao.com/
文 件 名: 	I2C.h
作    者:   喵呜实验室
版		本:   3.00
完成日期:   2017.03.01
概		要: 	




***********************************************************************/
#ifndef _I2C_H
#define _I2C_H


#include "stm32f10x.h"


#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define true 1
#define false 0 
#define   bool  uint8_t

#define TRUE  0
#define FALSE -1
/*
#define MPU6050_READRATE			1000	//6050读取频率
#define MPU6050_READTIME			0.001	//6050读取时间间隔
#define EE_6050_ACC_X_OFFSET_ADDR	0
#define EE_6050_ACC_Y_OFFSET_ADDR	1
#define EE_6050_ACC_Z_OFFSET_ADDR	2
#define EE_6050_GYRO_X_OFFSET_ADDR	3
#define EE_6050_GYRO_Y_OFFSET_ADDR	4
#define EE_6050_GYRO_Z_OFFSET_ADDR	5
*/
////0表示写
//#define	I2C_Direction_Transmitter   0
////１表示读
//#define	I2C_Direction_Receiver      1	 
/*====================================================================================================*/
/*====================================================================================================*/
bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
void i2cInit(void);
static void i2cUnstick(void);

int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
/*====================================================================================================*/
/*====================================================================================================*/
#endif

