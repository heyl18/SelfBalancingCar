/**********************************************************************
版权所有：	喵呜创新科技，2017.
官		网：	http://www.miaowlabs.com
淘		宝：	https://shop275516297.taobao.com/
文 件 名: 	ultrasonic.h
作    者:   喵呜实验室
版		本:   3.00
完成日期:   2017.03.01
概		要: 	




***********************************************************************/

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

extern int Distance;


void TIM1_Cap_Init(void);	
void Read_Distane(void);
char InfraredDetect(void);
void UltraSelfCheck(void);
int IsUltraOK(void);



#endif

