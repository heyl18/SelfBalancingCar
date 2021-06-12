/**********************************************************************
版权所有：	  喵呜实验室MiaowLabs，2017.
官		网：	http://www.miaowlabs.com
淘		宝：	https://miaowlabs.taobao.com/
文 件 名: 	  control.c
作    者:   喵呜实验室MiaowLabs
版		本:   3.00
完成日期:   2017.03.01
概		要: 	


***********************************************************************/
#include "math.h"
#include "stdio.h"
#include "control.h"
#include "debug.H"
#include "MPU6050.H"
#include "communicate.h"
#include "bsp.h"
#include "ultrasonic.h"
#include "infrare.h"
#include "manage.h"

unsigned char g_u8MainEventCount;
unsigned char g_u8SpeedControlCount;
unsigned char g_u8SpeedControlPeriod;
unsigned char g_u8DirectionControlPeriod;
unsigned char g_u8DirectionControlCount;

unsigned char g_cMotorDisable = 0;//值等于0时电机正常转动，否则停止转动


int g_iGravity_Offset = 0;

/******电机控制参数******/
float g_fSpeedControlOut;
float g_fSpeedControlOutOld;
float g_fSpeedControlOutNew;
float g_fAngleControlOut;
float g_fLeftMotorOut;
float g_fRightMotorOut;

/******速度控制参数******/

short  g_s16LeftMotorPulse;
short  g_s16RightMotorPulse;

int  g_s32LeftMotorPulseOld;
int  g_s32RightMotorPulseOld;
int  g_s32LeftMotorPulseSigma;
int  g_s32RightMotorPulseSigma;

float g_fCarSpeed;
float g_iCarSpeedSet;
float g_fCarSpeedOld;
float g_fCarPosition;

/*-----角度环和速度环PID控制参数-----*/
PID_t g_tCarAnglePID={17.0, 0, 23.0};	//*5 /10
PID_t g_tCarSpeedPID={15.25, 1.08, 0};	//i/10
/******蓝牙控制参数******/
float g_fBluetoothSpeed;
float g_fBluetoothDirection;
float g_fBluetoothDirectionOld;
float g_fBluetoothDirectionNew;
float g_fBluetoothDirectionOut;

float g_fCarAngle;         	// 控制车身平衡
float g_fGyroAngleSpeed;		//     			
float g_fGravityAngle;			
float g_fYawAngle;   // 偏航角
float g_fDx; 

int g_iLeftTurnRoundCnt = 0;
int g_iRightTurnRoundCnt = 0;

static int AbnormalSpinFlag = 0;

#define TURNING_LEFT 1
#define TURNING_RIGHT -1
#define MOVING_FORWARD 0
#define MOVING_LEFT 1
#define MOVING_RIGHT -1

#define ENDING_TAILING_THRESH 0 // 巡线几秒后才开始检测是否结束

#define MOVING_20CM 1460

int nowTurnAroundCnt = 0;
int myStep = 0; // 当前程序行进到第几步
int g_iMytimestamp = 0; // 用于判断程序状态的时间戳
float beginAngle = 0; // 转向前设置，记录开始转向前的角度
int turnOrMove = 0; // 行进转向的记录
int forward_cnt = 3; // 行进转向的记录
int MovingCar = 1; // 是否停车，为0则停
int lastTurn = 0; // 记录上次转向
int direct = 0; // 0朝前
int isTurn = 0; // 记录是否正在转弯，1为正在转弯
int directZeroCnt = 0; // 连续朝0方向前进的次数，检测到四个1设置为100
char detected = 0x01 | (0x01 << 1) | 0x01 << 2 | 0x01 << 3;// 记录红外传感器测量是否是四个1
int lastDetectedTime = 0;
char rrss;
const float DistanceThresh = 20;
const int speed_0 = 3;
const int speed_1 = 4;
const int speed_2 = 5;
const int speed_3 = 7;
const int speed_4 = 9;

 

int stopDetect(){
	char fraredresult  = InfraredDetectAll(); // 记录红外数据
	if(fraredresult == detected){
		return 1;
	}
	return 0;
}

void fixYaw(){// 控制电机，原地调整车头到初始方向
	if(g_fYawAngle < -1){
		Steer(-1, 0);
	}
	else if(g_fYawAngle > 1){
		Steer(1, 0);
	}
	else{
		Steer(0, 0);
	}
}

void makeSelfBanlance(){ // 控制小车原地站立
	Steer(0, 0);
}

void forward(){ // 朝着初始角度前进
	if(g_fYawAngle < -10){
		Steer(-4, speed_1);
	}
	else if(g_fYawAngle < -5){
		Steer(-2, speed_2);
	}
	else if(g_fYawAngle < -1){
		Steer(-1, speed_3);
	}
	else if(g_fYawAngle > 1){
		Steer(1, speed_3);
	}
	else if(g_fYawAngle > 5){
		Steer(2, speed_2);
	}
	else if(g_fYawAngle > 10){
		Steer(4, speed_1);
	}
	else{	
		Steer(0, speed_4);
	}
}
void moveForward(float a){ // 朝着角度a前进,a大于0为朝初始角的左边，小于0为右边
	if(g_fYawAngle < -10 + a){
		Steer(-4, speed_1);
	}
	else if(g_fYawAngle < -5 + a){
		Steer(-2, speed_2);
	}
	else if(g_fYawAngle < -1 + a){
		Steer(-1, speed_3);
	}
	else if(g_fYawAngle > 1 + a){
		Steer(1, speed_3);
	}
	else if(g_fYawAngle > 5 + a){
		Steer(2, speed_2);
	}
	else if(g_fYawAngle > 10 + a){
		Steer(4, speed_1);
	}
	else{	
		Steer(0, speed_4);
	}
}
void back(){ // 朝着初始角度后退
	if(g_fYawAngle < -10){
		Steer(-4, -3);
	}
	else if(g_fYawAngle < -5){
		Steer(-2, -4);
	}
	else if(g_fYawAngle < -1){
		Steer(-1, -5);
	}
	else if(g_fYawAngle > 1){
		Steer(1, -5);
	}
	else if(g_fYawAngle > 5){
		Steer(2, -4);
	}
	else if(g_fYawAngle > 10){
		Steer(4, -3);
	}
	else{	
		Steer(0, -6);
	}
}

/*
	原地罚站timeCnt秒，需要调用时应当20ms调用一次
		输入：timeCnt （第一次调用前需要更新g_iMytimestamp）
		输出：myStep加1，beginAngle更新，将turnOrMove置0
*/
void wait(int timeCnt){
	if( g_RunTime - g_iMytimestamp <= timeCnt){
		makeSelfBanlance();
	}
	else{
		myStep += 1;
		beginAngle = g_fYawAngle;
		turnOrMove = 0;
	}
}


/*
朝着左边转弯turnAngle度，边转弯边朝前,第一次调用前更新beginAngle
20ms调用一次，转一次前进forward_cnt次
*/
void turnLeft(float turnAngle){ 
	if(g_fYawAngle < beginAngle + turnAngle){
		if(turnOrMove == 0){
			Steer(-3, speed_2);
			turnOrMove ++;
		}
		else{
			Steer(0, speed_2);
			turnOrMove ++;
		}
		if(turnOrMove >= forward_cnt){
			turnOrMove = 0;
		}
	}
	else{	
		Steer(0, 0);
		myStep += 1;
		g_iMytimestamp = g_RunTime;
	}
}

/*
朝着右边转弯turnAngle度，边转弯边朝前,第一次调用前更新beginAngle
20ms调用一次，转一次前进forward_cnt次
*/
void turnRight(float turnAngle){
	if(g_fYawAngle > beginAngle - turnAngle){
		if(turnOrMove == 0){
			Steer(3, speed_2);
			turnOrMove ++;
		}
		else{
			Steer(0, speed_2);
			turnOrMove ++;
		}
		if(turnOrMove >= forward_cnt){
			turnOrMove = 0;
		}
	}
	else{	
		Steer(0, 0);
		myStep += 1;
		g_iMytimestamp = g_RunTime;
	}
}

/*
朝着左边转弯turnAngle度,第一次调用前更新beginAngle，20ms调用一次
转完之后将isTurn更新为0
*/
void realTurnLeft(float turnAngle){
	if(g_fYawAngle < beginAngle + turnAngle){
		Steer(-5,0);
	}
	else if(g_fYawAngle > beginAngle + turnAngle){
		Steer(-1,speed_0);
		isTurn = 0;
	}
}

/*
朝着右边转弯turnAngle度,第一次调用前更新beginAngle，20ms调用一次
转完之后将isTurn更新为0
*/
void realTurnRight(float turnAngle){
	if(g_fYawAngle > beginAngle - turnAngle){
		Steer(5,0);
	}
	else if(g_fYawAngle < beginAngle - turnAngle){
		Steer(-1,speed_0);
		isTurn = 0;
	}
}




void UltraControl(int mode) // 每隔20ms 执行一次
{	
	if(mode == 2){// 任务一
		if(myStep == 0 && (g_iLeftTurnRoundCnt + g_iRightTurnRoundCnt) / 2 < -7735){
			myStep += 1; // 前进且到达目的地
			g_iMytimestamp = g_RunTime; // 打上时间戳调用wait
			return;
		}
		if(myStep == 0){ // 前进
			forward();
		}
		else if(myStep == 1){// 原地等待2秒
			wait(2);
		}
		else if(myStep == 2){ // 等待一秒后开始后退
			if( (g_iLeftTurnRoundCnt + g_iRightTurnRoundCnt) / 2 < 0){
				back();
			}
			else{
				myStep += 1; // 后退完成
				g_iMytimestamp = g_RunTime; // 打上时间戳调用wait
				return;
			}
		}
		else if(myStep == 3){// 原地等待2秒，wait执行完会使myStep++，beginAngle更新
			wait(2);
		}
		else if(myStep == 4){// 左转135度
			turnLeft(135.0);
		}
		else if(myStep == 5){// 原地等待2秒，wait执行完会使myStep++，beginAngle更新
			wait(2);
		}
		else if(myStep == 6){// 右转135度
			turnRight(135.0);
		}
		else if(myStep == 7){// 所有任务完成，原地挂机
			makeSelfBanlance();
		}
	}
	else if(mode == 3){// 红外避障模式
		if(MovingCar == 1){ // 巡线阶段
			if(directZeroCnt >= 8){ // 走到最后一段直线了且遇到了横的
				MovingCar = 0; // 退出巡线模式
				lastDetectedTime = g_RunTime;
				fixYaw(); // 调整车头
				myStep++; // 走到下一步
			}
			else{ // 还没到最后的直线，继续巡线
				TailingControl();
			}
		}
		else if(myStep == 3){ // 所有任务完成，原地挂机，调整车头
			fixYaw();
		}
		else if(myStep == 2){ // 走20cm
			if((g_iLeftTurnRoundCnt + g_iRightTurnRoundCnt)/2 > nowTurnAroundCnt - MOVING_20CM){
				forward();
			}
			else{
				myStep++;
				fixYaw();
			}
		}
		else if(myStep == 1){// 避障前进模式
			if(isTurn){ // 正在转弯
				if(lastTurn == TURNING_LEFT){
					realTurnLeft(90); // 左转90
				}
				else{
					realTurnRight(90); // 右转90
				}
			}
			else if(Distance <= DistanceThresh){ // 遇到障碍
				Steer(0, -speed_4 );
				beginAngle = g_fYawAngle; // 初始化
				isTurn = 1; // 开启转弯模式
				if(direct == MOVING_FORWARD){
					if(lastTurn == TURNING_LEFT){ // 现在朝前遇到障碍，上次是左转，说明还要左转（策略为遇到障碍先右转走U型）
						direct = MOVING_LEFT;  // 更新车头朝向
						lastTurn = TURNING_LEFT; // 更新上次转向
					}
					else if(lastTurn == TURNING_RIGHT){ // 现在朝前遇到障碍，上次是右转，继续右转
						direct = MOVING_RIGHT;
						lastTurn = TURNING_RIGHT;
					}
					else{
						direct = MOVING_LEFT;  // 更新车头朝向
						lastTurn = TURNING_LEFT; // 更新上次转向
					}
				}
				else if(direct == MOVING_LEFT){ // 现在朝左，右转转到向前
					direct = MOVING_FORWARD;
					lastTurn = TURNING_RIGHT;
				}
				else if(direct == MOVING_RIGHT){ // 现在朝右，左转转到向前
					direct = MOVING_FORWARD; 
					lastTurn = TURNING_LEFT;
				}
			}
			else if(direct == MOVING_FORWARD){ // 没有障碍且向前
				forward(); // 一边走一边修正角度
				if(g_RunTime - lastDetectedTime >= 3) { // 四个都检测到了黑线
					if(stopDetect()){
						myStep++; // 进入停车区
						nowTurnAroundCnt = (g_iLeftTurnRoundCnt + g_iRightTurnRoundCnt) / 2;
					}
				
				}
			}
			else if(direct == MOVING_LEFT){ // 没有障碍且向左
				moveForward(90);
			}
			else if(direct == MOVING_RIGHT){// 没有障碍且向右
				moveForward(-90);
			}
		}

	}
}


/***************************************************************
** 函数名称: SetMotorVoltageAndDirection
** 功能描述: 电机转速及方向控制函数             
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2018年08月27日
***************************************************************/
void SetMotorVoltageAndDirection(int i16LeftVoltage,int i16RightVoltage)
{
	  if(i16LeftVoltage<0)
    {	
			GPIO_SetBits(GPIOA, GPIO_Pin_3 );				    
      GPIO_ResetBits(GPIOA, GPIO_Pin_4 );
      i16LeftVoltage = (-i16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOA, GPIO_Pin_4 );				    
      GPIO_ResetBits(GPIOA, GPIO_Pin_3 ); 
    }

    if(i16RightVoltage<0)
    {	
     	GPIO_SetBits(GPIOB, GPIO_Pin_0 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_1 );
      i16RightVoltage = (-i16RightVoltage);
    }
    else
    {
			GPIO_SetBits(GPIOB, GPIO_Pin_1 );				    
			GPIO_ResetBits(GPIOB, GPIO_Pin_0 );	      
    }

	if(i16RightVoltage > MOTOR_OUT_MAX)  
	{
		i16RightVoltage = MOTOR_OUT_MAX;
	}
	if(i16LeftVoltage > MOTOR_OUT_MAX)
	{
	   i16LeftVoltage = MOTOR_OUT_MAX;
	}  
	
	if(g_cMotorDisable)
	{
		TIM_SetCompare1(TIM3,0);
		TIM_SetCompare2(TIM3,0); 
	}
	else
	{
		TIM_SetCompare1(TIM3,i16RightVoltage);
		TIM_SetCompare2(TIM3,i16LeftVoltage);
	}
}


/***************************************************************
** 函数名称: MotorOutput
** 功能描述: 电机输出函数
             将直立控制、速度控制、方向控制的输出量进行叠加,并加
			 入死区常量，对输出饱和作出处理。
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/ 
** 日　期:   2014年08月01日
***************************************************************/
void MotorOutput(void)
{
	g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut - g_fBluetoothDirection ;	//这里的电机输出等于角度环控制量 + 速度环外环,这里的 - g_fSpeedControlOut 是因为速度环的极性跟角度环不一样，角度环是负反馈，速度环是正反馈
	g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirection ;


	/*增加死区常数*/
	if((int)g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
	if((int)g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	/*输出饱和处理，防止超出PWM范围*/			
	if((int)g_fLeftMotorOut  > MOTOR_OUT_MAX)	g_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((int)g_fLeftMotorOut  < MOTOR_OUT_MIN)	g_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((int)g_fRightMotorOut > MOTOR_OUT_MAX)	g_fRightMotorOut = MOTOR_OUT_MAX;
	if((int)g_fRightMotorOut < MOTOR_OUT_MIN)	g_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((int)g_fLeftMotorOut,(int)g_fRightMotorOut);
}



/***************************************************************
** 函数名称: CarUpstandInit
** 功能描述: 全局变量初始化函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void CarUpstandInit(void)
{
	//g_iAccelInputVoltage_X_Axis = g_iGyroInputVoltage_Y_Axis = 0;
	g_s16LeftMotorPulse = g_s16RightMotorPulse = 0;
	g_s32LeftMotorPulseOld = g_s32RightMotorPulseOld = 0;
	g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;

	g_fCarSpeed = g_fCarSpeedOld = 0;
	g_fCarPosition = 0;
	g_fCarAngle    = 0;
	g_fGyroAngleSpeed = 0;
	g_fGravityAngle   = 0;
	g_fYawAngle = 0;
	g_fDx = 0;

	g_fAngleControlOut = g_fSpeedControlOut = g_fBluetoothDirectionOut = 0;
	g_fLeftMotorOut    = g_fRightMotorOut   = 0;
	g_fBluetoothSpeed  = g_fBluetoothDirection = 0;
	g_fBluetoothDirectionNew = g_fBluetoothDirectionOld = 0;

  g_u8MainEventCount=0;
	g_u8SpeedControlCount=0;
 	g_u8SpeedControlPeriod=0;
}


/***************************************************************
** 函数名称: AbnormalSpinDetect
** 功能描述: 电机转速异常检测      
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 日　期:   2017年4月26日
***************************************************************/

void AbnormalSpinDetect(short leftSpeed,short rightSpeed)
{
	static unsigned short count = 0;
	
	//速度设置为0时检测，否则不检测
	if(g_iCarSpeedSet==0)
	{
		if(((leftSpeed>30)&&(rightSpeed>30)&&(g_fCarAngle > -30) && (g_fCarAngle < 30))
			||((leftSpeed<-30)&&(rightSpeed<-30))&&(g_fCarAngle > -30) && (g_fCarAngle < 30))
		{// 左右电机转速大于30、方向相同、持续时间超过250ms，且车身角度不超过30度，则判断为悬空空转
			count++;
			if(count>50){
				count = 0;
				AbnormalSpinFlag = 1;
			}
		}
		else{
			count = 0;
		}
	}
	else{
		count = 0;
	}
}

/***************************************************************
** 函数名称: LandingDetect
** 功能描述: 小车着地检测      
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 日　期:   2017年4月26日
***************************************************************/
void LandingDetect(void)
{
	static float lastCarAngle = 0;
	static unsigned short count = 0,count1 = 0;
	
	if(AbnormalSpinFlag == 0)return;
	
	// 小车角度5°~-5°启动检测
	if((g_fCarAngle > -5) && (g_fCarAngle < 5))
	{
		count1++;
		if(count1 >= 50)
		{//每隔250ms判断一次小车角度变化量，变化量小于0.8°或大于-0.8°判断为小车静止
			count1 = 0;
			if(((g_fCarAngle - lastCarAngle) < 0.8) && ((g_fCarAngle - lastCarAngle) > -0.8))
			{
				count++;
				if(count >= 4){
					count = 0;
					count1 = 0;
					g_fCarPosition = 0;
					AbnormalSpinFlag = 0;
				}
			}
			else{
				count = 0;
			}
			lastCarAngle = g_fCarAngle;
		}
	}
	else
	{
		count1 = 0;
		count = 0;
	}
}

/***************************************************************
** 函数名称: MotorManage
** 功能描述: 电机使能/失能控制      
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 日　期:   2017年4月26日
***************************************************************/
void MotorManage(void)
{

	AbnormalSpinDetect(g_s16LeftMotorPulse, g_s16RightMotorPulse);
		
	LandingDetect();
	
	if(AbnormalSpinFlag)
	{	
		g_cMotorDisable |= (0x01<<1);
	}
	else
	{
		g_cMotorDisable &= ~(0x01<<1);
	}
	
	if(g_fCarAngle > 30 || g_fCarAngle < (-30))
	{
		g_cMotorDisable |= (0x01<<2);
	}
	else
	{
		g_cMotorDisable &= ~(0x01<<2);
	}
	
}

void GetMotorPulse(void)  //采集电机速度脉冲
{ 	
  g_s16LeftMotorPulse = TIM_GetCounter(TIM2);     
  g_s16RightMotorPulse= -TIM_GetCounter(TIM4);
  TIM2->CNT = 0;
  TIM4->CNT = 0;   //清零

  g_s32LeftMotorPulseSigma +=  g_s16LeftMotorPulse;
  g_s32RightMotorPulseSigma += g_s16RightMotorPulse; 
	
	g_iLeftTurnRoundCnt -= g_s16LeftMotorPulse;
	g_iRightTurnRoundCnt -= g_s16RightMotorPulse;

}

/***************************************************************
** 作　  者: MiaowLabs Team
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 2015年11月29日
** 函数名称: AngleCalculate
** 功能描述: 角度环计算函数           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室MiaowLabs版权所有**************************
***************************************************************/
void AngleCalculate(void)
{
	//-------加速度--------------------------
	//量程为±2g时，灵敏度：16384 LSB/g
    g_fGravityAngle = atan2(g_fAccel_y/16384.0,g_fAccel_z/16384.0) * 180.0 / M_PI;
	g_fGravityAngle = g_fGravityAngle - g_iGravity_Offset;

	//-------角速度-------------------------
	//范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
	g_fGyro_x  = g_fGyro_x / 16.4;  //计算角速度值			   
	g_fGyroAngleSpeed = g_fGyro_x;

	//-------互补滤波---------------
	g_fCarAngle = 0.98 * (g_fCarAngle + g_fGyroAngleSpeed * 0.005) + 0.02 *	g_fGravityAngle;

	// 角速度积分计算yaw, GYRO_Z_OFFSET为修正 g_fYawAngle = g_fGyro_z / GYRO_SENSITIVITY;
	g_fYawAngle += (g_fGyro_z / GYRO_SENSITIVITY + GYRO_Z_OFFSET)*0.005;
}

/***************************************************************
** 作　  者: 喵呜实验室MiaowLabs
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 2018年08月27日
** 函数名称: AngleControl
** 功能描述: 角度环控制函数           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室MiaowLabs版权所有**************************
***************************************************************/

void AngleControl(void)	 
{
	g_fAngleControlOut =  (CAR_ANGLE_SET-g_fCarAngle) * g_tCarAnglePID.P *5 + \
	(CAR_ANGLE_SPEED_SET-g_fGyroAngleSpeed) * (g_tCarAnglePID.D /10);
}



/***************************************************************
** 函数名称: SpeedControl
** 功能描述: 速度环控制函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/

void SpeedControl(void)
{
  	float fP,fI;   	
	float fDelta;
	
	
	g_fCarSpeed = (g_s32LeftMotorPulseSigma  + g_s32RightMotorPulseSigma ) * 0.5 ;
    g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;	  //全局变量 注意及时清零
    	
	g_fCarSpeed = 0.7 * g_fCarSpeedOld + 0.3 * g_fCarSpeed ;//低通滤波，使速度更平滑
	g_fCarSpeedOld = g_fCarSpeed;

	fDelta = CAR_SPEED_SET;
	fDelta -= g_fCarSpeed;   
	
	fP = fDelta * (g_tCarSpeedPID.P);
    fI = fDelta * (g_tCarSpeedPID.I/10.0);

	g_fCarPosition += fI;
	g_fCarPosition += g_fBluetoothSpeed;	  
	
//积分上限设限
	if((s16)g_fCarPosition > CAR_POSITION_MAX)    g_fCarPosition = CAR_POSITION_MAX;
	if((s16)g_fCarPosition < CAR_POSITION_MIN)    g_fCarPosition = CAR_POSITION_MIN;
	
	g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  g_fSpeedControlOutNew = fP + g_fCarPosition;
}
/***************************************************************
** 函数名称: SpeedControlOutput
** 功能描述: 速度环控制输出函数-分多步逐次逼近最终输出，尽可能将对直立环的干扰降低。
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void SpeedControlOutput(void)
{
  float fValue;
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld ;
  g_fSpeedControlOut = fValue * (g_u8SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld; 
}


/***************************************************************
** 函数名称: Scale
** 功能描述: 量程归一化处理
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
float Scale(float input, float inputMin, float inputMax, float outputMin, float outputMax) { 
  float output;
  if (inputMin < inputMax)
    output = (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output = (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}

/***************************************************************
** 函数名称: Steer
** 功能描述: 遥控速度及方向处理函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void Steer(float direct, float speed)
{
	if(direct > 0)
		g_fBluetoothDirection = Scale(direct, 0, 10, 0, 400);
	else
		g_fBluetoothDirection = -Scale(direct, 0, -10, 0, 400);

	if(speed > 0)
		g_iCarSpeedSet = Scale(speed, 0, 10, 0, 70);
	else
		g_iCarSpeedSet = -Scale(speed, 0, -10, 0, 70);

}

/***************************************************************
** 作　  者: MiaowLabs Team
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 20160415
** 函数名称: TailingControl
** 功能描述: 红外寻迹           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室MiaowLabs版权所有**************************
***************************************************************/
// void TailingControl(void)
// {
// #if INFRARE_DEBUG_EN > 0
// 	char buff[32];	
// #endif
// 	char result;
// 	float direct = 0;
// 	float speed = 0;

// 	result = InfraredDetect();
// 	if(result & infrared_channel_Lb)
// 		direct = -10;
// 	else if(result & infrared_channel_La)
// 		direct = -4;
// 	else if(result & infrared_channel_Rb)
// 		direct = 10;
// 	else if(result & infrared_channel_Ra)
// 		direct = 4;
// 	else
// 		direct = 0;
// 	if(g_RunTime > ENDING_TAILING_THRESH){// 巡线最后一段直线检测，等过了x秒开始
// 		if(direct == 0){
// 			directZeroCnt ++;
// 		}
// 		if(direct != 0){
// 			directZeroCnt = 0;
// 		}
// 	}
// 	speed = 2.2;
// 	Steer(direct, speed);

// #if INFRARE_DEBUG_EN > 0
// 	sprintf(buff, "Steer:%d, Speed:%d\r\n",(int)direct,  (int)speed);
// 	DebugOutStr(buff);
// #endif
// }

void TailingControl(void)
{
	char result;
	float direct = 0;
	float speed = 0;

	if(g_RunTime > ENDING_TAILING_THRESH){// 巡线最后一段直线检测，等过了x秒开始
		result = InfraredDetectAll();
		if(result == detected){
			directZeroCnt = 100;
			Steer(0,0);
			return;
		}
	}
	else{
		result = InfraredDetect();
	}
	
	if(result & infrared_channel_La)
		direct = -4;
	else if(result & infrared_channel_Ra)
		direct = 4;
	else if(result & infrared_channel_Rb)
		direct = 6;
	else if(result & infrared_channel_Lb)
		direct = -6;
	else
		direct = 0;
	speed = 2.3;
	Steer(direct, speed);
	
}
