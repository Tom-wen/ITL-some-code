
#include "stm32f10x.h"                  // Device header
#include "headfile.h"
#include "OLED.h"
#include "Encoder.h"
#include "LINE_FOLLOWER.h"
#include "TIMER.h"
#include "OPEN_MV.h"
//------------------------JY61P-----------------------------//
uint16_t RxData0,RxData1,RxData2;
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
void AutoScanSensor(void);
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
void Start(void);
float angle_diff;
float fAcc[3], fGyro[3], fAngle[3];
int i;
int count;
volatile uint32_t system_tick = 0;
void TIM4_IRQHandler(void);
float turn_start_angle = 0.0f;
uint8_t turning = 0;  // 0:未转弯, 1:正在左转, 2:正在右转
char buffer[10];
uint16_t flag=0;
uint16_t flag_1=0;


uint16_t speed_normal = -30;
uint16_t speed_behind = 40;
uint16_t speed_go = -100;
uint16_t speed_afterturn = -10;

//-----------------------------------------------------//

uint8_t KeyNum;		//定义用于接收按键键码的变量
int16_t Speed;		//定义速度变量
int16_t Num=0;//定义待被旋转编码器调节的变量
int16_t key=0;
int16_t left_k=0;
int16_t key_num=0;
int16_t speed=30;//35  30
//修复相关问题
int main(void)
{
	SystemInit();
	//JY61P陀螺仪的初始化
	Usart2Init(9600);
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	AutoScanSensor(); 
	/*模块初始化*/
	OLED_Init();		//OLED初始化
//	Encoder_Init(ENCODER_LEFT_REAR);
	USART3_openmv_Init(9600);// PD8-TX,PD9-RX
	uart4_init(9600);
//  Encoder_Init(ENCODER_LEFT_FRONT);
//  Encoder_Init(ENCODER_RIGHT_FRONT);
	line_follower_init();
	Timer_Init(10000-1,7200-1);
//	OLED_ShowString(1, 1, "Num:");			//1行1列显示字符串Num:
	Key_Init();			//按键初始化	
	int8_t test_speeds[] = {10, 25, 50, 75, 100};
	Delay_ms(1000);
	while (1)
	{
//	Motor_SetSpeed(30);
key=Key_GetNum();
key_num=key_num+key;
//void Motor_MECNAMU_SetSpeed(int8_t chassis_vx,int8_t chassis_vy)
		//改变vx,vy的值，例如前进为（vx,0）,后退为（vy,0）如果颠倒则取负数，如果是左右问题交换xy,具体以实际为准
		//(x,y) x正：右  y正：后
	if (key_num % 2==1)
	{
		Motor_MECNAMU_SetSpeed(-speed,0);//30   左
		Delay_ms(6500);

		//第一个箱子
		Motor_MECNAMU_SetSpeed(0,-speed);//    前
		Delay_s(5);
		Motor_MECNAMU_SetSpeed(0,speed);//    后
		Delay_s(2);
		Motor_MECNAMU_SetSpeed(-speed,0);//  左
		Delay_ms(4500);

		//第二个箱子
		Motor_MECNAMU_SetSpeed(0,-speed);//    前
		Delay_s(2);
		Motor_MECNAMU_SetSpeed(0,speed);//    后
		Delay_s(2);
		Motor_MECNAMU_SetSpeed(-speed,0);//  左
		Delay_ms(4500);

		//第三个箱子
		Motor_MECNAMU_SetSpeed(0,-speed);//    前
		Delay_s(2);
		Motor_MECNAMU_SetSpeed(0,speed);//    后
		Delay_s(2);
		Motor_MECNAMU_SetSpeed(-speed,0);//  左
		Delay_s(2);
	}
	else
	{
		Motor_MECNAMU_SetSpeed(0,0);//30
	}		
		

		//test_motor_left(30);
	//	test_motor_right(30);

					//					set_motor_speeds(30, 00);//左转
//Start();
//OLED_ShowSignedNum(1,1,fAngle[2],3);
//planBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB
//			if ( key==0 )
//		{
//			set_motor_speeds(-35, -35); 
//			Delay_ms (6700);//
//			key=key+1;
//		}
//			else if (key==1)//第一个右转
//		{
//			
//			if (!turning) 	
//						{
//								// 开始右转，记录起始角度
//								turn_start_angle = fAngle[2];
//								turning = 2;  // 标记为正在右转
//						}
//						while(1)
//						{
//							Start();
//						// 检查是否转过90度
//						float angle_diff = fAngle[2] - turn_start_angle;
//						if (angle_diff <= -75.0f)  // 右转角度差约为-90度 85.0 angle_diff <= -85.0f//90.0
//						{
//								// 完成90度右转，恢复直行
//								set_motor_speeds(-35, -35);
//								turning = 0;  // 重置转弯状态
//								OLED_ShowString(3, 1, "Right90 Done ");
//								break;
//						}
//						else
//						{
//								// 继续右转
//								set_motor_speeds(30, -50);//(20, -90) (20,-80)
//								OLED_ShowSignedNum(2, 1, (int16_t)angle_diff, 3);
//								OLED_ShowString(3, 5, " R");
//						}
//						Delay_ms(100);  // 缩短延时以便更频繁地检查角度
//					}
//			
//			
//		key=key+1;
//					
//		}

//		else if (key==2)//拐弯第一个箱子
//		{

//			set_motor_speeds(-30, -30); 
//			Delay_ms (6000);
//			key=key+1;
//		}	
//		else if (key==3)//拐弯后的后退
//		{

//			set_motor_speeds(35, 35); 
//			Delay_ms (4000);//6000
//			key=key+1;

//		}
//		else if(key==4)//左拐弯
//		{
//							if (!turning) 
//						{
//								// 开始左转，记录起始角度
//								turn_start_angle = fAngle[2];
//								turning = 1;  // 标记为正在左转
//						}
//						
//						while(1)
//						{
//						Start();
//						// 检查是否转过90度
//						float angle_diff = fAngle[2] - turn_start_angle;
//						if (angle_diff >= 84.0f)  // 左转角度差约为+90度//90
//						{
//								// 完成90度左转，恢复直行
//								set_motor_speeds(-30, -30);
//								turning = 0;  // 重置转弯状态
//								OLED_ShowString(3, 1, "Left90 Done  ");
//							break;
//						}
//						else
//						{
//								// 继续左转
//								set_motor_speeds(-60, 30);
//								OLED_ShowSignedNum(2, 1, (int16_t)angle_diff, 3);
//								OLED_ShowString(3, 5, " L");
//						}
//						Delay_ms(100);  // 缩短延时以便更频繁地检查角度
//						}
//						key=key+1;

//		}
//		else if(key==5)//左拐弯后的直线
//		{
//			
//			set_motor_speeds(-45, -45); 
//			Delay_ms (2850);
//			key=key+1;
//		}
//		else if (key==6)//右拐弯
//		{
//							if (!turning) 	
//						{
//								// 开始右转，记录起始角度
//								turn_start_angle = fAngle[2];
//								turning = 2;  // 标记为正在右转
//						}
//						while(1)
//						{
//							Start();
//						// 检查是否转过90度
//						float angle_diff = fAngle[2] - turn_start_angle;
//						if (angle_diff <= -83.0f)  // 右转角度差约为-90度 85.0 angle_diff <= -85.0f
//						{
//								// 完成90度右转，恢复直行
//								set_motor_speeds(-20, -20);
//								turning = 0;  // 重置转弯状态
//								OLED_ShowString(3, 1, "Right90 Done ");
//								break;
//						}
//						else
//						{
//								// 继续右转
//								set_motor_speeds(40, -100);//(20, -90)（20,-80）
//								OLED_ShowSignedNum(2, 1, (int16_t)angle_diff, 3);
//								OLED_ShowString(3, 5, " R");
//						}
//						Delay_ms(100);  // 缩短延时以便更频繁地检查角度
//					}
//			key=key+1;
//		}
//		else if (key==7)//拐弯第二个箱子
//		{
//			set_motor_speeds(-40, -40); 
//			Delay_ms (2600);
//			key=key+1;
//		}	

//		else if (key==8)//拐弯后的后退
//		{

//			set_motor_speeds(40, 40); //20,20
//			Delay_ms (2000);
//			key=key+1;

//		}

//		else if (key==9)//左拐
//		{
//						if (!turning) 
//						{
//								// 开始左转，记录起始角度
//								turn_start_angle = fAngle[2];
//								turning = 1;  // 标记为正在左转
//						}
//						
//						while(1)
//						{
//						Start();
//						// 检查是否转过90度
//						float angle_diff = fAngle[2] - turn_start_angle;
//						if (angle_diff >= 90.0f)  // 左转角度差约为+90度
//						{
//								// 完成90度左转，恢复直行
//								set_motor_speeds(-20, -20);
//								turning = 0;  // 重置转弯状态
//								OLED_ShowString(3, 1, "Left90 Done  ");
//							break;
//						}
//						else
//						{
//								// 继续左转
//								set_motor_speeds(-90, 20);
//								OLED_ShowSignedNum(2, 1, (int16_t)angle_diff, 3);
//								OLED_ShowString(3, 5, " L");
//						}
//						Delay_ms(100);  // 缩短延时以便更频繁地检查角度
//						}
//						key=key+1;
//		}
//		else if(key==10)//左拐弯后的直线
//		{
//			
//			set_motor_speeds(-40, -40); 
//			Delay_ms (450);
//			key=key+1;
//		}

//		else if(key==11)//右拐弯
//		{
//							if (!turning) 	
//						{
//								// 开始右转，记录起始角度
//								turn_start_angle = fAngle[2];
//								turning = 2;  // 标记为正在右转
//						}
//						while(1)
//						{
//							Start();
//						// 检查是否转过90度
//						float angle_diff = fAngle[2] - turn_start_angle;
//						if (angle_diff <= -90.0f)  // 右转角度差约为-90度 85.0 angle_diff <= -85.0f
//						{
//								// 完成90度右转，恢复直行
//								set_motor_speeds(-20, -20);
//								turning = 0;  // 重置转弯状态
//								OLED_ShowString(3, 1, "Right90 Done ");
//								break;
//						}
//						else
//						{
//								// 继续右转
//								set_motor_speeds(20, -250);//(20, -90)（20,-80）
//								OLED_ShowSignedNum(2, 1, (int16_t)angle_diff, 3);
//								OLED_ShowString(3, 5, " R");
//						}
//						Delay_ms(100);  // 缩短延时以便更频繁地检查角度
//					}
//			key=key+1;

//		}
//		else if (key==12 )//冲刺
//		{
//			
//			set_motor_speeds(-100, -100); 
//			Delay_ms (700);
//			key=key+1;


//		}

//		else if(key==13)//停止
//		{
//		set_motor_speeds( 0, 0); 
//		}
		}

}


// 定时器中断服务函数 - 用于定期执行控制算法
void TIM4_IRQHandler(void) 
{           
//    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
//    {
//        // 执行巡线控制
//        line_following_control();
//			  system_tick++;
//        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//    }

}

void Start(void)
{
		if(s_cDataUpdate)
		{
			   for(i = 0; i < 3; i++)
			   {
			   	  fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			   }
			   if(s_cDataUpdate & ANGLE_UPDATE)
			   {
					 RxData0=fAngle[0];
					 RxData1=fAngle[1];
					 RxData2=fAngle[2];
			   	s_cDataUpdate &= ~ANGLE_UPDATE;
					 
			   }
		}    
}
void CopeCmdData(unsigned char ucData)
{
	 static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	 s_ucData[s_ucRxCnt++] = ucData;
	 if(s_ucRxCnt<3)return;										//Less than three data returned
	 if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	 if(s_ucRxCnt >= 3)
	 {
		 if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		 {
		  	s_cCmd = s_ucData[0];
			  memset(s_ucData,0,50);//
			  s_ucRxCnt = 0;
	   } 
		 else 
		 {
			 s_ucData[0] = s_ucData[1];
			 s_ucData[1] = s_ucData[2];
			 s_ucRxCnt = 2;
			}
	  }
}
void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Uart2Send(p_data, uiSize);
}

void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}
void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)
	{
		Usart2Init(c_uiBaud[i]);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Delay_ms(100);
			if(s_cDataUpdate != 0)
			{
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
}

