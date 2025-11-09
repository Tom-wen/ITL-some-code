#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "MOTOR.h"

/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Motor_Init(void)
{
    /*开启时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 只配置主电机控制引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_FORWARD | MOTOR_LEFT_BACKWARD | 
                                 MOTOR_RIGHT_FORWARD | MOTOR_RIGHT_BACKWARD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 如果需要额外的测试电机，单独配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_7;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    PWM_Init();
}

//void Motor_B_Init(void)
//{
//	/*开启时钟*/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; //|GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);						//将PA4和PA5引脚初始化为推挽输出	
//	PWM_Init();													//初始化直流电机的底层PWM
//}

/**
  * 函    数：直流电机设置速度
  * 参    数：Speed 要设置的速度，范围：-100~100
  * 返 回 值：无
  */
void Motor_SetSpeed(int8_t Speed)
{
    uint16_t pwm_value;
    
    // 将Speed(-100到100)映射到PWM(0到999)
    if (Speed >= 0)
    {
        // 正转
        GPIO_SetBits(GPIOC, GPIO_Pin_0);    // 左电机正转
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);  // 左电机反转
        GPIO_SetBits(GPIOC, GPIO_Pin_2);    // 右电机正转
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);  // 右电机反转
//---------------------------------test--------------------------------//
        GPIO_SetBits(GPIOC, GPIO_Pin_4);    // 左电机正转
        GPIO_ResetBits(GPIOC, GPIO_Pin_5);  // 左电机反转
        GPIO_SetBits(GPIOD, GPIO_Pin_3);    // 右电机正转
        GPIO_ResetBits(GPIOD, GPIO_Pin_7);  // 右电机反转
      
        pwm_value = (uint16_t)((999 * Speed) / 100);  // 映射到0-999
    }
    else
    {
        // 反转
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // 左电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_1);    // 左电机反转
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);  // 右电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_3);    // 右电机反转
//----------------------------------------TEST-------------------------------------//
        GPIO_ResetBits(GPIOC, GPIO_Pin_4);  // 左电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_5);    // 左电机反转
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);  // 右电机正转
        GPIO_SetBits(GPIOD, GPIO_Pin_7);    // 右电机反转
			
        pwm_value = (uint16_t)((999 * (-Speed)) / 100);  // 映射到0-999
    }
    
    // 设置两个电机的PWM
    PWM_SetCompare3(pwm_value);  // 左电机 - PB0
    PWM_SetCompare4(pwm_value);  // 右电机 - PB1
		test_motor_left(pwm_value);
		test_motor_right(pwm_value);
}
void set_motor_speeds(float left_speed, float right_speed)
{
    uint16_t left_pwm, right_pwm;
    
    // 处理左电机
    if (left_speed >= 0)
    {
        // 左电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_0);    // 左电机正转
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);  // 左电机反转
        left_pwm = (uint16_t)((999 * left_speed) / 100);  // 映射到0-999
    }
    else
    {
        // 左电机反转
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // 左电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_1);    // 左电机反转
        left_pwm = (uint16_t)((999 * (-left_speed)) / 100);  // 映射到0-999
    }
    
    // 处理右电机
    if (right_speed >= 0)
    {
        // 右电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_2);    // 右电机正转
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);  // 右电机反转
        right_pwm = (uint16_t)((999 * right_speed) / 100);  // 映射到0-999
    }
    else
    {
        // 右电机反转
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);  // 右电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_3);    // 右电机反转
        right_pwm = (uint16_t)((999 * (-right_speed)) / 100);  // 映射到0-999
    }
    
    // 设置两个电机的PWM
    PWM_SetCompare3(left_pwm);   // 左电机
    PWM_SetCompare4(right_pwm);  // 右电机
}




//麦轮解算公式 由于不需要转向wz=0
//    vt_lf = chassis_vx - chassis_vy - chassis_cmd_recv.wz * MOTOR_DISTANCE_TO_CENTER;
//    vt_rf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * MOTOR_DISTANCE_TO_CENTER;
//    vt_rb = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * MOTOR_DISTANCE_TO_CENTER;
//    vt_lb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * MOTOR_DISTANCE_TO_CENTER;

void Motor_MECNAMU_SetSpeed(float chassis_vx,float chassis_vy)
{
	  uint16_t vt_lf_pwm;
	  uint16_t vt_rf_pwm;
	  uint16_t vt_lb_pwm;
	  uint16_t vt_rb_pwm;
	
	  float vt_lf;
	  float vt_rf;
	  float vt_lb;
	  float vt_rb;
		vt_lf = -(-chassis_vx + chassis_vy); //(chassis_vx - chassis_vy);//右后轮//(chassis_vx - chassis_vy)
    vt_rf = -(chassis_vx + chassis_vy); ;//(-chassis_vx - chassis_vy);//左后轮//(-chassis_vx - chassis_vy)
    vt_rb = (chassis_vx - chassis_vy); ;//(-chassis_vx + chassis_vy);//左前轮//(-chassis_vx + chassis_vy);
    vt_lb = (-chassis_vx - chassis_vy); //(chassis_vx + chassis_vy);//右前轮//(chassis_vx + chassis_vy)
    
    // 处理左电机
    if (vt_lf >= 0)
    {
        // 左电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_0);    // 左电机正转
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);  // 左电机反转
        vt_lf_pwm = (uint16_t)((999 * vt_lf) / 100);  // 映射到0-999
    }
    else
    {
        // 左电机反转
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // 左电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_1);    // 左电机反转
        vt_lf_pwm = (uint16_t)((999 * (-vt_lf)) / 100);  // 映射到0-999
    }
    
    // 处理右电机
    if (vt_rf >= 0)
    {
        // 右电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_2);    // 右电机正转
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);  // 右电机反转
        vt_rf_pwm = (uint16_t)((999 * vt_rf) / 100);  // 映射到0-999
    }
    else
    {
        // 右电机反转
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);  // 右电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_3);    // 右电机反转
        vt_rf_pwm = (uint16_t)((999 * (-vt_rf)) / 100);  // 映射到0-999
    }
		    if (vt_rb >= 0)
    {
        // 左电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_4);    // 左电机正转
        GPIO_ResetBits(GPIOC, GPIO_Pin_5);  // 左电机反转
        vt_rb_pwm = (uint16_t)((999 * vt_rb) / 100);  // 映射到0-999
    }
    else
    {
        // 左电机反转
        GPIO_ResetBits(GPIOC, GPIO_Pin_4);  // 左电机正转
        GPIO_SetBits(GPIOC, GPIO_Pin_5);    // 左电机反转
        vt_rb_pwm = (uint16_t)((999 * (-vt_rb)) / 100);  // 映射到0-999
    }
    
    // 处理右电机
    if (vt_lb >= 0)
    {
        // 右电机正转
        GPIO_SetBits(GPIOD, GPIO_Pin_3);    // 右电机正转
        GPIO_ResetBits(GPIOD, GPIO_Pin_7);  // 右电机反转
        vt_lb_pwm = (uint16_t)((999 * vt_lb) / 100);  // 映射到0-999
    }
    else
    {
        // 右电机反转
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);  // 右电机正转
        GPIO_SetBits(GPIOD, GPIO_Pin_7);    // 右电机反转
        vt_lb_pwm = (uint16_t)((999 * (-vt_lb)) / 100);  // 映射到0-999
    }
		
		


    
    // 设置两个电机的PWM
    PWM_SetCompare3(vt_lf_pwm);  // 左电机 - PB0
    PWM_SetCompare4(vt_rf_pwm);  // 右电机 - PB1
	test_motor_left(1.19*vt_rb_pwm);
	test_motor_right(vt_lb_pwm);
}

