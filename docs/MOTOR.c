//
// Created by S0lux on 2026/10/16.
//
#include "MOTOR.h"
#include "ax_ps2.h"

extern JOYSTICK_TypeDef ps2;
//外部借用ps2
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
//借用计时器tim2和tim4

int16_t wheel_LF = 0;
int16_t wheel_RF = 0;
int16_t wheel_LB = 0;
int16_t wheel_RB = 0;


void MOTOR_Init()
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}
//初始化

//定义Single_Motor用于控制单一轮子
static void Single_Motor(int16_t Speed,TIM_HandleTypeDef *htim,uint32_t Channel1,uint32_t Channel2)
{
//防止菲绿灯模式导致的安全隐患
    if (ps2.mode != 115)
    {
        __HAL_TIM_SET_COMPARE(htim, Channel1, 0);
        __HAL_TIM_SET_COMPARE(htim, Channel2, 0);
        return;
    }
//------------------------------------------------------------------------------------------------
    if (Speed >80) Speed=80;
    if (Speed <-80) Speed=-80;
//-------------------------------------------限速--------------------------------------------------

    //if>0  正转
    if (Speed > 0)
    {
        __HAL_TIM_SET_COMPARE(htim, Channel1, Speed);
        __HAL_TIM_SET_COMPARE(htim, Channel2, 0);
    }
    //if<0  倒转
    else if (Speed < 0)
    {
        __HAL_TIM_SET_COMPARE(htim, Channel1, 0);
        __HAL_TIM_SET_COMPARE(htim, Channel2, -Speed);
    }
    //if=0  停转
    else
    {
        __HAL_TIM_SET_COMPARE(htim, Channel2, 0);
        __HAL_TIM_SET_COMPARE(htim, Channel1, 0);
    }
}

//开始执行控制代码
void MOTOR_Control_Task(void)
{
    AX_PS2_ScanKey(&ps2);
    //识别手柄
    int16_t ly = ps2.LJoy_UD - 128;
    int16_t lx = ps2.LJoy_LR - 128;
//ly上0下255  lx左0右255--车体移动
    int16_t rx = ps2.RJoy_LR - 128;
//同上--左右旋转

    if (ly > -15 && ly < 15) ly = 0;
    if (lx > -15 && lx < 15) lx = 0;
    if (rx > -15 && rx < 15) rx = 0;
//死区处理

    wheel_LF = -ly +lx + rx;
    wheel_LB = -ly - lx - rx;
    wheel_RF = -ly - lx +rx;
    wheel_RB = -ly +lx -rx;
//---------------------------局部变量无法监控故在此插入限幅------------------------------------------
    if (wheel_LF > 80) wheel_LF = 80; if (wheel_LF < -80) wheel_LF = -80;
    if (wheel_LB > 80) wheel_LB = 80; if (wheel_LB < -80) wheel_LB = -80;
    if (wheel_RF > 80) wheel_RF = 80; if (wheel_RF < -80) wheel_RF = -80;
    if (wheel_RB > 80) wheel_RB = 80; if (wheel_RB < -80) wheel_RB = -80;
//---------------------------------------------------------------------------------------------
//麦轮解算

    Single_Motor(wheel_LF,&htim2,TIM_CHANNEL_1,TIM_CHANNEL_2);
    Single_Motor(wheel_LB,&htim2,TIM_CHANNEL_3,TIM_CHANNEL_4);
    Single_Motor(wheel_RF,&htim4,TIM_CHANNEL_1,TIM_CHANNEL_2);
    Single_Motor(wheel_RB,&htim4,TIM_CHANNEL_3,TIM_CHANNEL_4);
//车轮运动

}

