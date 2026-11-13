#include "tsk_config_and_callback.h"

#include "tim.h"
#include "main.h"
#include "adc.h"

#include "AS5600_APP.h"
#include "CurrentSense.h"
#include "uart_printf.h"

#include "PWM.h"
#include "SVPWM.h"
#include "OpenLoop.h"
#include <math.h>


/*全局变量定义*/

// AS5600 串口打印调试变量
float AS5600_imformation[3] = {0.0f, 0.0f, 0.0f};//串口打印调试变量

// CurrentSense 串口打印调试变量
CurrentSense current_sense;
float Current_imformation[3] = {0.0f, 0.0f, 0.0f};

// 电机串口调试变量
float Motor_imformation[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 电机驱动模块
PWM pwm;
SVPWM svpwm;
OpenLoop open_loop;


/**
 * @brief 任务初始化函数
 * @retval None
 */
void Task_Init()
{
    // 初始化编码器
    AS5600_Init();

    // 初始化电流采样
    current_sense.Init();
    current_sense.CalibrateOffset(512);

    pwm.Init();
    open_loop.Init(pwm, svpwm, 24.0f);  // 与实测母线电压一致

    open_loop.SetAlignVoltage(1.0f);    // 预定位电压，V
    open_loop.SetAlignTime(1.0f);       // 预定位时间，s

    open_loop.SetVoltageMagnitude(1.0f);         // 旋转电压矢量幅值，V
    open_loop.SetTargetElectricalSpeed(7.33f);  // 14 对极，约 30 rpm
    open_loop.SetAcceleration(10.0f);           // 电角加速度，rad/s²

    open_loop.Start();
    open_loop.Update(0.001f);
    pwm.Start();
    pwm.Enable();

    // 启动控制中断，按开环参数旋转磁场
    HAL_TIM_Base_Start_IT(&htim6);

    uart_printf("FOC Init Complete!\r\n");
}

/**
 * @brief 1ms 定时器回调函数
 * @retval None
 */
void TIM_1ms_Callback()
{
    if (open_loop.GetState() != OpenLoop::State::IDLE)
    {
        if (!open_loop.Update(0.001f))
        {
            open_loop.Stop();
        }
    }
}


/**
 * @brief 定时器中断回调函数
 * @param htim 定时器句柄
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        TIM_1ms_Callback();
    }
}



/**
 * @brief 任务循环函数
 * @retval None
 */
void Task_loop()
{
    // AS5600 更新
    AS5600_Update();
    
    // AS5600_imformation[0] = as5600_angle_deg;
    // AS5600_imformation[1] = as5600_reg_agc;
    // AS5600_imformation[2] = as5600_magnet_detected;


    // CurrentSense 更新
    if (current_sense.Update())
    {
        PhaseCurrent v = current_sense.GetVPhase();
        PhaseCurrent w = current_sense.GetWPhase();

        // Current_imformation[0] = current_sense.GetUCurrent();
        // Current_imformation[1] = v.current;
        // Current_imformation[2] = w.current;
    }

    // 电机调试信息更新
    Motor_imformation[0] = open_loop.GetElectricalAngle() * (180.0f / 3.14159265359f); // 电角度，度
    Motor_imformation[1] = as5600_angle_deg;
    float mechanical_angle = fmodf(
        open_loop.GetAccumulatedElectricalAngle() * (180.0f / 3.14159265359f) / 14.0f,
        360.0f);
    Motor_imformation[2] = mechanical_angle < 0.0f ? mechanical_angle + 360.0f : mechanical_angle;
    Motor_imformation[3] = current_sense.GetUCurrent();
    Motor_imformation[4] = current_sense.GetVPhase().current;
    Motor_imformation[5] = current_sense.GetWPhase().current;

    vofa_justfloat_printf(Motor_imformation, 6);

    HAL_Delay(10);
}
