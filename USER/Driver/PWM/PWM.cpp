
#include "PWM.h"
#include "tim.h"

// CubeMX 生成的 TIM1 句柄
extern TIM_HandleTypeDef htim1;

bool PWM::Init()
{
    htim = &htim1;

    // 初始化时首先关闭驱动
    Disable();

    // 初始设置三相占空比为 50%
    // 此时三相输出相同，不产生理想线电压矢量
    duty_u = 0.5f;
    duty_v = 0.5f;
    duty_w = 0.5f;

    SetCompare(TIM_CHANNEL_1, duty_u);
    SetCompare(TIM_CHANNEL_2, duty_v);
    SetCompare(TIM_CHANNEL_3, duty_w);

    initialized = true;

    return true;
}

void PWM::Stop()
{
    if (!initialized || !started)
    {
        return;
    }

    // 停止三路 PWM 输出
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);

    started = false;
}

bool PWM::Start()
{
    if (!initialized)
    {
        return false;
    }

    // 确保驱动芯片处于关闭状态
    Disable();

    // 启动 TIM1 三路 PWM
    if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
        return false;
    }

    if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
        return false;
    }

    started = true;

    return true;
}


void PWM::Enable()
{
    if (!initialized || !started)
    {
        return;
    }

    // DRV8313 EN1、EN2、EN3 共用 DRV_EN
    HAL_GPIO_WritePin(
        DRV_EN_GPIO_Port,
        DRV_EN_Pin,
        GPIO_PIN_SET
    );

    enabled = true;
}


void PWM::Disable()
{
    // 拉低 DRV_EN，关闭三个半桥
    HAL_GPIO_WritePin(
        DRV_EN_GPIO_Port,
        DRV_EN_Pin,
        GPIO_PIN_RESET
    );

    enabled = false;
}


bool PWM::SetDuty(float u, float v, float w)
{
    if (!initialized)
    {
        return false;
    }

    duty_u = ClampDuty(u);
    duty_v = ClampDuty(v);
    duty_w = ClampDuty(w);

    SetCompare(TIM_CHANNEL_1, duty_u);
    SetCompare(TIM_CHANNEL_2, duty_v);
    SetCompare(TIM_CHANNEL_3, duty_w);

    return true;
}


bool PWM::SetAllDuty(float duty)
{
    return SetDuty(duty, duty, duty);
}


void PWM::SetCompare(uint32_t channel, float duty)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

    uint32_t compare =
        static_cast<uint32_t>(duty * (arr + 1));

    // CCR 不允许超过 ARR
    if (compare > arr) 
    {
        compare = arr;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, compare);
}


float PWM::ClampDuty(float duty)
{
    if (duty < 0.0f)
    {
        return 0.0f;
    }

    if (duty > 1.0f)
    {
        return 1.0f;
    }

    return duty;
}


float PWM::GetUDuty() const
{
    return duty_u;
}


float PWM::GetVDuty() const
{
    return duty_v;
}


float PWM::GetWDuty() const
{
    return duty_w;
}
