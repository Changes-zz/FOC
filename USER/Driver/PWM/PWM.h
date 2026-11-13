
#ifndef PWM_H
#define PWM_H

#include "main.h"
#include <stdint.h>

class PWM
{
public:
    PWM() = default;

    // 初始化三路 PWM，默认关闭驱动输出
    bool Init();

    // 启动三路 PWM 输出
    bool Start();

    // 停止三路 PWM 输出
    void Stop();

    // 使能 DRV8313
    void Enable();

    // 关闭 DRV8313
    void Disable();

    // 设置三相占空比，范围 0.0 ~ 1.0
    bool SetDuty(float u, float v, float w);

    // 设置三相相同占空比
    bool SetAllDuty(float duty);

    // 获取当前三相占空比
    float GetUDuty() const;
    float GetVDuty() const;
    float GetWDuty() const;

private:
    // 将占空比限制在 0~1
    float ClampDuty(float duty);

    // 更新单个通道的比较值
    void SetCompare(uint32_t channel, float duty);

private:
    TIM_HandleTypeDef* htim = nullptr;

    float duty_u = 0.5f;
    float duty_v = 0.5f;
    float duty_w = 0.5f;

    bool initialized = false;
    bool started = false;
    bool enabled = false;
};

#endif
