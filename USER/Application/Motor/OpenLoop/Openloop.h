
#ifndef OPENLOOP_H
#define OPENLOOP_H

#include "PWM.h"
#include "SVPWM.h"

class OpenLoop
{
public:

    enum class State
    {
        IDLE,
        ALIGNMENT,
        RUNNING
    };

    OpenLoop() = default;

    // 绑定 PWM、SVPWM 对象及直流母线电压
    bool Init(PWM& pwm, SVPWM& svpwm, float v_dc);

    // 启动开环算法，进入转子预定位状态
    void Start();

    // 停止开环算法，不操作硬件
    void Stop();

    // 周期性更新，dt 单位为秒
    bool Update(float dt);

    // 设置旋转电压矢量幅值，单位 V
    void SetVoltageMagnitude(float voltage);

    // 设置预定位电压，单位 V
    void SetAlignVoltage(float voltage);

    // 设置预定位时间，单位 s
    void SetAlignTime(float time);

    // 设置目标电角速度，单位 rad/s
    void SetTargetElectricalSpeed(float speed);

    // 设置电角速度变化率，单位 rad/s^2
    void SetAcceleration(float acceleration);

    // 获取当前电角度
    float GetElectricalAngle() const;

    // 获取当前电角速度
    float GetElectricalSpeed() const;

    // 获取当前状态
    State GetState() const;

private:

    // 根据 αβ 电压计算并更新 PWM
    bool ApplyVoltage(float v_alpha, float v_beta);

    // 角度限制到 [0, 2π)
    void NormalizeAngle();

    // 限制电压幅值
    float ClampVoltage(float voltage);

private:

    PWM* pwm = nullptr;
    SVPWM* svpwm = nullptr;

    // 直流母线电压
    float v_dc = 0.0f;

    // 旋转电压矢量幅值
    float voltage_magnitude = 0.5f;

    // 转子预定位电压
    float align_voltage = 0.5f;

    // 预定位持续时间
    float align_time = 0.5f;

    // 预定位计时
    float align_timer = 0.0f;

    // 当前电角度
    float electrical_angle = 0.0f;

    // 当前电角速度
    float electrical_speed = 0.0f;

    // 目标电角速度
    float target_electrical_speed = 6.2831853f;

    // 电角速度变化率
    float acceleration = 2.0f;

    State state = State::IDLE;

    bool initialized = false;

    static constexpr float TWO_PI = 6.28318530718f;
    static constexpr float MAX_MODULATION = 0.95f;
};

#endif
