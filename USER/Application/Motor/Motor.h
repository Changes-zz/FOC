#ifndef MOTOR_H
#define MOTOR_H

#include "PWM.h"
#include "SVPWM.h"
#include "OpenLoop.h"

/**
 * @brief 电机控制模式枚举
 */
enum Motor_Control_Mode
 {
    OpenLoop = 0,
    Current,
    Velocity,
    Position
};


class Motor
{
public:
    // 初始化底层硬件；未实现的闭环模式返回 false。
    bool Init(Motor_Control_Mode mode, float dc_voltage);

    // 停止状态下选择控制模式；未实现的模式返回 false。
    bool SetMode(Motor_Control_Mode mode);

    // 启动和停止
    bool Start();
    void Stop();

    // 开环速度为电角速度 rad/s；电流和位置目标暂不参与控制。
    void SetCurrent(float current);
    void SetVelocity(float velocity);
    void SetPosition(float position);

    // 周期更新
    bool Update(float dt);

    // 状态读取
    Motor_Control_Mode GetMode() const;
    bool IsRunning() const;
    float GetOpenLoopElectricalAngle() const;
    float GetOpenLoopAccumulatedElectricalAngle() const;

private:

    bool UpdateOpenLoop(float dt);
    bool UpdateCurrent(float dt);
    bool UpdateVelocity(float dt);
    bool UpdatePosition(float dt);

    PWM pwm;
    SVPWM svpwm;
    class OpenLoop open_loop;
    Motor_Control_Mode mode = Motor_Control_Mode::OpenLoop;
    bool initialized = false;
    bool running = false;
    float target_current = 0.0f;
    float target_velocity = 0.0f;
    float target_position = 0.0f;
};

#endif
