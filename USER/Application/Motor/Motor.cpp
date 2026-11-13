#include "Motor.h"
#include <math.h>

/**
 * @brief 初始化电机
 * @param requested_mode 请求的控制模式
 * @param dc_voltage 直流电压
 * @retval true 初始化成功，false 初始化失败
 */
bool Motor::Init(Motor_Control_Mode requested_mode, float dc_voltage)
{
    Stop();
    initialized = false;
    if (requested_mode != Motor_Control_Mode::OpenLoop ||
        !isfinite(dc_voltage) || dc_voltage <= 0.0f)
    {
        return false;
    }
    if (!pwm.Init() || !open_loop.Init(pwm, svpwm, dc_voltage))
    {
        pwm.Disable();
        return false;
    }
    mode = requested_mode;
    open_loop.SetAlignVoltage(1.0f);
    open_loop.SetAlignTime(1.0f);
    open_loop.SetVoltageMagnitude(5.0f);
    open_loop.SetAcceleration(10.0f);
    open_loop.SetTargetElectricalSpeed(target_velocity);
    initialized = true;
    return true;
}


/**
 * @brief 设置电机控制模式
 * @param requested_mode 请求的控制模式
 * @retval true 设置成功，false 设置失败
 */
bool Motor::SetMode(Motor_Control_Mode requested_mode)
{
    if (!initialized || running || requested_mode != Motor_Control_Mode::OpenLoop)
    {
        return false;
    }
    mode = requested_mode;
    return true;
}

/**
 * @brief 启动电机
 * @retval true 启动成功，false 启动失败
 */
bool Motor::Start()
{
    if (!initialized || running || mode != Motor_Control_Mode::OpenLoop)
    {
        return false;
    }
    open_loop.Start();
    if (!open_loop.Update(0.001f) || !pwm.Start())
    {
        Stop();
        return false;
    }
    pwm.Enable();
    running = true;
    return true;
}

/**
 * @brief 停止电机
 */
void Motor::Stop()
{
    open_loop.Stop();
    pwm.Disable();
    pwm.Stop();
    running = false;
}

/**
 * @brief 设置电流目标
 * @param current 电流目标
 */
void Motor::SetCurrent(float current)
{
    target_current = current;
}

/**
 * @brief 设置速度目标
 * @param velocity 速度目标
 */
void Motor::SetVelocity(float velocity)
{
    target_velocity = velocity;
    if (initialized && mode == Motor_Control_Mode::OpenLoop)
    {
        open_loop.SetTargetElectricalSpeed(velocity);
    }
}

/**
 * @brief 设置位置目标
 * @param position 位置目标
 */
void Motor::SetPosition(float position)
{
    target_position = position;
}

/**
 * @brief 更新电机状态
 * @param dt 时间步长
 * @retval true 更新成功，false 更新失败
 */
bool Motor::Update(float dt)
{
    if (!initialized || !running)
    {
        return false;
    }
    bool success = false;
    switch (mode)
    {
    case Motor_Control_Mode::OpenLoop: success = UpdateOpenLoop(dt); break;
    case Motor_Control_Mode::Current: success = UpdateCurrent(dt); break;
    case Motor_Control_Mode::Velocity: success = UpdateVelocity(dt); break;
    case Motor_Control_Mode::Position: success = UpdatePosition(dt); break;
    }
    if (!success)
    {
        Stop();
    }
    return success;
}

/**
 * @brief 更新开环控制
 * @param dt 时间步长
 * @retval true 更新成功，false 更新失败
 */
bool Motor::UpdateOpenLoop(float dt) { return open_loop.Update(dt); }
bool Motor::UpdateCurrent(float dt) { (void)dt; return false; }
bool Motor::UpdateVelocity(float dt) { (void)dt; return false; }
bool Motor::UpdatePosition(float dt) { (void)dt; return false; }
Motor_Control_Mode Motor::GetMode() const { return mode; }
bool Motor::IsRunning() const { return running; }
float Motor::GetOpenLoopElectricalAngle() const { return open_loop.GetElectricalAngle(); }
float Motor::GetOpenLoopAccumulatedElectricalAngle() const { return open_loop.GetAccumulatedElectricalAngle(); }
