#include "Motor.h"

bool Motor::Init(float dc_voltage)
{
    Stop();
    initialized = false;
    if (!pwm.Init() || !open_loop.Init(pwm, svpwm, dc_voltage))
    {
        pwm.Disable();
        return false;
    }
    initialized = true;
    return true;
}

bool Motor::Start()
{
    if (!initialized || running)
    {
        return false;
    }
    open_loop.Start();
    // 开启输出前先准备预定位占空比。
    if (!open_loop.Update(0.001f) || !pwm.Start())
    {
        Stop();
        return false;
    }
    pwm.Enable();
    running = true;
    return true;
}

void Motor::Stop()
{
    open_loop.Stop();
    pwm.Disable();
    pwm.Stop();
    running = false;
}

bool Motor::Update(float dt)
{
    if (!running)
    {
        return false;
    }
    if (!open_loop.Update(dt))
    {
        Stop();
        return false;
    }
    return true;
}
