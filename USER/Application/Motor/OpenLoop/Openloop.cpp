
#include "OpenLoop.h"
#include <math.h>

/*
 * 初始化开环算法
 * @param pwm_driver PWM驱动器
 * @param svpwm_algorithm SVPWM算法
 * @param dc_voltage 直流电压
 * @return 初始化是否成功
 */
bool OpenLoop::Init(PWM& pwm_driver,
                    SVPWM& svpwm_algorithm,
                    float dc_voltage)
{
    if (dc_voltage <= 0.0f)
    {
        return false;
    }

    pwm = &pwm_driver;
    svpwm = &svpwm_algorithm;

    v_dc = dc_voltage;

    voltage_magnitude = 0.5f;
    align_voltage = 0.5f;

    electrical_angle = 0.0f;
    electrical_speed = 0.0f;

    align_timer = 0.0f;

    state = State::IDLE;

    initialized = true;

    return true;
}

/*
 * 启动开环算法
 */
void OpenLoop::Start()
{
    if (!initialized)
    {
        return;
    }

    if (pwm != nullptr)
    {
        pwm->Enable();
    }

    electrical_angle = 0.0f;
    electrical_speed = 0.0f;
    align_timer = 0.0f;

    state = State::ALIGNMENT;
}

/*
 * 停止开环算法
 */
void OpenLoop::Stop()
{
    state = State::IDLE;

    electrical_speed = 0.0f;
    align_timer = 0.0f;

    if (pwm != nullptr)
    {
        // 先关闭驱动芯片
        pwm->Disable();

        // 设置三相相同占空比
        pwm->SetAllDuty(0.5f);
    }
}

/*
 * 周期性控制更新
 * @param dt 时间步长
 * @return 更新是否成功
 */
bool OpenLoop::Update(float dt)
{
    if (!initialized ||
        state == State::IDLE ||
        dt <= 0.0f ||
        dt > 0.01f)
    {
        return false;
    }

    // 第一阶段：转子预定位
    if (state == State::ALIGNMENT)
    {
        align_timer += dt;

        // 固定方向施加电压矢量
        if (!ApplyVoltage(align_voltage, 0.0f))
        {
            Stop();
            return false;
        }

        // 预定位结束，进入旋转阶段
        if (align_timer >= align_time)
        {
            electrical_angle = 0.0f;
            electrical_speed = 0.0f;

            state = State::RUNNING;
        }

        return true;
    }

    // 第二阶段：电角速度斜坡控制

    float speed_error =
        target_electrical_speed - electrical_speed;

    float speed_step = acceleration * dt;

    if (speed_error > speed_step)
    {
        electrical_speed += speed_step;
    }
    else if (speed_error < -speed_step)
    {
        electrical_speed -= speed_step;
    }
    else
    {
        electrical_speed = target_electrical_speed;
    }

    // 更新电角度
    electrical_angle += electrical_speed * dt;

    NormalizeAngle();

    // 计算旋转电压矢量
    float v_alpha =
        voltage_magnitude * cosf(electrical_angle);

    float v_beta =
        voltage_magnitude * sinf(electrical_angle);

    // 交给 SVPWM 和 PWM
    if (!ApplyVoltage(v_alpha, v_beta))
    {
        Stop();
        return false;
    }

    return true;
}

/*
 * 根据 αβ 电压计算并更新 PWM
 * @param v_alpha α轴电压
 * @param v_beta β轴电压
 * @return 更新是否成功
 */
// αβ 电压 -> SVPWM -> PWM
bool OpenLoop::ApplyVoltage(float v_alpha,
                            float v_beta)
{
    if (pwm == nullptr || svpwm == nullptr)
    {
        return false;
    }

    SVPWMOutput output;

    if (!svpwm->Calculate(
            v_alpha,
            v_beta,
            v_dc,
            output))
    {
        return false;
    }

    return pwm->SetDuty(
        output.duty_u,
        output.duty_v,
        output.duty_w
    );
}


// 设置旋转电压矢量幅值
void OpenLoop::SetVoltageMagnitude(float voltage)
{
    voltage_magnitude = ClampVoltage(voltage);
}


// 设置预定位电压
void OpenLoop::SetAlignVoltage(float voltage)
{
    align_voltage = ClampVoltage(voltage);
}


// 设置预定位时间
void OpenLoop::SetAlignTime(float time)
{
    if (time >= 0.0f)
    {
        align_time = time;
    }
}


// 设置目标电角速度
void OpenLoop::SetTargetElectricalSpeed(float speed)
{
    target_electrical_speed = speed;
}


// 设置电角速度变化率
void OpenLoop::SetAcceleration(float value)
{
    if (value > 0.0f)
    {
        acceleration = value;
    }
}


// 获取电角度
float OpenLoop::GetElectricalAngle() const
{
    return electrical_angle;
}


// 获取电角速度
float OpenLoop::GetElectricalSpeed() const
{
    return electrical_speed;
}


// 获取状态
OpenLoop::State OpenLoop::GetState() const
{
    return state;
}


// 限制电压幅值
float OpenLoop::ClampVoltage(float voltage)
{
    if (voltage < 0.0f)
    {
        voltage = 0.0f;
    }

    const float max_voltage =
        v_dc * MAX_MODULATION / 1.73205080757f;

    if (voltage > max_voltage)
    {
        voltage = max_voltage;
    }

    return voltage;
}


// 角度归一化
void OpenLoop::NormalizeAngle()
{
    while (electrical_angle >= TWO_PI)
    {
        electrical_angle -= TWO_PI;
    }

    while (electrical_angle < 0.0f)
    {
        electrical_angle += TWO_PI;
    }
}
