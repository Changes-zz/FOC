#include "CurrentLoop.h"

#include <math.h>

/**
 * @brief 初始化电流环
 */
bool CurrentLoop::Init(float kp_d,
                       float ki_d,
                       float kp_q,
                       float ki_q,
                       float control_period,
                       float voltage_limit,
                       float integral_limit)
{
    if (control_period <= 0.0f ||
        voltage_limit <= 0.0f ||
        integral_limit < 0.0f)
    {
        initialized = false;
        return false;
    }

    this->kp_d = kp_d;
    this->ki_d = ki_d;
    this->kp_q = kp_q;
    this->ki_q = ki_q;

    this->control_period = control_period;
    this->voltage_limit = voltage_limit;
    this->integral_limit = integral_limit;

    // d轴PI
    pi_d.Init(
        kp_d,
        ki_d,
        0.0f,               // Kd = 0，使用PI
        0.0f,               // 前馈
        integral_limit,
        voltage_limit,
        control_period
    );

    // q轴PI
    pi_q.Init(
        kp_q,
        ki_q,
        0.0f,
        0.0f,
        integral_limit,
        voltage_limit,
        control_period
    );

    Reset();

    initialized = true;

    return true;
}


/**
 * @brief 设置目标电流
 */
void CurrentLoop::SetTargetCurrent(float id_target,
                                   float iq_target)
{
    this->id_target = id_target;
    this->iq_target = iq_target;
}


/**
 * @brief 更新电流环
 */
bool CurrentLoop::Update(float phase_u,
                         float phase_v,
                         float phase_w,
                         float electrical_angle,
                         CurrentLoopOutput& output)
{
    if (!initialized)
    {
        return false;
    }

    // =========================================
    // 1. Clarke变换：UVW -> AlphaBeta
    // =========================================

    ClarkeOutput current_ab = {};

    clarke.Calculate(
        phase_u,
        phase_v,
        phase_w,
        current_ab
    );

    // =========================================
    // 2. Park变换：AlphaBeta -> DQ
    // =========================================

    AlphaBeta current_input;

    current_input.alpha = current_ab.alpha;
    current_input.beta = current_ab.beta;

    DQ current_dq = Park::Transform(
        current_input,
        electrical_angle
    );

    id = current_dq.d;
    iq = current_dq.q;

    // =========================================
    // 3. 计算电流误差并执行PI控制
    // =========================================

    pi_d.Set_Target(id_target);
    pi_d.Set_Now(id);
    pi_d.TIM_Calculate_PeriodElapsedCallback();

    pi_q.Set_Target(iq_target);
    pi_q.Set_Now(iq);
    pi_q.TIM_Calculate_PeriodElapsedCallback();

    vd = pi_d.Get_Out();
    vq = pi_q.Get_Out();

    // =========================================
    // 4. 电压矢量限幅
    // =========================================

    float voltage_magnitude = sqrtf(
        vd * vd + vq * vq
    );

    if (voltage_magnitude > voltage_limit)
    {
        float scale = voltage_limit / voltage_magnitude;

        vd *= scale;
        vq *= scale;
    }

    // =========================================
    // 5. 逆Park变换：DQ -> AlphaBeta
    // =========================================

    DQ voltage_dq;

    voltage_dq.d = vd;
    voltage_dq.q = vq;

    AlphaBeta voltage_ab = Park::InverseTransform(
        voltage_dq,
        electrical_angle
    );

    // =========================================
    // 6. 输出计算结果
    // =========================================

    output.id = id;
    output.iq = iq;

    output.vd = vd;
    output.vq = vq;

    output.v_alpha = voltage_ab.alpha;
    output.v_beta = voltage_ab.beta;

    return true;
}


/**
 * @brief 重置PI控制器
 */
void CurrentLoop::Reset()
{
    id = 0.0f;
    iq = 0.0f;

    vd = 0.0f;
    vq = 0.0f;

    id_target = 0.0f;
    iq_target = 0.0f;

    // 重新初始化，清除积分和历史误差
    if (control_period > 0.0f)
    {
        pi_d.Init(
            kp_d, ki_d, 0.0f, 0.0f,
            integral_limit, voltage_limit,
            control_period
        );

        pi_q.Init(
            kp_q, ki_q, 0.0f, 0.0f,
            integral_limit, voltage_limit,
            control_period
        );
    }
}


/**
 * @brief 获取目标电流
 */
float CurrentLoop::GetTargetId() const
{
    return id_target;
}

float CurrentLoop::GetTargetIq() const
{
    return iq_target;
}


/**
 * @brief 获取实际电流
 */
float CurrentLoop::GetId() const
{
    return id;
}

float CurrentLoop::GetIq() const
{
    return iq;
}


/**
 * @brief 获取电压输出
 */
float CurrentLoop::GetVd() const
{
    return vd;
}

float CurrentLoop::GetVq() const
{
    return vq;
}


/**
 * @brief 判断是否初始化
 */
bool CurrentLoop::IsInitialized() const
{
    return initialized;
}
