
#include "SVPWM.h"
#include <math.h>

// 常量
static constexpr float SQRT3 = 1.73205080757f;
static constexpr float HALF = 0.5f;
static constexpr float SQRT3_HALF = 0.86602540378f;

/**
 * @brief 计算 SVPWM 占空比
 * @param v_alpha α 轴电压
 * @param v_beta β 轴电压
 * @param v_dc 母线电压
 * @param output 输出结构体
 * @return true 表示计算成功，false 表示计算失败
 */
bool SVPWM::Calculate(float v_alpha,
                      float v_beta,
                      float v_dc,
                      SVPWMOutput& output)
{
    // 检查母线电压
    if (v_dc <= 0.0f)
    {
        return false;
    }

    // 限制电压矢量幅值，防止过调制
    LimitVoltage(v_alpha, v_beta, v_dc);

    // 1. Clarke 逆变换：αβ -> 三相电压
    float v_u = v_alpha;

    float v_v = -HALF * v_alpha
                + SQRT3_HALF * v_beta;

    float v_w = -HALF * v_alpha
                - SQRT3_HALF * v_beta;

    // 2. 找到三相电压最大值和最小值
    float v_max = fmaxf(v_u, fmaxf(v_v, v_w));
    float v_min = fminf(v_u, fminf(v_v, v_w));

    // 3. 注入零序电压
    float v_offset = -HALF * (v_max + v_min);

    v_u += v_offset;
    v_v += v_offset;
    v_w += v_offset;

    // 4. 电压转换为占空比
    output.duty_u = ClampDuty(
        HALF + v_u / v_dc
    );

    output.duty_v = ClampDuty(
        HALF + v_v / v_dc
    );

    output.duty_w = ClampDuty(
        HALF + v_w / v_dc
    );

    return true;
}
/**
 * @brief 限制电压矢量幅值，防止过调制
 * @param v_alpha α 轴电压
 * @param v_beta β 轴电压
 * @param v_dc 母线电压
 */
void SVPWM::LimitVoltage(float& v_alpha,
                         float& v_beta,
                         float v_dc)
{
    // 当前电压矢量幅值
    float magnitude = sqrtf(
        v_alpha * v_alpha +
        v_beta * v_beta
    );

    // SVPWM 线性调制区的最大矢量幅值
    float max_voltage =
        v_dc * max_modulation / SQRT3;

    // 超出范围时等比例缩小
    if (magnitude > max_voltage && magnitude > 0.0f)
    {
        float scale = max_voltage / magnitude;

        v_alpha *= scale;
        v_beta *= scale;
    }
}

/**
 * @brief 限制占空比在 [0, 1] 范围内
 * @param duty 占空比
 * @return 限制后的占空比
 */
float SVPWM::ClampDuty(float duty)
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

/**
 * @brief 设置最大调制度
 * @param modulation 最大调制度
 */
void SVPWM::SetMaxModulation(float modulation)
{
    if (modulation < 0.0f)
    {
        modulation = 0.0f;
    }

    if (modulation > 1.0f)
    {
        modulation = 1.0f;
    }

    max_modulation = modulation;
}
