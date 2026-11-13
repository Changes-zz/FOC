
#ifndef SVPWM_H
#define SVPWM_H

struct SVPWMOutput
{
    float duty_u;
    float duty_v;
    float duty_w;
};

class SVPWM
{
public:
    SVPWM() = default;

    // 根据 αβ 电压矢量计算三相占空比
    bool Calculate(float v_alpha,
                   float v_beta,
                   float v_dc,
                   SVPWMOutput& output);

    // 设置最大调制系数，默认 0.95
    void SetMaxModulation(float modulation);

private:
    // 限制电压矢量幅值
    void LimitVoltage(float& v_alpha,
                      float& v_beta,
                      float v_dc);

    // 限制占空比
    float ClampDuty(float duty);

private:
    float max_modulation = 0.95f;
};

#endif
