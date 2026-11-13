#ifndef CURRENTSENSE_H
#define CURRENTSENSE_H

#include "main.h"
#include <stdint.h>

struct PhaseCurrent
{
    uint16_t raw = 0;   // ADC 原始值
    float voltage = 0;  // ADC 输入电压
    float current = 0;  // 实际电流 A
};

class CurrentSense
{
public:
    // 初始化 ADC 硬件校准
    bool Init();

    // 读取两路 ADC，更新电流数据
    bool Update();

    // 零电流状态下进行偏置校准
    bool CalibrateOffset(uint16_t samples = 512);

    // 获取 V 相采样结果
    PhaseCurrent GetVPhase() const;

    // 获取 W 相采样结果
    PhaseCurrent GetWPhase() const;

    // 获取 U 相电流
    float GetUCurrent() const;

    // 是否完成零点校准
    bool IsCalibrated() const;

private:
    // 读取指定 ADC
    bool ReadADC(ADC_HandleTypeDef* hadc,
                 uint16_t* value);

    // ADC 原始值转换为电压
    float ConvertToVoltage(float raw) const;

private:
    // ADC 句柄
    ADC_HandleTypeDef* hadc_v = nullptr;
    ADC_HandleTypeDef* hadc_w = nullptr;

    // 两相电流数据
    PhaseCurrent v_phase = {};
    PhaseCurrent w_phase = {};

    // U 相电流由 V、W 相计算得到
    float u_current = 0.0f;

    // 零点偏置电压
    float v_offset = 0.0f;
    float w_offset = 0.0f;

    // 状态标志
    bool initialized = false;
    bool calibrated = false;

private:
    // ADC 参数
    static constexpr float ADC_VREF = 3.3f;
    static constexpr float ADC_MAX = 4095.0f;

    // 电流采样硬件参数
    static constexpr float SHUNT_RESISTOR = 0.01f; //采样电阻 0.01Ω
    static constexpr float AMPLIFIER_GAIN = 50.0f; //增益50倍

    // 电流灵敏度：0.5 V/A
    static constexpr float SENSITIVITY =
        SHUNT_RESISTOR * AMPLIFIER_GAIN;

    // ADC 轮询超时时间
    static constexpr uint32_t ADC_TIMEOUT = 10;
};

#endif