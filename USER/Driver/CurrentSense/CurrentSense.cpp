#include "CurrentSense.h"
#include "adc.h"

/**
 * @brief 初始化电流传感器
 * @retval true - 成功, false - 失败
 */
bool CurrentSense::Init()
{
    // CubeMX 生成的 ADC 句柄
    hadc_w = &hadc1;   // ADC1_IN1 -> PA0 -> W 相
    hadc_v = &hadc2;   // ADC2_IN2 -> PA1 -> V 相

    // 清空数据
    v_phase = {};
    w_phase = {};

    v_offset = 0.0f;
    w_offset = 0.0f;

    calibrated = false;
    initialized = false;

    // STM32G4 ADC 硬件校准
    if (HAL_ADCEx_Calibration_Start(
            hadc_w, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADCEx_Calibration_Start(
            hadc_v, ADC_SINGLE_ENDED) != HAL_OK)
    {
        return false;
    }

    initialized = true;

    return true;
}


/**
 * @brief 读取两路 ADC，更新电流数据
 * @retval true - 成功, false - 失败
 */
bool CurrentSense::ReadADC(
    ADC_HandleTypeDef* hadc,
    uint16_t* value)
{
    if (hadc == nullptr || value == nullptr)
    {
        return false;
    }

    // 启动 ADC
    if (HAL_ADC_Start(hadc) != HAL_OK)
    {
        return false;
    }

    // 等待转换完成
    HAL_StatusTypeDef status =
        HAL_ADC_PollForConversion(
            hadc,
            ADC_TIMEOUT
        );

    if (status != HAL_OK)
    {
        HAL_ADC_Stop(hadc);
        return false;
    }

    // 获取转换结果
    uint32_t raw = HAL_ADC_GetValue(hadc);

    // 停止 ADC
    HAL_ADC_Stop(hadc);

    // 检查 12 bit 数据范围
    if (raw > 4095U)
    {
        return false;
    }

    *value = static_cast<uint16_t>(raw);

    return true;
}


/**
 * @brief ADC 原始值转换为电压
 * @param raw ADC 原始值
 * @retval 电压值
 */
float CurrentSense::ConvertToVoltage(float raw) const
{
    return raw * ADC_VREF / ADC_MAX;
}


/**
 * @brief 零电流状态下进行偏置校准
 * @param samples 校准采样次数
 * @retval true - 成功, false - 失败
 */
bool CurrentSense::CalibrateOffset(uint16_t samples)
{
    if (!initialized || samples == 0)
    {
        return false;
    }

    // 校准期间不允许电机通电
    calibrated = false;

    uint64_t v_sum = 0;
    uint64_t w_sum = 0;

    uint16_t v_raw = 0;
    uint16_t w_raw = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        // 读取 V 相
        if (!ReadADC(hadc_v, &v_raw))
        {
            return false;
        }

        // 读取 W 相
        if (!ReadADC(hadc_w, &w_raw))
        {
            return false;
        }

        v_sum += v_raw;
        w_sum += w_raw;
    }

    // 计算平均 ADC 值并转换为电压
    float v_avg =
        static_cast<float>(v_sum) / samples;

    float w_avg =
        static_cast<float>(w_sum) / samples;

    v_offset = ConvertToVoltage(v_avg);
    w_offset = ConvertToVoltage(w_avg);

    calibrated = true;

    return true;
}


/**
 * @brief 读取两路 ADC，更新电流数据
 * @retval true - 成功, false - 失败
 */
bool CurrentSense::Update()
{
    if (!initialized || !calibrated)
    {
        return false;
    }

    uint16_t v_raw = 0;
    uint16_t w_raw = 0;

    // 读取两路 ADC
    if (!ReadADC(hadc_v, &v_raw))
    {
        return false;
    }

    if (!ReadADC(hadc_w, &w_raw))
    {
        return false;
    }

    // 转换为电压
    float v_voltage = ConvertToVoltage(v_raw);
    float w_voltage = ConvertToVoltage(w_raw);

    // 计算电流
    float v_current =
        (v_voltage - v_offset) / SENSITIVITY;

    float w_current =
        (w_voltage - w_offset) / SENSITIVITY;

    // 根据基尔霍夫电流定律计算 U 相
    u_current = -(v_phase.current + w_phase.current);

    // 更新 V 相数据
    v_phase.raw = v_raw;
    v_phase.voltage = v_voltage;
    v_phase.current = v_current;

    // 更新 W 相数据
    w_phase.raw = w_raw;
    w_phase.voltage = w_voltage;
    w_phase.current = w_current;

    return true;
}


/**
 * @brief 获取 V 相数据
 * @retval PhaseCurrent 结构体，包含原始值、电压和电流
 */
PhaseCurrent CurrentSense::GetVPhase() const
{
    return v_phase;
}


/**
 * @brief 获取 W 相数据
 * @retval PhaseCurrent 结构体，包含原始值、电压和电流
 */
PhaseCurrent CurrentSense::GetWPhase() const
{
    return w_phase;
}

/**
 * @brief 获取 U 相电流
 * @retval U 相电流值
 */
float CurrentSense::GetUCurrent() const
{
    return u_current;
}

/**
 * @brief 检查是否完成零点校准
 * @retval true - 已校准, false - 未校准
 */
bool CurrentSense::IsCalibrated() const
{
    return calibrated;
}