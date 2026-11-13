#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include "alg_pid.h"
#include "Clarke.h"
#include "Park.h"

#include <stdint.h>

/**
 * @brief 电流环计算结果
 */
struct CurrentLoopOutput
{
    // 实际电流
    float id;
    float iq;

    // PI 输出电压
    float vd;
    float vq;

    // 逆 Park 后的电压
    float v_alpha;
    float v_beta;
};

class CurrentLoop
{
public:

    /**
     * @brief 初始化电流环
     *
     * @param kp_d       d轴比例系数
     * @param ki_d       d轴积分系数
     * @param kp_q       q轴比例系数
     * @param ki_q       q轴积分系数
     * @param control_period 电流环周期，单位 s
     * @param voltage_limit  单轴PI输出限幅，单位 V
     * @param integral_limit 积分输出限幅，0表示不限制
     */
    bool Init(float kp_d,
              float ki_d,
              float kp_q,
              float ki_q,
              float control_period,
              float voltage_limit,
              float integral_limit = 0.0f);

    /**
     * @brief 设置目标电流
     *
     * @param id_target d轴目标电流，A
     * @param iq_target q轴目标电流，A
     */
    void SetTargetCurrent(float id_target,
                          float iq_target);

    /**
     * @brief 更新电流环
     *
     * @param phase_u U相电流，A
     * @param phase_v V相电流，A
     * @param phase_w W相电流，A
     * @param electrical_angle 转子电角度，rad
     * @param output 电流环计算结果
     */
    bool Update(float phase_u,
                float phase_v,
                float phase_w,
                float electrical_angle,
                CurrentLoopOutput& output);

    // 清除PI控制器积分及历史状态
    void Reset();

    // 获取目标电流
    float GetTargetId() const;
    float GetTargetIq() const;

    // 获取实际电流
    float GetId() const;
    float GetIq() const;

    // 获取电压输出
    float GetVd() const;
    float GetVq() const;

    bool IsInitialized() const;

private:

    // Clarke变换
    Clarke clarke;

    // 两路PI控制器
    Class_PID pi_d;
    Class_PID pi_q;

    // 目标电流
    float id_target = 0.0f;
    float iq_target = 0.0f;

    // 实际电流
    float id = 0.0f;
    float iq = 0.0f;

    // 电压输出
    float vd = 0.0f;
    float vq = 0.0f;

    // 控制器参数
    float kp_d = 0.0f;
    float ki_d = 0.0f;
    float kp_q = 0.0f;
    float ki_q = 0.0f;

    float control_period = 0.001f;
    float voltage_limit = 0.0f;
    float integral_limit = 0.0f;

    bool initialized = false;
};

#endif
