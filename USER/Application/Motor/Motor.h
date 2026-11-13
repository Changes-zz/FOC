#ifndef MOTOR_H
#define MOTOR_H

#include "PWM.h"
#include "SVPWM.h"
#include "OpenLoop.h"

class Motor
{
public:
    // CubeMX 外设须先完成初始化；电压单位为 V。
    bool Init(float dc_voltage);
    bool Start();
    void Stop();
    bool Update(float dt);

    // 初始化后、启动前设置开环参数。
    OpenLoop& OpenLoopControl() { return open_loop; }
    const OpenLoop& OpenLoopControl() const { return open_loop; }

private:
    PWM pwm;
    SVPWM svpwm;
    OpenLoop open_loop;
    bool initialized = false;
    bool running = false;
};

#endif
