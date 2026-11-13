#include "main.h"
#include "i2c.h"
#include "AS5600.h"
#include "AS5600_APP.h"

// 创建 AS5600 对象
static AS5600 encoder(&hi2c1);

// 调试变量
volatile uint8_t as5600_i2c_ok = 0;
volatile uint8_t as5600_magnet_detected = 0;
volatile uint8_t as5600_magnet_too_weak = 0;
volatile uint8_t as5600_magnet_too_strong = 0;

volatile uint16_t as5600_raw_angle = 0;
volatile float as5600_angle_deg = 0.0f;

volatile uint32_t as5600_hal_status = 0;

volatile float as5600_reg_agc = 0.0f;

// 初始化
extern "C" void AS5600_Init(void)
{
    as5600_i2c_ok = encoder.init() ? 1 : 0;

    as5600_hal_status =
        static_cast<uint32_t>(encoder.getLastStatus());

    if (as5600_i2c_ok)
    {
        AS5600_Update();
    }
}


// 周期性读取
extern "C" void AS5600_Update(void)
{
    AS5600::MagnetStatus status{};

    // 读取磁铁状态
    if (encoder.readMagnetStatus(status))
    {
        as5600_magnet_detected = status.detected;
        as5600_magnet_too_weak = status.tooWeak;
        as5600_magnet_too_strong = status.tooStrong;
    }
    else
    {
        as5600_i2c_ok = 0;
        as5600_hal_status =
            static_cast<uint32_t>(encoder.getLastStatus());
        return;
    }

    // 读取原始角度
    uint16_t raw = 0;

    if (encoder.readRawAngle(raw))
    {
        as5600_raw_angle = raw;

        // 转换为角度
        as5600_angle_deg =
            static_cast<float>(raw) * 360.0f / 4096.0f;
    }
    else
    {
        as5600_i2c_ok = 0;
        as5600_hal_status =
            static_cast<uint32_t>(encoder.getLastStatus());
        return;
    }

    as5600_i2c_ok = 1;
    as5600_hal_status =
        static_cast<uint32_t>(encoder.getLastStatus());

    uint8_t agc_raw = 0;

    if (encoder.readAGC(agc_raw))
    {
        as5600_reg_agc = static_cast<float>(agc_raw);
    }
    else
    {
        as5600_reg_agc = -1.0f;
    }
}