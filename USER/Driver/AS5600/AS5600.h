#ifndef AS5600_H
#define AS5600_H

#include <cstdint>
#include "stm32g4xx_hal.h"

class AS5600
{
public:
    static constexpr uint8_t I2C_ADDRESS = 0x36;
    static constexpr float TWO_PI = 6.283185307179586f;

    struct MagnetStatus
    {
        bool detected;
        bool tooWeak;
        bool tooStrong;
    };

    explicit AS5600(I2C_HandleTypeDef* hi2c);

    // 初始化：检测 I2C 设备是否响应
    bool init();

    // 检查设备连接
    bool isConnected();

    // 读取原始角度，范围 0~4095
    bool readRawAngle(uint16_t& angle);

    // 读取机械角度，范围 [0, 2PI)
    bool readMechanicalAngle(float& angleRad);

    // 读取机械角度，范围 [0, 360)
    bool readMechanicalAngleDeg(float& angleDeg);

    // 读取磁铁状态
    bool readMagnetStatus(MagnetStatus& status);

    // 检测磁铁是否存在
    bool magnetDetected();

    // 读取自动增益控制值
    bool readAGC(uint8_t& agc);

    // 读取磁场幅值
    bool readMagnitude(uint16_t& magnitude);

    // 最近一次 HAL I2C 操作结果
    HAL_StatusTypeDef getLastStatus() const;

private:
    I2C_HandleTypeDef* hi2c_;
    HAL_StatusTypeDef lastStatus_;

    static constexpr uint16_t DEVICE_ADDRESS =
        (I2C_ADDRESS << 1);

    static constexpr uint32_t TIMEOUT_MS = 10;

    static constexpr uint8_t REG_STATUS = 0x0B;// 状态寄存器地址
    static constexpr uint8_t REG_RAW_ANGLE = 0x0C;// 原始角度寄存器地址
    static constexpr uint8_t REG_AGC = 0x1A;// 自动增益控制寄存器地址
    static constexpr uint8_t REG_MAGNITUDE = 0x1B;// 磁场幅值寄存器地址

    bool readBytes(uint8_t reg,
                   uint8_t* data,
                   uint16_t length);

    bool readRegister(uint8_t reg, uint8_t& value);

    bool read12Bit(uint8_t reg, uint16_t& value);
};



#endif // AS5600_H