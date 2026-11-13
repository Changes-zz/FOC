#include "AS5600.h"

AS5600::AS5600(I2C_HandleTypeDef* hi2c)
    : hi2c_(hi2c),
      lastStatus_(HAL_ERROR)
{
}

bool AS5600::init()
{
    return isConnected();
}

bool AS5600::isConnected()
{
    if (hi2c_ == nullptr)
    {
        lastStatus_ = HAL_ERROR;
        return false;
    }

    lastStatus_ = HAL_I2C_IsDeviceReady(
        hi2c_,
        DEVICE_ADDRESS,
        2,
        TIMEOUT_MS
    );

    return lastStatus_ == HAL_OK;
}

bool AS5600::readBytes(uint8_t reg,
                       uint8_t* data,
                       uint16_t length)
{
    if (hi2c_ == nullptr ||
        data == nullptr ||
        length == 0)
    {
        lastStatus_ = HAL_ERROR;
        return false;
    }

    lastStatus_ = HAL_I2C_Mem_Read(
        hi2c_,
        DEVICE_ADDRESS,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        length,
        TIMEOUT_MS
    );

    return lastStatus_ == HAL_OK;
}

bool AS5600::readRegister(uint8_t reg,
                          uint8_t& value)
{
    uint8_t data = 0;

    if (!readBytes(reg, &data, 1))
    {
        return false;
    }

    value = data;
    return true;
}

bool AS5600::read12Bit(uint8_t reg,
                       uint16_t& value)
{
    uint8_t data[2] = {0, 0};

    if (!readBytes(reg, data, 2))
    {
        return false;
    }

    value = static_cast<uint16_t>(
        ((static_cast<uint16_t>(data[0]) & 0x0F) << 8)
        | data[1]
    );

    return true;
}

bool AS5600::readRawAngle(uint16_t& angle)
{
    return read12Bit(REG_RAW_ANGLE, angle);
}

bool AS5600::readMechanicalAngle(float& angleRad)
{
    uint16_t rawAngle = 0;

    if (!readRawAngle(rawAngle))
    {
        return false;
    }

    angleRad = static_cast<float>(rawAngle)
             * TWO_PI / 4096.0f;

    return true;
}

bool AS5600::readMechanicalAngleDeg(float& angleDeg)
{
    uint16_t rawAngle = 0;

    if (!readRawAngle(rawAngle))
    {
        return false;
    }

    angleDeg = static_cast<float>(rawAngle)
             * 360.0f / 4096.0f;

    return true;
}

bool AS5600::readMagnetStatus(MagnetStatus& status)
{
    uint8_t value = 0;

    if (!readRegister(REG_STATUS, value))
    {
        return false;
    }

    MagnetStatus result{};

    result.detected = (value & 0x20) != 0;
    result.tooWeak  = (value & 0x10) != 0;
    result.tooStrong = (value & 0x08) != 0;

    status = result;
    return true;
}

bool AS5600::magnetDetected()
{
    MagnetStatus status{};

    if (!readMagnetStatus(status))
    {
        return false;
    }

    return status.detected;
}

bool AS5600::readAGC(uint8_t& agc)
{
    return readRegister(REG_AGC, agc);
}

bool AS5600::readMagnitude(uint16_t& magnitude)
{
    return read12Bit(REG_MAGNITUDE, magnitude);
}

HAL_StatusTypeDef AS5600::getLastStatus() const
{
    return lastStatus_;
}
