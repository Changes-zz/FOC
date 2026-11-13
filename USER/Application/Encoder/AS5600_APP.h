#ifndef AS5600_APP_H
#define AS5600_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void AS5600_Init(void);
void AS5600_Update(void);

// 调试观察变量
extern volatile uint8_t as5600_i2c_ok;
extern volatile uint8_t as5600_magnet_detected;
extern volatile uint8_t as5600_magnet_too_weak;
extern volatile uint8_t as5600_magnet_too_strong;

extern volatile uint16_t as5600_raw_angle;
extern volatile float as5600_angle_deg;

extern volatile uint32_t as5600_hal_status;

extern volatile float as5600_reg_agc;
#ifdef __cplusplus
}
#endif

#endif