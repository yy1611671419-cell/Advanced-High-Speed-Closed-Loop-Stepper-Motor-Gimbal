#ifndef __GIMBAL_CTRL_H
#define __GIMBAL_CTRL_H

#include "stm32h7xx_hal.h"
#include "protocol.h"

#define TILT_MIN_DEG  -135.0f
#define TILT_MAX_DEG   135.0f
#define PAN_MIN_DEG   -180.0f
#define PAN_MAX_DEG    180.0f

#define DEFAULT_MAX_SPEED  80.0f
#define DEFAULT_ACCEL      200.0f

typedef struct {
    MotorState_t motors[GIMBAL_AXIS_COUNT];
    IMUState_t imu[GIMBAL_AXIS_COUNT];
    SystemState_t state;
    uint8_t homing;
    uint8_t estop;
    float pan_speed;
    float tilt_speed;
    float pan_accel;
    float tilt_accel;
    uint32_t tick;
} Gimbal_t;

void Gimbal_Init(Gimbal_t *g);
void Gimbal_Update(Gimbal_t *g);
void Gimbal_MoveTo(Gimbal_t *g, uint8_t axis, float angle);
void Gimbal_Stop(Gimbal_t *g);
void Gimbal_EStop(Gimbal_t *g);
void Gimbal_Enable(Gimbal_t *g, uint8_t axis, uint8_t en);
void Gimbal_Home(Gimbal_t *g, uint8_t axis, uint8_t mode);
float Gimbal_ClampAngle(uint8_t axis, float a);

#endif
