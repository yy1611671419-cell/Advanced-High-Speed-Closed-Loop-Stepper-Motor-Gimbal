#include "gimbal_ctrl.h"
#include "can_motor.h"
#include <math.h>

extern CANMotor_t g_motor_pan;
extern CANMotor_t g_motor_tilt;

void Gimbal_Init(Gimbal_t *g)
{
    g->motors[GIMBAL_PAN_AXIS].addr = GIMBAL_PAN_ADDR;
    g->motors[GIMBAL_TILT_AXIS].addr = GIMBAL_TILT_ADDR;
    g->motors[GIMBAL_PAN_AXIS].current_angle = 0;
    g->motors[GIMBAL_TILT_AXIS].current_angle = 0;
    g->motors[GIMBAL_PAN_AXIS].target_angle = 0;
    g->motors[GIMBAL_TILT_AXIS].target_angle = 0;

    g->state = SYS_INIT;
    g->homing = 0;
    g->estop = 0;
    g->pan_speed = DEFAULT_MAX_SPEED;
    g->tilt_speed = DEFAULT_MAX_SPEED;
    g->pan_accel = DEFAULT_ACCEL;
    g->tilt_accel = DEFAULT_ACCEL;
    g->tick = 0;
}

float Gimbal_ClampAngle(uint8_t axis, float a)
{
    if (axis == GIMBAL_TILT_AXIS) {
        if (a < TILT_MIN_DEG) a = TILT_MIN_DEG;
        if (a > TILT_MAX_DEG) a = TILT_MAX_DEG;
    }
    return a;
}

void Gimbal_MoveTo(Gimbal_t *g, uint8_t axis, float angle)
{
    if (g->estop) return;
    if (axis >= GIMBAL_AXIS_COUNT) return;

    angle = Gimbal_ClampAngle(axis, angle);
    g->motors[axis].target_angle = angle;

    CANMotor_t *m = (axis == GIMBAL_PAN_AXIS) ? &g_motor_pan : &g_motor_tilt;
    float speed = (axis == GIMBAL_PAN_AXIS) ? g->pan_speed : g->tilt_speed;
    float accel = (axis == GIMBAL_PAN_AXIS) ? g->pan_accel : g->tilt_accel;

    CANMotor_Position(m, angle, speed, accel, SYNC_IMMEDIATE);
}

void Gimbal_Stop(Gimbal_t *g)
{
    CANMotor_Stop(&g_motor_pan);
    CANMotor_Stop(&g_motor_tilt);
}

void Gimbal_EStop(Gimbal_t *g)
{
    g->estop = 1;
    CANMotor_Stop(&g_motor_pan);
    CANMotor_Stop(&g_motor_tilt);
    g->state = SYS_DISABLED;
}

void Gimbal_Enable(Gimbal_t *g, uint8_t axis, uint8_t en)
{
    if (axis >= GIMBAL_AXIS_COUNT) return;

    CANMotor_t *m = (axis == GIMBAL_PAN_AXIS) ? &g_motor_pan : &g_motor_tilt;
    CANMotor_Enable(m, en);
    g->motors[axis].enabled = en;
}

void Gimbal_Home(Gimbal_t *g, uint8_t axis, uint8_t mode)
{
    if (axis >= GIMBAL_AXIS_COUNT) return;

    CANMotor_t *m = (axis == GIMBAL_PAN_AXIS) ? &g_motor_pan : &g_motor_tilt;
    CANMotor_StartHome(m, mode);
    g->homing = 1;
    g->state = SYS_HOMING;
}

void Gimbal_Update(Gimbal_t *g)
{
    g->tick++;

    CANMotor_ReadPosition(&g_motor_pan);
    CANMotor_ReadPosition(&g_motor_tilt);

    if (g_motor_pan.rx_ready) {
        CANMotor_ProcessResponse(&g_motor_pan, &g->motors[GIMBAL_PAN_AXIS]);
    }
    if (g_motor_tilt.rx_ready) {
        CANMotor_ProcessResponse(&g_motor_tilt, &g->motors[GIMBAL_TILT_AXIS]);
    }

    if (g->motors[GIMBAL_PAN_AXIS].enabled && g->motors[GIMBAL_TILT_AXIS].enabled) {
        if (g->state == SYS_INIT || g->state == SYS_DISABLED) {
            g->state = SYS_READY;
        }
    }

    if (g->state == SYS_READY || g->state == SYS_RUNNING) {
        g->state = SYS_RUNNING;
    }
}
