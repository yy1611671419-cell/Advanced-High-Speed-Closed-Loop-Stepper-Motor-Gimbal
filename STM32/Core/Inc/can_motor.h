#ifndef __CAN_MOTOR_H
#define __CAN_MOTOR_H

#include "stm32h7xx_hal.h"
#include "protocol.h"

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t addr;
    volatile uint8_t rx_ready;
    uint8_t rx_buf[16];
    uint8_t rx_len;
    uint32_t rx_id;
} CANMotor_t;

void CANMotor_Init(CANMotor_t *m, FDCAN_HandleTypeDef *hfdcan, uint8_t addr);
void CANMotor_Enable(CANMotor_t *m, uint8_t en);
void CANMotor_Position(CANMotor_t *m, float angle, float speed, float accel, uint8_t sync);
void CANMotor_PositionBuffered(CANMotor_t *m, float angle, float speed, float accel);
void CANMotor_Stop(CANMotor_t *m);
void CANMotor_SyncTrigger(void);
void CANMotor_ReadPosition(CANMotor_t *m);
void CANMotor_ReadSpeed(CANMotor_t *m);
void CANMotor_ReadStatus(CANMotor_t *m);
void CANMotor_ReadPosError(CANMotor_t *m);
void CANMotor_SetZero(CANMotor_t *m, uint8_t save);
void CANMotor_StartHome(CANMotor_t *m, uint8_t mode);
void CANMotor_AbortHome(CANMotor_t *m);
void CANMotor_ReadHomeStatus(CANMotor_t *m);
void CANMotor_ProcessResponse(CANMotor_t *m, MotorState_t *state);

void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#endif
