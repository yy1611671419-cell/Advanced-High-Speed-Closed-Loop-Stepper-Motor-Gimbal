#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "can_motor.h"
#include "imu_mpu6500.h"
#include "usb_comm.h"
#include "gimbal_ctrl.h"

FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;
USBD_HandleTypeDef hUsbDeviceFS;

CANMotor_t g_motor_pan;
CANMotor_t g_motor_tilt;
MPU6500_t g_imu_pan;
MPU6500_t g_imu_tilt;
USBComm_t g_usb;
Gimbal_t g_gimbal;

QueueHandle_t qCAN_Tx;
QueueHandle_t qUSB_Rx;

SemaphoreHandle_t semI2C;
SemaphoreHandle_t semCAN;
SemaphoreHandle_t semUSB;

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 10;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1 = 13;
    hfdcan1.Init.NominalTimeSeg2 = 2;
    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 1;
    hfdcan1.Init.DataTimeSeg1 = 1;
    hfdcan1.Init.DataTimeSeg2 = 1;
    hfdcan1.Init.StdFiltersNbr = 0;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10909CEC;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

void MX_USART1_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_D3PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_USART1
                                             | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_USB;

    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.PLL3.PLL3M = 25;
    PeriphClkInitStruct.PLL3.PLL3N = 192;
    PeriphClkInitStruct.PLL3.PLL3P = 4;
    PeriphClkInitStruct.PLL3.PLL3Q = 4;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }

    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();
}

static void Task_CAN(void *param)
{
    (void)param;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);

    CANMotor_Init(&g_motor_pan, &hfdcan1, GIMBAL_PAN_ADDR);
    CANMotor_Init(&g_motor_tilt, &hfdcan1, GIMBAL_TILT_ADDR);

    HAL_Delay(2000);

    CANMotor_Enable(&g_motor_pan, 1);
    HAL_Delay(100);
    CANMotor_Enable(&g_motor_tilt, 1);
    HAL_Delay(100);

    g_gimbal.state = SYS_READY;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        Gimbal_Update(&g_gimbal);

        MotorStatus_t pan_status;
        pan_status.axis = GIMBAL_PAN_AXIS;
        pan_status.position = g_gimbal.motors[GIMBAL_PAN_AXIS].current_angle;
        pan_status.speed = g_gimbal.motors[GIMBAL_PAN_AXIS].current_speed_rpm;
        pan_status.position_error = g_gimbal.motors[GIMBAL_PAN_AXIS].position_error;
        pan_status.status_flags = g_gimbal.motors[GIMBAL_PAN_AXIS].status_flags;
        pan_status.temperature = g_gimbal.motors[GIMBAL_PAN_AXIS].temperature;
        USBComm_SendMotorStatus(&g_usb, &pan_status);

        MotorStatus_t tilt_status;
        tilt_status.axis = GIMBAL_TILT_AXIS;
        tilt_status.position = g_gimbal.motors[GIMBAL_TILT_AXIS].current_angle;
        tilt_status.speed = g_gimbal.motors[GIMBAL_TILT_AXIS].current_speed_rpm;
        tilt_status.position_error = g_gimbal.motors[GIMBAL_TILT_AXIS].position_error;
        tilt_status.status_flags = g_gimbal.motors[GIMBAL_TILT_AXIS].status_flags;
        tilt_status.temperature = g_gimbal.motors[GIMBAL_TILT_AXIS].temperature;
        USBComm_SendMotorStatus(&g_usb, &tilt_status);
    }
}

static void Task_IMU(void *param)
{
    (void)param;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);

    MPU6500_Init(&g_imu_pan, &hi2c1, MPU_ADDR_LO);
    MPU6500_Init(&g_imu_tilt, &hi2c1, MPU_ADDR_HI);

    HAL_Delay(200);
    MPU6500_Calibrate(&g_imu_pan, 500);
    MPU6500_Calibrate(&g_imu_tilt, 500);

    g_gimbal.imu[GIMBAL_PAN_AXIS].ready = 1;
    g_gimbal.imu[GIMBAL_TILT_AXIS].ready = 1;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        if (xSemaphoreTake(semI2C, pdMS_TO_TICKS(5)) == pdTRUE) {
            MPU6500_ReadAll(&g_imu_pan);
            MPU6500_ReadAll(&g_imu_tilt);
            xSemaphoreGive(semI2C);
        }

        IMUData_t imu_data;
        uint32_t now = HAL_GetTick();

        imu_data.axis = GIMBAL_PAN_AXIS;
        memcpy(imu_data.accel, g_imu_pan.accel, sizeof(float) * 3);
        memcpy(imu_data.gyro, g_imu_pan.gyro, sizeof(float) * 3);
        imu_data.temperature = g_imu_pan.temperature;
        imu_data.timestamp = now;
        USBComm_SendIMU(&g_usb, &imu_data);

        imu_data.axis = GIMBAL_TILT_AXIS;
        memcpy(imu_data.accel, g_imu_tilt.accel, sizeof(float) * 3);
        memcpy(imu_data.gyro, g_imu_tilt.gyro, sizeof(float) * 3);
        imu_data.temperature = g_imu_tilt.temperature;
        imu_data.timestamp = now;
        USBComm_SendIMU(&g_usb, &imu_data);
    }
}

static void Task_USB(void *param)
{
    (void)param;

    USBComm_Init(&g_usb);

    for (;;) {
        if (g_usb.frame_ready) {
            g_usb.frame_ready = 0;
            uint8_t cmd = g_usb.cur_cmd;
            uint8_t seq = g_usb.cur_seq;
            uint8_t payload[56];
            uint8_t len = g_usb.payload_len;
            memcpy(payload, g_usb.payload, len);

            switch (cmd) {
            case USB_CMD_POS_CMD: {
                if (len >= 10) {
                    uint8_t axis = payload[0];
                    int32_t pos_raw = ((int32_t)payload[1] << 24) |
                                     ((int32_t)payload[2] << 16) |
                                     ((int32_t)payload[3] << 8)  |
                                     payload[4];
                    float angle = (float)pos_raw / 10.0f;
                    Gimbal_MoveTo(&g_gimbal, axis, angle);
                    USBComm_SendAck(&g_usb, cmd, 0x00);
                }
                break;
            }
            case USB_CMD_ESTOP:
                Gimbal_EStop(&g_gimbal);
                USBComm_SendAck(&g_usb, cmd, 0x00);
                break;

            case USB_CMD_ENABLE: {
                if (len >= 2) {
                    Gimbal_Enable(&g_gimbal, payload[0], payload[1]);
                    USBComm_SendAck(&g_usb, cmd, 0x00);
                }
                break;
            }
            case USB_CMD_HOME: {
                if (len >= 2) {
                    Gimbal_Home(&g_gimbal, payload[0], payload[1]);
                    USBComm_SendAck(&g_usb, cmd, 0x00);
                }
                break;
            }
            case USB_CMD_HEARTBEAT: {
                uint8_t hb_resp[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
                USBComm_SendFrame(&g_usb, 0x30, seq, hb_resp, 6);
                break;
            }
            default:
                USBComm_SendAck(&g_usb, cmd, 0xE2);
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void Task_Monitor(void *param)
{
    (void)param;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);

        if (g_gimbal.state == SYS_ERROR) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        }

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
}

void Tasks_StartAll(void)
{
    semI2C = xSemaphoreCreateMutex();
    semCAN = xSemaphoreCreateMutex();
    semUSB = xSemaphoreCreateMutex();

    qCAN_Tx = xQueueCreate(5, 16);
    qUSB_Rx = xQueueCreate(5, 64);

    xTaskCreate(Task_CAN, "CAN", 512, NULL, 4, NULL);
    xTaskCreate(Task_IMU, "IMU", 256, NULL, 3, NULL);
    xTaskCreate(Task_USB, "USB", 1024, NULL, 4, NULL);
    xTaskCreate(Task_Monitor, "Mon", 256, NULL, 1, NULL);

    vTaskStartScheduler();
}
