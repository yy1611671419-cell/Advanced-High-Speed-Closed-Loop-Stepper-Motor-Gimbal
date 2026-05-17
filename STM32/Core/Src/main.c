#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern void Tasks_StartAll(void);

void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_I2C1_Init();
    MX_USART1_Init();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    Tasks_StartAll();

    while (1) {
    }
}
