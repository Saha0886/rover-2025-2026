#include "main.h"

#include <stdbool.h>
#include <string.h>
#include <dma.h>
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "../Tarantul/motor_controller.h"
#include "../Tarantul/uart_protocol.h"

#define SIZE 4
#define PACKET_SIZE 8

#define LED_GPIO_PORT GPIOC
#define LED_GPIO_PIN  GPIO_PIN_13

// Добавляем переменные для управления светодиодом
volatile uint32_t led_blink_timer = 0;
volatile bool led_should_blink = false;
volatile uint32_t system_tick = 0;

void SystemClock_Config(void);

uint8_t msg[PACKET_SIZE] = {0};
volatile bool uart_tx_ready = true;

const uint8_t core_id = 0x43;

MotorController_t m1;
MotorController_t m2;

uint8_t tx_buffer_enc_1[PACKET_SIZE] = {0};
uint8_t tx_buffer_enc_2[PACKET_SIZE] = {0};

// Функция для инициализации мигания светодиода
void StartLedBlink(void)
{
    led_should_blink = true;
    led_blink_timer = system_tick;
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET); // Включить светодиод
}

// Функция для обработки мигания светодиода (вызывается в main loop)
void HandleLedBlink(void)
{
    if (led_should_blink)
    {
        // Проверяем, прошло ли 100ms с момента включения
        if ((system_tick - led_blink_timer) >= 100)
        {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET); // Выключить светодиод
            led_should_blink = false;
        }
    }
}

// Обработчик системного тика для подсчета времени
void HAL_IncTick(void)
{
    uwTick += uwTickFreq;
    system_tick++;
}

// Короткое мигание светодиодом (оставляем для совместимости)
void BlinkLedShort(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET); // включить
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET); // выключить
}

void createEncMessage(const uint8_t CMD, int32_t count, uint8_t* tx_data)
{
    protocol msg;
    msg.core_id = core_id;
    msg.cmd = CMD;
    msg.data[0] = 0;
    msg.data[1] = 0;
    msg.data[2] = 0;
    msg.data[3] = 0;
    uint8_t bytes[SIZE] = {0};
    int_32_to_bytes(count, bytes);
    memcpy(msg.data, bytes, SIZE);
    create_msg(msg, tx_data);
}

bool validate_msg(const uint8_t* msg)
{
    if (msg[0] != 0xA5) return false;

    // Calculate checksum
    uint8_t calculated_checksum = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++)
    {
        calculated_checksum ^= msg[i];
    }

    // Check if calculated checksum matches received checksum
    if (calculated_checksum != msg[PACKET_SIZE - 1]) return false;

    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        protocol data;

        // Мигаем светодиодом при получении ЛЮБОГО пакета
        StartLedBlink();

        if (validate_msg(msg))
        {
            parse_msg(&data, msg);
            if (data.core_id == core_id)
            {
                if (data.cmd == m1.id)
                {
                    MotorController_SetSpeed(&m1, bytes_to_float(data.data));
                }
                else if (data.cmd == m2.id)
                {
                    MotorController_SetSpeed(&m2, bytes_to_float(data.data));
                }
            }
        }

        HAL_UART_Receive_DMA(&huart3, msg, PACKET_SIZE);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        uart_tx_ready = true;
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    m1.id = M1_CMD_PWM;
    m1.enc_htim = TIM1;
    m1.motor_htim = &htim3;
    m1.forward_ch = TIM_CHANNEL_4;
    m1.backward_ch = TIM_CHANNEL_3;
    m1.counter = 0;

    m2.id = M2_CMD_PWM;
    m2.enc_htim = TIM4;
    m2.motor_htim = &htim3;
    m2.forward_ch = TIM_CHANNEL_1;
    m2.backward_ch = TIM_CHANNEL_2;
    m2.counter = 0;

    // Убеждаемся, что светодиод изначально выключен
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);

    HAL_UART_Receive_DMA(&huart3, msg, PACKET_SIZE);

    while (1)
    {
        // Обрабатываем мигание светодиода
        HandleLedBlink();

        MotorController_UpdateEnc(&m1);
        MotorController_UpdateEnc(&m2);

        if (uart_tx_ready)
        {
            uart_tx_ready = false;
            createEncMessage(M1_CMD_ENC, MotorController_GetCounter(&m1), tx_buffer_enc_1);
            createEncMessage(M2_CMD_ENC, MotorController_GetCounter(&m2), tx_buffer_enc_2);
            HAL_UART_Transmit_DMA(&huart3, tx_buffer_enc_1, PACKET_SIZE);
            HAL_Delay(50);
            HAL_UART_Transmit_DMA(&huart3, tx_buffer_enc_2, PACKET_SIZE);
        }
        HAL_Delay(100);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */