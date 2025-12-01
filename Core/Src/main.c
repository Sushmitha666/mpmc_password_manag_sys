/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Keypad + OLED + LED example
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1306.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* Keypad pins (STM32 pin mapping) */
#define ROW1_PIN    GPIO_PIN_0   // PA0
#define ROW2_PIN    GPIO_PIN_1   // PA1
#define ROW3_PIN    GPIO_PIN_2   // PA2
#define ROW4_PIN    GPIO_PIN_3   // PA3

#define COL1_PIN    GPIO_PIN_4   // PA4
#define COL2_PIN    GPIO_PIN_5   // PA5
#define COL3_PIN    GPIO_PIN_6   // PA6

/* LED pin */
#define LED_PIN     GPIO_PIN_0   // PB0

/* Private helper prototypes */
char keypad_scan_once(void);
char keypad_getkey_debounced(void);

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
    /* MCU init */
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();

    /* LED init */
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = LED_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOB, LED_PIN, GPIO_PIN_RESET);
    }

    /* OLED init */
    ssd1306_Init();
    ssd1306_Fill(0x00);
    ssd1306_UpdateScreen();      // blank at start

    /* -------------------- MAIN LOOP -------------------- */
    while (1)
    {
        char key = keypad_getkey_debounced();

        /* If a key is pressed */
        if (key != 0)
        {
            HAL_GPIO_WritePin(GPIOB, LED_PIN, GPIO_PIN_SET); // LED ON

            ssd1306_Fill(0x00);
            ssd1306_SetCursor(0, 0);
            ssd1306_WriteString("hello");
            ssd1306_UpdateScreen();

            HAL_Delay(1000);
            continue;
        }

        /* If NO key is pressed */
        HAL_GPIO_WritePin(GPIOB, LED_PIN, GPIO_PIN_RESET); // LED OFF

        ssd1306_Fill(0x00);   // blank screen
        ssd1306_UpdateScreen();

        HAL_Delay(1000);
    }
}

/* ======================= KEYPAD HELPERS ======================= */

char keypad_scan_once(void)
{
    const char keymap[4][3] = {
        {'1','2','3'},
        {'4','5','6'},
        {'7','8','9'},
        {'*','0','#'}
    };

    HAL_GPIO_WritePin(GPIOA, ROW1_PIN|ROW2_PIN|ROW3_PIN|ROW4_PIN, GPIO_PIN_SET);

    const uint16_t rows[4] = { ROW1_PIN, ROW2_PIN, ROW3_PIN, ROW4_PIN };
    const uint16_t cols[3] = { COL1_PIN, COL2_PIN, COL3_PIN };

    for (int r = 0; r < 4; ++r)
    {
        HAL_GPIO_WritePin(GPIOA, rows[r], GPIO_PIN_RESET);
        HAL_Delay(1);

        if (HAL_GPIO_ReadPin(GPIOA, cols[0]) == GPIO_PIN_RESET) {
            HAL_GPIO_WritePin(GPIOA, rows[r], GPIO_PIN_SET);
            return keymap[r][0];
        }
        if (HAL_GPIO_ReadPin(GPIOA, cols[1]) == GPIO_PIN_RESET) {
            HAL_GPIO_WritePin(GPIOA, rows[r], GPIO_PIN_SET);
            return keymap[r][1];
        }
        if (HAL_GPIO_ReadPin(GPIOA, cols[2]) == GPIO_PIN_RESET) {
            HAL_GPIO_WritePin(GPIOA, rows[r], GPIO_PIN_SET);
            return keymap[r][2];
        }

        HAL_GPIO_WritePin(GPIOA, rows[r], GPIO_PIN_SET);
    }

    return 0;
}

char keypad_getkey_debounced(void)
{
    char k = keypad_scan_once();
    if (k == 0) return 0;

    HAL_Delay(20);
    char k2 = keypad_scan_once();
    if (k2 == k)
    {
        while (keypad_scan_once() == k) {
            HAL_Delay(20);
        }
        return k;
    }
    return 0;
}

/* ===================== SYSTEM CONFIG ====================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = ROW1_PIN|ROW2_PIN|ROW3_PIN|ROW4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, ROW1_PIN|ROW2_PIN|ROW3_PIN|ROW4_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = COL1_PIN|COL2_PIN|COL3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
