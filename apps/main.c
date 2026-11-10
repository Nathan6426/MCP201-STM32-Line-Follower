
/* USER CODE BEGIN Header */
/**
 * MCP201 - STM32F103C8T6 Blue Pill
 * LINE FOLLOWER - ON/OFF PWM v3.0
 *
 * LEFT MOTOR : PB6/PB7 - TIM4 CH1/CH2
 * RIGHT MOTOR: PB4/PB5 - TIM3 CH1/CH2
 *
 * S1 PA0 | S2 PA1 | S3 PA2 | S4 PA3 | S5 PA4
 *
 * BLACK = 0, WHITE = 1
 *
 * Motor commands: ONLY 0% or 65%
 * TIM3 requires partial remap PB4/PB5.
 * JTAG disabled, SWD enabled.
 */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>

/* Private variables */
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PD */

#define RUN_MODE            2

// 0 = Sensor test (motors off)
// 1 = Straight motor test
// 2 = Automatic line following

#define MOTOR_ON            65
#define MOTOR_OFF           0

#define LOOP_MS             10U
#define LOST_CONFIRM_MS     30U
#define LOST_TIMEOUT_MS     500U

#define LINE_ACTIVE         0U

/* USER CODE END PD */

/* USER CODE BEGIN PV */

volatile uint8_t sensor[5] = {1, 1, 1, 1, 1};
volatile uint8_t sensor_mask = 0;
volatile uint8_t black_count = 0;

volatile float line_error = 0.0f;

volatile int16_t target_left = 0;
volatile int16_t target_right = 0;
volatile int16_t actual_left = 0;
volatile int16_t actual_right = 0;

volatile int8_t last_direction = 0;

volatile uint32_t lost_since = 0;
volatile uint8_t lost_active = 0;
volatile uint8_t emergency_latched = 0;

volatile uint8_t robot_state = 0;

// robot_state:
// 0 = STOP
// 1 = TRACKING
// 2 = LOST DEBOUNCE
// 3 = SHARP TURN SEARCH
// 4 = INVALID SENSOR PATTERN
// 5 = EMERGENCY STOP
// 7 = MOTOR TEST
// 8 = SENSOR TEST

/* USER CODE END PV */

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN 0 */

static void Motor_Write(
    TIM_HandleTypeDef *htim,
    uint32_t ch_forward,
    uint32_t ch_reverse,
    int16_t speed)
{
    // Strict ON/OFF output: 0 or 65 only.
    uint32_t pwm = (speed != 0) ? 260U : 0U;

    __HAL_TIM_SET_COMPARE(htim, ch_forward, 0);
    __HAL_TIM_SET_COMPARE(htim, ch_reverse, 0);

    if (speed > 0)
    {
        __HAL_TIM_SET_COMPARE(htim, ch_forward, pwm);
    }
    else if (speed < 0)
    {
        __HAL_TIM_SET_COMPARE(htim, ch_reverse, pwm);
    }
}

static void Motors_Set(int16_t left, int16_t right)
{
    // LEFT - TIM4 PB6/PB7
    Motor_Write(
        &htim4,
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        left
    );

    // RIGHT - TIM3 PB4/PB5
    Motor_Write(
        &htim3,
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        right
    );
}

static void Motors_Update(void)
{
    actual_left =
        (target_left > 0) ? MOTOR_ON : MOTOR_OFF;

    actual_right =
        (target_right > 0) ? MOTOR_ON : MOTOR_OFF;

    Motors_Set(actual_left, actual_right);
}

static void Motors_Stop(void)
{
    target_left = MOTOR_OFF;
    target_right = MOTOR_OFF;

    actual_left = MOTOR_OFF;
    actual_right = MOTOR_OFF;

    Motors_Set(MOTOR_OFF, MOTOR_OFF);
}

static void Emergency_Stop(void)
{
    emergency_latched = 1;
    robot_state = 5;
    Motors_Stop();
}

static void Sensors_Read(void)
{
    uint32_t pins = GPIOA->IDR;

    sensor_mask = 0;
    black_count = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
        sensor[i] =
            (uint8_t)((pins >> i) & 1U);

        if (sensor[i] == LINE_ACTIVE)
        {
            sensor_mask |= (uint8_t)(1U << i);
            black_count++;
        }
    }
}

static uint8_t Sensors_Valid(void)
{
    // All black is ambiguous
    if (black_count == 5)
        return 0;

    uint8_t first = 5;
    uint8_t last = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
        if (sensor[i] == LINE_ACTIVE)
        {
            if (first == 5)
                first = i;

            last = i;
        }
    }

    if (first == 5)
        return 0;

    // Reject non-contiguous black patterns
    for (uint8_t i = first; i <= last; i++)
    {
        if (sensor[i] != LINE_ACTIVE)
            return 0;
    }

    return 1;
}


static void Line_Follow_Update(void)
{
    Sensors_Read();

    // Remember direction using OUTER sensors only.
    // BLACK = 0
    if (sensor[0] == 0 && sensor[4] != 0)
    {
        last_direction = -1; // LEFT
    }
    else if (sensor[4] == 0 && sensor[0] != 0)
    {
        last_direction = 1;  // RIGHT
    }

    // ALL WHITE: keep searching until line returns
    if (black_count == 0)
    {
        robot_state = 3;

        if (last_direction < 0)
        {
            target_left = MOTOR_OFF;
            target_right = MOTOR_ON;
        }
        else if (last_direction > 0)
        {
            target_left = MOTOR_ON;
            target_right = MOTOR_OFF;
        }
        else
        {
            // No direction recorded yet
            target_left = MOTOR_OFF;
            target_right = MOTOR_OFF;
        }

        Motors_Update();
        return;
    }

    // ALL BLACK or invalid pattern
    if (!Sensors_Valid())
    {
        robot_state = 4;
        Motors_Stop();
        return;
    }

    // NORMAL LINE FOLLOWING
    // Weighted position S1..S5: -2,-1,0,+1,+2
    static const int8_t weights[5] =
    {
        -2, -1, 0, 1, 2
    };

    int16_t sum = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
        if (sensor[i] == LINE_ACTIVE)
        {
            sum += weights[i];
        }
    }

    line_error = (float)sum / black_count;

    if (line_error < -0.01f)
    {
        target_left = MOTOR_OFF;
        target_right = MOTOR_ON;
    }
    else if (line_error > 0.01f)
    {
        target_left = MOTOR_ON;
        target_right = MOTOR_OFF;
    }
    else
    {
        target_left = MOTOR_ON;
        target_right = MOTOR_ON;
    }

    robot_state = 1;
    Motors_Update();
}


/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM4_Init();
    MX_TIM3_Init();

    /* USER CODE BEGIN 2 */

    if (HAL_TIM_PWM_Start(
        &htim3, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Start(
        &htim3, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Start(
        &htim4, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_Start(
        &htim4, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    Motors_Stop();

    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */

#if RUN_MODE == 0

        Sensors_Read();
        Motors_Stop();
        robot_state = 8;

#elif RUN_MODE == 1

        Sensors_Read();

        target_left = MOTOR_ON;
        target_right = MOTOR_ON;

        robot_state = 7;
        Motors_Update();

#elif RUN_MODE == 2

        Line_Follow_Update();

#else

        Motors_Stop();

#endif

        HAL_Delay(LOOP_MS);

        /* USER CODE END 3 */
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(
        &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
        &RCC_ClkInitStruct,
        FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/* TIM3 Initialization */
static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 399;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(
        &htim3, &sMasterConfig) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(
        &htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_ConfigChannel(
        &htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim3);
}

/* TIM4 Initialization */
static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 399;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(
        &htim4, &sMasterConfig) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(
        &htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    if (HAL_TIM_PWM_ConfigChannel(
        &htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin =
        GPIO_PIN_0 |
        GPIO_PIN_1 |
        GPIO_PIN_2 |
        GPIO_PIN_3 |
        GPIO_PIN_4;

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* Error Handler */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */

    if (__HAL_RCC_TIM3_IS_CLK_ENABLED())
    {
        TIM3->CCR1 = 0;
        TIM3->CCR2 = 0;
    }

    if (__HAL_RCC_TIM4_IS_CLK_ENABLED())
    {
        TIM4->CCR1 = 0;
        TIM4->CCR2 = 0;
    }

    __disable_irq();

    while (1)
    {
    }

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    (void)file;
    (void)line;
    /* USER CODE END 6 */
}
#endif
