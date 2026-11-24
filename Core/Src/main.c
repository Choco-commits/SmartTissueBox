/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "oled.h"
#include "FreeRTOS.h"   
#include "task.h"    
#include "queue.h"
/* Face bitmaps (defined in face/cat_happy.c, cat-sleep.c, cat-cry.c) */
extern const uint8_t face_happy[];
extern const uint8_t face_sleep[];
extern const uint8_t face_cry[];


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- Servo control parameters (TIM3, PSC=7/ARR=19999 -> 50 Hz; CCR = deg/180*2000 + 500) ---- */
#define SERVO_ANGLE_SIT     0      /* sit = 0 deg */
#define SERVO_ANGLE_STAND   90     /* stand = 90 deg */
#define SERVO_CCR(deg)  ((uint16_t)((deg) * 2000 / 180 + 500))

#define SERVO_STEP_MS        10    /* sit/stand slow motion: 1 deg per step, hold 10 ms */
#define SERVO_STEP_DELAY_MS  200   /* walk: how long to hold each half-step */
#define SERVO_STEP_TIMES     10     /* full box (>=80%): walk forward 10 steps */

#define SERVO_MIRROR_RIGHT   1     /* right servo mirrored (=1 is correct, verified by 'in sync' test) */

/* Ultrasonic calibration (meters) */
#define DIST_FULL_M    0.02f       /* full: 2 cm */
#define DIST_EMPTY_M   0.065f      /* empty: 6.5 cm */

/* Trigger thresholds (percent) */
#define TH_WALK    75.0f           /* >=75%: walk forward */
#define TH_SLEEP   30.0f           /* <30%: sleep face + reminder */
#define TH_CRY 		 5.0f

/* Measurement cadence */
#define MEASURE_PERIOD_MS  200     /* sample every 200 ms (real-time) */
#define MEDIAN_WINDOW      7       /* sliding-median window: last N readings */
#define DISPLAY_STEP_PCT   5       /* round displayed percent to this step (5 -> 0/5/10/.../100) */
#define DIST_MAX_VALID_M   0.10f   /* discard readings beyond this (spurious echo reflections) */

/* Task parameters -----------------------------------------------------------*/
#define START_TASK_PRIO	0
#define START_STK_SIZE  256

#define CONTROL_TASK_PRIO	3
#define CONTROL_STK_SIZE  256

#define BUTTON_TASK_PRIO	3
#define BUTTON_STK_SIZE   256

#define DETECT_TASK_PRIO	2
#define DETECT_STK_SIZE   256

#define SERVO_TASK_PRIO	1
#define SERVO_STK_SIZE  256

#define OLED_TASK_PRIO	0
#define OLED_STK_SIZE   256


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
OLED_TypeDef OLED;
/* TASK Variables ------------------------------------------------------------*/
TaskHandle_t StartTask_Handler;
TaskHandle_t ControlTask_Handler;
TaskHandle_t ButtonTask_Handler;
TaskHandle_t DetectTask_Handler;
TaskHandle_t ServoTask_Handler;
TaskHandle_t OLEDTask_Handler;
/* Queue Variables -----------------------------------------------------------*/
QueueHandle_t g_percentQueue;

typedef enum { SERVO_NONE=0, SERVO_WALK=1, SERVO_SLEEP=2, SERVO_CRY=3 } ServoAction_t;
// static float percent,distance; not a good method to use in rtos

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* Task function prototypes --------------------------------------------------*/
void start_task(void *pvParameters);
void control_task(void *pvParameters);
void button_task(void *pvParameters);
void detect_task(void *pvParameters);
void servo_task(void *pvParameters);
void oled_task(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int i2c_write_bytes(uint8_t addr, const uint8_t *pdata, uint16_t size)
{
	if (HAL_I2C_Master_Transmit(&hi2c1, addr, (uint8_t *)pdata, size, 100) != HAL_OK)
	{
		return -1;
	}
	return 0;
}

void My_OLEDScreen_Init(void)
{
	OLED_InitTypeDef OLED_InitStruct;
	OLED_InitStruct.i2c_write_cb=i2c_write_bytes;
	HAL_Delay(5);
	OLED_Init(&OLED,&OLED_InitStruct);
}

/* standby: all white (clears garbage screen on power-up / stop) */
static void OLED_ShowWhite(void)
{
    OLED_Clear(&OLED);
    OLED_SetPen(&OLED, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(&OLED, BRUSH_WHITE);
    OLED_SetCursor(&OLED, 0, 0);
    OLED_DrawRect(&OLED, 128, 64);
    OLED_SendBuffer(&OLED);
}

/* distance -> tissue percent (linear calibration, 0~100) */
static float Tissue_Percent(float distance)
{
    if (distance <= DIST_FULL_M)  return 100.0f;
    if (distance >= DIST_EMPTY_M) return 0.0f;
    return (DIST_EMPTY_M - distance) / (DIST_EMPTY_M - DIST_FULL_M) * 100.0f;
}

/* set both servo angles at once (left = CH1, right = CH2) */
static void Servo_SetDegrees(int leftDeg, int rightDeg)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_CCR(leftDeg));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
        SERVO_MIRROR_RIGHT ? SERVO_CCR(180 - rightDeg) : SERVO_CCR(rightDeg));
}

/* move gradually from fromDeg to toDeg (increment + delay) */
static void Servo_MoveTo(int fromDeg, int toDeg)
{
    int step = (toDeg >= fromDeg) ? 1 : -1;
    for (int a = fromDeg; a != toDeg + step; a += step)
    {
        Servo_SetDegrees(a, a);
        vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_MS));
    }
}



void OLED_ShowLevel(OLED_TypeDef *OLED, float distance)
{
	uint8_t percent = (uint8_t)Tissue_Percent(distance);


	OLED_Clear(OLED);

	// distance text
	OLED_SetCursor(OLED, 0, OLED_GetFontHeight(OLED));
	OLED_Printf(OLED, "Distance: %.2f m", distance);

	// progress bar frame
	OLED_SetPen(OLED, PEN_COLOR_WHITE, 1);
	OLED_SetBrush(OLED, BRUSH_TRANSPARENT);
	OLED_SetCursor(OLED, 14, 20);
	OLED_DrawRect(OLED, 100, 12);

	// progress bar fill
	if (percent > 0)
	{
		uint16_t fillW = (uint16_t)((100 - 2) * percent / 100);
		OLED_SetPen(OLED, PEN_COLOR_TRANSPARENT, 1);
		OLED_SetBrush(OLED, BRUSH_WHITE);
		OLED_SetCursor(OLED, 15, 21);
		OLED_DrawRect(OLED, fillW, 10);
	}

	// percent label (centered)
	char pct[8];
	snprintf(pct, sizeof(pct), "%d%%", percent);
	OLED_SetPen(OLED, PEN_COLOR_WHITE, 1);
	OLED_SetBrush(OLED, BRUSH_TRANSPARENT);
	OLED_SetCursor(OLED, (int16_t)((OLED_GetScreenWidth(OLED) - OLED_GetStrWidth(OLED, pct)) / 2), 48);
	OLED_DrawString(OLED, pct);
}
/* show tissue level (0~100): a progress bar plus the centered percentage */
static void OLED_ShowPercent(OLED_TypeDef *OLED, uint8_t percent)
{
    OLED_Clear(OLED);

    /* title */
    OLED_SetPen(OLED, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(OLED, BRUSH_TRANSPARENT);
    OLED_SetCursor(OLED, 0, 0);
    OLED_DrawString(OLED, "Tissue Level");

    /* progress bar frame */
    OLED_SetCursor(OLED, 14, 24);
    OLED_DrawRect(OLED, 100, 12);

    /* progress bar fill */
    if (percent > 0)
    {
        uint16_t fillW = (uint16_t)((100 - 2) * percent / 100);
        OLED_SetPen(OLED, PEN_COLOR_TRANSPARENT, 1);
        OLED_SetBrush(OLED, BRUSH_WHITE);
        OLED_SetCursor(OLED, 15, 25);
        OLED_DrawRect(OLED, fillW, 10);
    }

    /* percent number, centered */
    char pct[8];
    snprintf(pct, sizeof(pct), "%d%%", percent);
    OLED_SetPen(OLED, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(OLED, BRUSH_TRANSPARENT);
    OLED_SetCursor(OLED, (int16_t)((OLED_GetScreenWidth(OLED) - OLED_GetStrWidth(OLED, pct)) / 2), 44);
    OLED_DrawString(OLED, pct);
}

/* sit / stand */
static void Servo_Sit(void)   { Servo_SetDegrees(SERVO_ANGLE_SIT,   SERVO_ANGLE_SIT); }


/* walk (forward): the two legs must be opposite; we pass 'opposite logical angles' here, and with SERVO_MIRROR_RIGHT=1 they become
   the same physical angle (45,45)/(135,135), i.e. legs alternate; your earlier (45,45)/(135,135) was the root cause of both legs swinging in sync */
static void Servo_Step(uint8_t steps)
{
    for (uint8_t s = 0; s < steps; s++)
    {
        Servo_SetDegrees(45, 135);      /* left forward, right back */
        vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_DELAY_MS));
        Servo_SetDegrees(135, 45);      /* swap */
        vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_DELAY_MS));
    }
    Servo_Sit();                        /* sit down after walking */
}

/* ===== Faces ===== */
static void Show_Face(const uint8_t *bitmap)
{
    OLED_Clear(&OLED);
    OLED_SetCursor(&OLED, 0, 0);
    OLED_DrawBitmap(&OLED, 128, 64, bitmap);
    OLED_SendBuffer(&OLED);
}



static const uint8_t *Get_Face(float percent)
{
    if (percent >= TH_SLEEP) return face_happy;   /* >=40%: happy */
    if (percent >= TH_CRY)     return face_sleep;   /* 1~39%: sleep */
    return face_cry;                              /* 0%: cry */
}

/* ===== Button (internal pull-up, pressed = low), debounced, one report per press ===== */
static uint8_t Button_IsPressed(void)
{
    static uint8_t last = 1;
    uint8_t now = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
    if (now == 0 && last == 1)
    {
        HAL_Delay(30);                            /* debounce */
        if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0)
        {
            last = 0;
            return 1;
        }
    }
    last = now;
    return 0;
}

/* single ultrasonic measurement; returns 1 on success and writes the distance to *pDistance */
static uint8_t Read_Once(float *pDistance)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC2);
    HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    for (volatile uint32_t i = 0; i < 80; i++);   /* ~50 us trigger pulse (>10 us min) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    uint8_t success = 0;
    uint32_t expire = HAL_GetTick() + 50;
    while (HAL_GetTick() < expire)
    {
        if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1) &&
            __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC2)) { success = 1; break; }
    }
    HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_2);
    if (!success) return 0;

    uint16_t ccr1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    uint16_t ccr2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    *pDistance = 340.0f * (ccr2 - ccr1) * 1e-6f / 2.0f;
    return 1;
}



/* ===== State ===== */
static uint8_t Running   = 0;   /* 0 = stopped, 1 = running */
static uint8_t Ran_Happy = 0;   /* whether the >=75% walk-forward has run */
static uint8_t Ran_Sleep = 0;   /* whether the <30% reminder has run */
static uint8_t Ran_Cry   = 0;   /* whether the <5% reminder has run */

/* Task Function----------------------------------------------------*/
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();
	//create queue
	g_percentQueue = xQueueCreate(2, sizeof(float));
	//create control task
	xTaskCreate(control_task,"control_task",CONTROL_STK_SIZE,NULL,CONTROL_TASK_PRIO,&ControlTask_Handler);
	xTaskCreate(button_task,"button_task",BUTTON_STK_SIZE,NULL,BUTTON_TASK_PRIO,&ButtonTask_Handler);
	xTaskCreate(detect_task,"detect_task",DETECT_STK_SIZE,NULL,DETECT_TASK_PRIO,&DetectTask_Handler);
	xTaskCreate(servo_task,"servo_task",SERVO_STK_SIZE,NULL,SERVO_TASK_PRIO,&ServoTask_Handler);
	xTaskCreate(oled_task,"oled_task",OLED_STK_SIZE,NULL,OLED_TASK_PRIO,&OLEDTask_Handler);
	taskEXIT_CRITICAL();
	vTaskDelete(NULL);
}

void detect_task(void *pvParameters)
{
	float hist[MEDIAN_WINDOW];
	uint8_t n = 0, idx = 0;
	int8_t lastSent = -1;   /* -1 = nothing sent yet */

	for(;;)
	{
		float distance = 0;
		if (Read_Once(&distance) && distance <= DIST_MAX_VALID_M)
		{
			/* push into the sliding window */
			hist[idx] = distance;
			idx = (uint8_t)((idx + 1) % MEDIAN_WINDOW);
			if (n < MEDIAN_WINDOW) n++;

			/* median of the last n readings */
			float s[MEDIAN_WINDOW];
			for (uint8_t i = 0; i < n; i++) s[i] = hist[i];
			for (uint8_t i = 0; i + 1 < n; i++)
				for (uint8_t j = i + 1; j < n; j++)
					if (s[j] < s[i]) { float t = s[i]; s[i] = s[j]; s[j] = t; }

			/* percent rounded to DISPLAY_STEP_PCT, so 1~3% jitter doesn't flicker the display */
			float p = Tissue_Percent(s[n / 2]);
			int q = (int)(p / DISPLAY_STEP_PCT + 0.5f) * DISPLAY_STEP_PCT;
			if (q > 100) q = 100;
			if (q < 0)   q = 0;

			/* only send when the level actually changed */
			if (q != lastSent)
			{
				lastSent = q;
				float percent = (float)q;
				xQueueSend(g_percentQueue, &percent, portMAX_DELAY);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(MEASURE_PERIOD_MS));
	}
}
void button_task(void *pvParameters) {
    for (;;) {
        if (Button_IsPressed()) {
            xTaskNotifyGive(ControlTask_Handler);   // send notify to control task
        }
        vTaskDelay(pdMS_TO_TICKS(10));              // 10ms 
    }
}

/* evaluate a level (0~100): fire the one-shot servo action and show it on the OLED */
static void Apply_Level(float percent)
{
	ServoAction_t act = SERVO_NONE;
	if      (percent >= TH_WALK  && !Ran_Happy) { act = SERVO_WALK;  Ran_Happy = 1; }
	else if (percent < TH_CRY    && !Ran_Cry)    { act = SERVO_CRY;   Ran_Cry   = 1; }
	else if (percent < TH_SLEEP  && !Ran_Sleep)  { act = SERVO_SLEEP; Ran_Sleep = 1; }

	xTaskNotify(OLEDTask_Handler, (uint32_t)percent, eSetValueWithOverwrite);
	xTaskNotify(ServoTask_Handler, (uint32_t)act, eSetValueWithOverwrite);
}

void control_task(void *pvParameters)
{
	float currentPercent = -1.0f;   /* last received level, -1 = none yet */

	for(;;)
	{
		/* on / off toggle */
		if (ulTaskNotifyTake(pdTRUE, 0))
		{
			Running = !Running;
			if (Running)
			{
				/* reset one-shot flags, and re-show the last level immediately */
				Ran_Happy = Ran_Cry = Ran_Sleep = 0;
				if (currentPercent >= 0.0f) Apply_Level(currentPercent);
			}
			else
			{
				xTaskNotify(ServoTask_Handler, SERVO_NONE, eSetValueWithOverwrite);
				xTaskNotify(OLEDTask_Handler, 0xFF, eSetValueWithOverwrite);
			}
		}

		/* receive a new level (detect only sends when it changed) */
		float percent;
		if (xQueueReceive(g_percentQueue, &percent, pdMS_TO_TICKS(10)) == pdTRUE)
		{
			currentPercent = percent;
			if (Running) Apply_Level(percent);
		}
	}
}
void servo_task(void *pvParameters)
{
	
	// reseive notify from control task x=ultasknotifytake(),if x== ,switch case?
	for(;;)
	{
		uint32_t val;
		if(xTaskNotifyWait(0,0xFFFFFFFF,&val,portMAX_DELAY))
		{
			//if use switch ,the parameters only can be int type. how can i transfer
			ServoAction_t act=(ServoAction_t) val;
			switch (act)
			{
				case SERVO_WALK:
					Servo_Step(SERVO_STEP_TIMES);
					break;
				case SERVO_CRY:
					for (int i = 0; i < 5; i++)
					{
						Servo_MoveTo(SERVO_ANGLE_SIT, SERVO_ANGLE_STAND);
						Servo_MoveTo(SERVO_ANGLE_STAND, SERVO_ANGLE_SIT);
					}
					break;
				case SERVO_SLEEP:
					Servo_MoveTo(SERVO_ANGLE_SIT, SERVO_ANGLE_STAND);
					Servo_MoveTo(SERVO_ANGLE_STAND, SERVO_ANGLE_SIT);
					//need send something to control? where to delay?
					break;
				case SERVO_NONE:
					break;
				default:
					break;
				}
			
			}
		
		
	 }
	
}

void oled_task(void *pvParameters)
{
	OLED_ShowWhite();   /* boot: white screen */

	for(;;)
	{
		uint32_t val;
		if (xTaskNotifyWait(0, 0xFFFFFFFF, &val, portMAX_DELAY))
		{
			if (val == 0xFF)
			{
				OLED_ShowWhite();   /* off */
			}
			else
			{
				uint8_t percentInt = (uint8_t)val;

				/* show the new percent for 3 s, then keep that level's face */
				OLED_ShowPercent(&OLED, percentInt);
				OLED_SendBuffer(&OLED);
				vTaskDelay(pdMS_TO_TICKS(3000));

				Show_Face(Get_Face((float)percentInt));
			}
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* may be the reason why the sys behavior wierd*/
	HAL_Delay(5);
	My_OLEDScreen_Init();
	char test[64];
	int len = snprintf(test, sizeof(test), "OK");
	HAL_UART_Transmit(&huart1, (uint8_t *)test, len, 100);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	Servo_Sit();   /* on boot, put both legs in the 'sit' position */
	// create start task
	xTaskCreate(start_task,"start_task",START_STK_SIZE,NULL,START_TASK_PRIO,&StartTask_Handler);
	vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
