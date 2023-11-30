/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DTek_TLE5012B.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

osThreadId defaultTaskHandle;
osThreadId commTaskHandle;
osThreadId sensorTaskHandle;
osThreadId motorTaskHandle;
/* USER CODE BEGIN PV */

// Sensor communication
errorTypes checkError = NO_ERROR;
errorTypes checkError2 = NO_ERROR;
errorTypes checkError3 = NO_ERROR;
errorTypes checkError4 = NO_ERROR;
errorTypes checkError5 = NO_ERROR;
errorTypes checkError6 = NO_ERROR;
errorTypes checkError7 = NO_ERROR;

uint16_t errorCounter = 0;

// Joint angles
static double angle1_raw = 0.0;
static double angle2_raw = 0.0;
static double angle3_raw = 0.0;
static double angle4_raw = 0.0;
static double angle5_raw = 0.0;
static double angle6_raw = 0.0;
static double angle7_raw = 0.0;

static double angle1_temp = 0.0;
static double angle2_temp = 0.0;
static double angle3_temp = 0.0;
static double angle4_temp = 0.0;
static double angle5_temp = 0.0;
static double angle6_temp = 0.0;
static double angle7_temp = 0.0;

static double angle1 = 0.0;
static double angle2 = 0.0;
static double angle3 = 0.0;
static double angle4 = 0.0;
static double angle5 = 0.0;
static double angle6 = 0.0;
static double angle7 = 0.0;

static double offset1 = -104.25;//0.0;
static double offset2 = 32.37;//0.0;
static double offset3 = 111.05;//0.0;
static double offset4 = 11.1;//0.0;
static double offset5 = -176.17;//0.0;
static double offset6 = 95.75;//0.0;
static double offset7 = 0.0;

// Feedback
static uint8_t feedback = 0;
static uint8_t feedback_temp = 0;
static uint8_t motorPattern = 0;

// Button status
static uint8_t b1Status = 0;
static uint8_t b2Status = 0;

// USB CDC communication
char txBuf[256];
char rxBuf[256];
uint8_t receiveState = 1;
static volatile uint32_t timeStamp = 0;
static volatile uint32_t timeOutGuard = 0;
static volatile uint32_t timeOutGuardMax = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void StartCommTask(void const * argument);
void StartSensorTask(void const * argument);
void StartMotorTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  TIM1->CCR1 = 0*3600/100;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of commTask */
  osThreadDef(commTask, StartCommTask, osPriorityNormal, 0, 256);
  commTaskHandle = osThreadCreate(osThread(commTask), NULL);

  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of motorTask */
  osThreadDef(motorTask, StartMotorTask, osPriorityIdle, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS2_Pin|CS3_Pin|CS4_Pin
                          |CS5_Pin|CS6_Pin|CS7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin B2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin CS2_Pin CS3_Pin CS4_Pin
                           CS5_Pin CS6_Pin CS7_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS2_Pin|CS3_Pin|CS4_Pin
                          |CS5_Pin|CS6_Pin|CS7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief EXTI handler
  * @param GPIO_Pin: Triggered pin
  * @retval none
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch ( GPIO_Pin ) {
  case GPIO_PIN_0:
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)){
      b1Status = 0;
    }
    else {
      b1Status = 1;
    }
    break;
  case GPIO_PIN_1:
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)){
	  b2Status = 0;
	}
	else {
	  b2Status = 1;
	}
	break;
  default:
	break;
  }
}

/**
  * @brief Count characters in char array
  * @param ptr: pointer to char array
  * @retval Number of characters in array
  */
uint16_t SizeofCharArray(char *ptr)
{
  /* Local variables */
  uint16_t len = 0;

  /* Search until end char */
  while (ptr[len] != '\0') {
    len++;
  }
  return len;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(200);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */
  uint16_t Len = 0;
  /* Infinite loop */
  for(;;)
  {
	if (receiveState == 1){
	  // Dummy command
	  if ((rxBuf[0] == 'K') && (rxBuf[1] == 'A') && (rxBuf[2] == '\r')){ // KA = Keep alive (response echo)
		__ASM("NOP");
	  }
	  // Clear error counter
	  else if ((rxBuf[0] == 'C') && (rxBuf[1] == 'E') && (rxBuf[2] == 'C') && (rxBuf[3] == '\r')){
		errorCounter = 0;
		sprintf(txBuf, "OK;%s\r\n", rxBuf);
		Len = SizeofCharArray((char*)txBuf);
		CDC_Transmit_FS((uint8_t*)txBuf, Len);
	  }
	  // Read error counter
	  else if ((rxBuf[0] == 'R') && (rxBuf[1] == 'E') && (rxBuf[2] == 'C') && (rxBuf[3] == '\r')){
		sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, errorCounter);
		Len = SizeofCharArray((char*)txBuf);
		CDC_Transmit_FS((uint8_t*)txBuf, Len);
	  }
	  // Read joint offsets
	  else if ((rxBuf[0] == 'R') && (rxBuf[1] == 'O') && (rxBuf[3] == '\r')){
		if (rxBuf[2] == '1') {
			sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, (int16_t)(offset1*100));
		}
		else if (rxBuf[2] == '2') {
			sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, (int16_t)(offset2*100));
		}
		else if (rxBuf[2] == '3') {
			sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, (int16_t)(offset3*100));
		}
		else if (rxBuf[2] == '4') {
			sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, (int16_t)(offset4*100));
		}
		else if (rxBuf[2] == '5') {
			sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, (int16_t)(offset5*100));
		}
		else if (rxBuf[2] == '6') {
			sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, (int16_t)(offset6*100));
		}
		else if (rxBuf[2] == '7') {
			sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, (int16_t)(offset7*100));
		}
		else {
			sprintf(txBuf, "ERR1;%s\r\n", rxBuf);
		}
		Len = SizeofCharArray((char*)txBuf);
		CDC_Transmit_FS((uint8_t*)txBuf, Len);
	  }
	  // Set joint offsets
	  else if ((rxBuf[0] == 'S') && (rxBuf[1] == 'O') && (rxBuf[3] == '\r')){
		//ToDo: implement writing offsets
		sprintf(txBuf, "OK;%s;%s\r\n", rxBuf, "To be implemented...");
		Len = SizeofCharArray((char*)txBuf);
		CDC_Transmit_FS((uint8_t*)txBuf, Len);
	  }
	  // Read device status, returns joint angles and button status
	  else if ((rxBuf[0] == 'R') && (rxBuf[1] == 'D') && (rxBuf[2] == 'S') && (rxBuf[3] == '\r')){
		sprintf(txBuf, "OK;%s;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n", rxBuf, (int16_t)(angle1*100), (int16_t)(angle2*100), (int16_t)(angle3*100), (int16_t)(angle4*100), (int16_t)(angle5*100), (int16_t)(angle6*100), (int16_t)(angle7*100), b1Status, b2Status);
		Len = SizeofCharArray((char*)txBuf);
		CDC_Transmit_FS((uint8_t*)txBuf, Len);
	  }
	  // Read feedback status 0...100
	  else if ((rxBuf[0] == 'R') && (rxBuf[1] == 'F') && (rxBuf[2] == 'B') && (rxBuf[3] == '\r')){
		sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, feedback);
		Len = SizeofCharArray((char*)txBuf);
		CDC_Transmit_FS((uint8_t*)txBuf, Len);
	  }
	  // Set feedback status 0...100
	  else if ((rxBuf[0] == 'S') && (rxBuf[1] == 'F') && (rxBuf[2] == 'B') && (rxBuf[6] == '\r')){
		feedback_temp = (int16_t)((rxBuf[3]  - '0')*100 + (rxBuf[4]  - '0')*10 + (rxBuf[5]  - '0')*1);
		if (feedback_temp > 100) feedback_temp = 100;
		feedback = feedback_temp;
		sprintf(txBuf, "OK;%s;%d\r\n", rxBuf, feedback);
		Len = SizeofCharArray((char*)txBuf);
		CDC_Transmit_FS((uint8_t*)txBuf, Len);
	  }
	  // Unrecognized command
      else{
        sprintf(txBuf, "ERR2;%s\r\n", rxBuf);
        Len = SizeofCharArray((char*)txBuf);
        CDC_Transmit_FS((uint8_t*)txBuf, Len);
      }
      timeStamp = HAL_GetTick();
      receiveState = 0;
    }

    timeOutGuard = HAL_GetTick() - timeStamp;
    if (timeOutGuard > timeOutGuardMax){
      timeOutGuardMax = timeOutGuard;
    }

    osDelay(10);
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  SPI_CS_Disable(1);
  SPI_CS_Disable(2);
  SPI_CS_Disable(3);
  SPI_CS_Disable(4);
  SPI_CS_Disable(5);
  SPI_CS_Disable(6);
  SPI_CS_Disable(7);

  checkError = NO_ERROR;
  checkError = readBlockCRC(1);
  if (checkError != NO_ERROR) errorCounter++;
  HAL_Delay(1);

  checkError2 = NO_ERROR;
  checkError2 = readBlockCRC(2);
  if (checkError2 != NO_ERROR) errorCounter++;
  HAL_Delay(1);

  checkError3 = NO_ERROR;
  checkError3 = readBlockCRC(3);
  if (checkError3 != NO_ERROR) errorCounter++;
  HAL_Delay(1);

  checkError4 = NO_ERROR;
  checkError4 = readBlockCRC(4);
  if (checkError4 != NO_ERROR) errorCounter++;
  HAL_Delay(1);

  checkError5 = NO_ERROR;
  checkError5 = readBlockCRC(5);
  if (checkError5 != NO_ERROR) errorCounter++;
  HAL_Delay(1);

  checkError6 = NO_ERROR;
  checkError6 = readBlockCRC(6);
  if (checkError6 != NO_ERROR) errorCounter++;
  HAL_Delay(1);

  checkError7 = NO_ERROR;
  checkError7 = readBlockCRC(7);
  if (checkError7 != NO_ERROR) errorCounter++;
  HAL_Delay(1);

  /* Infinite loop */
  for(;;)
  {
    osDelay(10);

    checkError = NO_ERROR;
    checkError = getAngleValue(&angle1_raw,1);
    if (checkError != NO_ERROR) errorCounter++;
    HAL_Delay(1);

    checkError2 = NO_ERROR;
    checkError2 = getAngleValue(&angle2_raw,2);
    if (checkError2 != NO_ERROR) errorCounter++;
    HAL_Delay(1);

    checkError3 = NO_ERROR;
    checkError3 = getAngleValue(&angle3_raw,3);
    if (checkError3 != NO_ERROR) errorCounter++;
    HAL_Delay(1);

    checkError4 = NO_ERROR;
    checkError4 = getAngleValue(&angle4_raw,4);
    if (checkError4 != NO_ERROR) errorCounter++;

    checkError5 = NO_ERROR;
    checkError5 = getAngleValue(&angle5_raw,5);
    if (checkError5 != NO_ERROR) errorCounter++;
    HAL_Delay(1);

    checkError6 = NO_ERROR;
    checkError6 = getAngleValue(&angle6_raw,6);
    if (checkError6 != NO_ERROR) errorCounter++;

    checkError7 = NO_ERROR;
    checkError7 = getAngleValue(&angle7_raw,7);
    if (checkError7 != NO_ERROR) errorCounter++;


    // processing raw angles

    angle1_temp = angle1_raw - offset1;
    if (angle1_temp >= 0){
    	angle1 = angle1_temp;
    }
    else{
    	angle1 = -1 * (360 - (360 + angle1_temp));
    }

    angle2_temp = -1 * (angle2_raw - offset2);
    if (angle2_temp >= 0){
    	angle2 = angle2_temp;
    }
    else{
    	angle2 = -1 * (360 - (360 + angle2_temp));
    }

// PROBLÉMA
    angle3_temp = angle3_raw;
    if (angle3_temp >= 0){
    	angle3 = angle3_temp - offset3;
    }
    else{
    	angle3 = angle3_temp + offset3;
    }
// PROBLÉMA

    angle4_temp = -1 * (angle4_raw - offset4);
    if (angle4_temp >= 0){
    	angle4 = angle4_temp;
    }
    else{
    	angle4 = -1 * (360 - (360 + angle4_temp));
    }

    angle5_temp = (angle5_raw - offset5);
	if (angle5_temp >= 0){
		angle5 = angle5_temp;
	}
	else{
		angle5 = (360 - (360 + angle5_temp));
	}

	angle6_temp = -1 * (angle6_raw - offset6);
	if (angle6_temp >= 0){
		angle6 = angle6_temp;
	}
	else{
		angle6 = -1 * (360 - (360 + angle6_temp));
	}

	angle7_temp = (angle7_raw - offset7);
	if (angle7_temp >= 0){
		angle7 = angle7_temp;
	}
	else{
		angle7 = (360 + angle7_temp);
	}

  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
//    if (motorPattern == 1){
//
//    	for (int i = 0; i < 10; i++){
//        	TIM1->CCR1 = 40*3600/100;
//        	osDelay(500);
//        	TIM1->CCR1 = 0*3600/100;
//        	osDelay(500);
//    	}
//
//    	motorPattern = 0;
//    }
//    else {
//    	TIM1->CCR1 = feedback*3600/100;
//    }
  }
  /* USER CODE END StartMotorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */