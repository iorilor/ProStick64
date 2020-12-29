/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body v3 for Y axis processor
  ******************************************************************************
  ******************************************************************************
  * All the code OUTSIDE the "USER CODE" blocks is copyrighted as follows:
  *
  * Copyright (c) 2020 STMicroelectronics
  * All rights reserved
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  *
  * All the code INSIDE the "USER CODE" blocks is copyrighted as follows:
  *
  * Copyright (c) 2020 Lorenzo Iori
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_RANGE 		90			//Maximum number of steps in every direction
#define MAX_RANGE_EXT	120			//Maximum number of steps in every direction (ext)
#define DEADZONE 		  250			//Area that will be ignored (0-2048)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
int16_t   getCalFlashSlot	(void);
void 		  calibration		  (uint8_t, uint8_t);
void 		  draw			      (int16_t);
void 		  writeToFlash	  (uint32_t, uint32_t, uint8_t);
void 		  eraseFlash 		  (void);
uint16_t 	getValue		    (void);
void 		  rotateLeft 		  (volatile uint32_t*);
void 		  rotateRight 	  (volatile uint32_t*);
void      swdioInput      (void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t* FLASH_SR  	  = (uint32_t*)0x40022010;  //MEMORY ADDRESSES
volatile uint32_t* FLASH_CR  	  = (uint32_t*)0x40022014;
volatile uint32_t* PAGE15_SLOT1 = (uint32_t*)0x08007800;
volatile uint32_t* GPIOA_IDR 	  = (uint32_t*)0x50000010;
volatile uint32_t* GPIOB_IDR 	  = (uint32_t*)0x50000410;
volatile uint32_t* GPIOA_ODR 	  = (uint32_t*)0x50000014;
volatile uint32_t* ADC_ISR 		  = (uint32_t*)0x40012400;
volatile uint32_t* ADC_DR 		  = (uint32_t*)0x40012440;
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  //The next line configures the output register with the bit wheel
  *GPIOA_ODR = (*GPIOA_ODR & 0xFFFF0000) + 0x6600;
  uint16_t calSwRead = (uint16_t)(*GPIOB_IDR) & 0x80;//calSw = GPIOB_PIN7 status
  uint8_t  calSw = 0;
  if (calSwRead) calSw = 1;
  int16_t 	calFlashSlot = getCalFlashSlot();

  //If getCalFlashSlot returns -1 it means there is no calibration data stored
  if (calFlashSlot == -1) calibration(calSw, 0);

  //If it returns 255 it means the memory is full. Erase + calibration needed.
  if (calFlashSlot == 255){
    eraseFlash();
    calibration(calSw, 0);
  }
  uint32_t 	w1 = *(PAGE15_SLOT1 + ((calFlashSlot)*2));
  uint32_t 	w2 = *(PAGE15_SLOT1 + ((calFlashSlot)*2) + 1);
  uint16_t 	w1a = (uint16_t)(w1>>16);
  uint16_t 	neutral16 = (uint16_t) w1;
  uint16_t 	calFactAInt = (uint16_t)(w2>>16);
  uint16_t 	calFactBInt = (uint16_t) w2;
  uint16_t 	min = w1a & 0x7FFF;
  uint8_t  	calSwOld = (uint8_t)(w1a >> 15);
  if (calSwOld != calSw){
    calFlashSlot++;
    calibration(calSw, (uint8_t)calFlashSlot);
  }
  float    	calFactA = (float)calFactAInt / 10000;
  float    	calFactB = (float)calFactBInt / 10000;

  uint16_t 	value16, value16Old, value16Raw;
  int16_t 	steps = 0, pos = 0;
  uint16_t	stepScale = 0;
  value16Old = 2048;
  value16Raw = getValue();
  if (value16Raw < 1500 || value16Raw > 2500){
    stepScale = 4096 / (MAX_RANGE_EXT * 2);
  } else {
    stepScale = 4096 / (MAX_RANGE * 2);
  }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
      value16Raw = getValue();
      if (value16Raw < neutral16){
        if (value16Raw < min) value16Raw = min;
        value16 = value16Raw - min;
        value16 = value16 * calFactA;
      }
      if (value16Raw == neutral16)	value16 = neutral16;
      if (value16Raw > neutral16){
        value16 = value16Raw - neutral16;
        value16 = value16 * calFactB;
        value16 = value16 + 2048;
      }
      if (value16 > 2048 - DEADZONE && value16 < 2048 + DEADZONE){
        value16Old = value16;
        while (pos > 0){
          rotateLeft(GPIOA_ODR);
          pos--;
        }
        while (pos < 0){
          rotateRight(GPIOA_ODR);
          pos++;
        }
      }
      steps = steps + ((int16_t)value16 - value16Old);
      value16Old = value16;
      while (steps < -stepScale){
        rotateLeft(GPIOA_ODR);
        steps = steps + stepScale;
        pos--;
      }
      while (steps > stepScale){
        rotateRight(GPIOA_ODR);
        steps = steps - stepScale;
        pos++;
      }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /*
    Configure the global features of the ADC
    (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analog WatchDog 2
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT_0_Pin|OUT_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAL_SW_Pin */
  GPIO_InitStruct.Pin = CAL_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CAL_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_0_Pin OUT_1_Pin */
  GPIO_InitStruct.Pin = OUT_0_Pin|OUT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//Find the FLASH memory slot currently in use
int16_t getCalFlashSlot(void){
	int16_t i = -1;
	uint32_t data = *PAGE15_SLOT1;
	while (((data != 0xFFFFFFFF) && i < 255)){
		i++;
		data = *(PAGE15_SLOT1 + i*2);
	}
	return i;
}
//Calibration 3.0
void calibration(uint8_t calSw, uint8_t calFlashSlot){

//PHASE 1 - Calculating neutral value
uint32_t 	value32=0, i=0, j=0, neutral32=0, w1=0xFFFFFFFF,w2=0xFFFFFFFF;
uint16_t 	value16=0, count=0, neutralAverage=0, neutralAverageOld=0;
uint16_t  max=0, min=0, MCU_OK=0;
uint16_t 	w1a=0xFFFF, w1b=0xFFFF, w2a=0xFFFF, w2b=0xFFFF;
uint8_t		neutralOK=0;
float			calFactA=0.0, calFactB=0.0;

HAL_Delay(5000);

swdioInput();

//Set GPIOA_PIN11 and PIN12 HIGH to communicate that MCU is ready to proceed
*GPIOA_ODR = (*GPIOA_ODR & 0xFFFF0000) + 0x1800;

//Wait until GPIOA_PIN13 becomes 1 - both MCUs are ready to proceed
while (!MCU_OK){
  MCU_OK = (uint16_t)(*GPIOB_IDR) & 0x2000;
}

HAL_Delay(100);

//The next line reconfigures the output register with the bit wheel
*GPIOA_ODR = (*GPIOA_ODR & 0xFFFF0000) + 0x6600;

MCU_OK = 0;
draw(3);

value16 = getValue();

while (value16<1600 || value16>2400){   //Wait for joystick to be centered
	value16 = getValue();
}
//Step 1.1 - Pull the stick to any corner and let go until neutral is OK
while (neutralOK<3){
	count++;
	while (value16>1500 && value16<2500){  //Wait for joystick to be moved
		value16 = getValue();
	}
	HAL_Delay(100);
	while (value16<1600 || value16>2400){  //Wait for joystick to return centered
		value16 = getValue();
	}
	HAL_Delay(500);									//Wait for spring to stabilize
	for (i=0; i<16; i++){						//Get neutral value by averaging 16 readings
		value32 = value32 + getValue();
	}
	neutral32 = neutral32 + (value32/16);	 //Add current neutral reading to pool
	value32 = 0;								           //Reset value32
	neutralAverage = (neutral32/count);
	if (neutralAverage == neutralAverageOld) neutralOK++;
	else neutralOK = 0;
	neutralAverageOld = neutralAverage;
}
//Set GPIOA_PIN11 and PIN12 HIGH to communicate that MCU is ready to proceed
*GPIOA_ODR = (*GPIOA_ODR & 0xFFFF0000) + 0x1800;

//Wait until GPIOA_PIN13 becomes 1 - both MCUs are ready to proceed
while (!MCU_OK){
  MCU_OK = (uint16_t)(*GPIOA_IDR) & 0x2000;
}

HAL_Delay(100);

//The next line reconfigures the output register with the bit wheel
*GPIOA_ODR = (*GPIOA_ODR & 0xFFFF0000) + 0x6600;

MCU_OK = 0;
draw(3);
//PHASE 2 - Calculating min and max while the stick is rotated
min = neutralAverage;
max = neutralAverage;
for (i=0; i<625000; i++){
	for (j=0; j<16; j++){
		value32 = value32 + getValue();
	}
	value16 = value32 / 16;
	value32 = 0;
	if (value16 > max){
		max = value16;
	}else if(value16 < min){
		min = value16;
	}
}
//PHASE 3 - Calculating calFactA and calFactB
calFactA = ((float)2047) / (neutralAverage - min);
calFactB = ((float)2048) / (max - neutralAverage);
//PHASE 4 - Calculating w1 and w2 and write to flash
/* Calibration data structure:

	w1 = 						w1a								  w1b
				|--1bit--|---------15bit----------|---------------16bit---------------|
				  calSw				min							neutral16

	w2 =						w2a									w2b
				|--------------16bit--------------|---------------16bit---------------|
							 calFactA							calFactB
*/
w1a = min + ((uint16_t)calSw << 15);
w1b = neutralAverage;
w2a = (uint16_t)(calFactA * 10000);
w2b = (uint16_t)(calFactB * 10000);
w1 = ((uint32_t)w1a << 16) + w1b;
w2 = ((uint32_t)w2a << 16) + w2b;
writeToFlash(w1, w2, calFlashSlot);

//Set GPIOA_PIN11 and PIN12 HIGH to communicate that MCU is ready to proceed
*GPIOA_ODR = (*GPIOA_ODR & 0xFFFF0000) + 0x1800;

//Wait until GPIOA_PIN13 becomes 1 - both MCUs are ready to proceed
while (!MCU_OK){
  MCU_OK = (uint16_t)(*GPIOA_IDR) & 0x2000;
}

HAL_Delay(100);

//The next line reconfigures the output register with the bit wheel
*GPIOA_ODR = (*GPIOA_ODR & 0xFFFF0000) + 0x6600;

MCU_OK = 0;
draw(-1);
}

//Move the cursor in a waving pattern for n times
void draw(int16_t n){
	uint8_t  i=0;
  uint16_t j=0;
  //if times_to_repeat = -1 loop indefinitely
  if (n == -1){
    while(1){
      for (i=0; i<40; i++){
  			rotateRight(GPIOA_ODR);
  			HAL_Delay(1);
  		}
  		HAL_Delay(500);
  		for (i=0; i<40; i++){
  			rotateLeft(GPIOA_ODR);
  			HAL_Delay(1);
  		}
  		for (i=0; i<40; i++){
  			rotateLeft(GPIOA_ODR);
  			HAL_Delay(1);
  		}
  		HAL_Delay(500);
  		for (i=0; i<40; i++){
  			rotateRight(GPIOA_ODR);
  			HAL_Delay(1);
  		}
    }
  }

  //otherwise loop n times
  else {
  	for (j=0; j<n; j++){
  		for (i=0; i<40; i++){
  			rotateRight(GPIOA_ODR);
  			HAL_Delay(1);
  		}
  		HAL_Delay(500);
  		for (i=0; i<40; i++){
  			rotateLeft(GPIOA_ODR);
  			HAL_Delay(1);
  		}
  		for (i=0; i<40; i++){
  			rotateLeft(GPIOA_ODR);
  			HAL_Delay(1);
  		}
  		HAL_Delay(500);
  		for (i=0; i<40; i++){
  			rotateRight(GPIOA_ODR);
  			HAL_Delay(1);
  		}
  	}
  }
}

//Write data into MCU flash memory
void writeToFlash(uint32_t w1, uint32_t w2, uint8_t calFlashSlot){

	HAL_FLASH_Unlock();
	while (*FLASH_SR&(1<<16));          			    //wait for BSY1 flag to be 0
	*FLASH_SR = *FLASH_SR & 0x00050000; 			    //reset all flash error flags
	while (*FLASH_SR&(1<<18));          			    //wait for CFGBSY1 flag to be 0
	*FLASH_CR = *FLASH_CR | 0x01; 					      //set PG flag
	*(PAGE15_SLOT1 + (calFlashSlot*2)) = w1;      //write w1 into Flash
	__ISB();				//Ensures that the two 32bit words are not merged by compiler
	*(PAGE15_SLOT1 + (calFlashSlot*2) + 1) = w2;  //write w2 into Flash
	while (*FLASH_SR&(1<<16));          			    //wait for BSY1 flag to be 0
	while (*FLASH_SR&(1<<18));          			    //wait for CFGBSY1 flag to be 0
	*FLASH_CR = *FLASH_CR & 0xFFFFFFFE;			     	//reset PG flag
	HAL_FLASH_Lock();
}

//Erase all data in pages 15 and 16 of MCU flash memory
void eraseFlash (void){
	HAL_FLASH_Unlock();
	while (*FLASH_SR&(1<<16));          	//wait for BSY1 flag to be 0
	*FLASH_SR = *FLASH_SR & 0x00050000; 	//reset all flash error flags
	*FLASH_CR = *FLASH_CR & 0xC0000000; 	//reset page erase and number flags
	*FLASH_CR = *FLASH_CR | 0x7A; 				//set page erase flag and page number 15
	*FLASH_CR = *FLASH_CR | (1<<16);    	//set start flag
	while (*FLASH_SR&(1<<16));          	//wait for BSY1 flag to be 0
	*FLASH_CR = *FLASH_CR & 0xC0000000; 	//reset page erase and number flags
	HAL_FLASH_Lock();
}

//Wait till new value is available from ADC data register and read it
uint16_t getValue(void){
  uint16_t value;
	while (!(*ADC_ISR & 1<<3));
	value = (uint16_t)(*ADC_DR);
	return value;
}

//Simulate an optical disc rotation (LEFT)
void rotateLeft (volatile uint32_t* GPIOA_ODR){
	uint8_t result, data;
	data = (uint8_t)(*(GPIOA_ODR)>>8);
	if ( data & 1<<7 ){
		result = (data<<1)|(1<<0);
	}else{
		result = (data<<1);
	}
	*GPIOA_ODR = (*GPIOA_ODR & 0xFFFF00FF) + ((uint16_t)result << 8);
}

//Simulate an optical disc rotation (RIGHT)
void rotateRight (volatile uint32_t* GPIOA_ODR){
	uint8_t result, data;
	data = (uint8_t)(*(GPIOA_ODR)>>8);
	if ( data & (1<<0) ){
		result = (data>>1)|(1<<7);
	}else{
		result = (data>>1);
	}
	*GPIOA_ODR = (*GPIOA_ODR & 0xFFFF00FF) + ((uint16_t)result << 8);
}

//Configure SWDIO pin (GPIOA_PIN13) as input
void swdioInput (void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/* USER CODE END 4 */

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

/*******************************END OF FILE************************************/
