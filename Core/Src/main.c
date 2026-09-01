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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Servo_SetAngle(uint8_t angle);
void mot_test(void);
void sonar_test(float distanza_min,float distanza_max);
void set_mot_spin_destra(uint16_t speed);
void set_mot_spin_sinistra(uint16_t speed);
void set_mot_curva_destra(void);
void set_mot_curva_sinistra(void);
void mot_turn(uint8_t direzione, uint16_t speed);
void path_finding(void);
float scan_direction(uint8_t verso, uint16_t durata_ms, uint16_t speed);
void DWT_Init(void);
static void mot_dir_lato_sinistro(uint8_t avanti);
static void mot_dir_lato_destro(uint8_t avanti);

void set_mot_speed_individual(uint16_t speed_asx, uint16_t speed_adx, uint16_t speed_psx, uint16_t speed_pdx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --------------------------------------------------------------
   Init del DWT cycle counter: contatore libero della CPU, usato
   per la misura di tempo in misura()/delay_us(). Indipendente
   dai timer periferici -> non disturba mai il PWM dei motori.
   -------------------------------------------------------------- */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Servo_startup(void){
	for (int i = SERVO_posDestra; i <=SERVO_posSinistra ; i++){
		Servo_SetAngle(i);
		HAL_Delay(5);
	 }
	for (int i = SERVO_posSinistra; i > SERVO_posDestra; i--){
		 Servo_SetAngle(i);
		 HAL_Delay(5);
	}
	Servo_SetAngle(SERVO_posDefault);
}


void Servo_SetAngle(uint8_t angle)
{
    uint32_t pulse;

    if (angle > 180U) {
        angle = 180U;
    }

    //
    pulse = SERVO_MIN_TICK + (((uint32_t)angle * (SERVO_MAX_TICK - SERVO_MIN_TICK)) / 180U);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);
}

/* --------------------------------------------------------------
   delay_us basata sul DWT (prima usava htim2, che è lo stesso
   timer che genera il PWM di 2 motori -> conflitto rimosso).
   -------------------------------------------------------------- */
void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

/* --------------------------------------------------------------
   misura(): stessa formula di distanza di prima, ma il timing
   ora viene dal DWT invece che da htim2, quindi ogni lettura
   ad ultrasuoni non disturba più il PWM dei motori PSX/ADX.
   -------------------------------------------------------------- */
float misura(void)
{
    uint32_t t_trig, t1, t2;
    uint32_t timeout_ticks = (SystemCoreClock / 1000U) * 30U; // timeout 30 ms

    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    delay_us(10);   // impulso di trigger corretto: 10us reali
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

    t_trig = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)
    {
        if ((DWT->CYCCNT - t_trig) > timeout_ticks)
            return -1.0f;
    }
    t1 = DWT->CYCCNT;

    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
    {
        if ((DWT->CYCCNT - t1) > timeout_ticks)
            return -1.0f;
    }
    t2 = DWT->CYCCNT;

    float t_us = (float)(t2 - t1) / (float)(SystemCoreClock / 1000000U);
    return t_us * 0.01715f; // distanza in cm
}

void ricerca(void){
	float distanzaMax = 0;
	int posMax = 0;
	float tempDist = 0;
	for (int i = SERVO_posDestra; i <=SERVO_posSinistra ; i++){
			Servo_SetAngle(i);
			tempDist = 0;
			tempDist = misura();
			if (tempDist > distanzaMax){
				distanzaMax = tempDist;
				posMax = i;
			}


			HAL_Delay(10);
		 }
	Servo_SetAngle(posMax);

}
void config_mot(void){
	  // Avvia PWM
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   // servo
}
/* ================================================================
   MAPPATURA FISICA REALE (confermata dall'utente):
     motASX = Posteriore DESTRA   (RR)
     motPDX = Anteriore  DESTRA   (FR)  -> pin invertiti rispetto agli altri 3
     motADX = Anteriore  SINISTRA (FL)
     motPSX = Posteriore SINISTRA (RL)

   Stato "AVANTI" calibrato da set_mot_drive() originale (funzionante):
     ASX avanti = (1:SET, 2:RESET)
     PDX avanti = (1:RESET, 2:SET)   <- invertito
     ADX avanti = (1:SET, 2:RESET)
     PSX avanti = (1:SET, 2:RESET)

   Da qui in poi si comanda sempre per LATO FISICO reale (sinistra =
   ADX+PSX, destra = ASX+PDX), non più per etichetta di canale, così
   le due ruote dello stesso lato girano sempre nella stessa direzione.
   ================================================================ */

static void mot_dir_ASX(uint8_t avanti){ // Posteriore Destra
    HAL_GPIO_WritePin(motASX1_GPIO_Port, motASX1_Pin, avanti ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motASX2_GPIO_Port, motASX2_Pin, avanti ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static void mot_dir_PDX(uint8_t avanti){ // Anteriore Destra (pin invertiti)
    HAL_GPIO_WritePin(motPDX1_GPIO_Port, motPDX1_Pin, avanti ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(motPDX2_GPIO_Port, motPDX2_Pin, avanti ? GPIO_PIN_SET   : GPIO_PIN_RESET);
}
static void mot_dir_ADX(uint8_t avanti){ // Anteriore Sinistra
    HAL_GPIO_WritePin(motADX1_GPIO_Port, motADX1_Pin, avanti ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motADX2_GPIO_Port, motADX2_Pin, avanti ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static void mot_dir_PSX(uint8_t avanti){ // Posteriore Sinistra
    HAL_GPIO_WritePin(motPSX1_GPIO_Port, motPSX1_Pin, avanti ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motPSX2_GPIO_Port, motPSX2_Pin, avanti ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void mot_dir_lato_sinistro(uint8_t avanti){ mot_dir_ADX(avanti); mot_dir_PSX(avanti); }
static void mot_dir_lato_destro(uint8_t avanti){ mot_dir_ASX(avanti); mot_dir_PDX(avanti); }

void set_mot_reverse(void){
	mot_dir_lato_sinistro(0);
	mot_dir_lato_destro(0);
}

void set_mot_drive(void){
	mot_dir_lato_sinistro(1);
	mot_dir_lato_destro(1);
}
void mot_on_speed(short SPEED){
		  // MOTASX
		 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SPEED);

		  // MOTPDX
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SPEED);

		  // MOTPSX
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SPEED);

		  // MOTADX
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, SPEED);
}
void mot_stop(void){
	HAL_GPIO_WritePin(motPSX2_GPIO_Port, motPSX2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motPSX1_GPIO_Port, motPSX1_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(motASX1_GPIO_Port, motASX1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motASX2_GPIO_Port, motASX2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(motPDX1_GPIO_Port, motPDX1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motPDX2_GPIO_Port, motPDX2_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(motADX1_GPIO_Port, motADX1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motADX2_GPIO_Port, motADX2_Pin, GPIO_PIN_RESET);

}

void mot_on(void){
	set_mot_drive();

		// MOTPSX - ruota rotta
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOT_VEL_MAX);
		// MOT ADX
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MOT_VEL_MAX);

		// Velocità (0–999)


		//HAL_Delay(220);
		// MOTASX
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOT_VEL_MAX);


		// MOTPDX
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOT_VEL_MAX);




}
void mot_test(void){
	 mot_stop();
	 HAL_Delay(2000);
		 mot_on();
		 HAL_Delay(2000);
}
void sonar_test(float distanza_min,float distanza_max){
	float d = misura();
	if(d>distanza_min && d<distanza_max){
		mot_on();
	}else{
		mot_stop();
	}
}
void mot_turn(uint8_t direzione, uint16_t speed)
{
    mot_stop();
    HAL_Delay(50); // Breve pausa per proteggere i ponti H da picchi di corrente

    if (direzione == MOT_DIREZIONE_DX) {
        set_mot_spin_destra(speed);
    } else if (direzione == MOT_DIREZIONE_SX) {
        set_mot_spin_sinistra(speed);
    }
}
/* Rotazione sul posto: un lato avanti, l'altro indietro, stessa velocità.
   "destra"/"sinistra" restano nomi convenzionali - se scopri che il verso
   fisico è invertito rispetto a quello che ti aspetti, scambia i due
   corpi funzione qui sotto (non toccare mot_dir_*). */
void set_mot_spin_destra(uint16_t speed)
{
    mot_dir_lato_sinistro(1); // avanti
    mot_dir_lato_destro(0);   // indietro
    set_mot_speed_individual(speed, speed, speed, speed);
}
void set_mot_spin_sinistra(uint16_t speed)
{
    mot_dir_lato_sinistro(0); // indietro
    mot_dir_lato_destro(1);   // avanti
    set_mot_speed_individual(speed, speed, speed, speed);
}

/* Curva ad arco: entrambi i lati AVANTI, ma a velocità diverse
   (lato esterno veloce, lato interno lento) -> a differenza di prima
   ora è una vera curva, non un altro spin travestito. */
#define MOT_VEL_INNER   (MOT_VEL_MAX / 3)   // velocità lato interno - da tarare

void set_mot_curva_destra(void) // curva verso destra: interno = destra
{
    mot_dir_lato_sinistro(1);
    mot_dir_lato_destro(1);
    // set_mot_speed_individual(asx, adx, psx, pdx)
    set_mot_speed_individual(MOT_VEL_INNER, MOT_VEL_MAX, MOT_VEL_MAX, MOT_VEL_INNER);
    HAL_Delay(RIT_SPIN);
    mot_stop();
}
void set_mot_curva_sinistra(void) // curva verso sinistra: interno = sinistra
{
    mot_dir_lato_sinistro(1);
    mot_dir_lato_destro(1);
    set_mot_speed_individual(MOT_VEL_MAX, MOT_VEL_INNER, MOT_VEL_INNER, MOT_VEL_MAX);
    HAL_Delay(RIT_SPIN);
    mot_stop();
}
void set_mot_speed_individual(uint16_t speed_asx, uint16_t speed_adx, uint16_t speed_psx, uint16_t speed_pdx)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed_asx); // ASX
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed_adx); // ADX
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed_psx); // PSX
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed_pdx); // PDX
}
/* --------------------------------------------------------------
   scan_direction(): ruota il robot sul posto in una direzione
   per un tempo FISSO (durata_ms), campionando la distanza max
   rilevata. Sostituisce le vecchie routa_destra/sinistra
   _path_finding(), che scansionavano per un numero di cicli
   diverso tra destra (N_RICERCHE/2) e sinistra (N_RICERCHE):
   ecco perché il confronto tra i due lati non aveva senso.
   -------------------------------------------------------------- */
#define SCAN_TIME_MS   600u   // durata di ogni scansione laterale - da tarare

float scan_direction(uint8_t verso, uint16_t durata_ms, uint16_t speed)
{
    float dist_max = 0.0f;
    uint32_t t_start = HAL_GetTick();

    if (verso == MOT_DIREZIONE_DX)
        set_mot_spin_destra(speed);
    else
        set_mot_spin_sinistra(speed);

    while ((HAL_GetTick() - t_start) < durata_ms)
    {
        float d = misura();
        if (d > dist_max)
            dist_max = d;
        HAL_Delay(10);
    }

    mot_stop();
    return dist_max;
}

/* --------------------------------------------------------------
   path_finding(): macchina a stati simmetrica.
   stop -> scansiona DX (tempo fisso) -> torna al centro (SX,
   stesso tempo) -> scansiona SX (stesso tempo fisso) -> torna
   al centro (DX, stesso tempo) -> confronta -> gira verso il
   lato più libero -> riparte dritto.
   Ogni fase usa la stessa rotazione (spin sul posto), quindi le
   due scansioni sono confrontabili e il robot torna davvero
   all'orientamento di partenza tra una e l'altra.
   -------------------------------------------------------------- */
void path_finding(void){
	mot_stop();
	HAL_Delay(100);

	// 1. Scansiona a destra
	float destra = scan_direction(MOT_DIREZIONE_DX, SCAN_TIME_MS, MOT_VEL_MAX);
	HAL_Delay(200);

	// 2. Torna al centro (stessa rotazione, verso opposto, stesso tempo)
	mot_turn(MOT_DIREZIONE_SX, MOT_VEL_MAX);
	HAL_Delay(SCAN_TIME_MS);
	mot_stop();
	HAL_Delay(200);

	// 3. Scansiona a sinistra, STESSO tempo di scansione della destra
	float sinistra = scan_direction(MOT_DIREZIONE_SX, SCAN_TIME_MS, MOT_VEL_MAX);
	HAL_Delay(200);

	// 4. Torna al centro
	mot_turn(MOT_DIREZIONE_DX, MOT_VEL_MAX);
	HAL_Delay(SCAN_TIME_MS);
	mot_stop();
	HAL_Delay(200);

	// 5. Gira verso il lato con più spazio libero
	if(destra > sinistra){

		mot_turn(MOT_DIREZIONE_DX, MOT_VEL_MAX);
	}else{

		mot_turn(MOT_DIREZIONE_SX, MOT_VEL_MAX);
	}


	HAL_Delay(SCAN_TIME_MS ); // gira circa a metà strada verso lo spazio libero
	mot_stop();
	HAL_Delay(100);

	// 6. Riparte dritto
	set_mot_drive();
	mot_on_speed(MOT_VEL_MAX);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){

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
  DWT_Init(); // abilita il cycle counter usato da misura()/delay_us()
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);


  config_mot();
  set_mot_drive();

  mot_stop();
  HAL_Delay(startup_delay);







  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  //test



	  /* Se rileva un ostacolo tra 2cm e 20cm */
	 if (misura() > 3.0f && misura() < 20.0f) {
		 	mot_stop();

		 	set_mot_reverse();
		 		      mot_on_speed(MOT_VEL_MAX);
		 		      HAL_Delay(250);


		 	mot_stop();
		 	HAL_Delay(1500);



		 	// 1. Torna indietro per 0.5 secondi
		 	set_mot_reverse();
		 	mot_on_speed(MOT_VEL_MAX);
		 	HAL_Delay(REVERSE_TIME);
		 	mot_stop();
		 	HAL_Delay(1500);


		 	//ricerca punto
		 	path_finding();
		 	mot_stop();
		 	HAL_Delay(2000);





		 	HAL_Delay(50);
	  } else {
	      // Prosegue dritto
	      set_mot_drive();
	      mot_on_speed(MOT_VEL_MAX);
	  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, motASX2_Pin|motASX1_Pin|motPDX1_Pin|motPDX2_Pin
                          |motADX1_Pin|TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, motADX2_Pin|motPSX2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(motPSX1_GPIO_Port, motPSX1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : motASX2_Pin motASX1_Pin motPDX1_Pin motPDX2_Pin
                           motADX1_Pin TRIG_Pin */
  GPIO_InitStruct.Pin = motASX2_Pin|motASX1_Pin|motPDX1_Pin|motPDX2_Pin
                          |motADX1_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : motADX2_Pin motPSX2_Pin */
  GPIO_InitStruct.Pin = motADX2_Pin|motPSX2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : motPSX1_Pin */
  GPIO_InitStruct.Pin = motPSX1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(motPSX1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
