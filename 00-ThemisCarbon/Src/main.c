/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>

#include "configuration.h"
#include "serial.h"
#include "protocol.h"
#include "protocol_interpreter.h"
#include "imu.h"
#include "pid.h"
#include "stepper.h"
#include "ewma.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart8;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart8_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static float const distance_CoG = 0.13; // 13cm above the floor
static float const wheel_distance = 0.11; // 12cm from center

static float *pitch_ref = &configuration_data[IMU_PITCH_REF];

StepperMotor_HandleTypeDef m1;
StepperMotor_HandleTypeDef m2;

ewma_handler m1_ewma = {&configuration_data[M_FILTER], 0.0};
ewma_handler m2_ewma = {&configuration_data[M_FILTER], 0.0};

static float m1_speed = 0.0;
static float m2_speed = 0.0;

static bool configured = false;
static bool armed = false;

pid_win_handler pid_pitch =
{
		&configuration_data[CTRL_PKP],
		&configuration_data[CTRL_PKI],
		&configuration_data[CTRL_PKD],
	-400.0*16.0*5.0, //configuration_data[M_SPEED],
	400.0*16.0*5.0, //configuration_data[M_SPEED],
	PID_WIN_SIZE,
	0.1 // 10xT filter
};
static int32_t pid_pitch_output = 0;

int32_t abs_pos_0 = 0;
int32_t abs_pos_1 = 0;
int32_t abs_pos = 0;
int32_t abs_pos_last = 0;
int32_t delta_pos = 0;

float actual_xspeed = 0.0;
float actual_wspeed = 0.0;
float reference_xspeed = 0.0f;
float reference_wspeed = 0.0f;

ewma_handler xspeed_ewma = {&configuration_data[CTRL_XFILTER], 0.0};
ewma_handler wspeed_ewma = {&configuration_data[CTRL_YFILTER], 0.0};

pid_handler pid_x =
{
		&configuration_data[CTRL_XKP],
		&configuration_data[CTRL_XKI],
		&configuration_data[CTRL_XKD],
	-10.0,
	10.0,
	0.1 // no filter
};

static float pid_x_output = 0;

static float distance = 0.0;

float e_pitch_alpha = 1.0/2000.0;
ewma_handler e_ptich_correction = {&e_pitch_alpha, 0.0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART8_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define ToDeg(x) ((x) * 57.2957795131)


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback(void)
{
	if(configured)
	{

	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART8_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	// PWM inputs 1 & 2 (no HAL class at the moment)
	HAL_TIM_IC_Start(&htim8,TIM_CHANNEL_1); // Rx1
	HAL_TIM_IC_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_IC_Start(&htim4,TIM_CHANNEL_1); // Rx2
	HAL_TIM_IC_Start(&htim4,TIM_CHANNEL_2);

	// Configuration : hard-coded first flash
	if(0)
	{
		configuration_data[IMU_PITCH_REF] = 0.0;
		configuration_data[IMU_ROLL_REF] = 0.0;
		configuration_data[IMU_YAW_REF] = 0.0;
		configuration_data[IMU_SPARE1] = 0.0;
		configuration_data[IMU_SPARE2] = 0.0;
		configuration_data[IMU_SPARE3] = 0.0;

		configuration_data[M_ACC] = 	0.0; //! steps/s^2
		configuration_data[M_SPEED] = 	0.0; // step/s
		configuration_data[M_FILTER] = 0.5;
		configuration_data[M_SPARE1] = 0.0;
		configuration_data[M_SPARE2] = 0.0;
		configuration_data[M_SPARE3] = 0.0;

		configuration_data[CTRL_PKP] = 1000.0;
		configuration_data[CTRL_PKI] = 10.0;
		configuration_data[CTRL_PKD] = 1000.0;

		configuration_data[CTRL_YKP] = 0.0;
		configuration_data[CTRL_YKI] = 0.0;
		configuration_data[CTRL_YKD] = 0.0;

		configuration_data[CTRL_XKP] = 13.0;
		configuration_data[CTRL_XKI] = 0.09;
		configuration_data[CTRL_XKD] = 0.0;

		configuration_data[CTRL_XFILTER] = 0.5;
		configuration_data[CTRL_YFILTER] = 1.0;
		configuration_data[CTRL_PFILTER] = 1.0;

		HAL_Configuration_Save();
	}
	// Configuration restore
	HAL_Configuration_Reload();


	HAL_Imu_Init();
	if(0)
	{
		HAL_Imu_Gyr_Drift_Calibration();
	}
	if(0)
	{
		HAL_Imu_Mag_Calibration();
	}
	HAL_Imu_Process();

	// Motors
	HAL_Stepper_Motor_Add(
		  &m1,
		  M1_STEP_GPIO_Port,
		  M1_STEP_Pin,
		  M1_DIR_GPIO_Port,
		  M1_DIR_Pin
	);
	HAL_Stepper_Motor_Add(
		  &m2,
		  M2_STEP_GPIO_Port,
		  M2_STEP_Pin,
		  M2_DIR_GPIO_Port,
		  M2_DIR_Pin
	);
	HAL_Stepper_Motor_Init(&htim2);

	HAL_Stepper_Motor_Set_Speed(&m1,0);
	HAL_Stepper_Motor_Set_Speed(&m2,0);
	HAL_GPIO_WritePin(M1_ENABLE_GPIO_Port,M1_ENABLE_Pin,GPIO_PIN_SET); //! RESET to run
	HAL_GPIO_WritePin(M2_ENABLE_GPIO_Port,M2_ENABLE_Pin,GPIO_PIN_SET);

	// Telemetry
	HAL_Serial_Init(&huart8,&com);
	{
		float tx_data_buffer[32];
		tx_data_buffer[0]=configuration_data[IMU_PITCH_REF];
		tx_data_buffer[1]=configuration_data[CTRL_PKP];
		tx_data_buffer[2]=configuration_data[CTRL_PKI];
		tx_data_buffer[3]=configuration_data[CTRL_PKD];
		tx_data_buffer[4]=configuration_data[CTRL_XKP];
		tx_data_buffer[5]=configuration_data[CTRL_XKI];
		tx_data_buffer[6]=configuration_data[CTRL_XKD];
		tx_data_buffer[7]=configuration_data[CTRL_XFILTER];
		tx_data_buffer[8]=configuration_data[CTRL_YFILTER];
		tx_data_buffer[9]=configuration_data[CTRL_PFILTER];

		int enc_ret = protocol_encode(
			tx_protocol_buffer,
			CMD_CONFIGURATION,
			(uint8_t const*)tx_data_buffer,
			4*21 );
		if(enc_ret>0)
		HAL_Serial_Write(&com, tx_protocol_buffer, enc_ret );
		HAL_Delay(3);
	}

	reset_pid_win(&pid_pitch);
	reset_pid(&pid_x);


	reset_ewma( &m1_ewma );
	reset_ewma( &m2_ewma );
	reset_ewma( &xspeed_ewma );
	reset_ewma( &wspeed_ewma );
	reset_ewma( &e_ptich_correction );


	// end of startup
	  armed = false;
	configured = true;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t last_time = HAL_GetTick();
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  APP_Protocol_Interpreter_Process();
	  uint32_t current_time = HAL_GetTick();
	  if(current_time>=last_time+5)
	  {
			last_time = current_time;
			HAL_Imu_Process();
			// actual x and w speeds (m/s)
			abs_pos_0 = HAL_Stepper_Motor_Get_Position(&m1);
			abs_pos_1 = HAL_Stepper_Motor_Get_Position(&m2);
			abs_pos_last = abs_pos;
			abs_pos = (abs_pos_1-abs_pos_0)/2;
			delta_pos = (abs_pos_1+abs_pos_0)/2;
			distance = (float)(abs_pos)*(M_PI*0.1/(400.0*16.0))+distance_CoG*sin(HAL_Imu_Angle(IMU_ANGLE_PITCH)*M_PI/180); // m






			if( armed && ( (HAL_Imu_Angle(IMU_ANGLE_PITCH)>10.0) || (HAL_Imu_Angle(IMU_ANGLE_PITCH)<-10.0) ) )
			{
				armed = false;
				HAL_Stepper_Motor_Set_Speed(&m1,0);
				HAL_Stepper_Motor_Set_Speed(&m2,0);
				HAL_GPIO_WritePin(M1_ENABLE_GPIO_Port,M1_ENABLE_Pin,GPIO_PIN_SET); //! RESET to run
				HAL_GPIO_WritePin(M2_ENABLE_GPIO_Port,M2_ENABLE_Pin,GPIO_PIN_SET);

			}
			else if(armed)
			{
				// read PWM 1 & 2
				uint32_t period1 = htim8.Instance->CCR1; // 1000 .. 2000 Rx1
				uint32_t period2 = htim4.Instance->CCR1; // 1000 .. 2000 Rx2
				uint32_t width1 = htim8.Instance->CCR2; // 1000 .. 2000
				uint32_t width2 = htim4.Instance->CCR2; // 1000 .. 2000
				if(period1>0 && period2>0 && width1>0 && width2>0)
				{
					if((1450<width1)&&(width1<1550)) width1 = 1500; // deadzone
					if((1450<width2)&&(width2<1550)) width2 = 1500; // deadzone
					//reference_wspeed = fmap( (float)((int32_t)(width1) - 1500), -500.0, 500.0, -300.0, 300.0); // m/s
					//reference_xspeed = fmap( (float)((int32_t)(width2) - 1500), -500.0, 500.0, -800.0, 800.0); // m/s
					reference_xspeed = (float)((int32_t)(width1)-1500)/1000.0; // m/s ... 0.5m/s max
					reference_wspeed = (float)((int32_t)(width2)-1500)*10.0; // deg/s ... 10 dps max
				}
				else
				{
					reference_xspeed = 0.0f;
					reference_wspeed = 0.0f;
				}

				actual_xspeed = process_ewma(&xspeed_ewma,(float)(abs_pos-abs_pos_last)*(M_PI*0.1/(400.0*16.0))*200.0); //+distance_CoG*sin(HAL_Imu_Angle(IMU_ANGLE_PITCH)*M_PI/180); // m/s
				actual_wspeed = process_ewma(&wspeed_ewma,200.0*ToDeg(fastAtan2((float)(delta_pos)*(M_PI*0.1/(400.0*16.0)),wheel_distance)));

				pid_x_output = process_pid(&pid_x,reference_xspeed-actual_xspeed);
				pid_pitch_output = process_pid_win(&pid_pitch,HAL_Imu_Angle(IMU_ANGLE_PITCH) - pid_x_output + *pitch_ref );

				process_ewma(&e_ptich_correction,pid_x_output); // bizarre filtrage apres etre utilisé

				m1_speed = process_ewma(&m1_ewma,pid_pitch_output-reference_wspeed);
				m2_speed = process_ewma(&m2_ewma,pid_pitch_output+reference_wspeed);

				HAL_Stepper_Motor_Set_Speed(&m1,-(int32_t)m1_speed);
				HAL_Stepper_Motor_Set_Speed(&m2,(int32_t)m2_speed);

			}
			else // not amred
			{
				HAL_GPIO_WritePin(M1_ENABLE_GPIO_Port,M1_ENABLE_Pin,GPIO_PIN_RESET); //! RESET to run
				HAL_GPIO_WritePin(M2_ENABLE_GPIO_Port,M2_ENABLE_Pin,GPIO_PIN_RESET);
				HAL_Stepper_Motor_Set_Speed(&m1,0);
				HAL_Stepper_Motor_Set_Speed(&m2,0);
				HAL_Stepper_Motor_Reset_Position(&m1);
				HAL_Stepper_Motor_Reset_Position(&m2);
				reset_pid_win(&pid_pitch);
				reset_pid(&pid_x);
				reset_ewma( &m1_ewma );
				reset_ewma( &m2_ewma );
				reset_ewma( &xspeed_ewma );
				reset_ewma( &wspeed_ewma );
				pid_x_output = 0.0;
				pid_pitch_output = 0.0;
				m1_speed = 0.0;
				m2_speed = 0.0;
				armed = (HAL_Imu_Angle(IMU_ANGLE_PITCH)>-1.5) && (HAL_Imu_Angle(IMU_ANGLE_PITCH)<1.5);
				if(armed)
				{
				}
			}

			if(0 && HAL_Imu_Get_State()!=0)
			{
				int16_t * raw_data = HAL_Imu_Raw_Sensor_Data();
				HAL_Serial_Print(&com,"%d %d %d\n",raw_data[6],raw_data[7],raw_data[8]);

			}
			if(0)
			{
				float * scaled_data = HAL_Imu_Scaled_Sensor_Data();
								HAL_Serial_Print(&com,"%d %d %d\n",(int32_t)(1000.0*scaled_data[6]),(int32_t)(1000.0*scaled_data[7]),(int32_t)(1000.0*scaled_data[8]));
			}
			if(1)
			{
				static int32_t tx_data_buffer_int[32];
				tx_data_buffer_int[0]=(int32_t)(HAL_Imu_Scaled_Sensor_Data()[1]*180/3.1415*10.0);
				tx_data_buffer_int[1]=(int32_t)(HAL_Imu_Angle(IMU_ANGLE_PITCH)*10.0);
				tx_data_buffer_int[2]=(int32_t)(distance*1000.0);
				tx_data_buffer_int[3]=(int32_t)(pid_x_output*10.0);
				tx_data_buffer_int[4]=(int32_t)pid_pitch_output;
				tx_data_buffer_int[5]=(int32_t)(e_ptich_correction.mean*100.0);
				tx_data_buffer_int[6]=(int32_t)(actual_xspeed*1000.0);
				tx_data_buffer_int[7]=0;
				int enc_ret = protocol_encode(
						tx_protocol_buffer,
						CMD_TELEMETRY_TICK,
						(uint8_t const*)tx_data_buffer_int,
						7*4 );
				if(enc_ret>0)
					HAL_Serial_Write(&com, tx_protocol_buffer, enc_ret );
			}
	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART8|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Uart8ClockSelection = RCC_UART8CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x6000030D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 107;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 107;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 215;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0xffff;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim8, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART8 init function */
static void MX_UART8_Init(void)
{

  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_nCS_Pin|M1_STEP_Pin|M2_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M2_DIR_Pin|M1_DIR_Pin|M2_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M1_ENABLE_GPIO_Port, M1_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_nRESET_GPIO_Port, SPI_nRESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : L6470_nBusy_Pin L6470_nFlag_Pin */
  GPIO_InitStruct.Pin = L6470_nBusy_Pin|L6470_nFlag_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_nCS_Pin */
  GPIO_InitStruct.Pin = SPI1_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_nCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_STEP_Pin M2_STEP_Pin */
  GPIO_InitStruct.Pin = M1_STEP_Pin|M2_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M2_DIR_Pin M1_DIR_Pin M2_ENABLE_Pin */
  GPIO_InitStruct.Pin = M2_DIR_Pin|M1_DIR_Pin|M2_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : M1_ENABLE_Pin */
  GPIO_InitStruct.Pin = M1_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M1_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_nRESET_Pin */
  GPIO_InitStruct.Pin = SPI_nRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_nRESET_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
