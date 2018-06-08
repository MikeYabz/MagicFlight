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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */	
	
	
	
/************************************************************************************/
//	*		Initialize Timers foir PWM
/************************************************************************************/	
	HAL_TIM_Base_Init(&htim3);
		
	TIM3->CCR1 = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		
	TIM3->CCR2 = 0;
	HAL_TIMEx_PWMN_Start(&htim3,TIM_CHANNEL_2);
		
	TIM3->CCR4 = 0;
	HAL_TIMEx_PWMN_Start(&htim3,TIM_CHANNEL_4);		
/*************************************END********************************************/



		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		HAL_Delay(1);
		
		
		
		
/************************************************************************************/
//	*		Read Input and Output Voltages
/************************************************************************************/		
		uint16_t outputVoltageReading;
		uint16_t inputVoltageReading;	
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc,5);
		inputVoltageReading = HAL_ADC_GetValue(&hadc);
		HAL_ADC_PollForConversion(&hadc,5);
		outputVoltageReading = HAL_ADC_GetValue(&hadc);
		HAL_ADC_Stop(&hadc);
/*************************************END********************************************/
		



/************************************************************************************/
//	*		Calculate "Temperature" of Output Voltage
/************************************************************************************/

			//range should be from 0.6V(819) to 1.5V(2047)
		#define MINIMUM 819
		#define MAXIMUM 2047
		const uint16_t DIFFERENCE = MAXIMUM - MINIMUM;
		const double PWM_PERIOD = (double)htim3.Init.Period;
		double Red;
		double Green;
		double Blue;		
		double voltagePercentageOfMax; 

		
		voltagePercentageOfMax = (outputVoltageReading-MINIMUM)/DIFFERENCE;
		
					//Fade From Blue to Turquoise
		if (voltagePercentageOfMax < 0.25)
		{
			Red = 0;
			Green = PWM_PERIOD * (voltagePercentageOfMax / 0.25);
			Blue = PWM_PERIOD;
		}
		
				//Fade From Turquoise to Green
		else if ((voltagePercentageOfMax >= 0.25) && (voltagePercentageOfMax < 0.5))
		{
			Red = 0;
			Green = PWM_PERIOD;
			Blue = PWM_PERIOD - (PWM_PERIOD * ((voltagePercentageOfMax - 0.25) / 0.25));
		} 
		
				//Fade From Green to Yellow
		else if ((voltagePercentageOfMax >= 0.5) && (voltagePercentageOfMax < 0.75))
		{
			Red = (PWM_PERIOD * ((voltagePercentageOfMax - 0.50) / 0.25));
			Green = PWM_PERIOD;
			Blue = 0;
		} 
		
				//Fade From Yellow to Red
		else if (voltagePercentageOfMax > 0.75)
		{
			Red = PWM_PERIOD;
			Green = PWM_PERIOD - (PWM_PERIOD * ((voltagePercentageOfMax - 0.75) / 0.25));
			Blue = 0;
		}
	
		static double filteredRed = 0;
		static double filteredGreen = 0;
		static double filteredBlue = 0;
		
		#define LPF_Strength 100		
		filteredRed = (Red + (filteredRed * LPF_Strength-1))/LPF_Strength;
		filteredRed = (Green + (filteredGreen * LPF_Strength-1))/LPF_Strength;
		filteredRed = (Blue + (filteredBlue * LPF_Strength-1))/LPF_Strength;						
		
		int16_t pwmRed;
		int16_t pwmGreen;
		int16_t pwmBlue;		
		pwmRed = (uint16_t)filteredRed;
		pwmGreen = (uint16_t)filteredGreen;
		pwmBlue = (uint16_t)filteredBlue;		
		
			//Checking Boundaries of 0-(max pwm period) to make sure nothing broke
		if (pwmRed < 0)
		{
			pwmRed = 0;
		}
		else if (pwmRed > PWM_PERIOD)
		{
			pwmRed = PWM_PERIOD;
		}
		
		
		if (pwmGreen < 0)
		{
			pwmGreen = 0;
		}
		else if (pwmGreen > PWM_PERIOD)
		{
			pwmGreen = PWM_PERIOD;
		}
		
		
		if (pwmBlue < 0)
		{
			pwmBlue = 0;
		}
		else if (pwmBlue > PWM_PERIOD)
		{
			pwmBlue = PWM_PERIOD;
		}
/*************************************END********************************************/


		
		
		
		
/************************************************************************************/
//	*		Low Battery Indicator.  Flahses white 3 time quickly every 2.5s
/************************************************************************************/
		#define placeholder 0
		if (inputVoltageReading < placeholder)	//if input voltage is under threshold which means the battery is low
		{
			uint32_t loopTimer = HAL_GetTick() % 2500;	//will count to 2500ms then restart
			if ( 
				((loopTimer > 100) && (loopTimer < 200)) || 
				((loopTimer > 400) && (loopTimer < 500)) || 
				((loopTimer > 700) && (loopTimer < 800))	)			
			{
				pwmRed = PWM_PERIOD;
				pwmGreen = PWM_PERIOD;
				pwmBlue = PWM_PERIOD;
			}
		}	
/*************************************END********************************************/		
		
		
		
		
		
/************************************************************************************/
//	*		Set RGB PWMs
/************************************************************************************/		
		TIM3->CCR1 = pwmRed;
		TIM3->CCR2 = pwmGreen;
		TIM3->CCR4 = pwmBlue;
/*************************************END********************************************/	


		
		
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
