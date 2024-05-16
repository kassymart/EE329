/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void SystemClock_Config(void);
void SPI_init(void);
void DAC_init(void);
uint16_t DAC_volt_conv(uint16_t voltage);
void DAC_write(uint32_t volt_code);
void delay_us(const uint32_t time_us);


int main(void)
{

  HAL_Init();
  SystemClock_Config();
  DAC_init();
 // SPI_init();


  while (1)
  {
	uint16_t volt_1 = 1000;
	uint16_t volt_2 = 2000;
    DAC_write(DAC_volt_conv(volt_1));
    delay_us(75);
    DAC_write(DAC_volt_conv(volt_2));
    delay_us(225);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SPI_init(void) {
   // SPI config as specified @ STM32L4 RM0351 rev.9 p.1459
   // called by or with DAC_init()
   // build control registers CR1 & CR2 for SPI control of peripheral DAC
   // assumes no active SPI xmits & no recv data in process (BSY=0)
   // CR1 (reset value = 0x0000)
   DAC_init();
   SPI1->CR1 &= ~( SPI_CR1_SPE );             	// disable SPI for config
   SPI1->CR1 &= ~( SPI_CR1_RXONLY );          	// recv-only OFF
   SPI1->CR1 &= ~( SPI_CR1_LSBFIRST );        	// data bit order MSb:LSb
   SPI1->CR1 &= ~( SPI_CR1_CPOL | SPI_CR1_CPHA ); // SCLK polarity:phase = 0:0
   SPI1->CR1 |=	 SPI_CR1_MSTR;              	// MCU is SPI controller
   // CR2 (reset value = 0x0700 : 8b data)
   SPI1->CR2 &= ~( SPI_CR2_TXEIE | SPI_CR2_RXNEIE ); // disable FIFO intrpts
   SPI1->CR2 &= ~( SPI_CR2_FRF);              	// Moto frame format
   SPI1->CR2 |=	 SPI_CR2_NSSP;              	// auto-generate NSS pulse
   SPI1->CR2 |=	 SPI_CR2_DS;                	// 16-bit data
   SPI1->CR2 |=	 SPI_CR2_SSOE;              	// enable SS output
   // CR1
   SPI1->CR1 |=	 SPI_CR1_SPE;               	// re-enable SPI for ops
}

void DAC_init(void) {
	// enable clock for GPIOA & SPI1
		RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);                // GPIOA: DAC NSS/SCK/SDO
		RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);                 // SPI1 port
		/* USER ADD GPIO configuration of MODER/PUPDR/OTYPER/OSPEEDR registers HERE */
		// configure AFR for SPI1 function (1 of 3 SPI bits shown here)
		GPIOA->AFR[0] &= ~((0x000F << (GPIO_AFRL_AFSEL7_Pos)) |(0x000F << (GPIO_AFRL_AFSEL5_Pos)) |(0x000F << (GPIO_AFRL_AFSEL4_Pos))); // clear nibble for bit 7 AF
		GPIOA->AFR[0] |=  ((0x0005 << (GPIO_AFRL_AFSEL7_Pos)) |(0x0005 << (GPIO_AFRL_AFSEL5_Pos)) | (0x0005 << (GPIO_AFRL_AFSEL4_Pos))); // set b7 AF to SPI1 (fcn 5)
	    GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 |GPIO_MODER_MODE7);
	    GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1);
	    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4 |GPIO_OTYPER_OT5 | GPIO_OTYPER_OT7);
	    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5  | GPIO_PUPDR_PUPD7);
	    GPIOA->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED4_Pos) | (3 << GPIO_OSPEEDR_OSPEED5_Pos) | (3 << GPIO_OSPEEDR_OSPEED7_Pos));
}



uint16_t DAC_volt_conv(uint16_t voltage) {
	uint16_t Vref = 3300;
//	uint16_t control = (0x03 << 12);
//	uint16_t Vout = (Vref * voltage * 1000) / (4095);
//	return (control | Vout);

	  if (voltage > Vref) {
	    voltage = Vref;
	  }

	  // convert to digital value
	  uint32_t Vout = (voltage * 4095 * 0.805) / Vref;
	  return (Vout - 6);
}



void DAC_write(uint32_t volt_code) {
	//uint16_t check;
	while (!(SPI1->SR & SPI_SR_TXE)){

	}
	SPI1->DR = volt_code + 0x1000;
	//if (SPI1->SR & SPI_SR_RXNE){
	//	check = SPI1->DR;
	// }

}
	// ignore the first 4 MSB
	// read out the 12 bits
//	SPI1->CR2 &= ~(SPI_CR2_NSSP);	// setting the CS pin low
//	SPI1->DR |= (SPI_CR2_DS); 		// set data in the data register
	//if ()

//}

void delay_us(const uint32_t time_us) {
	// set the counts for the specified delay
	SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
	SysTick->VAL = 0;                                  	 // clear timer count
	SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);    	 // clear count flag
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // wait for flag
}


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
