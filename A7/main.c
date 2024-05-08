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
#include <stdio.h>

void SystemClock_Config(void);
void LPUART_Print();
void LPUART_Esc();
void LPUART1_IRQHandler();
void delay_us(const uint32_t time_us);
void splash_screen(void);
void update_position(void);
void pop_bubbles();

int row = 20;
int col = 40;

int main(void)
{
  const int baud_divisor = 0x115C; // calculated LPUARTDIV, clock is 2 MHz

  HAL_Init();
  SystemClock_Config();

  PWR->CR2 |= (PWR_CR2_IOSV);              // power avail on PG[15:2] (LPUART1)
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);   // enable GPIOG clock
  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; // enable LPUART clock bridge

  /* USER: configure GPIOG registers MODER/PUPDR/OTYPER/OSPEEDR then
     select AF mode and specify which function with AFR[0] and AFR[1] */
  GPIOG->AFR[0] &= ~(GPIO_AFRL_AFSEL7);
  GPIOG->AFR[0] |= (GPIO_AFRL_AFSEL7_3); // configure PG7 (TX)

  GPIOG->AFR[1] &= ~(GPIO_AFRH_AFSEL8);
  GPIOG->AFR[1] |= (GPIO_AFRH_AFSEL8_3); // configure PG8 (RX)

  GPIOG->OTYPER &= ~(GPIO_OTYPER_OT7); // TX push-pull

  GPIOG->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED7_Pos)
                   | (3 << GPIO_OSPEEDR_OSPEED8_Pos)); //highest speed

  GPIOG->PUPDR |= (GPIO_PUPDR_PUPD8_1); //no pull up/pull down

  GPIOG->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);
  GPIOG->MODER |= (GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1);

  LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit data
  LPUART1->CR1 |= USART_CR1_UE;                   // enable LPUART1
  LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // enable xmit & recv
  LPUART1->CR1 |= USART_CR1_RXNEIE;        // enable LPUART1 recv interrupt
  LPUART1->ISR &= ~(USART_ISR_RXNE);       // clear Recv-Not-Empty flag

  /* USER: set baud rate register (LPUART1->BRR) */
  LPUART1->BRR = baud_divisor;

  NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));   // enable LPUART1 ISR
  __enable_irq();                          // enable global interrupts
  	  LPUART_Esc("2J");
  	  LPUART_Esc("H");
  	  LPUART_Esc("0J");
  	  LPUART_Esc("3B");
  	  LPUART_Esc("5C");
  	  LPUART_Print("All good students read the");
  	  LPUART_Esc("1B");
  	  LPUART_Esc("21D");
  	  LPUART_Esc("5m");
  	  LPUART_Print("reference manual");
  	  LPUART_Esc("H");
  	  LPUART_Esc("0m");
  	  LPUART_Print("Input:");

  	LPUART_Esc("2J");
	splash_screen();
	delay_us(2000000);
	LPUART_Esc("0m");    // remove attributes
	update_position();


  while (1)
  {

  }
}

void delay_us(const uint32_t time_us) {
	// set the counts for the specified delay
	SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
	SysTick->VAL = 0;                                  	 // clear timer count
	SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);    	 // clear count flag
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // wait for flag
}


void LPUART_Print(const char* message ) {
   uint16_t iStrIdx = 0;
   while ( message[iStrIdx] != 0 ) {
      while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
         ;
      LPUART1->TDR = message[iStrIdx];       // send this character
	iStrIdx++;                             // advance index to next char
   }
}

void LPUART_Esc(const char* code) {
	LPUART_Print("\x1B["); //Escape code heading
	LPUART_Print(code); //escape code value
}

// referenced from James Savella | 'S23 EE329 Student
void update_position(){
	LPUART_Esc("2J");	//clear all
	LPUART_Esc("1m");	// make the char bolder

	char str[8];
	sprintf(str, "%d;%dH",  row, col);	//able to see the char move
	LPUART_Esc(str);
	LPUART_Print("X");
}

void splash_screen(void) {

	__disable_irq();			//disables input

	LPUART_Esc("11;30H"); //center
	LPUART_Esc("32m");	//blinking mode (sometimes it blinks)
	LPUART_Print("The Adventures of X");		//Blink Penvene's plaything
	LPUART_Esc("34m");

	//Heart
	//pop_bubbles();
	LPUART_Esc("18;40H");	//line:column
	LPUART_Print("*");
	LPUART_Esc("17;39H");
	LPUART_Print("* *");
	LPUART_Esc("16;38H");
	LPUART_Print("*   *");
	LPUART_Esc("15;37H");
	LPUART_Print("*     *");
	LPUART_Esc("14;36H");
	LPUART_Print("*   *   *");
	LPUART_Esc("13;36H");
	LPUART_Print("* *   * *");

	__enable_irq();                          // enable global interrupts
}


//void pop_bubbles(){
//	LPUART_Esc("37m");
//	LPUART_Esc("18;40H");	//line:column
//	LPUART_Print("*");
//	LPUART_Esc("32m");
//
//}
void LPUART1_IRQHandler(void) {
   uint8_t charRecv;
   if (LPUART1->ISR & USART_ISR_RXNE) {
      charRecv = LPUART1->RDR;
      switch ( charRecv ) {
	     case 'R':
            //for colors, format is '3xm' (1<x<7)
		    LPUART_Esc("31m");//x=1 is red
	        break;

	     case 'G':
	        LPUART_Esc("32m"); //x=2 is green
	     	break;

	     case 'B':
	    	LPUART_Esc("34m"); //x=4 is blue
	    	break;

	     case 'W':
		 	LPUART_Esc("37m"); //x=7 is white
	     	break;

	     case 'w':
	    	row -= 1;
	    	if (row > 40){
	    		row = 1;
	    	}
	    	break;
	     case 'a':
	    	 col -= 1;
		     if (col < 1){
			    col = 80;
			 }
			 break;
	     case 's':
	    	 row += 1;
	    	  if (row > 40){
	    		row = 1;
	    	  }
	    	 break;
	     case 'd':
	    	 col += 1;
	    	if (col > 80){
	           col = 1;
	    	}
	    	 break;
	   default:
	      while( !(LPUART1->ISR & USART_ISR_TXE) )
               ;    // wait for empty TX buffer
		LPUART1->TDR = charRecv;  // echo char to terminal
	}  // end switch
   }
   update_position();
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


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
