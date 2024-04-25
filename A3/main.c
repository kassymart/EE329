
#include "main.h"
#include "keypad.h"
#include "lcd.h"
#include "ctype.h"

void SystemClock_Config(void);

int main(void)
{


  HAL_Init();

  SystemClock_Config();
  Keypad_Init();
  Lcd_Init();


  Lcd_write_string("EE 329 A3 TIMER", 0);
  char bottom_string[] = "*=SET #=GO 00:00";


  uint8_t state = 0;
  uint8_t time_idx;

  uint8_t current_input;
  uint8_t last_input = '\0';

  uint8_t minutes;
  uint8_t seconds;


  while (1)
  {
	  //Get currently pressed key
	  current_input = Keypad_Read();

	  if (last_input != current_input){
		  if(current_input == '*'){
			  state = 0;
		  }
	  }

	  Lcd_write_string(bottom_string, 1);

	  //debug:
//	  bottom_string[10] = state | 0x30;

	  switch (state){

	  case 0: //Idle or 'Reset' state
		  Lcd_backlight_on();
		  Lcd_write_string("EE 329 A3 TIMER", 0);
		  bottom_string[11] = '0';
		  bottom_string[12] = '0';
		  bottom_string[14] = '0';
		  bottom_string[15] = '0';
		  minutes = 0;
		  seconds = 0;
		  time_idx = 0;

		  delay_us(500000);
		  if (last_input != current_input){
		  		  if(current_input == '*'){
		  			  state++;
		  			  break;
		  		  }
		  }
		  break;
	  case 1:

		  // Let user know that they can input now
		  Lcd_write_string("Enter a time:", 0);

		  //If the current input is a number
		  if (isdigit(current_input) && current_input != '\0' && current_input != '#'){
			  //And if there is an input delta
			  if (last_input != current_input){
				  //if the index is at the first 2 digits
				  if(time_idx < 2){
					  bottom_string[time_idx + 11] = current_input;
				  }
				  //if the index is at the last 2 digits
				  else{
					  bottom_string[time_idx + 12] = current_input;
				  }
				  //Add to time digit index
				  time_idx++;
			  }

		  }
		  //Check if we have reached the 4th digit
		  if (time_idx >= 4) {
			  //Jump to state 2
			  state++;
			  //Number conditioning
			  if( ((int)bottom_string[11] & 0x0f) > 5){
				  bottom_string[11] = '5';
			  }
			  if( ((int)bottom_string[14] & 0x0f) > 5){
				  bottom_string[14] = '5';
			  }

			  minutes = ((bottom_string[11] & 0x0f) * 10) + (bottom_string[12] & 0x0f);
			  seconds = ((bottom_string[14] & 0x0f) * 10) + (bottom_string[15] & 0x0f);
			  break;
		  }
		  break;

	  case 2:
		  Lcd_write_string("'#' to start", 0);
		  if(Keypad_Read() == '#'){
			  state++;
		  }


		  break;
	  case 3:
		  Lcd_write_string("Counting down!", 0);

		  if (minutes == 0 && seconds == 0){
			  state++;
			  break;

		  }
		  else if(seconds == 0){
			  seconds = 59;
			  --minutes;
		  }

		  else{
			  seconds = seconds - 1;
		  }

		  bottom_string[11] = (minutes / 10) | 0x30;
		  bottom_string[12] = (minutes - (minutes / 10)*10) | 0x30;


		  bottom_string[14] = (seconds / 10) | 0x30;
		  bottom_string[15] = (seconds - (seconds / 10)*10) | 0x30;

		  delay_us(1000000);
		  break;
	  case 4:
		  Lcd_write_string("Timer done!!", 0);
		  Lcd_backlight_off();
		  delay_us(250000);
		  Lcd_backlight_on();
		  delay_us(250000/2);
		  Lcd_backlight_on();
		  delay_us(250000/4);
		  Lcd_backlight_on();
		  delay_us(250000);
		  Lcd_backlight_off();
		  delay_us(250000);
		  break;


	  }

	  //Record last known input for checking delta later
	  last_input = current_input;
  }
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
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
