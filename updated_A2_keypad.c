
/*******************************************************************************
* EE 329 A2 KEYPAD INTERFACE
*******************************************************************************
* @file : keypad.c
* @brief : keypad configuration and debounced detection of keypresses
* project : EE 329 S'24 Assignment 2
* authors : Kassandra Martinez-Mejia (KMM) -> kmart169@calpoly.edu
* version : ?
* date : 04/17/2024
* compiler : STM32CubeIDE v.1.15.0
* target : NUCLEO-L496ZG
* clocks : 4 MHz MSI to AHB2
* @attention : (c) 2023 STMicroelectronics. All rights reserved.
*******************************************************************************
. . .
* 45678-1-2345678-2-2345678-3-2345678-4-2345678-5-2345678-6-2345678-7-234567 */
/* USER CODE END Header */
// ------------------------ keypad.c file begins here -----------------------
#include <keypad.h>
#include <stdint.h>
#include <main.h>

//int KEYPAD_PORT, NUM_OF_ROWS, COL_PORT, ROW_PORT;


/*void Delay(int delay_time) {                      	// quick delay if key was not pressed for a long time
	for(int delay = 0; delay < delay_time; delay++); */

void Keypad_Config(void)  {
RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOEEN);
RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
// rows are the outputs and columns are inputs
GPIOE->BRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
GPIOB->BSRR = (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
// first four should be rows (PB), second four should be columns (PE)
// pins on board: PD0, PD1, PD2, PD3
GPIOE ->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3); // Clear bits
GPIOE ->MODER |= (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);

// PB0, PB1, PB2, PB3
GPIOB->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7); // Clear bits
GPIOB->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0);

GPIOE->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 |
		  	  	  	  GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);

GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 |
		  	  	  	  GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
// columns will be set without any pull down/up
GPIOE->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 |
		  	  	  	  GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);
// rows have the pull down/up resistors
GPIOE->PUPDR   |= (GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 |
		  	  	  	  GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);

GPIOE->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED0_Pos) | (3 << GPIO_OSPEEDR_OSPEED1_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED2_Pos) | (3 << GPIO_OSPEEDR_OSPEED3_Pos));

GPIOB->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED4_Pos) | (3 << GPIO_OSPEEDR_OSPEED5_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED6_Pos) | (3 << GPIO_OSPEEDR_OSPEED7_Pos));

}
// -----------------------------------------------------------------------------
int Keypad_IsAnyKeyPressed(void) {
// drive all COLUMNS HI; see if any ROWS are HI
// return true if a key is pressed, false if not
// currently no debounce here - just looking for a key twitch

	GPIOB -> BSRR = ROW_PINS;			      /// set all ROWs HI


	for (uint16_t idx=0; idx < 3 ; idx++ ) {  	// let it settle! | settle time is 3 "seconds"
      ;
	}
if ((GPIOE->IDR & COL_PINS) != 0 ) {       		// got a keypress!
      return 1;									// return true if key is pressed
   }
   else {
      return 0;                          		// nope.
   }
}

// -----------------------------------------------------------------------------
int Keypad_WhichKeyIsPressed(void) {
// detect and encode a pressed key at {row,col}
// assumes a previous call to Keypad_IsAnyKeyPressed() returned TRUE
// verifies the Keypad_IsAnyKeyPressed() result (no debounce here),
// determines which key is pressed and returns the encoded key ID

   int8_t iRow=0, iCol=0, iKey=0;  								// keypad row & col index, key ID result
   int8_t bGotKey = 0;             								// bool for keypress, 0 = no press

   GPIOB->BSRR = ROW_PINS;                       	 			// set all rows HI
   for ( iCol = 0; iCol < 4; iCol++ ) {      	 				// check all COLUMNS
      if ( GPIOE->IDR & (GPIO_PIN_0 << iCol) ) {      	 		// keypress in iCol!!
    	  GPIOB->BRR = ( ROW_PINS );    						// set all cols LO
         for ( iRow = 0; iRow < 4; iRow++ ) {   				// 1 row at a time
        	 GPIOB->BSRR = ( GPIO_PIN_0 << (4+iRow) );     		// set this row HI
            if ( GPIOD->IDR & (GPIO_PIN_0 << iCol) ) {    		// keypress in iCol!!
               bGotKey = 1;
               break;                                  			// exit for iRow loop
            }
         }

         if ( bGotKey )
        	//Delay(100);									// set up debounce time!
            break;
         }
      }

   //	encode {iRow,iCol} into LED word : row 1-3 : numeric, ‘1’-’9’
   //	                                   row 4   : ‘*’=10, ‘0’=15, ‘#’=12
   //                                    no press: send NO_KEYPRESS
 /*  int iKey_nums [iRow] [iCol] = {{0x1, 0x2, 0x3, 0xA}, {0x4, 0x5, 0x6, 0xB}, {0x7, 0x8, 0x9, 0xC}, {0XE, 0x0, 0xF, 0xD}
   };*/

   if ( bGotKey ) {
      iKey = (iRow * 4) + iCol + 1;  // handle numeric keys ...
      if ( iKey == 4 ) {                //    works for ‘*’, ‘#’ too
         GPIOC -> BSRR = iKey;
      }
 	return GPIOC -> BSRR = iRow | ~(iCol << 16);
   }		// nothing else -> read out the last iKey
    // turn on the LEDs?

    // return encoded keypress
    return -1;
   ///return( NO_KEYPRESS );                     // unable to verify keypress
}


// polling the rows to set the logic high for open circuit state
// polling the rows to set the logic low to detect a keypress
int poll_for_row(void)
{
	if (GPIOD -> IDR & (GPIO_PIN_4))
	{
			return 1;
	}
	if (GPIOD -> IDR & (GPIO_PIN_5))
	{
			return 2;
	}
	if (GPIOD -> IDR & (GPIO_PIN_6))
	{
			return 3;
	}
	else
	{
			return 4;
	}

}

