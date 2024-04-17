/*
 * keypad.c
 *
 *  Created on: Apr 15, 2024
 *      Author: kassy
 */

// ------------------------ keypad.c file begins here -----------------------
#include <keypad.h>
#include <stdint.h>
#include <main.h>

//int KEYPAD_PORT, NUM_OF_ROWS, COL_PORT, ROW_PORT;


/*void Delay(int delay_time) {                      	// quick delay if key was not pressed for a long time
	for(int delay = 0; delay < delay_time; delay++); */


void Keypad_Config(void)  {
RCC->AHB2ENR |= (RCC_AHB2ENR_GPIODEN);
RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
// first four should be rows (PD), second four should be columns (PB)
GPIOD->MODER &= ~(GPIO_MODER_MODE3); // Clear bits
GPIOD->MODER |= (GPIO_MODER_MODE3_0);

GPIOD->MODER &= ~(GPIO_MODER_MODE4); // Clear bits
GPIOD->MODER |= (GPIO_MODER_MODE4_0);

GPIOD->MODER &= ~(GPIO_MODER_MODE5); // Clear bits
GPIOD->MODER |= (GPIO_MODER_MODE5_0);

GPIOD->MODER &= ~(GPIO_MODER_MODE6); // Clear bits
GPIOD->MODER |= (GPIO_MODER_MODE6_0);
// PB0 (CN 10 PIN D33)
GPIOB->MODER &= ~(GPIO_MODER_MODE0); // Clear bits
GPIOB->MODER |= (GPIO_MODER_MODE0_0);
// PB2 (CN 9 PIN D70)
GPIOB->MODER &= ~(GPIO_MODER_MODE2); // Clear bits
GPIOB->MODER |= (GPIO_MODER_MODE2_0);
// PB6 (CN 9 PIN D71)
GPIOB->MODER &= ~(GPIO_MODER_MODE6); // Clear bits
GPIOB->MODER |= (GPIO_MODER_MODE6_0);
// PB10 (CH 10 PIN D36)
GPIOB->MODER &= ~(GPIO_MODER_MODE10); // Clear bits
GPIOB->MODER |= (GPIO_MODER_MODE10_0);

GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 |
		  	  	  	  GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);

// columns will be set without any pull down/up
GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 |
		  	  	  	  GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);
// rows have the pull down/up resistors
GPIOD->PUPDR   |= (GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 |
		  	  	  	  GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);

GPIOC->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED0_Pos) | (3 << GPIO_OSPEEDR_OSPEED1_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED2_Pos) | (3 << GPIO_OSPEEDR_OSPEED3_Pos));

// rows are the outputs and columns are inputs
GPIOD->BSRR = (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
GPIOB->BRR = (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
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
if ((GPIOD->IDR & COL_PINS) != 0 ) {       		// got a keypress!
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

   int8_t iRow=0, iCol=0, iKey=0;  // keypad row & col index, key ID result
   int8_t bGotKey = 0;             // bool for keypress, 0 = no press

   GPIOB->BSRR = ROW_PINS;                       	 // set all rows HI
   for ( iCol = 0; iCol < 4; iCol++ ) {      	 // check all COLUMNS
      if ( GPIOD->IDR & (GPIO_PIN_0 << iCol) ) {      	 // keypress in iCol!!
    	  GPIOB->BRR = ( ROW_PINS );    				// set all cols LO
         for ( iRow = 0; iRow < 4; iRow++ ) {   // 1 row at a time
        	 GPIOB->BSRR = ( GPIO_PIN_0 << (4+iRow) );     // set this row HI
            if ( GPIOD->IDR & (GPIO_PIN_0 << iCol) ) {    // keypress in iCol!!
               bGotKey = 1;
               break;                                  // exit for iRow loop
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
   if ( bGotKey ) {
      iKey = ( iRow * 4 ) + iCol + 1;  // handle numeric keys ...
      if ( iKey == 4 ) {                //    works for ‘*’, ‘#’ too
         GPIOC -> BSRR = iRow | ~(iCol << 16);
      }
 	return GPIOC -> BSRR = iRow | ~(iCol << 16);
   }		// nothing else -> read out the last iKey
    // turn on the LEDs?

    // return encoded keypress
    return -1;
   ///return( NO_KEYPRESS );                     // unable to verify keypress
}
