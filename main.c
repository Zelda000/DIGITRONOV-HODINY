/**
  ******************************************************************************
  * @file    main.c
  * @author  Ondřej Zelený & Tomáš Paseka, Brno University of Technology, Czechia
  * @version V1.0
  * @date    3.12.2018
  * @brief   Digitron clock
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "settings.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>         /* itoa() function */
#include <util/delay.h>
/* Constants and macros ------------------------------------------------------*/
uint8_t seconds;
uint8_t ones;
uint8_t tens;
uint8_t hours;
uint8_t days;
uint8_t month;

/* Function prototypes -------------------------------------------------------*/
void setup(void);

/* Global variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
  * @brief Main function.
*/
int main(void)
{
      // Initializations
    setup();

      // Enables interrupts by setting the global interrupt mask
    sei();

      // Forever loop
    while (1) {
      // Cycle here, do nothing, and wait for an interrupt
    }

    return 0;
}

//SETUP FUNCTION - SETUP OF I/O, TIMERS ETC.
void setup(void)
{

      // Set pins PB0,PB1,PB2,PB3 as outputs
    DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB3);
      // Outputs are equal to zero
    PORTB &= ~_BV(PB0) | ~_BV(PB1) | ~_BV(PB2) | ~_BV(PB3);

      // Set whole port D as output
    DDRD = 0xFF;
      // Clear every pin on port D
    PORTD = 0x00;;
      // Set pins PB0,PB1,PB2,PB3 as outputs
    DDRC |= _BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3)| _BV(PC4);
    PORTC &= ~_BV(PC0) | ~_BV(PC1) | ~_BV(PC2) | ~_BV(PC3)| ~_BV(PC4);

      // set pins 12 and 13 as inputs
    DDRB &= ~_BV(PB4)| ~_BV(PB5);
      // activate pull-up rezistor
    PORTB |= _BV(PB4)| _BV(PB5);

      // Timer/Counter1 setup
      // Clock prescaler 256 => overflows every 1.05 ms
    TCCR1B |= _BV(CS02);
      //overflow interrupt enable
    TIMSK1 |= _BV(TOIE0);
    TCNT1 = 0xBDC;


      //external pin-change interrupt
      // Enable pin change interrupt 0
    PCICR |= _BV(PCIE0);
      // Enable pin change interrupt at pin PCINT4(pin 12) and PCINT5(pin 13)
    PCMSK0 |= _BV(PCINT4)| _BV(PCINT5);

}

// TIMER 1 OVERFLOW FUNCTION
ISR(TIMER1_OVF_vect)
{
    seconds++;                        //increment seconds
    if (seconds == 60){               //if seconds = 60 - clear seconds,increment ones
      seconds = 0;
      ones++;
    }

    if (ones == 10){                 //if ones are equal to 10, clear ones, increment tens
      ones = 0;
      tens++;
    }

    if (tens == 6){                 //if tens are equal to 6 = 60mins, clear tens, increment hours
      tens = 0;
      hours++;
    }

    if (hours == 24){               // if hours are equal to 24, then clear hours
      hours = 0;
    }

    switch (hours){                       //output hours on port D (2 digitrons)
      case 0:PORTD = 0b00000000; break;
      case 1:PORTD = 0b00000001;break;
      case 2:PORTD = 0b00000010;break;
      case 3:PORTD = 0b00000011;break;
      case 4:PORTD = 0b00000100;break;
      case 5:PORTD = 0b00000101;break;
      case 6:PORTD = 0b00000110;break;
      case 7:PORTD = 0b00000111;break;
      case 8:PORTD = 0b00001000;break;
      case 9:PORTD = 0b00001001;break;
      case 10:PORTD = 0b00010000;break;
      case 11:PORTD = 0b00010001;break;
      case 12:PORTD = 0b00010010;break;
      case 13:PORTD = 0b00010011;break;
      case 14:PORTD = 0b00010100;break;
      case 15:PORTD = 0b00010101;break;
      case 16:PORTD = 0b00010110;break;
      case 17:PORTD = 0b00010111;break;
      case 18:PORTD = 0b00011000;break;
      case 19:PORTD = 0b00011001;break;
      case 20:PORTD = 0b00100000;break;
      case 21:PORTD = 0b00100001;break;
      case 22:PORTD = 0b00100010;break;
      case 23:PORTD = 0b00100011;break;
      default: PORTD = 0b00000000;break;
    }
    PORTC = ones;                         //output ones on port C
    PORTB = tens;                         //output ones on port B
    PORTB |= _BV(PB4)| _BV(PB5);          //reset pull-up on inputs on PB4 and PB5
    TCNT1 = 0xBDC;
}

ISR(PCINT4_vect)
{
    char one =0, zero =0, i;
    for(i=0;i<9;i++){                  //cycle 9x
      if(bit_is_clear(PINB, 5)){       //button is pressed
        ones++;
      }
      else {                           // button is not pressed
        zeroes++;
      }
    _delay_ms(10);                     // wait 10 ms
    }
    if (one>zero){                    // if one>zero  button was pressed
      seconds == 0;
      hours++;}                       // increase minutes
}

ISR(PCINT5_vect)
{

    char one =0, zero =0, i;
    for(i=0;i<9;i++){                  // cycle 9 times
      if(bit_is_clear(PINB, 4)){       // button is pressed
        ones++;
      }
      else {                           // button is not pressed
        zeroes++;
      }
    _delay_ms(10);                     // wait 10 ms
    }
    if (one>zero){                     // if one>zero  button was pressed
      seconds == 0;
      ones++;}                         // increase minutes
}

/* END OF FILE ****************************************************************/
