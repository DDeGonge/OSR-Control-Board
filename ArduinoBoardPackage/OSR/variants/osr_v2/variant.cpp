/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

/*
 * Pins descriptions
 */

//TCC0 IOSET 6
//TCC1 IOSET 1
//TC0 IOSET 1
//TC1 IOSET 1
//TC2 IOSET 2
//TC3 IOSET 1
//TC4 IOSET 1
const PinDescription g_APinDescription[]=
{
  // 0..6 - Analog Pins
  { PORTB,  7, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel9, NOT_ON_TIMER, EXTERNAL_INT_7 },
  { PORTB,  8, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel0,  TC4_CH0, TC4_CH0, EXTERNAL_INT_8 },
  { PORTB,  9, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel3, TC4_CH1, TC4_CH1, EXTERNAL_INT_9 },
  { PORTA,  4, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel4, TC0_CH0, TC0_CH0, EXTERNAL_INT_4 },
  { PORTA,  5, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel5, TC0_CH1, TC0_CH1, EXTERNAL_INT_5 },
  { PORTA,  6, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel6, TC1_CH0, TC1_CH0, EXTERNAL_INT_6 },
  { PORTA,  7, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel7, TC1_CH1, TC1_CH1, EXTERNAL_INT_7 },

  // 7..14 - Digital Pins
  { PORTA, 10, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 11, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTB, 10, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },
  { PORTB, 11, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },
  { PORTB, 12, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },
  { PORTB, 13, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },
  { PORTB, 14, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },
  { PORTB, 15, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },

  // 15..16 - SERCOM/UART (Serial1)
  { PORTB, 17, PIO_SERCOM, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH4, NOT_ON_TIMER, EXTERNAL_INT_1 }, // RX: SERCOM5/PAD[1]
  { PORTB, 16, PIO_SERCOM, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH5, NOT_ON_TIMER, EXTERNAL_INT_0 }, // TX: SERCOM5/PAD[0]

  // 17..18 - Gates
  { PORTA, 8, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, TC0_CH0, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Gate 0
  { PORTA, 9, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, TC0_CH1, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Gate 1

  // 19..20 - I2C pins (SDA/SCL)
  { PORTA, 12, PIO_SERCOM, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH0, TC2_CH0, EXTERNAL_INT_12 }, // SDA: SERCOM2/PAD[0]
  { PORTA, 13, PIO_SERCOM, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH1, TC2_CH1, EXTERNAL_INT_13 }, // SCL: SERCOM2/PAD[1]

  // 21..23 - SPI pins (MISO,MOSI,SCK)
  { PORTB, 22, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // MISO: SERCOM1/PAD[2]
  { PORTB, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // MOSI: SERCOM1/PAD[3]
  { PORTA, 17, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // SCK: SERCOM1/PAD[1]

  // 24..26 - USB
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable DOES NOT EXIST ON THIS BOARD
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 27, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

  // 27 - AREF
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

  // 28..33 - Stepper driver 0 pins
  { PORTB,  6, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D0_CS
  { PORTB, 30, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D0_EN
  { PORTB, 31, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D0_S0_STEP
  { PORTB,  0, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D0_S0_DIR
  { PORTB,  5, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D0_S1_STEP
  { PORTB,  4, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D0_S1_DIR

  // 34..39 - Stepper driver 1 pins
  { PORTA, 18, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D1_CS
  { PORTA, 19, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D1_EN
  { PORTA, 20, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D1_S0_STEP
  { PORTA, 21, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D1_S0_DIR
  { PORTA, 22, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D1_S1_STEP
  { PORTA, 23, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // D1_S1_DIR

  // 40 - SD Chip Select
  { PORTA, 27,  PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // CS

  // 41..43 - Servo Pins
  { PORTA, 14, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // Servo0
  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // Servo1
  { PORTA, 16, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // Servo2

  // 44 - VSense
  { PORTB,  1, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5 } ;
const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM5_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM5_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM5_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM5_3_Handler()
{
  Serial1.IrqHandler();
}