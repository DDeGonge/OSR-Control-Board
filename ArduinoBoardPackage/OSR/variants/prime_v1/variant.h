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

#ifndef _VARIANT_FEATHER_M4_
#define _VARIANT_FEATHER_M4_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK        (F_CPU)

#define VARIANT_GCLK0_FREQ (F_CPU)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (53u)
#define NUM_DIGITAL_PINS     (35u)
#define NUM_ANALOG_INPUTS    (1u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1) // Currently unused

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
#define PIN_LED_5            (5u)
#define PIN_LED              PIN_LED_5
#define LED_BUILTIN          PIN_LED_5


/*
 * Analog pins
 */
// #define PIN_A0               (17ul)
// #define PIN_A1               (PIN_A0 + 1)

// #define PIN_DAC0             PIN_A0
// #define PIN_DAC1             PIN_A1

// static const uint8_t A0  = PIN_A0;
// static const uint8_t A1  = PIN_A1;

// static const uint8_t DAC0 = PIN_DAC0;
// static const uint8_t DAC1 = PIN_DAC1;

// #define ADC_RESOLUTION		12


// Other pins
#define PIN_ATN              (27ul)
static const uint8_t ATN = PIN_ATN;


// Stepper driver pins
#define S0_EN             (17u)
#define S0_CS             (S0_EN + 1)
#define S0_STEP           (S0_EN + 2)
#define S0_DIR            (S0_EN + 3)

#define S1_EN             (21u)
#define S1_CS             (S1_EN + 1)
#define S1_STEP           (S1_EN + 2)
#define S1_DIR            (S1_EN + 3)

#define S2_EN             (25u)
#define S2_CS             (S2_EN + 1)
#define S2_STEP           (S2_EN + 2)
#define S2_DIR            (S2_EN + 3)

#define S3S4_EN           (29u)
#define S3S4_CS           (S3S4_EN + 1)
#define S3_STEP           (S3S4_EN + 2)
#define S3_DIR            (S3S4_EN + 3)
#define S4_STEP           (S3S4_EN + 4)
#define S4_DIR            (S3S4_EN + 5)

#define S5S6_EN           (35u)
#define S5S6_CS           (S5S6_EN + 1)
#define S5_STEP           (S5S6_EN + 2)
#define S5_DIR            (S5S6_EN + 3)
#define S6_STEP           (S5S6_EN + 4)
#define S6_DIR            (S5S6_EN + 5)

static const uint8_t S0EN   = S0_EN ;
static const uint8_t S0CS   = S0_CS ;
static const uint8_t S0STEP = S0_STEP ;
static const uint8_t S0DIR  = S0_DIR ;

static const uint8_t S1EN   = S1_EN ;
static const uint8_t S1CS   = S1_CS ;
static const uint8_t S1STEP = S1_STEP ;
static const uint8_t S1DIR  = S1_DIR ;

static const uint8_t S2EN   = S2_EN ;
static const uint8_t S2CS   = S2_CS ;
static const uint8_t S2STEP = S2_STEP ;
static const uint8_t S2DIR  = S2_DIR ;

static const uint8_t S3S4EN = S3S4_EN ;
static const uint8_t S3S4CS = S3S4_CS ;
static const uint8_t S3STEP = S3_STEP ;
static const uint8_t S3DIR  = S3_DIR ;
static const uint8_t S4STEP = S4_STEP ;
static const uint8_t S4DIR  = S4_DIR ;

static const uint8_t S5S6EN = S5S6_EN ;
static const uint8_t S5S6CS = S5S6_CS ;
static const uint8_t S5STEP = S5_STEP ;
static const uint8_t S5DIR  = S5_DIR ;
static const uint8_t S6STEP = S6_STEP ;
static const uint8_t S6DIR  = S6_DIR ;


// Peripheral pins
#define Servo_0              (41u)
#define Servo_1              (Servo_0 + 1)

static const uint8_t SERV0 = Servo_0 ;
static const uint8_t SERV1 = Servo_1 ;

/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (5ul)
#define PIN_SERIAL1_TX       (6ul)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (11u)
#define PIN_SPI_MOSI         (12u)
#define PIN_SPI_SCK          (13u)
#define PERIPH_SPI           sercom1
#define PAD_SPI_TX           SPI_PAD_3_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_2

static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (9u)
#define PIN_WIRE_SCL         (10u)
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler
#define WIRE_IT_HANDLER_0    SERCOM2_0_Handler
#define WIRE_IT_HANDLER_1    SERCOM2_1_Handler
#define WIRE_IT_HANDLER_2    SERCOM2_2_Handler
#define WIRE_IT_HANDLER_3    SERCOM2_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * CAN
 */

// TODO

/*
 * USB
 */
// #define PIN_USB_HOST_ENABLE (24ul)
#define PIN_USB_DM          (7ul)
#define PIN_USB_DP          (8ul)

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0

#define I2S_DEVICE          0
#define I2S_CLOCK_GENERATOR 3

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial1;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_FEATHER_M4_ */

