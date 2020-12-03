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
#define PINS_COUNT           (54u)
#define NUM_DIGITAL_PINS     (34u)
#define NUM_ANALOG_INPUTS    (1u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)

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
#define PIN_LED_14           (14u)
#define PIN_LED              PIN_LED_14
#define LED_BUILTIN          PIN_LED_14


/*
 * Analog pins
 */
#define PIN_A0               (17ul)
#define PIN_A1               (PIN_A0 + 1)

#define PIN_DAC0             PIN_A0
#define PIN_DAC1             PIN_A1

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		12


// Other pins
#define PIN_ATN              (27ul)
static const uint8_t ATN = PIN_ATN;

// Stepper driver pins
#define D0_EN                (29u)
#define D0_S0_STEP           (D0_EN + 1)
#define D0_S0_DIR            (D0_EN + 2)
#define D0_S1_STEP           (D0_EN + 3)
#define D0_S1_DIR            (D0_EN + 4)
#define D1_EN                (35u)
#define D1_S0_STEP           (D1_EN + 1)
#define D1_S0_DIR            (D1_EN + 2)
#define D1_S1_STEP           (D1_EN + 3)
#define D1_S1_DIR            (D1_EN + 4)

static const uint8_t D0EN  = D0_EN ;
static const uint8_t D0S0S = D0_S0_STEP ;
static const uint8_t D0S0D = D0_S0_DIR ;
static const uint8_t D0S1S = D0_S1_STEP ;
static const uint8_t D0S1D = D0_S1_DIR ;
static const uint8_t D1EN  = D1_EN ;
static const uint8_t D1S0S = D1_S0_STEP ;
static const uint8_t D1S0D = D1_S0_DIR ;
static const uint8_t D1S1S = D1_S1_STEP ;
static const uint8_t D1S1D = D1_S1_DIR ;

// Peripheral pins
#define Servo_0              (41u)
#define Servo_1              (Servo_0 + 1)
#define Servo_2              (Servo_0 + 2)
#define Gate_0               (17u)
#define Gate_1               (Gate_0 + 1)

static const uint8_t S0 = Servo_0 ;
static const uint8_t S1 = Servo_1 ;
static const uint8_t S2 = Servo_2 ;
static const uint8_t G0 = Gate_0 ;
static const uint8_t G1 = Gate_1 ;

/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (15ul)
#define PIN_SERIAL1_TX       (16ul)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (21u)
#define PIN_SPI_MOSI         (22u)
#define PIN_SPI_SCK          (23u)
#define PERIPH_SPI           sercom1
#define PAD_SPI_TX           SPI_PAD_3_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_2

#define SD_CHIP_SEL          (40u)
#define D0_CHIP_SEL          (28u)
#define D1_CHIP_SEL          (34u)

static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

static const uint8_t SDCS	= SD_CHIP_SEL;
static const uint8_t D0CS	= D0_CHIP_SEL;
static const uint8_t D1CS	= D1_CHIP_SEL;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (19u)
#define PIN_WIRE_SCL         (20u)
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler
#define WIRE_IT_HANDLER_0    SERCOM2_0_Handler
#define WIRE_IT_HANDLER_1    SERCOM2_1_Handler
#define WIRE_IT_HANDLER_2    SERCOM2_2_Handler
#define WIRE_IT_HANDLER_3    SERCOM2_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (24ul)
#define PIN_USB_DM          (25ul)
#define PIN_USB_DP          (26ul)

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

