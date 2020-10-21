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

#ifndef _VARIANT_OSR_V1_
#define _VARIANT_OSR_V1_

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
#define PINS_COUNT           (45u)
#define NUM_DIGITAL_PINS     (20u)
#define NUM_ANALOG_INPUTS    (0u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1) // This is old bad nogood

#define FIRST_DIGITAL_PIN 26

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P + FIRST_DIGITAL_PIN].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P + FIRST_DIGITAL_PIN].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P + FIRST_DIGITAL_PIN].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P + FIRST_DIGITAL_PIN].ulTCChannel != NOT_ON_TIMER )

// // LEDs
#define PIN_LED_13           (39u)
// #define PIN_LED_RXL          (25u)
// #define PIN_LED_TXL          (26u)
#define PIN_LED              PIN_LED_13
// #define PIN_LED2             PIN_LED_RXL
// #define PIN_LED3             PIN_LED_TXL
#define LED_BUILTIN          PIN_LED_13
// #define PIN_NEOPIXEL         (8)

/*
 * Analog pins
 */
#define PIN_A0               (46ul)
// #define PIN_A1               (PIN_A0 + 1)
// #define PIN_A2               (PIN_A0 + 2)
// #define PIN_A3               (PIN_A0 + 3)
// #define PIN_A4               (PIN_A0 + 4)
// #define PIN_A5               (PIN_A0 + 5)
// #define PIN_A6               (PIN_A0 + 6)

#define PIN_DAC0             (11ul)
#define PIN_DAC1             (47ul)

static const uint8_t A0  = PIN_A0;
// static const uint8_t A1  = PIN_A1;
// static const uint8_t A2  = PIN_A2;
// static const uint8_t A3  = PIN_A3;
// static const uint8_t A4  = PIN_A4;
// static const uint8_t A5  = PIN_A5;
// static const uint8_t A6  = PIN_A6 ;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		12

// Stepper driver pins
#define D0_EN                (14u)
#define D0_S0_STEP           (D0_EN + 1)
#define D0_S0_DIR            (D0_EN + 2)
#define D0_S1_STEP           (D0_EN + 3)
#define D0_S1_DIR            (D0_EN + 4)
#define D1_EN                (20u)
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
#define Servo_0              (40u)
#define Servo_1              (Servo_0 + 1)
#define Servo_2              (Servo_0 + 2)
#define Gate_0               (Servo_0 + 3)
#define Gate_1               (Servo_0 + 4)

static const uint8_t S0 = Servo_0 ;
static const uint8_t S1 = Servo_1 ;
static const uint8_t S2 = Servo_2 ;
static const uint8_t G0 = Gate_0 ;
static const uint8_t G1 = Gate_1 ;

// AREF
#define PIN_ATN              (10ul)
static const uint8_t ATN = PIN_ATN;

/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (4u)
#define PIN_SPI_MOSI         (5u)
#define PIN_SPI_SCK          (6u)
#define PERIPH_SPI           sercom1
#define PAD_SPI_TX           SPI_PAD_3_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_2

#define SD_CHIP_SEL          (25u)
#define D0_CHIP_SEL          (13u)
#define D1_CHIP_SEL          (19u)

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

#define PIN_WIRE_SDA         (2u)
#define PIN_WIRE_SCL         (3u)
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
#define PIN_USB_HOST_ENABLE (7ul)
#define PIN_USB_DM          (8ul)
#define PIN_USB_DP          (9ul)

/*
 * I2S Interfaces
 */
// #define I2S_INTERFACES_COUNT 1

// #define I2S_DEVICE          0
// #define I2S_CLOCK_GENERATOR 3

// #define PIN_I2S_SDO          (11u)
// #define PIN_I2S_SDI          (12u)
// #define PIN_I2S_SCK          PIN_SERIAL1_TX
// #define PIN_I2S_FS           (10u)
// #define PIN_I2S_MCK          PIN_SERIAL1_RX

// On-board QSPI Flash
// #define EXTERNAL_FLASH_DEVICES   GD25Q16C
// #define EXTERNAL_FLASH_USE_QSPI

//QSPI Pins
// #define PIN_QSPI_SCK    (34u)
// #define PIN_QSPI_CS     (35u)
// #define PIN_QSPI_IO0    (36u)
// #define PIN_QSPI_IO1    (37u)
// #define PIN_QSPI_IO2    (38u)
// #define PIN_QSPI_IO3    (39u)

#if !defined(VARIANT_QSPI_BAUD_DEFAULT)
  // TODO: meaningful value for this
  #define VARIANT_QSPI_BAUD_DEFAULT 5000000
#endif

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

