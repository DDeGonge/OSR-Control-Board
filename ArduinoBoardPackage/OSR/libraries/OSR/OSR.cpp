#include <Arduino.h>
#include "OSR.h"

TMC2041::TMC2041(uint8_t en_pin, uint8_t cs_pin, uint8_t step0_pin, uint8_t step1_pin, uint8_t dir0_pin, uint8_t dir1_pin)
{
    // Save and set up CS and enable pins
    CS_PIN = cs_pin;
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    EN_PIN = en_pin;
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);

    // Start SPI if it aint running - TODO maybe this is bad idk
    SPI.begin();

    // Configure driver with defaults
    write_all();

    // Configure contained steppers
    motor0.set_pins(step0_pin, dir0_pin);
    motor1.set_pins(step1_pin, dir1_pin);
}

// Public - enable stepper driver
void TMC2041::enable()
{
  digitalWrite(EN_PIN, LOW);
}

// Public - disable stepper motor
void TMC2041::disable()
{
  digitalWrite(EN_PIN, HIGH);
}

void TMC2041::write_cmd(uint8_t addr, uint8_t chunk[4])
{
    SPI.beginTransaction(TMCspiSettings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer(addr + 0x80);
    for (uint8_t i = 0; i < 4; i++)
        SPI.transfer(chunk[i]);

    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);
}

int32_t TMC2041::read_cmd(uint8_t addr)
{
    SPI.beginTransaction(TMCspiSettings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer(addr);
    SPI.transfer16(0x0000);
    SPI.transfer16(0x0000);

    digitalWrite(CS_PIN, HIGH);
    digitalWrite(CS_PIN, LOW);
    
    uint8_t status_resp = SPI.transfer(addr);
    uint32_t out = SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);

    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);

    return out;
}

void TMC2041::write_gconf()
{
    write_cmd(a_gconf, gconf);
}

void TMC2041::write_iholdirun(uint8_t motor_index)
{
    if (motor_index == 0)
        write_cmd(a_iholdirun[0], motor0.ihold);
    else if (motor_index == 1)
        write_cmd(a_iholdirun[1], motor1.ihold);
}

void TMC2041::write_chop(uint8_t motor_index)
{
    if (motor_index == 0)
        write_cmd(a_chop[0], motor0.chop);
    else if (motor_index == 1)
        write_cmd(a_chop[1], motor1.chop);
}

void TMC2041::write_cool(uint8_t motor_index)
{
    if (motor_index == 0)
        write_cmd(a_cool[0], motor0.cool);
    else if (motor_index == 1)
        write_cmd(a_cool[1], motor1.cool);
}

void TMC2041::write_all()
{
    write_gconf();
    for(uint8_t i = 0; i < 2; i++)
    {
        write_iholdirun(i);
        write_chop(i);
        write_cool(i);
    }
}



TMCstep::TMCstep(uint8_t index)
{
    set_index(index);
}

TMCstep::TMCstep(uint8_t index, uint8_t step_pin, uint8_t dir_pin)
{
    set_index(index);
    set_pins(step_pin, dir_pin);
}

void TMCstep::set_pins(uint8_t step_pin, uint8_t dir_pin)
{
    STEP_PIN = step_pin;
    pinMode(STEP_PIN, OUTPUT);
    digitalWrite(STEP_PIN, LOW);
    DIR_PIN = dir_pin;
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
}

void TMCstep::set_index(uint8_t index)
{
    INDEX = index;
}

uint8_t TMCstep::get_index()
{
    return INDEX;
}

void TMCstep::step()
{
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(step_pulse_len_us);
    digitalWrite(STEP_PIN, LOW);

    step_count = motor_dir ? step_count + 1 : step_count - 1;
}

int32_t TMCstep::get_step()
{
    return step_count;
}

void TMCstep::set_dir(bool dir)
{
    motor_dir = dir;
    if (motor_dir)
        digitalWrite(DIR_PIN, HIGH);
    else
        digitalWrite(DIR_PIN, LOW);
}

bool TMCstep::get_dir()
{
    return motor_dir;
}

