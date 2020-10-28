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
    if (motor_dir != dir)
    {
        motor_dir = dir;
        if (motor_dir)
            digitalWrite(DIR_PIN, HIGH);
        else
            digitalWrite(DIR_PIN, LOW);
    }
}

bool TMCstep::get_dir()
{
    return motor_dir;
}



// Constructor
motorDrive::motorDrive(TMCstep &new_stepper)
{
    stepper = new_stepper;
}

// Public - Update stepper parameters. Feed NOVALUE to not change any particular parameter
void motorDrive::update_config(int32_t steps_per_mm_new, float max_vel_new, float max_accel_new)
{
    if (max_vel_new != NOVALUE)
        max_vel = max_vel_new;
    if (max_accel_new != NOVALUE)
        max_accel = max_accel_new;
    if (steps_per_mm_new != NOVALUE)
    {
        steps_per_mm = steps_per_mm_new;
        step_size_mm = 1 / steps_per_mm;
    }
}

// Public - Overwrite the current position to be any mm value designated
void motorDrive::set_current_pos_mm(double target)
{
    target = target == NOVALUE ? 0 : target;
    double working_count = target * steps_per_mm;
    working_count += 0.4999; // For the rounding
    current_step_count = (int32_t) working_count;
}

// Public - Update target position in mm. Respects present joint momentum.
void motorDrive::set_pos_target_mm_async(double target, float feedrate = NOVALUE)
{
    target_mm = target;
    next_step_us = micros();
    if (feedrate != NOVALUE)
        max_vel = feedrate;
}

// Public - Update target position in mm. Respects present joint momentum. Will stay in loop until move is compelte
void motorDrive::set_pos_target_mm_sync(double target, float feedrate = NOVALUE)
{
    set_pos_target_mm_async(target, feedrate);
    while(true)
    {
        if(!step_if_needed())
            break;
    }
}

// Public - The magic sauce. Tracks motor motion and calculates if a step is needed now to stay on track.
bool motorDrive::step_if_needed()
{
    uint32_t t_now = micros();
    int32_t step_target = (steps_per_mm * target_mm);

    // Check if motor is in right place already
    if((abs(current_velocity) < 0.001) && (step_target == current_step_count))
        return false;
    else if((abs(current_velocity) < 0.001) && (current_step_count > step_target))
        stepper.set_dir(false);
    else if((abs(current_velocity) < 0.001) && (current_step_count < step_target))
        stepper.set_dir(true);
    
    if(micros() > next_step_us)
    {
        stepper.step();

        uint32_t cur_step_us = next_step_us;
        double stop_dist_mm = pow(current_velocity, 2) / (max_accel);
        double stop_pos_mm = current_step_count / steps_per_mm;

        // This mess determines if we need to slow down
        if((current_dir && ((stop_pos_mm + stop_dist_mm) > target_mm)) || (!current_dir && ((stop_pos_mm - stop_dist_mm) < target_mm)))
        {
            // First check if we are coming to a stop
            if((pow(current_velocity, 2) - 2 * max_accel * step_size_mm) < 0)
            {
                // See if we should turn round
                if(current_step_count > step_target)
                {
                next_step_us = (2 * next_step_us) - last_step_us;
                stepper.set_dir(false);
                }
                else if(current_step_count < step_target)
                {
                next_step_us = (2 * next_step_us) - last_step_us;
                stepper.set_dir(true);
                }
                else
                {
                next_step_us = 4294967294;
                diff_exact_us = 4294967294;
                }
            }

            // Otherwise just decelerate normally
            else
            {
                double t0, t1;
                quad_solve(t0, t1, -max_accel, current_velocity, step_size_mm);
                double next_step_temp = 1000000 * min(t0, t1);
                diff_exact_us = next_step_temp;
                next_step_us = (uint32_t) (next_step_temp + 0.5);
                next_step_us += cur_step_us;
            }
        }

        // Otherwise check if we can speed up
        else if(abs(current_velocity) < max_vel)
        {
            // Quadratic has 2 roots, only use one that results in positive time
            double t0, t1;
            quad_solve(t0, t1, max_accel, current_velocity, step_size_mm);
            double next_step_temp = 1000000 * max(t0, t1);
            diff_exact_us = next_step_temp;
            next_step_us = (uint32_t) (next_step_temp + 0.5);
            next_step_us += cur_step_us;
        }

        // Last resort is maintain max speed
        else
        {
            next_step_us += 1000000 * step_size_mm / max_vel;
        }

        // Update current motor velocity
        current_velocity = step_size_mm * 1000000;
        current_velocity /= diff_exact_us;
        current_velocity = current_dir ? current_velocity : -current_velocity;
        current_velocity = abs(current_velocity) < 0.01 ? 0 : current_velocity;
        last_step_us = cur_step_us;
    }
    return true;
}

// Public - Return current position in mm
double motorDrive::get_current_pos_mm()
{
    return current_step_count / steps_per_mm;
}

// Public - Return current velocity in mm/sec
double motorDrive::get_current_vel_mmps()
{
    return current_velocity;
}

// Private - Quadratic equation yo
void motorDrive::quad_solve(double &t_0, double &t_1, double a, double b, double c)
{
    double temp0 = -abs(b);
    double temp1 = sqrt(pow(b, 2) + 2 * a * c);
    t_0 = (temp0 + temp1) / a;
    t_1 = (temp0 - temp1) / a;
}

void motorDrive::zero()
{
    set_current_pos_mm(0);
}
