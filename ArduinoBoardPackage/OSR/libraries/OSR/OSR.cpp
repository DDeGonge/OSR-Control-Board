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
    motor0.set_index(0);
    motor1.set_pins(step1_pin, dir1_pin);
    motor1.set_index(1);
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

void TMC2041::update_driver_status(uint8_t motor_index)
{
    int32_t new_status = read_cmd(a_status[motor_index]);
    if (motor_index == 0)
        motor0.update_status(new_status);
    else if (motor_index == 1)
        motor1.update_status(new_status);
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

void TMCstep::set_step(int32_t steps)
{
    step_count = steps;
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

void TMCstep::update_status(uint32_t new_status)
{
    status_bits = new_status;
}



// Constructor
motorDrive::motorDrive(TMCstep &new_stepper, int32_t steps_per_mm_new)
{
    stepper = new_stepper;
    steps_per_mm = steps_per_mm_new;
    step_size_mm = 1 / steps_per_mm;
}

// Public - Update stepper default velocity in mm/s if input is not NOVALUE
void motorDrive::set_default_vel_mmps(float max_vel_new)
{
    if (max_vel_new != NOVALUE)
        max_vel = max_vel_new;
}

// Public - Update stepper default acceleration in mm/s^2 if input is not NOVALUE
void motorDrive::set_default_acc_mmps2(float max_accel_new)
{
    if (max_accel_new != NOVALUE)
        max_accel = max_accel_new;
}

// Public - Overwrite the current position to be any mm value designated
void motorDrive::set_current_pos_mm(double target)
{
    target = target == NOVALUE ? 0 : target;
    double working_count = target * steps_per_mm;
    working_count += 0.4999; // For the rounding
    int32_t current_step_count = (int32_t) working_count;
    stepper.set_step(current_step_count);
}

// Public - Set softstop limit for linear drive
void motorDrive::set_max_move_dist_mm(float new_lim_mm)
{
    max_dist_mm = new_lim_mm;
}

// Public - Update target position in mm. Respects present joint momentum.
void motorDrive::set_pos_target_mm_async(double target, float feedrate)
{
    target_mm = check_target(target);
    next_step_us = micros();
    if (feedrate != NOVALUE)
        max_vel = feedrate;
}

// Public - Plan and execute acceleration profile Will stay in loop until move is complete
void motorDrive::set_pos_target_mm_sync(double target, float feedrate)
{
    plan_move(target, feedrate);
    execute_move_async();
    while (true)
    {
        if(async_move_step_check(micros()))
            return;
    }
}

// Public - Creates and stores move plan to later be executed asynchronously
void motorDrive::plan_move(double target, float feedrate, bool ignore_limits)
{
    float maxvel = feedrate == NOVALUE ? max_vel : feedrate / 60;
    double clean_target = ignore_limits ? target : check_target(target);
    int32_t step_target = clean_target * steps_per_mm;
    // Set stepper turn direction
    if((step_target - stepper.get_step()) < 0)
        stepper.set_dir(false);
    else
        stepper.set_dir(true);

    // Calculate some motion paramters
    plan_nsteps = abs(step_target - stepper.get_step());
    float accel_dist = (pow(maxvel, 2) / (2 * max_accel));
    float move_dist = abs(clean_target - (stepper.get_step() / steps_per_mm));

    // Determine if move will be all accelerations or if it will have plateau
    if(abs(2 * accel_dist) < move_dist)
    {
        plan_tmin = (1000000 * step_size_mm) / maxvel;
        plan_asteps = steps_per_mm * accel_dist;
    }
    else
    {
        plan_tmin = 0;
        plan_asteps = floor(plan_nsteps / 2);
    }

    // Serial.print("accel_dist");
    // Serial.println(accel_dist);
    // Serial.print("move_dist");
    // Serial.println(move_dist);
    // Serial.print("plan_tmin");
    // Serial.println(plan_tmin);
    // Serial.print("plan_asteps");
    // Serial.println(plan_asteps);
    // Serial.print("plan_nsteps");
    // Serial.println(plan_nsteps);

    // Construct acceleration ramp timings
    plan_accel_timings.clear();
    int32_t next_target_delta_us = 0;
    int32_t totaltime_us = 0;
    float v_now = 0;
    for(int i = 0; i < plan_asteps; i++)
    {
        next_target_delta_us = solve_for_t_us(v_now, max_accel, step_size_mm);
        plan_accel_timings.push_back(next_target_delta_us);
        totaltime_us += next_target_delta_us;
        v_now = (totaltime_us * max_accel) / 1000000;
    }
}

// Public - Start asynchronous move, requires motion plan and calling async_move_step_check frequently
void motorDrive::execute_move_async()
{
    plan_stepstaken = 1;
    plan_nextstep_us = micros() + plan_accel_timings[0];
    stepper.step();
}

// Public - Take a step if ready. Call this in a loop until it returns true
bool motorDrive::async_move_step_check(uint32_t t_now)
{
    if (plan_stepstaken >= plan_nsteps)
        return true;
    else if (t_now > plan_nextstep_us)
    {
        // Accelerating
        if (plan_stepstaken < plan_asteps)
        {
            plan_nextstep_us += plan_accel_timings[plan_stepstaken];
        }
        // Decelerating
        else if (plan_stepstaken >= (plan_nsteps - plan_asteps))
        {
            plan_nextstep_us += plan_accel_timings[plan_nsteps - plan_stepstaken - 1];
        }
        // Constant vel no plateau
        else if (plan_tmin == 0)
        {
            plan_nextstep_us += plan_accel_timings[plan_asteps - 1];
        }
        // Constant vel with plateau
        else
        {
            plan_nextstep_us += plan_tmin; // TODO keep track of tmin error for more accurate velocities
        } 
        stepper.step();
        plan_stepstaken++;
    }
    return false;
}

// Private - Kinematic equation solving for time
int32_t motorDrive::solve_for_t_us(float v, float a, float d)
{
    float t = -v;
    if(v >= 0)
    {
        t += sqrt(pow(v, 2) + 2 * a * d);
        t /= a;
    }
    else
    {
        t -= sqrt(pow(v, 2) - 2 * a * d);
        t /= a;
    }
    t *= 1000000;
    t = (int32_t)(t - 1); // Subtract 1 to account for the time of the step pulse
    if(t > 50000)
    {
        return 50000;
    }
    return t;
}

// Public - The magic sauce. Tracks motor motion and calculates if a step is needed now to stay on track.
bool motorDrive::step_if_needed()
{
    uint32_t t_now = micros();
    int32_t step_target = (steps_per_mm * target_mm);

    // Serial.print(stepper.get_step());
    // Serial.print("\t");
    // Serial.print(t_now);
    // Serial.print("\t");
    // Serial.print(next_step_us);
    // Serial.print("\n");

    // Check if motor is in right place already
    if((abs(current_velocity) < 0.001) && (step_target == stepper.get_step()))
        return false;
    else if((abs(current_velocity) < 0.001) && (stepper.get_step() > step_target))
        stepper.set_dir(false);
    else if((abs(current_velocity) < 0.001) && (stepper.get_step() < step_target))
        stepper.set_dir(true);
    
    if(micros() > next_step_us)
    {
        stepper.step();

        uint32_t cur_step_us = next_step_us;
        double stop_dist_mm = pow(current_velocity, 2) / (max_accel);
        double stop_pos_mm = stepper.get_step() / steps_per_mm;

        // This mess determines if we need to slow down
        if((stepper.get_dir() && ((stop_pos_mm + stop_dist_mm) > target_mm)) || (!stepper.get_dir() && ((stop_pos_mm - stop_dist_mm) < target_mm)))
        {
            // First check if we are coming to a stop
            if((pow(current_velocity, 2) - 2 * max_accel * step_size_mm) < 0)
            {
                // See if we should turn round
                if(stepper.get_step() > step_target)
                {
                next_step_us = (2 * next_step_us) - last_step_us;
                stepper.set_dir(false);
                }
                else if(stepper.get_step() < step_target)
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
        current_velocity = stepper.get_dir() ? current_velocity : -current_velocity;
        current_velocity = abs(current_velocity) < 0.01 ? 0 : current_velocity;
        last_step_us = cur_step_us;
    }
    return true;
}

// Public - Return current position in mm
double motorDrive::get_current_pos_mm()
{
    return stepper.get_step() / steps_per_mm;
}

// Public - Return current velocity in mm/sec
double motorDrive::get_current_vel_mmps()
{
    return current_velocity;
}

// Public - Sensorless homing
// bool motorDrive::home(bool to_min)
// {
//     if (to_min)
//         stepper.set_dir(false);
//     else
//         stepper.set_dir(true);

//     uint32_t steptime = 1000000 * step_size_mm / home_vel;
//     uint32_t stepcount = 0;
//     uint32_t nextstep = micros() + steptime;

//     while (true)
//     {
//         stepper.step();
//         stepcount += 1;

//         while (micros() < nextstep);

//         nextstep += steptime;
//         if (stepcount % 16 == 0)
//         {
//             // stepper.update_driver_status(stepper.get_index());
//         }
//     }
// }

// Private - Quadratic equation yo
void motorDrive::quad_solve(double &t_0, double &t_1, double a, double b, double c)
{
    double temp0 = -abs(b);
    double temp1 = sqrt(pow(b, 2) + 2 * a * c);
    t_0 = (temp0 + temp1) / a;
    t_1 = (temp0 - temp1) / a;
}

// Private - Filter move target to ensure valid and not below min (0) or above max(max_dist_mm)
double motorDrive::check_target(double target)
{
    target = target == NOVALUE ? target_mm : target;
    target = target < 0 ? 0 : target;
    target = target > max_dist_mm ? 0 : max_dist_mm;
    return target;
}

// Public - Set current position to zero
void motorDrive::zero()
{
    set_current_pos_mm(0);
}
