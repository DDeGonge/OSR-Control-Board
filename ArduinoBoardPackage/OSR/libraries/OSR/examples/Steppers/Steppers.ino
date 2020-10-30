#include <OSR.h>

#define Serial SERIAL_PORT_USBVIRTUAL

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  Serial.println("BEGINNING!");
}

void loop() {
  Serial.println("Initializing motor drivers");
  TMC2041 Driver0 = TMC2041(D0EN, D0CS, D0S0S, D0S1S, D0S0D, D0S1D);
  TMC2041 Driver1 = TMC2041(D1EN, D1CS, D1S0S, D1S1S, D1S0D, D1S1D);

  Serial.println("Initializing motor drives for 320 microsteps per mm");
  motorDrive step0 = motorDrive(Driver0.motor0, 320);
  motorDrive step1 = motorDrive(Driver0.motor1, 320);
  motorDrive step2 = motorDrive(Driver1.motor1, 320);
  motorDrive step3 = motorDrive(Driver1.motor0, 320);

  Serial.println("Updating default motor velocities and accelerations to various values");
  step0.set_default_vel_mmps(150);
  step0.set_default_acc_mmps2(500);
  step1.set_default_vel_mmps(150);
  step1.set_default_acc_mmps2(2000);
  step2.set_default_vel_mmps(50);
  step2.set_default_acc_mmps2(500);
  step3.set_default_vel_mmps(50);
  step3.set_default_acc_mmps2(2000);

  Serial.println("Enabling all motors");
  Driver0.enable();
  Driver1.enable();

  Serial.println("Zeroing all motors");
  step0.zero();
  step1.zero();
  step2.zero();
  step3.zero();

  while (true)
  {
    Serial.println("Moving all motors to 200mm simeltaneously");
    // Plan each motors move
    step0.plan_move(200);
    step1.plan_move(200);
    step2.plan_move(200);
    step3.plan_move(200);
  
    // Begin move
    step0.execute_move_async();
    step1.execute_move_async();
    step2.execute_move_async();
    step3.execute_move_async();
  
    // Loop until all are done stepping
    while(true)
    {
      uint32_t tnow = micros();
      bool r0 = step0.async_move_step_check(tnow);
      bool r1 = step1.async_move_step_check(tnow);
      bool r2 = step2.async_move_step_check(tnow);
      bool r3 = step3.async_move_step_check(tnow);
      if(r0 && r1 && r2 && r3)
        break;
    }
  
    Serial.println("Moving motors one at a time synchronously at 100mmps");

    Serial.println("Moving motor0 to 0mm");
    step0.set_pos_target_mm_sync(0, 6000);
    Serial.println("Moving motor1 to 0mm");
    step1.set_pos_target_mm_sync(0, 6000);
    Serial.println("Moving motor2 to 0mm");
    step2.set_pos_target_mm_sync(0, 6000);
    Serial.println("Moving motor3 to 0mm");
    step3.set_pos_target_mm_sync(0, 6000);
  }
}