#include <OSR.h>

#define Serial SERIAL_PORT_USBVIRTUAL

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  while(!Serial);
  Serial.println("BEGINNING!");
}

void loop() {
  Serial.println("Initializing motor drivers");
  TMC2041 Driver0 = TMC2041(D0EN, D0CS, D0S0S, D0S1S, D0S0D, D0S1D);
  TMC2041 Driver1 = TMC2041(D1EN, D1CS, D1S0S, D1S1S, D1S0D, D1S1D);

  Serial.println("Initializing motor drives");
  motorDrive step0 = motorDrive(Driver0.motor0);
  motorDrive step1 = motorDrive(Driver0.motor1);
  motorDrive step2 = motorDrive(Driver1.motor0);
  motorDrive step3 = motorDrive(Driver1.motor1);

  Serial.println("Enabling all motors");
  Driver0.enable();
  Driver1.enable();

  Serial.println("Zeroing all motors");
  step0.zero();

  while(true)
  {
    Serial.println("Moving all motors to 100mm");
    // Set target for async move and loop to keep motor activated until done moving
    step0.set_pos_target_mm_async(10);
    step1.set_pos_target_mm_async(10);
    step2.set_pos_target_mm_async(10);
    step3.set_pos_target_mm_async(10);
    while(true)
    {
      if(!step0.step_if_needed() && !step1.step_if_needed() && !step2.step_if_needed() && !step3.step_if_needed())
        break;
    }
  
    // Set target for synchronous move
    Serial.println("Moving motor0 to 0mm");
    step0.set_pos_target_mm_sync(0);
    Serial.println("Moving motor1 to 0mm");
    step1.set_pos_target_mm_sync(0);
    Serial.println("Moving motor2 to 0mm");
    step2.set_pos_target_mm_sync(0);
    Serial.println("Moving motor3 to 0mm");
    step3.set_pos_target_mm_sync(0);
  }
}
