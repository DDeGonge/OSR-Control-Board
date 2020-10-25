#include <OSR.h>

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  TMC2041 Driver0 = TMC2041(D0EN, D0CS, D0S0S, D0S1S, D0S0D, D0S1D);
  TMC2041 Driver1 = TMC2041(D1EN, D1CS, D1S0S, D1S1S, D1S0D, D1S1D);

  Driver0.enable();
  delay(10);
  Driver0.motor1.set_dir(true);

  for(uint32_t i = 0; i < 51200; i++)
  {
    Driver0.motor1.step();
    delayMicroseconds(10);
  }

  delay(500);
  Driver0.motor1.set_dir(false);
  for(uint32_t i = 0; i < 51200; i++)
  {
    Driver0.motor1.step();
    delayMicroseconds(10);
  }

  delay(10);
  Driver0.disable();
  delay(500);
}