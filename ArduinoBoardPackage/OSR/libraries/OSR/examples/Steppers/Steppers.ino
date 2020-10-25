#include <OSR.h>

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  Driver0 = OSR.TMC2041(D0EN, D0CS, D0S0S, D0S1S, D0S0D, D0S1D);
  Driver1 = OSR.TMC2041(D1EN, D1CS, D1S0S, D1S1S, D1S0D, D1S1D);
}
