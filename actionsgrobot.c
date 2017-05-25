#include <wiringPiI2C>

void initMoteurs() {
  wiringPiSetup();
  pinMode(1, PWM_OUTPUT); // Motor 1
  pinMode(7, OUTPUT); // Motor 2, lanceur de balles ?
  AX12setSpeed(143, SPEED);
  AX12setSpeed(144, SPEED);
}

void testLanceurDeBalles() {
  digitalWrite(7, 1);
  waitFor(500); // NEED TESTING
  digitalWrite(7, 0);
}

void activerLanceurDeBalles() {
  digitalWrite(7, 1);
  waitFor(3000); // NEED TESTING
  digitalWrite(7, 0);
}

void expandGrobot() {
  //deploy the cylinder with the mat
  move_to(143, 0);
  move_to(144, 0);
  //make sure these angles are OK...

  // expand with AX12
  pwmWrite(1, 512); // Half value, NEED TESTING
}
