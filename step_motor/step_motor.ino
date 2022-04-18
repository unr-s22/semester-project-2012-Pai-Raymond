
/*
 * Unipolar stepper motor speed and direction control with Arduino.
 * Full step control.
 * This is a free software with NO WARRANTY.
 * https://simple-circuit.com/
 */
 
// include Arduino stepper motor library
#include <Stepper.h>
 
// change this to the number of steps on your motor
#define STEPS 32
 
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 8, 10, 9, 11);
 
const int button =  4; // direction control button is connected to Arduino pin 4
const int pot    = A0; // speed control potentiometer is connected to analog pin 0
 
void setup()
{
  // configure button pin as input with internal pull up enabled
  pinMode(button, INPUT_PULLUP);
}
 
int direction_ = 1, speed_ = 0;
 
void loop()
{
  if ( digitalRead(button) == 0 )  // if button is pressed
    if ( debounce() )  // debounce button signal
    {
      direction_ *= -1;  // reverse direction variable
      while ( debounce() ) ;  // wait for button release
    }
 
  // read analog value from the potentiometer
  int val = analogRead(pot);
 
  // map digital value from [0, 1023] to [2, 500]
  // ===> min speed = 2 and max speed = 500 rpm
  if ( speed_ != map(val, 0, 1023, 2, 500) )
  { // if the speed was changed
    speed_ = map(val, 0, 1023, 2, 500);
    // set the speed of the motor
    stepper.setSpeed(speed_);
  }
 
  // move the stepper motor
  stepper.step(direction_);
 
}
 
// a small function for button debounce
bool debounce()
{
  byte count = 0;
  for(byte i = 0; i < 5; i++) {
    if (digitalRead(button) == 0)
      count++;
    delay(10);
  }
  if(count > 2)  return 1;
  else           return 0;
}
