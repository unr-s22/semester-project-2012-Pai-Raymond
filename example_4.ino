// CPE 301 - REGISTER-LEVEL Blink Example
// Written By Frank Mascarich, Spring 2018

// Define Port B Register Pointers
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

void setup() 
{
  // Initialize the Serial Port
  Serial.begin(9600);
  //set PB4 to INPUT
  *ddr_b &= 0xEF;
  // enable the pullup resistor on PB4
  *port_b |= 0x10;
}

void loop() 
{
  // if the pin is high
  if(*pin_b & 0x10)
  {
    Serial.println("PIN IS HIGH");
  }
  // if the pin is low
  else
  {
    Serial.println("PIN IS LOW");
  }
  delay(250);
}

