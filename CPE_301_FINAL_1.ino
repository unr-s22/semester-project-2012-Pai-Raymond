
#include <dht.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Wire.h>
#include <ds3231.h>

volatile unsigned char* ddr_a  = (unsigned char*) 0x21;
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* pin_a  = (unsigned char*) 0x20;
volatile unsigned char* ddr_b  = (unsigned char*) 0x24;
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* pin_b  = (unsigned char*) 0x23;
volatile unsigned char* ddr_c  = (unsigned char*) 0x27;
volatile unsigned char* port_c = (unsigned char*) 0x28;
volatile unsigned char* pin_c = (unsigned char*) 0x26;
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A;
volatile unsigned char* port_d = (unsigned char*) 0x2B;
volatile unsigned char* pin_d = (unsigned char*) 0x29;
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D;
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* pin_e = (unsigned char*) 0x2C;
volatile unsigned char* ddr_f  = (unsigned char*) 0x30;
volatile unsigned char* port_f = (unsigned char*) 0x31;
volatile unsigned char* pin_f = (unsigned char*) 0x2F;
volatile unsigned char* ddr_g  = (unsigned char*) 0x33;
volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* pin_g = (unsigned char*) 0x32;
volatile unsigned char* ddr_h  = (unsigned char*) 0x101;
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* pin_h = (unsigned char*) 0x100;
volatile unsigned char* ddr_j  = (unsigned char*) 0x104;
volatile unsigned char* port_j = (unsigned char*) 0x105;
volatile unsigned char* pin_j = (unsigned char*) 0x103;
volatile unsigned char* ddr_k  = (unsigned char*) 0x107;
volatile unsigned char* port_k = (unsigned char*) 0x108;
volatile unsigned char* pin_k = (unsigned char*) 0x106;
volatile unsigned char* ddr_l  = (unsigned char*) 0x10A;
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* pin_l = (unsigned char*) 0x109;

#define dht_apin A2 // Analog Pin sensor is connected to
#define POWER_PIN 7 // water sensor power pin
#define SIGNAL_PIN 0 // water sensor analog pin
#define THRESHOLD 20 // water sensor threshold limit
#define STEPS 32 // stepper motor steps
#define TEMPHOLD 23.0 // Temperatrue Threshold


//water sensor var
unsigned int value = 0; // variable to store the sensor value

//stepper motor var
const int button = 6; // direction control button is connected to Arduino pin 6
const int pot = A0; // speed control potentiometer is connected to analog pin 0

//LCD var
const int RS = 12, EN = 7, D4 = 2, D5 = 3, D6 = 4, D7 = 5;

//Humidity and Temperature var
dht DHT;

//Real-Time Clock var
struct ts t;

/*const int buttonStart = 52;
const int buttonStop = 50;
const int buttonReset = 48;*/

const int buttonStart = 1;
const int buttonStop = 3;
const int buttonReset = 1;

const int ledPinG = 0;
const int ledPinY = 2;
const int ledPinB = 4;
const int ledPinR = 6;
const int dcMotor = 0;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
Stepper stepper(STEPS, 8, 10, 9, 11);

boolean Idle = false, Running = true, Error = false, Disabled = false;
int direction_ = 1, speed_ = 1000;
volatile int state = 0;
volatile bool stateSys = false;
void setup(){
 Serial.begin(9600);
 setInput(ddr_b, buttonStart);
 enablePullup(port_b, buttonStart);
 setInput(ddr_b, buttonStop);
 enablePullup(port_b, buttonStop);
 setInput(ddr_l, buttonReset);
 enablePullup(port_l, buttonReset);
 
 setOutput(ddr_a, ledPinG);
 setOutput(ddr_a, ledPinY);
 setOutput(ddr_a, ledPinB);
 setOutput(ddr_a, ledPinR);
 setOutput(ddr_b, dcMotor);
 
 write_port(port_a, ledPinG, 0);
 write_port(port_a, ledPinY, 0);
 write_port(port_a, ledPinB, 0);
 write_port(port_a, ledPinR, 0);
 
 //Stepper Motor Setup
 pinMode(button, INPUT_PULLUP);
 //Water Sensor Setup
 adc_init();
 setOutput(ddr_b, POWER_PIN);
 write_port(port_b,POWER_PIN, 0);

 //LCD Setup
 lcd.begin(16, 2); // set up number of columns and rows
 delay(500);//Delay to let Temp/Hum Sensor system boot
 delay(1000);//Wait before accessing Temp/Hum Sensor
 
 //Real Time Clock Setup
 Wire.begin();
 DS3231_init(DS3231_CONTROL_INTCN);
  /*----------------------------------------------------------------------------
  In order to synchronise your clock module, insert timetable values below !
  ----------------------------------------------------------------------------*/
  t.sec=0;
 
 DS3231_set(t); 
  
 //PCICR  |= 1 << PCIE0;
 //PCMSK0 |= 1 << PCINT1;
 
}
/*ISR(PCINT1_vect){
}*/
void loop(){
   if(state == 0 && digital_read(pin_b, buttonStart) == HIGH){
      state = 1;
      stateSys =! stateSys;
      lcd.clear();
   }
   if(state == 1 && digital_read(pin_b, buttonStart) == LOW){
      state = 0;
      stateSys = false;
      lcd.clear();
   }
   if(!stateSys){
      write_port(port_b,POWER_PIN, 1);              
      value = adc_read(SIGNAL_PIN);
      write_port(port_b,POWER_PIN, 0);
      if(Running == true){
        write_port(port_b, dcMotor, 1);
        lcd.setCursor(0, 0);
        lcd.print("R");
        displayLCD();
        //write_port(port_a, ledPinR, 1);
        if(DHT.temperature <= TEMPHOLD){
          lcd.clear();
          write_port(port_b, dcMotor, 0);
          Idle = true;
          Running = false;
          //write_port(port_a, ledPinB, 0);
          serialTime("Running -> Idle");
        }
        else if(value < THRESHOLD){
          lcd.clear();
          write_port(port_b, dcMotor, 0);
          Error = true;
          Running = false;
          //write_port(port_a, ledPinB, 0);
          serialTime("Running -> Error");
        }
      }
      if(Idle == true){
        lcd.setCursor(0, 0);
        lcd.print("I");
        displayLCD();
        //write_port(port_a, ledPinG, 1);
        if(DHT.temperature > TEMPHOLD){
          lcd.clear();
          Running = true;
          Idle = false;
          //write_port(port_a, ledPinG, 0);
          serialTime("Idle -> Running");
        }
        else if(value <= THRESHOLD){
          lcd.clear();
          Error = true;
          Idle = false;
          //write_port(port_a, ledPinG, 0);
          serialTime("Idle -> Error");
        }
      }
      if(Error == true){
        //write_port(port_a, ledPinR, 1);
        ErrorLCD();
        if(value > THRESHOLD){
            if(digital_read(pin_l, buttonReset) == HIGH){
              lcd.clear();
              Idle = true;
              Error = false;
              //write_port(port_a, ledPinR, 0);
              serialTime("Error -> Idle");
          }
        }
     }
  }
  else{
      lcd.setCursor(0, 0);
      lcd.print("Disabled");
  }
}
void motorAct(int speed_){
  if (digitalRead(button) == 0 && Error == false) { // if button is pressed
      if (stepDebounce()){  // debounce button signal
         direction_ *= -1;  // reverse direction variable
         while (stepDebounce());  // wait for button release
      }
  }
  stepper.setSpeed(speed_);
  stepper.step(direction_); 
}
void motorRun(){
  for(int i = 0; i < 20; i++){
    motorAct(speed_);
  }
}
void serialTime(String state){
  DS3231_get(&t);
  Serial.print(state);
  Serial.print("\n");
  Serial.print("Seconds: ");
  Serial.println(t.sec);
}
void ErrorLCD(){
  motorRun();
  lcd.setCursor(0, 0);
  lcd.print("Error:");
  lcd.setCursor(0, 1);
  lcd.print("Water too low");
}
void displayLCD(){
  motorRun();
  DHT.read11(dht_apin);
  lcd.setCursor(3, 0);
  lcd.print("Hum.   ");
  lcd.print(DHT.humidity);
  lcd.setCursor(3, 1);
  lcd.print("Temp.  ");
  lcd.print(DHT.temperature);

}
boolean digital_read(volatile unsigned char* pin, volatile unsigned char pin_num){
    if((*pin & (0x01 << pin_num)) != 0){
      return HIGH;
    }
    return LOW;
}
void setOutput(volatile unsigned char* ddr, unsigned char pin_num){
    *ddr |= 0x01 << pin_num;
}
void setInput(volatile unsigned char* ddr, unsigned char pin_num){
    *ddr &= ~(0x01 << pin_num);
}
void enablePullup(volatile unsigned char *port, volatile unsigned char pin_num){
    *port |= 0x01 << pin_num;
}
void write_port(volatile unsigned char* port, unsigned char pin_num, unsigned char state){
  if(state == 0)
  {
    *port &= ~(0x01 << pin_num);
  }
  else
  {
    *port |= 0x01 << pin_num;
  }
}
bool stepDebounce()
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
void adc_init()
{
  // set up the A register
  ADCSRA |= (1<<7); // set bit   7 to 1 to enable the ADC
  ADCSRA &= ~(1<<5); // clear bit 5 to 0 to disable the ADC trigger mode
  ADCSRA &= ~(1<<3); // clear bit 3 to 0 to disable the ADC interrupt
  ADCSRA &= (0b11111000); // clear bit 2-0 to 0 to set prescaler selection to slow reading
  
  // set up the B register
  ADCSRB &= ~(1<<3); // clear bit 3 to 0 to reset the channel and gain bits
  ADCSRB &= (0b11111000); // clear bit 2-0 to 0 to set free running mode
  
  // set up the MUX Register
  ADMUX  &= ~(1<<7); // clear bit 7 to 0 for AVCC analog reference
  ADMUX  |= (1<<6); // set bit   6 to 1 for AVCC analog reference
  ADMUX  &= ~(1<<5); // clear bit 5 to 0 for right adjust result
  ADMUX  &= (0b11100000); // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num){
  // reset the channel and gain bits
  ADMUX  &= (0b11100000);
  
  // clear the channel selection bits
  ADCSRB &= ~(1<<3);
  
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    
    // set MUX bit 
    ADCSRB |= (1<<3);
  }
  
  // set the channel selection bits
  ADMUX  += adc_channel_num;
  
  // set bit ?? of ADCSRA to 1 to start a conversion
  ADCSRA |= (1<<6);
  
  // wait for the conversion to complete
  while((ADCSRA) & (1<<6));
  // return the result in the ADC data register
  return ADCL + ADCH * 256;
}
