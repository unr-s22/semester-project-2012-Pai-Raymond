
#include <dht.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <Wire.h>
#include <ds3231.h>
 
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;
volatile unsigned char* ddr_b  = (unsigned char*) 0x24;
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* pin_b  = (unsigned char*) 0x23;

#define dht_apin A2 // Analog Pin sensor is connected to
#define POWER_PIN 7 // water sensor power pin
#define SIGNAL_PIN 0 // water sensor analog pin
#define THRESHOLD 8 // water sensor threshold limit
#define STEPS 55 // stepper motor steps
#define TEMPHOLD 20.0 // Temperatrue Thresholt


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

const int buttonStart = 52;
const int buttonStop = 50;
const int buttonReset = 48;
const int ledPinG = 22;
const int ledPinY = 24;
const int ledPinB = 26;
const int ledPinR = 28;
const int pin31 = 2;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
//Stepper stepper(STEPS, 53, 51, 49, 47);
Stepper stepper(STEPS, 8, 10, 9, 11);
boolean Idle = false, Running = true, Error = false;
void setup(){
 Serial.begin(9600);

 pinMode(pin31, OUTPUT);
 pinMode(buttonStart, INPUT_PULLUP);
 pinMode(buttonReset, INPUT_PULLUP);
 pinMode(buttonStop, INPUT);
 
 pinMode(ledPinG, OUTPUT);
 pinMode(ledPinY, OUTPUT);
 pinMode(ledPinB, OUTPUT);
 pinMode(ledPinR, OUTPUT);
 
 write_pb(ledPinG, 0);
 write_pb(ledPinY, 0);
 write_pb(ledPinB, 0);
 write_pb(ledPinR, 0);
 digitalWrite(pin31, HIGH);
 //Stepper Motor Setup
 pinMode(button, INPUT_PULLUP);
 //Water Sensor Setup
 adc_init();
 set_PB_as_output(POWER_PIN);
 write_pb(POWER_PIN, 0);

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
  t.hour=12; 
  t.min=30;
  t.sec=0;
  t.mday=27;
  t.mon=4;
  t.year = 2122;
 
 DS3231_set(t); 
}
bool pressed;
int direction_ = 1, speed_ = 500, state = 0, stateSys = 0;
void loop(){
   if(state == 0 && digitalRead(buttonStart) == HIGH){
      state = 1;
      stateSys =! stateSys;
   }
   if(digitalRead(buttonStop) == HIGH){
        state = 0;
        stateSys = 0;
   }
   if(stateSys){
      write_pb(POWER_PIN, 1);               
      value = adc_read(SIGNAL_PIN);
      write_pb(POWER_PIN, 0);
      if(Running == true){
        write_pb(ledPinB, 1);
        if(DHT.temperature <= TEMPHOLD){
          Idle = true;
          Running = false;
          write_pb(ledPinB, 0);
          serialTime("Running -> Idle");
          lcd.clear();
        }
        else if(value < THRESHOLD){
          Error = true;
          Running = false;
          write_pb(ledPinB, 0);
          serialTime("Running -> Error");
          lcd.clear();
        }
        lcd.setCursor(0, 0);
        lcd.print("R");
        displayLCD();
      }
      if(Idle == true){
        write_pb(ledPinG, 1);
        if(DHT.temperature > TEMPHOLD){
          Running = true;
          Idle = false;
          write_pb(ledPinG, 0);
          serialTime("Idle -> Running");
          lcd.clear();
        }
        else if(value <= THRESHOLD){
          Error = true;
          Idle = false;
          write_pb(ledPinG, 0);
          serialTime("Idle -> Error");
          lcd.clear();
        }
        lcd.setCursor(0, 0);
        lcd.print("I");
        displayLCD();
      }
      if(Error == true){
        lcd.display();
        lcd.setCursor(0, 0);
        lcd.print("E");
        displayLCD();
        if(value > THRESHOLD){
            Idle = true;
            Error = false;
            write_pb(ledPinR, 0);
            lcd.clear();
            serialTime("Error -> Idle");
            speed_ = 500;
        }
        if(digitalRead(buttonReset) == 1){
            Idle = true;
            Error = false;
            write_pb(ledPinR, 0);
            lcd.clear();
            serialTime("Error -> Idle");
            speed_ = 500;
        }
      }
  }
  else{
    lcd.noDisplay();
  }
}
 /*
 motorAct(speed_);
      if(Running == true){
        write_pb(ledPinB, 1);
        if(DHT.temperature <= TEMPHOLD){
          Idle = true;
          Running = false;
          write_pb(ledPinB, 0);
          serialTime("Running -> Idle");
        }
        else if(value < THRESHOLD){
          Error = true;
          Running = false;
          write_pb(ledPinB, 0);
          serialTime("Running -> Error");
        }
        else{
          lcd.setCursor(0, 0);
          lcd.print("R");
          displayLCD();
        }
      }
      if(Idle == true){
        write_pb(ledPinG, 1);
        if(DHT.temperature > TEMPHOLD){
          Running = true;
          Idle = false;
          write_pb(ledPinG, 0);
          serialTime("Idle -> Running");
        }
        else if(value <= THRESHOLD){
          Error = true;
          Idle = false;
          write_pb(ledPinG, 0);
          serialTime("Idle -> Error");
        }
        else{
          lcd.setCursor(0, 0);
          lcd.print("I");
          displayLCD();
        }
      }
      if(Error == true){
        motorAct(0);
        write_pb(ledPinR, 1);
        lcd.setCursor(0, 0);
        lcd.print("Low");
        ErrorLCD();
        /*if(value > THRESHOLD){
            write_pb(buttonReset, 1);
        }
        if(digitalRead(buttonReset) == 1){
            Idle = true;
            motorAct(speed_);
            Error = false;
            write_pb(ledPinR, 0);
            write_pb(buttonReset, 0);
            serialTime("Error -> Idle");
        }
        */

    //LCD Monitor with Temp/Hum Sensor 
   /*DHT.read11(dht_apin);
   lcd.setCursor(0, 0);
   lcd.print("R");
   lcd.setCursor(3, 0);
   lcd.print("Hum.   ");
   lcd.print(DHT.humidity);
   lcd.setCursor(3, 1);
   lcd.print("Temp.  ");
   lcd.print(DHT.temperature); 
   delay(5000);*/
  
  //Water Sensor 
  /*write_pb(POWER_PIN, 1);               
  value = adc_read(SIGNAL_PIN);
  write_pb(POWER_PIN, 0);
  //digitalWrite(POWER_PIN, 0);

  Serial.print("Sensor value: ");
  Serial.println(value);*/

  //Stepper Motor 
  /*if ( digitalRead(button) == 0 )  // if button is pressed
    if ( debounce() )  // debounce button signal
    {
      direction_ *= -1;  // reverse direction variable
      while ( debounce() ) ;  // wait for button release
    }
 
  // read analog value from the potentiometer
    int val = analogRead(pot);
    stepper.setSpeed(speed_);
    stepper.step(direction_);
  */

  //Real-Time Clock
  /*DS3231_get(&t);
  Serial.print("Date : ");
  Serial.print(t.mday);
  Serial.print("/");
  Serial.print(t.mon);
  Serial.print("/");
  Serial.print(t.year);
  Serial.print("\t Hour : ");
  Serial.print(t.hour);
  Serial.print(":");
  Serial.print(t.min);
  Serial.print(".");
  Serial.println(t.sec);
 
  delay(1000);*/
void motorAct(int speed_){
  if (digitalRead(button) == 0 ) { // if button is pressed
      if (stepDebounce()){  // debounce button signal
         direction_ *= -1;  // reverse direction variable
         while (stepDebounce());  // wait for button release
      }
  }
  stepper.setSpeed(speed_);
  stepper.step(direction_); 
}
void serialTime(String state){
  DS3231_get(&t);
  Serial.print(state);
  Serial.print("\n");
  Serial.print("Date : ");
  Serial.print(t.mday);
  Serial.print("/");
  Serial.print(t.mon);
  Serial.print("/");
  Serial.print(t.year);
  Serial.print("\t Hour : ");
  Serial.print(t.hour);
  Serial.print(":");
  Serial.print(t.min);
  Serial.print(".");
  Serial.println(t.sec);
 
  //delay(1000);
}
void ErrorLCD(){
  DHT.read11(dht_apin);
  lcd.setCursor(3, 0);
  lcd.print("   Hum.  ");
  lcd.print(DHT.humidity);
  lcd.setCursor(3, 1);
  lcd.print("   Temp. ");
  lcd.print(DHT.temperature);
}
void displayLCD(){
  DHT.read11(dht_apin);
  lcd.setCursor(3, 0);
  lcd.print("Hum.   ");
  lcd.print(DHT.humidity);
  lcd.setCursor(3, 1);
  lcd.print("Temp.  ");
  lcd.print(DHT.temperature);

  //delay(2000);
}
void set_PB_as_output(unsigned char pin_num)
{
    *ddr_b |= 0x01 << pin_num;
}
void write_pb(unsigned char pin_num, unsigned char state)
{
  if(state == 0)
  {
    *port_b &= ~(0x01 << pin_num);
  }
  else
  {
    *port_b |= 0x01 << pin_num;
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

void my_delay(unsigned int ticks){
  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);
  *myTCCR1B |= 0x01;
  while((*myTIFR1 & 0x01) == 0);
  *myTCCR1B &= 0xF8;
  *myTIFR1 |= 0x01;
}
