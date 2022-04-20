#include <dht.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23;

#define dht_apin A2 // Analog Pin sensor is connected to
#define POWER_PIN 7 // water sensor power pin
#define SIGNAL_PIN 1 // water sensor analog pin
#define THRESHOLD 8 // water sensor threshold limit
#define STEPS 32 // stepper motor steps

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
Stepper stepper(STEPS, 8, 10, 9, 11);

//water sensor var
unsigned int value = 0; // variable to store the sensor value

//stepper motor var
const int button =  6; // direction control button is connected to Arduino pin 6
const int pot = A0; // speed control potentiometer is connected to analog pin 0
int direction_ = 1, speed_ = 500;

//LCD var
const int RS = 11, EN = 10, D4 = 2, D5 = 3, D6 = 4, D7 = 5;

//Humidity and Temperature var
dht DHT;

void setup(){
 Serial.begin(9600);
 pinMode(button, INPUT_PULLUP);
 
 adc_init();

 set_PB_as_output(POWER_PIN);
 write_pb(POWER_PIN, 0);
 
 lcd.begin(16, 2); // set up number of columns and rows
 delay(500);//Delay to let Temp/Hum Sensor system boot
 delay(1000);//Wait before accessing Temp/Hum Sensor
}
//_Bool Disabled, Idle, Running, Error;
void loop(){
  //LCD Monitor with Temp/Hum Sensor 
   /*DHT.read11(dht_apin);
   lcd.setCursor(0, 0);
   lcd.print("Hum. ");
   lcd.print(DHT.humidity);
   lcd.setCursor(0, 1);
   lcd.print("Temp. ");
   lcd.print(DHT.temperature); 
   delay(5000);*/
  
  //Water Sensor 
  /*write_pb(POWER_PIN, 1);
  
  delay(10);                   
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
unsigned int adc_read(unsigned char adc_channel_num)
{
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
