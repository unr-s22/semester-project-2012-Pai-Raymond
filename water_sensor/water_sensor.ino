/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-water-sensor
 */
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 
#define LED_PIN 6
#define POWER_PIN  7
#define SIGNAL_PIN 0
#define THRESHOLD 156
unsigned int value = 0; // variable to store the sensor value

void setup() {
  Serial.begin(9600);
  adc_init();
  set_PB_as_output(LED_PIN);
  set_PB_as_output(POWER_PIN);
  write_pb(POWER_PIN, 0);
  write_pb(LED_PIN, 0);
  /*pinMode(7, OUTPUT);   // configure D7 pin as an OUTPUT
  digitalWrite(POWER_PIN, LOW); // turn the sensor OFF
  pinMode(2, OUTPUT);   // configure D2 pin as an OUTPUT
  digitalWrite(2, LOW); // turn LED OFF*/
}

void loop() {
  write_pb(POWER_PIN, 1);
  //digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
  delay(10);                      // wait 10 milliseconds
  value = adc_read(SIGNAL_PIN);
  write_pb(POWER_PIN, 0);
  //digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF
  
  Serial.print("Sensor value: ");
  Serial.println(value);
  
  if (value > THRESHOLD) {
    write_pb(LED_PIN, 1);
    //digitalWrite(LED_PIN, HIGH);  // turn LED ON
  } else {
    write_pb(LED_PIN, 0);
    //digitalWrite(LED_PIN, LOW);   // turn LED OFF
  }
  delay(10);
  
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
