#include <LiquidCrystal.h>

// LCD pins <--> Arduino pins
const int RS = 13, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup()
{
  lcd.begin(16, 2); // set up number of columns and rows

  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("Hello World!");        // print message at (0, 0)
  lcd.setCursor(0, 1);         // move cursor to   (0, 1)
  lcd.print("Hello World!"); // print message at (0, 1)
}

void loop()
{
  
}
