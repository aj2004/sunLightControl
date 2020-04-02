#include "outputLCD.h"
#include <Arduino.h>
//#include "global.h"

void outputLCD(int LCDscreen){

  /* LCD DISPLAY:
   * The LCD is a '1602': 16 characters/columns, 2 rows
   * 
   * Main Screen:
   *        0123456789012345
   * Line0: MMM.DD  HH:mm:ss (Current month/day, and time. Adjustable) TODO
   * Line1: R.HH:mm  S.HH:mm (Today's Sunrise and Sunset. Calculated)
   * 
   * Setup Screen: TODO
   *        0123456789012345
   * Line0: >Off: Rise +01m (Turn lights off at Sunrise +/- XX minutes. Adjustable)
   * Line1: >On : Set  -01m (Turn lights on at Sunset +/- XX minutes. Adjustable)
   * 
   * Avoid using 'lcd.clear()' because it clears the whole screen and causes flicker.
   * Instead, overwrite existing characters and use 'lcd.write(254)' to print a blank character.
   * 
   * lcd.setCursor(Column,Row); to move the cursor.
   * lcd.home(); to put the cursor at 0,0 (top-left).
   * lcd.print(variable/string); to print something. Data types cannot be combined.
   */ 

  
  uint8_t _LCDyear    = (bools.timeAdjust) ? timeYear_temp % 2000   : now.year() % 2000;
  uint8_t _LCDmonth   = (bools.timeAdjust) ? timeMonth_temp  : now.month();
  uint8_t _LCDday     = (bools.timeAdjust) ? timeDay_temp    : now.day();
  uint8_t _LCDhour    = (bools.timeAdjust) ? timeHour_temp   : now.hour();
  uint8_t _LCDminute  = (bools.timeAdjust) ? timeMinute_temp : now.minute();
  uint8_t _LCDsecond  = (bools.timeAdjust) ? timeSecond_temp : now.second();
  

  switch (LCDscreen) {
    case 0:
      // 0 = MAIN SCREEN
      // LINE 0
      // Print the date
      lcd.noCursor();
      lcd.setCursor(0,0);
      if (_LCDyear < 10) { lcd.print(0);}
      lcd.print(_LCDyear);
      
      lcd.print(monthNameShort[_LCDmonth]);
      if (_LCDday < 10) { lcd.print(0); } // Pad single digit with a leading zero
      lcd.print(_LCDday);
      
      lcd.write(254);

      if (_LCDhour < 10) { lcd.print(0); } // Pad single digit with a leading zero
      lcd.print(_LCDhour);
      lcd.print(":");
      if (_LCDminute < 10){ lcd.print(0); } // Pad single digit with a leading zero
      lcd.print(_LCDminute);
      lcd.print(":");
      if (_LCDsecond < 10){ lcd.print(0); } // Pad single digit with a leading zero
      lcd.print(_LCDsecond);

      // LINE 1
      // Print today's sunrise

      lcd.setCursor(0,1);
      lcd.print("R.");
      lcd.print(timeBurnabySunrise); // Single digits are padded with zero elsewhere

      lcd.write(254);
      lcd.write(254);

      lcd.print("S.");
      lcd.print(timeBurnabySunset); // Single digits are padded with zero elsewhere

      if (bools.timeAdjust){ lcd.cursor(); }
      else { lcd.noCursor(); }

      lcd.setCursor(cursorPos.col, cursorPos.row);

      break;

    case 1:
      screenTimeoutTimer = millis();
      lcd.noBlink();
      // Print the Sunrise Offset
      lcd.setCursor(0, 0);

      #ifdef DEBUG
        lcd.print("  col=");
        lcd.print(cursorPos.col);
        lcd.print("  ");
      #else
        lcd.print("  Sunrise");
              
              if(burnabySunriseOffset_temp < 0){lcd.print(" -");}
        else  if(burnabySunriseOffset_temp >= 0){lcd.print(" +");}
        if( abs(burnabySunriseOffset_temp) / 10 < 1 ){lcd.print(" ");}
        lcd.print( abs(burnabySunriseOffset_temp) );
        lcd.print("m");
        if( abs(burnabySunriseOffset_temp) < 100 ){lcd.print(" ");}
      #endif

      // Print the Sunset Offset
      lcd.setCursor(0, 1);

      #ifdef DEBUG
        lcd.print("  row=");
        lcd.print(cursorPos.row);
        lcd.print("  ");
      #else
        lcd.print("   Sunset");
              
              if(burnabySunsetOffset_temp < 0){lcd.print(" -");}
        else  if(burnabySunsetOffset_temp >= 0){lcd.print(" +");}
        if( abs(burnabySunsetOffset_temp) / 10 < 1 ){lcd.print(" ");}
        lcd.print( abs(burnabySunsetOffset_temp) );
        lcd.print("m");
        if( abs(burnabySunsetOffset_temp) < 100 ){lcd.print(" ");}
      #endif
      

      lcd.setCursor(0, cursorPos.row);
      lcd.print(">");

      lcd.setCursor(LCD_COLUMNS - 1, 0);
      if (burnabySunriseOffset_temp != burnabySunriseOffset){
        lcd.print("*");
      }else{
        lcd.print(" ");
      }
      lcd.setCursor(LCD_COLUMNS - 1, 1);
      if (burnabySunsetOffset_temp != burnabySunsetOffset){
        lcd.print("*");
      }else{
        lcd.print(" ");
      }
      if(cursorPos.col > 2){
        lcd.cursor();
      }
      else{
        lcd.noCursor();
      }

      lcd.setCursor(cursorPos.col, cursorPos.row);

      
      
      break;


  }

}