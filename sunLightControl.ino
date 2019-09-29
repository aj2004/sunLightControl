/* 2018-08-09
 * 
 * 
 * Outdoor light timer based on sunrise and sunset.
 * This program is meant to control the puck lights (in the soffet) out front
 * so that the timer doesn't have to be reset twice a year and so the lights
 * turn on/off at the same time each day, relative to the daylight.
 *  
 * 
 * *********************
 * * TABLE OF CONTENTS *
 * *********************
 * 
 * 1. #includes (libraries)
 * 2. #defines
 *  2a. Pin Definitions
 *  2b. Config Values
 * 3. Global variables
 * 4. Object creation
 * 5. Function declarations
 * 6. void setup()  (main function, runs once)
 * 7. void loop()   (main function, runs forever)
 * 8. function prototypes
 * 
 */






///////////////////////////////////////////////
//  1. INCLUDED LIBRARIES                    //
///////////////////////////////////////////////
#include <Arduino.h>
#include <Dusk2Dawn.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <Button.h>
#include <TimedAction.h>
#include <EEPROM.h>
#include "PWM_RampLinear.h"
#include "isDST_Canada.h"







///////////////////////////////////////////////
//  2. DEFINED CONSTANTS                     //
///////////////////////////////////////////////

//DEBUGGING:
// Anywhere that
//    '#ifdef DEBUG' ... #endif
// is seen in this code denotes a block of code that can be enabled
// to provide serial port debugging features.
// Normally, this code would be disabled once the program is running smoothly.
//
// Comment out the following line to disable debugging. (put // on the left)
// Baud rate may be defined here. Standard baud rates:
//  300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
#define DEBUG

#ifdef DEBUG
  #define SERIAL_BAUD 115200
#endif

#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Setup the pushbutton pins here.
#define PIN_PB_UP     A0
#define PIN_PB_DOWN   A1
#define PIN_PB_LEFT   A2
#define PIN_PB_RIGHT  A3
#define PIN_PB_ENTER  12
// How long (in milliseconds) to hold a PB before it repeats its function?
#define REPEAT_MS     300
// Specify the Debounce time, in milliseconds
#define PB_DBNC_MS    30

// Setup the relay pin here
#define PIN_RELAY 8

// Setup the LED pins here.
// The RGB LED should be wired to pins 3,5,6,9,10,11
// These are the hardware PWM pins.
#define PIN_LED_R 9
#define PIN_LED_G 10
#define PIN_LED_B 11
// Set the LCD backlight PWM pin here.
#define PIN_LCD_A  6

// This digital LED is on the Arduino circuit board, hard-wired to pin 13.
#define PIN_LED_L 13

// How many seconds of idle time before the screen goes to sleep?
#define SCREEN_TIMEOUT_SEC 10


// Lamp Flash times, in milliseconds
#define FLASH_BLIP_ON 100
#define FLASH_BLIP_OFF 1750

#define FLASH_SLOW_ON 900
#define FLASH_SLOW_OFF 900

#define FLASH_FAST_ON 300
#define FLASH_FAST_OFF 300

// These values shouldn't need to be modified
#define BURNABY_LATITUDE 49.2488
#define BURNABY_LONGITUDE -122.9805
#define BURNABY_UTC_OFFSET -8






/*
 * Values below this comment should not have to be changed.
 */

// Don't change these values
#define RED   255, 000, 000
#define GREEN 000, 255, 000
#define BLUE  000, 000, 255

// Don't change these values
#define SOLID 1
#define FLASH_BLIP 2
#define FLASH_SLOW 3
#define FLASH_FAST 4

// These are EEPROM addresses. Offsets are in minutes, so 1 Byte is enough
// NOTE: Each EEPROM address has a MTBF of ~10,000 writes.
//       So, avoid excessive writing.
#define ADDR_SUNRISE_OFFSET 0
#define ADDR_SUNSET_OFFSET 1







///////////////////////////////////////////////
//  3. GLOBAL VARIABLES                      //
///////////////////////////////////////////////

/*
 * This struct will allow 8 bools to be packed into a single byte,
 * rather than taking up 8 bytes.
 * The number after the colon is the size, in BITS
 */
struct boolByte
{
    bool screen:1;
    bool lcdFading:1;
    bool timedOut:1;
    bool var3:1;
    bool var4:1;
    bool var5:1;
    bool var6:1;
    bool var7:1;
}__attribute__((packed));
// make a new variable of type "boolByte"
// address each member like so: bools.screen = 0;
boolByte bools;

// to store the previous elapsed time
uint32_t screenTimeoutTimer = 0;
// to grab a snapshot of the current time, in minutes after midnight
uint16_t currentMinutes = 0;
// these will hold the sunrise/sunset, in minutes after midnight
uint16_t burnabySunrise;
uint16_t burnabySunset ;
// Sunrise time, in HH:mm 24-hour format
char timeBurnabySunrise[] = "00:00";
// Sunset time, in HH:mm 24-hour format
char timeBurnabySunset[] = "00:00";

// These will hold the +/- offset
// Range is -127...+127 (approx. +/- 2 hours)
int8_t burnabySunriseOffset = 0;
int8_t burnabySunsetOffset = 0;

// These can be used when displaying the date, or for debugging
// use something like: lcd.print(dayName[now.dayOfTheWeek]) or dayNameShort[now.dayOfTheWeek]
char dayName[7][10] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char dayNameShort[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

char monthName[13][10] = {"ERROR", "January", "February", "March", "April", "May", "June",
                          "July", "August", "September", "October", "November", "December"};
char monthNameShort[13][4] = {"ERR","Jan", "Feb", "Mar", "Apr", "May", "Jun",
                              "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

struct two_nibbles
{
 uint8_t col:4;
 uint8_t row:4;
}__attribute__((packed));

two_nibbles cursorPos;

#ifdef DEBUG
  // This timer is for serial debugging updates.
  // Change the value of interval_Serial to the refresh rate, in milliseconds.
  long prevTime_Serial = 0;
  long currTime_Serial = 0;
  uint16_t interval_Serial = 2000;
  // Preset is how many scans to average the reported scan time over
  uint16_t scanTimePreset = 100;
  uint16_t scanTimeCount = 0;
  uint32_t scanTimeCurr = 0;
  uint32_t scanTimePrev = 0;
  uint32_t scanTimeAverage = 0;

#endif







///////////////////////////////////////////////
//  4. OBJECT CREATION                       //
///////////////////////////////////////////////

/* Create a real-time clock object
 * There are no arguments here.
 */ 
RTC_DS3231 rtc;
// Create a DateTime object. The current date and time are grabbed from this object.
DateTime now;

/* Create an LCD object.
 * Arguments: I2C address (hex), columns, rows
 */
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);

/* Create PWM Ramping objects
 * Argument: Pin Number
 */
PWM_RampLinear LCD_Backlight_PWM(PIN_LCD_A);

/* Create the Button objects.
 *  
 * Make a new Button object using this class:
 * 
 * Button PB_1(uint8_t pin, uint8_t puEnable, uint8_t invert, uint32_t dbTime);
 * 
 * pin = Arduino pin number of the button input
 * puEnable (true|false) = enable the internal pull-up resistor. wire the button as sinking.
 * invert (true|false) = invert the logic. if enabled, and button wired as sinking, pressing the button will return true.
 * dbTime (32-bit int) = debounce time, in milliseconds. 20 is generally a good minimum.
 * 
 * 
 * These functions are included in this library:
 * 
 * uint8_t read(); = updates the inputs. this should be called at the start of the main loop()
 * uint8_t isPressed(); = returns true if button is pressed (after debouncing)
 * uint8_t isReleased(); = returns true if button is not pressed (after debouncing)
 * uint8_t wasPressed(); = one-shot rising. returns true for one scan/loop when button pressed
 * uint8_t wasReleased(); = one-shot falling. returns true for one scan/loop when button released
 * uint8_t pressedFor(uint32_t ms); = time-delay on, in milliseconds. returns true if button held for this long
 * uint8_t releasedFor(uint32_t ms); = time-delay off, in milliseconds. returns true if button released for this long
 * uint32_t lastChange(); = elapsed program time, in milliseconds, when the last button event occured.
                         use with millis();, which returns the current elapsed program time.
 *                          
 * Each button should be wired to the input pin and COM/GND.
 * Enable the pull-up resistor and inverting logic for each button
 *  to make it behave like a high-active input in the code.
 * 20ms is a good start for debounce time.
 * 
 */

//    |Name  | Pin Number  |Pull Up|Invrt|Debnc|
//----|------|-------------|-------|-----|-----|
Button pbUp   (PIN_PB_UP,     true, true, PB_DBNC_MS);
Button pbDown (PIN_PB_DOWN,   true, true, PB_DBNC_MS);
Button pbLeft (PIN_PB_LEFT,   true, true, PB_DBNC_MS);
Button pbRight(PIN_PB_RIGHT,  true, true, PB_DBNC_MS);
Button pbEnter(PIN_PB_ENTER,  true, true, PB_DBNC_MS);

/* Create a Dusk2Dawn object. (for sunrise/sunset times)
 * Arguments: latitude, longitude, UTC offset
 * PST = UTC-8:00
 * 
 */
Dusk2Dawn burnaby(BURNABY_LATITUDE, BURNABY_LONGITUDE, BURNABY_UTC_OFFSET);







///////////////////////////////////////////////
//  5. FUNCTION DECLARATIONS                 //
///////////////////////////////////////////////


bool burnabyDST;
bool burnabyDST_last;

/* Call this function to illuminate and/or flash the LED(s)
 * Arguments:
 *  - colour (RED, GREEN, BLUE)
 *    RED   = 255, 0, 0
 *    GREEN = 0, 255, 0
 *    BLUE  = 0, 0, 255
 *    Other custom colours may be defined.
 *  - animation (SOLID, FLASH_BLIP, FLASH_SLOW, FLASH_FAST)
 */
void outputLED_RGB(uint8_t val_red, uint8_t val_grn, uint8_t val_blu, uint8_t animation);


/* Call this function to illuminate and/or flash a single LED
 * Arguments:
 *  - Pin number of the LED 
 *  - animation (SOLID, FLASH_BLIP, FLASH_SLOW, FLASH_FAST)
 */
void outputLED_digital(uint8_t LED_pin, uint8_t animation);


/* Call this void function to print the screen to the LCD
 * Argument: screen number
 *  0 = main screen
 *  1 = setup screen
 */
void outputLCD(void);
TimedAction outputLCD_action = TimedAction(250,outputLCD);



// Call this void function to control the relay/lights
void outputRelay(void);


#ifdef DEBUG
  // Call this function when DEBUG is enabled to output info to the Serial Port
  void outputSerialDebug(void);
  TimedAction outputSerialDebug_action = TimedAction(2000, outputSerialDebug);
#endif










///////////////////////////////////////////////
//  6. setup() -- MAIN FUNCTION (runs once)  //
///////////////////////////////////////////////

/* setup() function: runs once
 * 1. Init Serial, LCD, RTC, I/O, 
 * 
 * 
 */

void setup() {

  #ifdef DEBUG 
    //DEBUG: start serial comms
    Serial.begin (SERIAL_BAUD);
      Serial.print("Serial Port is running at ");
      Serial.print(SERIAL_BAUD);
    Serial.println(" baud.");
    
  #endif

  // Setup the Input pins
  /*
   * NOTE: These lines of code are redundant.
   * The Button library takes care of the pinModes.
   * This code is being left here to make it more obvious. 
   */
  pinMode(PIN_PB_UP,    INPUT_PULLUP);
  pinMode(PIN_PB_DOWN,  INPUT_PULLUP);
  pinMode(PIN_PB_LEFT,  INPUT_PULLUP);
  pinMode(PIN_PB_RIGHT, INPUT_PULLUP);
  pinMode(PIN_PB_ENTER, INPUT_PULLUP);

  // Setup the Output pins
  /* 
   * It is recommended to use digitalWrite(PIN, STATE) to put the outputs
   * into a known state before the main loop() begins.
   */
  pinMode(PIN_RELAY, OUTPUT);
  /* The relay contacts will be wired N.C. so if this program/circuit fails, the original
   *  timer unit (in series with this system) can still control the lights
   * The output is configured as a CURRENT SINK. This "double-negative" means that:
   *  - Setting the output HIGH means the lights will be ON (relay coil is off, NC contacts closed)
   *  - Setting the output LOW means the lights will be OFF (relay coil is on, NC contacts open)
   */
  digitalWrite(PIN_RELAY, LOW);
  

  // Setup the LED pins here.
  pinMode(PIN_LED_R, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);
  pinMode(PIN_LED_G, OUTPUT);
  digitalWrite(PIN_LED_G, HIGH);
  pinMode(PIN_LED_B, OUTPUT);
  digitalWrite(PIN_LED_B, HIGH);
  // And the onboard LED: Labelled 'L'
  pinMode(PIN_LED_L, OUTPUT);
  digitalWrite(PIN_LED_L, LOW);


  // Initialize the LCD & clear any junk left in screen memory
  lcd.init();
  lcd.clear();
  lcd.backlight();
  // analogWrite is an 8-bit PWM output
  analogWrite(PIN_LCD_A, 255);
  LCD_Backlight_PWM.ramp(255, 500);
  //set the cursor to 0,0 (top-left)
  lcd.home();  

  #ifdef DEBUG
    //DEBUG: print the location information
    Serial.println("Location: Burnaby, BC, CA");
      Serial.print("Latitude = ");
    Serial.println(BURNABY_LATITUDE);
      Serial.print("Longitude = ");
    Serial.println(BURNABY_LONGITUDE);
      Serial.print("UTC offset: ");
    Serial.println(BURNABY_UTC_OFFSET);
    Serial.println();

    
    
    // LCD Debug Enabled Message
    // This will show the configured serial baud rate
    //         0123456789012345
    lcd.print("Debug Enabled");
    lcd.setCursor(0,1);
    lcd.print("Baud: ");
    lcd.print(SERIAL_BAUD);

    // This function literally delays the program for X milliseconds
    delay(3000);

    lcd.clear();

  #endif

  // Clear the LCD one more time for good measure.
  
}









///////////////////////////////////////////////
//  7. loop() -- MAIN FUNCTION               //
///////////////////////////////////////////////

// Main function: runs forever
void loop() {


  //-------------------------//
  //------ Read Inputs ------//
  //-------------------------//

  // Grab the stored Offsets from EEPROM
  burnabySunriseOffset = EEPROM.read(ADDR_SUNRISE_OFFSET);
  burnabySunsetOffset = EEPROM.read(ADDR_SUNSET_OFFSET);

  // Grab the current state of the pushbuttons.
  pbUp.read();
  pbDown.read();
  pbLeft.read();
  pbRight.read();
  pbEnter.read();

  


  //-------------------------//
  //--- Time Calculations ---//
  //-------------------------//

  /* Grab the current time.
   * This avoids the current time changing
   *  in the middle of the loop. 
   */
  
  now = rtc.now();

  

  // Convert the current time into minutes after midnight
  currentMinutes = (now.hour()*60) + now.minute();  

  /* Grab the sunrise and sunset for Burnaby using the current date. 
   * Available methods are sunrise() and sunset(). Arguments are year, month,
   *  day, and if Daylight Saving Time is in effect.
   */
  burnabyDST = isDST_Canada(now.month(), now.day(), now.dayOfTheWeek());
  burnabySunrise  = burnaby.sunrise(now.year(), now.month(), now.day(), burnabyDST);
  burnabySunset   = burnaby.sunset(now.year(), now.month(), now.day(), burnabyDST);

  if (burnabyDST != burnabyDST_last){
    // If the current DST is different than the last loop (it has changed), adjust the time
    if(burnabyDST){
    rtc.adjust(DateTime(now.year(),now.month(),now.day(),now.hour()+1,now.minute(),now.second()));
    }
    else if(!burnabyDST){
    rtc.adjust(DateTime(now.year(),now.month(),now.day(),now.hour()-1,now.minute(),now.second()));
    }
    // and equate the comparison variable so we're not writing the RTC chip every loop
    burnabyDST_last = burnabyDST;
  }
  

  // convert the sunrise to 24-hour time for display
  Dusk2Dawn::min2str(timeBurnabySunrise, burnabySunrise);
  // convert the sunset to 24-hour time for display
  Dusk2Dawn::min2str(timeBurnabySunset, burnabySunset);
  

  //-------------------------//
  //------ Misc. Stuff ------//
  //-------------------------//

  if (pbUp.wasPressed() || pbDown.wasPressed() || pbLeft.wasPressed() || pbRight.wasPressed() || pbEnter.wasPressed()){
    // If ANY button is pressed, reset the screenTimeoutTimer
    screenTimeoutTimer = millis();
    
    lcd.display();
    // If the display happened to be timed out, change to screen 0 and brighten the LCD backlight
    if (bools.timedOut) {
      lcd.backlight();
      LCD_Backlight_PWM.ramp(255, 500);
      bools.screen = 0;
    }
    // screen's not timed out anymore...
    bools.timedOut = false;
  }

  if (bools.screen == 0 && pbRight.wasPressed()){
    // if at the left-most screen and "right" was pressed, go right
    lcd.clear();
    bools.screen = 1;
  }

  if (bools.screen == 1){
    // if at the right-most screen and "left" was pressed, go left
    if (pbLeft.wasPressed()){
      lcd.clear();
      bools.screen = 0;
    }
    cursorPos.row = 0;
    cursorPos.col = 0;
    if (cursorPos.row < LCD_ROWS && pbDown.wasPressed())cursorPos.row++;
    if (cursorPos.row > 0 && pbDown.wasPressed())cursorPos.row--;
    if (cursorPos.col < LCD_COLUMNS && pbRight.wasPressed()){
      if (cursorPos.col < 12 )cursorPos.col = 12;
      if (cursorPos.row == 0 && cursorPos.col > 13 ){
        EEPROM.update(ADDR_SUNSET_OFFSET, burnabySunsetOffset);
      }
      if (cursorPos.row == 1 && cursorPos.col > 13 ){
        EEPROM.update(ADDR_SUNRISE_OFFSET, burnabySunriseOffset);
      }
      cursorPos.col++;
    }
    if (cursorPos.col > 0 && pbLeft.wasPressed())cursorPos.col--;

  }

  if (millis() - screenTimeoutTimer > (SCREEN_TIMEOUT_SEC * 1000)){
    // If the system times out, set the flag and dim the LCD
    bools.timedOut = true;
    LCD_Backlight_PWM.ramp(0, 1500);
    if(LCD_Backlight_PWM.rampDoneOS){
      lcd.noBacklight();
      lcd.clear();
    }
    
  }

  //-------------------------//
  //----- Write Outputs -----//
  //-------------------------//

  // TODO: Output to the LEDs

  #ifdef DEBUG
    if(pbLeft.isPressed()){
      outputLED_RGB(255,0,0, FLASH_FAST);
      
    }
  #endif
  outputLED_digital(PIN_LED_L, FLASH_SLOW);
  

  
  // Control the relay/lights.
  //TODO: outputRelay();
  LCD_Backlight_PWM.update();
   // Display screen on LCD (timed action)
  if (!bools.timedOut) outputLCD_action.check();
  
  #ifdef DEBUG
    // If DEBUG is enabled, send some info to the Serial Port (timed action)
    scanTimeCount++;
    if (scanTimeCount >= scanTimePreset){
      scanTimeCount = 0;
      scanTimeCurr = micros();
      scanTimeAverage = (scanTimeCurr-scanTimePrev)/scanTimePreset;
      scanTimePrev = scanTimeCurr;
    }

    /* if (test_flash){
      outputLED_digital(9, FLASH_FAST);
    }*/
    
    char SerialKey;
    SerialKey = Serial.read();

    switch (SerialKey){

      case 'w':
        //LED_R_Ramp.ramp(250, 2000);
        break;

      case 's':
        //LED_R_Ramp.ramp(25,1000);
        break;

      case 'd':
        //test_flash = true;
        break;

      case 'a':
        break;
        //test_flash = false;
    }

    outputSerialDebug_action.check(); 
  #endif


}// END OF MAIN LOOP





////////////////////////////////////////////////
//  8. FUNCTION PROTOTYPES                    //
////////////////////////////////////////////////


void outputLED_RGB(uint8_t val_red, uint8_t val_grn, uint8_t val_blu, uint8_t animation){
  /*
    Use this function to flash an RGB LED.
    Arguments: RGB value (red byte, green byte, blue byte,) and animation
  */
  uint32_t flashTimer = 0;

  switch (animation){

    case SOLID:
      //digitalWrite(LED_pin, HIGH);
      analogWrite(PIN_LED_R, val_red);
      analogWrite(PIN_LED_G, val_grn);
      analogWrite(PIN_LED_B, val_blu);
      break;

    case FLASH_BLIP:
      flashTimer = millis() % (FLASH_BLIP_ON + FLASH_BLIP_OFF);
      if (flashTimer <= FLASH_BLIP_ON){
        analogWrite(PIN_LED_R, val_red);
        analogWrite(PIN_LED_G, val_grn);
        analogWrite(PIN_LED_B, val_blu);
      }
      else {
        analogWrite(PIN_LED_R, 0);
        analogWrite(PIN_LED_G, 0);
        analogWrite(PIN_LED_B, 0);
      }
      //digitalWrite(LED_pin, (flashTimer <= FLASH_BLIP_ON));
      break;

    case FLASH_SLOW:
      flashTimer = millis() % (FLASH_SLOW_ON + FLASH_SLOW_OFF);
      if (flashTimer <= FLASH_SLOW_ON){
        analogWrite(PIN_LED_R, val_red);
        analogWrite(PIN_LED_G, val_grn);
        analogWrite(PIN_LED_B, val_blu);
      }
      else {
        analogWrite(PIN_LED_R, 0);
        analogWrite(PIN_LED_G, 0);
        analogWrite(PIN_LED_B, 0);
      }
      //digitalWrite(LED_pin, (flashTimer <= FLASH_SLOW_ON));
      break;

    case FLASH_FAST:
      flashTimer = millis() % (FLASH_FAST_ON + FLASH_FAST_OFF);
      if (flashTimer <= FLASH_FAST_ON){
        analogWrite(PIN_LED_R, val_red);
        analogWrite(PIN_LED_G, val_grn);
        analogWrite(PIN_LED_B, val_blu);
      }
      else {
        analogWrite(PIN_LED_R, 0);
        analogWrite(PIN_LED_G, 0);
        analogWrite(PIN_LED_B, 0);
      }
      //digitalWrite(LED_pin, (flashTimer <= FLASH_FAST_ON));
      break;

    default:
      break;

  }
}




void outputLED_digital(uint8_t LED_pin, uint8_t animation){
  /*
   * Use this function to flash a single LED.
   * Arguments: LED pin, and animation
   * 
   */
  uint32_t flashTimer = 0;

  switch (animation){

    case SOLID:
      digitalWrite(LED_pin, HIGH);
      break;

    case FLASH_BLIP:
      flashTimer = millis() % (FLASH_BLIP_ON + FLASH_BLIP_OFF);
      digitalWrite(LED_pin, (flashTimer <= FLASH_BLIP_ON));
      break;

    case FLASH_SLOW:
      flashTimer = millis() % (FLASH_SLOW_ON + FLASH_SLOW_OFF);
      digitalWrite(LED_pin, (flashTimer <= FLASH_SLOW_ON));
      break;

    case FLASH_FAST:
      flashTimer = millis() % (FLASH_FAST_ON + FLASH_FAST_OFF);
      digitalWrite(LED_pin, (flashTimer <= FLASH_FAST_ON));
      break;

    default:
      break;

  }
}




void outputLCD(void){

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

  switch (bools.screen) {
    case 0:
      // 0 = MAIN SCREEN
      // LINE 0
      // Print the date
      lcd.setCursor(0,0);

      lcd.print(monthNameShort[now.month()]);
      lcd.print(".");
      if (now.day() < 10) { lcd.print(0); } // Pad single digit with a leading zero
      lcd.print(now.day());
      lcd.write(254);
      lcd.write(254);

      if (now.hour() < 10) { lcd.print(0); } // Pad single digit with a leading zero
      lcd.print(now.hour());
      lcd.print(":");
      if (now.minute() < 10){ lcd.print(0); } // Pad single digit with a leading zero
      lcd.print(now.minute());
      lcd.print(":");
      if (now.second() < 10){ lcd.print(0); } // Pad single digit with a leading zero
      lcd.print(now.second());

      // LINE 1
      // Print today's sunrise
      lcd.setCursor(0,1);
      lcd.print("R.");
      lcd.print(timeBurnabySunrise); // Single digits are padded with zero elsewhere
      lcd.write(254);
      lcd.write(254);

      
      lcd.print("S.");
      lcd.print(timeBurnabySunset); // Single digits are padded with zero elsewhere

      

      break;

    case 1:
      //TODO: 'Setup' screen
      lcd.setCursor(0, cursorPos.row);
      lcd.print(">");
      if (cursorPos.col > 0){
        lcd.setCursor(cursorPos.col, cursorPos.row);
        lcd.blink();
      }
      
      break;


  }

}




void outputRelay(void){

  /* Relay Output Logic:
   * 
   * The outdoor lights are controlled by the relay.
   * Setting the output HIGH turns the lights ON and vice versa.
   * The lights will turn ON at sunset, +/- the adjustable offset.
   * The lights will turn OFF at sunrise, +/- the adjustable offset.
   * 
   */

  // If the current time is after sunset...
  if (currentMinutes > (burnabySunset + burnabySunsetOffset)){
    // Turn the lights ON.
    digitalWrite(PIN_RELAY, HIGH);
    outputLED_RGB(GREEN, SOLID);
  }

  // Or if the current time is before sunrise...
  else if (currentMinutes < (burnabySunrise + burnabySunriseOffset)){
    // Turn the lights ON.
    digitalWrite(PIN_RELAY, HIGH);
    outputLED_RGB(GREEN, SOLID);
  }

  // Otherwise, it's daytime.
  else {
    // Turn the lights OFF.
    digitalWrite(PIN_RELAY, LOW);
  }

}




void outputSerialDebug(void){
  
    
    /* Main Debug Serial Message:
     * 
     * This block of Serial.print() statements will output some useful information
     * to the serial port. The built-in Arduino serial monitor, or a program such
     * as PuTTY, may be used to connect to the COM port with 115200 baud.
     * 
     * Additional Serial.print() statements may be added after the first if() statement.
     * This serial message will be printed every X milliseconds, defined by: interval_Serial
     * 
     */

    // grab the current elapsed milliseconds
    currTime_Serial = millis();

    /* if the elapsed program time since the last time this block
     * of code was run is greater than the interval, run the code
     */
    if ((currTime_Serial - prevTime_Serial) >= interval_Serial){

      // reset the last time the code was run to the current time
      prevTime_Serial = currTime_Serial;
      
      Serial.println("============================");
      //DEBUG: print the current date/time
      Serial.println("Current date/time:");
        Serial.print(dayNameShort[now.dayOfTheWeek()]);
        Serial.print(", ");
        Serial.print(monthNameShort[now.month()]);
        Serial.print(" ");
        Serial.print(now.day());
        Serial.print(" @ ");
        Serial.print(now.hour());
        Serial.print(":");
      if (now.minute()<10){Serial.print(0);}
        Serial.print(now.minute());
        Serial.print(":");
      if (now.second()<10){Serial.print(0);}
      Serial.println(now.second());
      Serial.println();

      //DEBUG: print out today's sunrise and sunset
        Serial.print("Burnaby sunrise: ");
      Serial.println(timeBurnabySunrise);
        Serial.print("Min. after midnight: ");
      Serial.println(burnabySunrise);
      Serial.println();

        Serial.print("Burnaby sunset: ");
      Serial.println(timeBurnabySunset);
        Serial.print("Min. after midnight: ");
      Serial.println(burnabySunset);
      Serial.println();
        Serial.print("Scan Time: ");
      Serial.println(scanTimeAverage);
      Serial.println();
    }
    
  
}

