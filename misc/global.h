#include <Dusk2Dawn.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <Button.h>
#include <TimedAction.h>
#include <EEPROM.h>
#include "PWM_RampLinear.h"
#include "isDST_Canada.h"
#include "leapYear.h"
#include "LED_Flash.h"
#include "outputLCD.h"

///////////////////////////////////////////////
//  2. DEFINED ALIASES & SETTINGS            //
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

#define DEBUG // Comment out to disable debug mode

#ifdef DEBUG
  #define SERIAL_BAUD 115200
#endif

// Set the size of the LCD here
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Setup the PUSHBUTTON pins here.
#define PIN_PB_UP     A0
#define PIN_PB_DOWN   A1
#define PIN_PB_LEFT   A2
#define PIN_PB_RIGHT  A3
#define PIN_PB_ENTER  12
// How long (in milliseconds) to hold a PB before it repeats its function?
#define REPEAT_MS     300
// Specify the Debounce time, in milliseconds. At least 20ms usually
#define PB_DBNC_MS    30

// Setup the RELAY COIL pin here
#define PIN_RELAY 8

// Setup the LED pins here.
// The RGB LED should be wired to pins 3,5,6,9,10,11
// These are the hardware PWM pins.
#define PIN_LED_R 9
#define PIN_LED_G 10
#define PIN_LED_B 11
// Set the LCD BACKLIGHT PWM pin here.
#define PIN_LCD_A  6

// This digital LED is on the Arduino circuit board, hard-wired to pin 13.
// The board labels this LED as 'L'
#define PIN_LED_L 13

// How many SECONDS of idle time before the screen goes to sleep?
#define SCREEN_TIMEOUT_SEC 10


// Lamp Flash times, in milliseconds
/*#define FLASH_BLIP_ON 100
#define FLASH_BLIP_OFF 1750

#define FLASH_SLOW_ON 900
#define FLASH_SLOW_OFF 900

#define FLASH_FAST_ON 300
#define FLASH_FAST_OFF 300*/

// These values shouldn't need to be modified
#define BURNABY_LATITUDE 49.2488
#define BURNABY_LONGITUDE -122.9805
#define BURNABY_UTC_OFFSET -8






//
// Values below this comment should not have to be changed.
///

// Colours, in decimal
#define REDd   255, 000, 000
#define GREENd 000, 255, 000
#define BLUEd  000, 000, 255

// Colours, in hex
#define REDx   0xFF0000
#define GREENx 0x00FF00
#define BLUEx  0x0000FF

// Don't change these values
/*#define SOLID 1
#define FLASH_BLIP 2
#define FLASH_SLOW 3
#define FLASH_FAST 4*/


// These are EEPROM addresses.
// ATmega328P (Arduino Nano) has 1024 bytes of EEPROM.
// Each address is 8 bits (1 byte)
// Offsets are in minutes, so 1 byte is enough for 2 hours (+/- 127 min)
// DST_last is a BOOL, so 1 byte is more than enough
// NOTE: Each EEPROM address has a MTBF of ~10,000 writes.
// Avoid excessive writing by verifying that the code does not continuously write to an address.
// Using the update() function will first READ and then WRITE, but ONLY if the value differs.
#define ADDR_DST_LAST       10
#define ADDR_SUNRISE_OFFSET 20
#define ADDR_SUNSET_OFFSET  30





///////////////////////////////////////////////
//  3. GLOBAL VARIABLES                      //
///////////////////////////////////////////////



// This struct will allow 8 bools to be packed into a single byte,
//  rather than taking up 8 bytes.
// The number after the colon is the size, in BITS of each member
// NOTE: Using a bitfield-packed struct like this is smaller, but slower.

struct {
  bool screen				:1;
  bool timingOut		:1;
  bool timedOut			:1;
  bool anyPbPressed	:1;
  bool anyPbHeld		:1;
  bool anyPbReleased:1;
  bool timeAdjust		:1;
  bool timeDayLast	:1;
}__attribute__((packed))
  bools;
// Address each member like so: bools.screen = 0;


//howdy


// This keeps track of how long a PB has been pressed,
//  so that we can add a preset amount of time to that.
// The PB will then have to continue to be pressed for
//  that new total length of time to execute some code.
// i.e. press and hold "UP" to auto-increment a value
uint16_t pbRepeatTimer = 0;

// To store the previous elapsed milliseconds so we know how
//  much time has elapsed between two moments.
uint32_t screenTimeoutTimer = 0;
// To grab a snapshot of the current time, in minutes after midnight.
// It is much easier to convert time (HH:MM) into just minutes
//  in order to do math on it.
uint16_t currentMinutes = 0;
// These will hold today's sunrise/sunset, in minutes after midnight
uint16_t burnabySunrise;
uint16_t burnabySunset;

// These are used to detect a change in the DST state
// If the state changes, the RTC gets set back/forward 1 hour
// The new state will be written to EEPROM so that the
//  RTC isn't adjusted every time the power is cycled.
bool burnabyDST = false;
bool burnabyDST_last = false;


// Sunrise time, in HH:mm 24-hour format (string)
char timeBurnabySunrise[] = "00:00";
// Sunset time, in HH:mm 24-hour format (string)
char timeBurnabySunset[] = "00:00";


#ifdef DEBUG
  uint16_t debugSunrise;
  uint16_t debugSunset;
  bool debugDST;
  bool debugDST_last;
  char debugTimeSunrise[] = "00:00";
  char debugTimeSunset[] = "00:00";
#endif

// These will hold the actual +/- offset
// Range is -127...+127 minutes (approx. +/- 2 hours)
int8_t burnabySunriseOffset = 0;
int8_t burnabySunsetOffset  = 0;
// These are used to hold the offset while it is being adjusted
int8_t burnabySunriseOffset_temp = 0;
int8_t burnabySunsetOffset_temp  = 0;

// These are SIGNED because adjustment could go below zero temporarily
int16_t timeYear_temp;
int8_t  timeMonth_temp;
int8_t  timeDay_temp;
int8_t  timeHour_temp;
int8_t  timeMinute_temp;
int8_t  timeSecond_temp;

// Days in each month.
// NOTE: there are 13 numbers here.
// Months from the RTC are numbered 1-12. 0 = Error
// Since both January and December have 31 days,
//  the zeroth element is 31 so that "ERR" doesn't get
//  displayed temporarily while adjusting the time.
uint8_t daysInMonth [] = { 31,31,28,31,30,31,30,31,31,30,31,30,31 };



// These can be used when displaying the date, or for debugging.
// Use something like: lcd.print(dayName[now.dayOfTheWeek()]) or dayNameShort[now.dayOfTheWeek()]
char dayName[7][10] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char dayNameShort[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
// Months from the RTClib are numbered 1-12. 0 = invalid
char monthName[13][10] = {"ERROR", "January", "February", "March", "April", "May", "June",
                          "July", "August", "September", "October", "November", "December"};
char monthNameShort[13][4] = {"ERR","Jan", "Feb", "Mar", "Apr", "May", "Jun",
                              "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};



// This is used to virtually move the cursor, like when navigating the screen.
// At the end of a block of code which moves the cursor using this variable,
//  the outputLCD() function is called and the cursor is actually moved to that location.
//
// Address the position like so:
// cursorPos.col = 10;
// cursorPos.row = 1;
// lcd.setCursor(cursorPos.col, cursorPos.row);
//
// This struct packs 2x 4-bit intergers into a single Byte
struct {
 uint8_t col:4;
 uint8_t row:4;
}__attribute__((packed))
  cursorPos;





///////////////////////////////////////////////
//  4. OBJECT CREATION                       //
///////////////////////////////////////////////

// Create a real-time clock object
// This will retrieve the raw datetime from the RTC module
RTC_DS3231 rtc;
// Create a DateTime object. The current date and time are grabbed from this object.
DateTime now;

// Create an LCD object.
//
// Arguments: I2C address (hex), columns, rows
// Typ.addr: 0x27
// Typ.sizes: 8,2  16,2  20x4
extern LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);

// Create PWM Ramping objects
// 
//
// Argument: Pin Number
PWM_RampLinear LCD_Backlight_PWM(PIN_LCD_A);

// Create a flashing RGB LED, common anode (inverted)
LED_Flash_RGB LEDrgb(PIN_LED_R, PIN_LED_G, PIN_LED_B, true);

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
 * These functions are included in this library:
 * 
 * uint8_t read(); = updates the inputs. this should be called at the start of the main loop()
 * uint8_t isPressed(); = returns true if button is pressed (after debouncing)
 * uint8_t isReleased(); = returns true if button is not pressed (after debouncing)
 * uint8_t wasPressed(); = one-shot rising. returns true for one scan/loop when button pressed
 * uint8_t wasReleased(); = one-shot falling. returns true for one scan/loop when button released
 * uint8_t pressedFor(uint32_t ms); = time-delay on, in milliseconds. returns true if button held for this long
 *    Note: you can do -> if(pb.pressedFor(ms)){ some stuff; ms += repeatInterval;} to enable auto-repeat
 * uint8_t releasedFor(uint32_t ms); = time-delay off, in milliseconds. returns true if button released for this long
 *                          
 * Each button should be wired to the input pin and COM/GND.
 * Enable the pull-up resistor and inverting logic for each button
 *  to make it behave like a high-active input in the code.
 * 20ms is a good start for debounce time.
 * 
 */

//    |Name  | Pin Number  |Pull Up|Invrt|Debnc Time|
//----|------|-------------|-------|-----|----------|
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


