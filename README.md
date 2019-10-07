# sunLightControl
Arduino on/off timer based on sunrise/sunset with LCD interface

The sketch is written for an Arduino Nano/Uno, DS3231 RTC, and 16x2 I2C LCD.
The code is certainly not the most efficient. It is written to be easy to understand rather than slick.

At my place of employment (a small office) there are lights mounted in the soffit at the front of the building. The original controller is a 7-day programmable timer (i.e. a unique on/off time can be selected for each day of the week). It has a battery backup in case of a power outage. However, there is no automatic Daylight Savings Time adjustment, so twice per year, the time must manually be set. Also, the sunrise and sunset drift throughout the year, so these lights may turn off before the sun rises, or may stay on for too long after that.

This is how this project came about. This self-contained controller is designed to turn the lights on at/around sunset and off at/around sunrise. The offset for both of these times is adjustable (e.g. lights turn on 30 minutes before sunset) through the LCD and pushbuttons. The lights themselves are controlled via a mechanical relay. Refer to the drawing and schematic for details.

The following libraries are used in this sketch and are available in the /lib folder.
The original repositories are linked below:

Dusk2Dawn (https://github.com/dmkishi/Dusk2Dawn) to calculate the sunrise/sunset based on date/time and coordinates.

RTClib (https://github.com/adafruit/RTClib) to interface to the DS3231 RTC module.

LiquidCrystal_I2C (https://github.com/johnrickman/LiquidCrystal_I2C) for the LCD module.

Button (https://github.com/JChristensen/Button) a pushbutton library (additional functions over the standard Arduino lib).

TimedAction (https://github.com/Glumgad/TimedAction) to run a function at a specified interval.

EEPROMEx (https://github.com/thijse/Arduino-EEPROMEx) an extended EEPROM library.
