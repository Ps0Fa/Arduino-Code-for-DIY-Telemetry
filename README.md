# Arduino-Code-for-DIY-Telemetry
Full source code that i use for my telemetry

INIT.txt must be on the SD card of arduino (has the tracks and initial imu positions), plus some scrap that the code is ready to emplement as
sound (for rpm), hall sensors for throttle position and braking

Ghost.csv must be on the sd card if u want to add some guides on the track you have to add the lat, lon the rad of the cicle that u want to see the msg
and at the String u have to add the number of the code that will tell u if u have to open the thorttle etc.

SyncVideoReady is the main .ino that u have to load on your arduino board. Don't change the Position of #include <SPI.h>  it will not work
when u refreshing your screen u refresh only the parts that u need (it's trully difficult to make 10Hz with screen refreshing) when the seconds changes
you will lose 1hz because of the screens refresh

Don't change the pins 51,52 of screen will not work the PDQ_GFX.h library it's perfect because it's really fast but u have to use directly that pins

Don't Try to add the code in Arduino Uno it has no memmory to run it
