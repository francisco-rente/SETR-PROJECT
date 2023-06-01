// this->AIN1 = 12;
// this->AIN2 = 13;
// this->ENA = 6;
// this->BIN1 = 20;
// this->BIN2 = 21;
// this->ENB = 26;
// this->speed_left = 10;
// this->speed_right = 10;

#include <pigpio.h>
#define HIGH 1
#define LOW 0

gpioSetMode(12, PI_OUTPUT);
gpioSetMode(13, PI_OUTPUT);
gpioSetMode(20, PI_OUTPUT);
gpioSetMode(21, PI_OUTPUT);
gpioSetMode(6, PI_OUTPUT);
gpioSetMode(26, PI_OUTPUT);

gpioSetPWMfrequency(6, 500);
gpioSetPWMfrequency(26, 500);
gpioSetPWMrange(6, 100);
gpioSetPWMrange(26, 100);

gpioPWM(6, 10);
gpioPWM(26, 10);
gpioWrite(13, LOW);
gpioWrite(13, HIGH); 
gpioWrite(20, LOW);
gpioWrite(21, HIGH);


gpioDelay(1000); // Idk the unit of this
// STOP
gpioPWM(6, 0);
gpioPWM(26, 0);
gpioWrite(13, LOW);
gpioWrite(13, LOW); 
gpioWrite(20, LOW);
gpioWrite(21, LOW);