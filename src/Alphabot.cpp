#include "Alphabot.h"
#include <iostream>
#include <pigpio.h>

Alphabot::Alphabot() {
    this->AIN1 = 12;
    this->AIN2 = 13;
    this->ENA = 6;
    this->BIN1 = 20;
    this->BIN2 = 21;
    this->ENB = 26;
    this->speed_left = 10;
    this->speed_right = 10;
}

int Alphabot::init(){
    std::cout << "init" << std::endl;

    std::cout << "set pins" << std::endl;
    gpioSetMode(AIN1, PI_OUTPUT);
    gpioSetMode(AIN2, PI_OUTPUT);
    gpioSetMode(BIN1, PI_OUTPUT);
    gpioSetMode(BIN2, PI_OUTPUT);
    gpioSetMode(ENA, PI_OUTPUT);
    gpioSetMode(ENB, PI_OUTPUT);

    gpioSetPWMfrequency(ENA, 500);
    gpioSetPWMfrequency(ENB, 500);
    gpioSetPWMrange(ENA, 100);
    gpioSetPWMrange(ENB, 100);
    return 0;
}

int Alphabot::setSpeed(int speed_left, int speed_right) {
    this->speed_left = speed_left;
    this->speed_right = speed_right;
    return 0;
}

int Alphabot::stop() {
    gpioPWM(this->ENA, 0);
    gpioPWM(this->ENB, 0);
    gpioWrite(this->AIN1, LOW);
    gpioWrite(this->AIN2, LOW); 
    gpioWrite(this->BIN1, LOW);
    gpioWrite(this->BIN2, LOW);
    return 0;
}

int Alphabot::forward() {
    gpioPWM(this->ENA, this->speed_left);
    gpioPWM(this->ENB, this->speed_right);
    gpioWrite(this->AIN1, LOW);
    gpioWrite(this->AIN2, HIGH); 
    gpioWrite(this->BIN1, LOW);
    gpioWrite(this->BIN2, HIGH);
    return 0;
}

int Alphabot::backward() {
    gpioPWM(this->ENA, this->speed_left);
    gpioPWM(this->ENB, this->speed_right);
    gpioWrite(this->AIN1, HIGH);
    gpioWrite(this->AIN2, LOW); 
    gpioWrite(this->BIN1, HIGH);
    gpioWrite(this->BIN2, LOW);
    return 0;
}

int Alphabot::left() {
    gpioPWM(this->ENA, this->speed_left);
    gpioPWM(this->ENB, this->speed_right);
    gpioWrite(this->AIN1, HIGH);
    gpioWrite(this->AIN2, LOW); 
    gpioWrite(this->BIN1, LOW);
    gpioWrite(this->BIN2, HIGH);
    return 0;
}

int Alphabot::right() {
    gpioPWM(this->ENA, this->speed_left);
    gpioPWM(this->ENB, this->speed_right);
    gpioWrite(this->AIN1, LOW);
    gpioWrite(this->AIN2, HIGH); 
    gpioWrite(this->BIN1, HIGH);
    gpioWrite(this->BIN2, LOW);
    return 0;
}

Alphabot::~Alphabot(){
    this->stop();
}