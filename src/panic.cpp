#include <iostream>
#include <pigpio.h>

#define HIGH 1
#define LOW 0

class Alphabot {
    private:
        int AIN1;
        int AIN2;
        int BIN1;
        int BIN2;
        int ENA;
        int ENB;
        int speed;

    public:
        Alphabot() {
            this->AIN1 = 12;
            this->AIN2 = 13;
            this->ENA = 6;
            this->BIN1 = 20;
            this->BIN2 = 21;
            this->ENB = 26;
            this->speed = 10;
        }

        int init(){
            std::cout << "init" << std::endl;
            if (gpioInitialise() < 0)
            {
                return 1;
            }

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

        int setSpeed(int speed) {
            this->speed = speed;
            return 0;
        }

        int stop() {
            gpioPWM(this->ENA, 0);
            gpioPWM(this->ENB, 0);
            gpioWrite(this->AIN1, LOW);
            gpioWrite(this->AIN2, LOW); 
            gpioWrite(this->BIN1, LOW);
            gpioWrite(this->BIN2, LOW);
            return 0;
        }

        ~Alphabot(){
            this->stop();
        }
};

int main() {
    Alphabot alphabot = Alphabot();
    if(alphabot.init()) {
        std::cout << "exited with error code 1";
        return 1;
    }
    alphabot.stop();
    gpioTerminate();
    return 0;
}
