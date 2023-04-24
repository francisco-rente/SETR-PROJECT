#include <iostream>
#include <bcm2835.h>

class Alphabot {
    private:
        int AIN1;
        int AIN2;
        int BIN1;
        int BIN2;
        int ENA;
        int ENB;
    public:
        Alphabot() {
            this->AIN1 = 12;
            this->AIN2 = 13;
            this->ENA = 6;
            this->BIN1 = 20;
            this->BIN2 = 21;
            this->ENB = 26; 
        }

        int init(){
            std::cout << "init" << std::endl;
            if(!bcm2835_init()) {
                return 1;
            }

            std::cout << "set pins" << std::endl;
            bcm2835_gpio_fsel(this->AIN1, BCM2835_GPIO_FSEL_OUTP);
            bcm2835_gpio_fsel(this->AIN2, BCM2835_GPIO_FSEL_OUTP);
            bcm2835_gpio_fsel(this->BIN1, BCM2835_GPIO_FSEL_OUTP);
            bcm2835_gpio_fsel(this->BIN2, BCM2835_GPIO_FSEL_OUTP);
            bcm2835_gpio_fsel(this->ENA, BCM2835_GPIO_FSEL_ALT5);
            bcm2835_gpio_fsel(this->ENB, BCM2835_GPIO_FSEL_ALT5);

            bcm2835_gpio_fsel(4, BCM2835_GPIO_FSEL_OUTP);

            std::cout << "pwm 1" << std::endl;
            bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_2048);
            bcm2835_pwm_set_mode(0, 1, 1);
            bcm2835_pwm_set_range(0, 1024);
            bcm2835_pwm_set_mode(1, 1, 1);
            bcm2835_pwm_set_range(1, 1024);

            bcm2835_pwm_set_data(0, 128);
            bcm2835_pwm_set_data(1, 128);

            std::cout << "pwm 2" << std::endl;
            // bcm2835_pwm_set_data(0, 1);
            // bcm2835_pwm_set_data(1, 1);

            std::cout << "gpio write" << std::endl;
            bcm2835_gpio_write(4, HIGH);
            // this->backward();
            // bcm2835_gpio_write(this->AIN1, LOW); // tHIS WORKS
            // bcm2835_gpio_write(this->AIN2, HIGH); // THIS WORKS
            // bcm2835_gpio_write(this->BIN1, LOW);
            // bcm2835_gpio_write(this->BIN2, HIGH);
            this->left();
            bcm2835_delay(1500);
            bcm2835_gpio_write(4, LOW);
            // this->left();
            // bcm2835_delay(1000);
            // this->forward();
            // bcm2835_delay(1000);
            // this->right();
            // bcm2835_delay(1000);

            // self.PWMA = GPIO.PWM(self.ENA,500)
            // self.PWMB = GPIO.PWM(self.ENB,500)
            // self.PWMA.start(self.PA)
            // self.PWMB.start(self.PB)
            // self.stop()

            return 0;
        }

        int stop() {
            bcm2835_gpio_write(this->AIN1, LOW);
            bcm2835_gpio_write(this->AIN2, LOW); 
            bcm2835_gpio_write(this->BIN1, LOW);
            bcm2835_gpio_write(this->BIN2, LOW);
            return 0;
        }

        int forward() {
            bcm2835_gpio_write(this->AIN1, LOW);
            bcm2835_gpio_write(this->AIN2, HIGH); 
            bcm2835_gpio_write(this->BIN1, LOW);
            bcm2835_gpio_write(this->BIN2, HIGH);
            return 0;
        }

        int backward() {
            bcm2835_gpio_write(this->AIN1, HIGH);
            bcm2835_gpio_write(this->AIN2, LOW); 
            bcm2835_gpio_write(this->BIN1, HIGH);
            bcm2835_gpio_write(this->BIN2, LOW);
            return 0;
        }

        int left() {
            bcm2835_gpio_write(this->AIN1, HIGH);
            bcm2835_gpio_write(this->AIN2, LOW); 
            bcm2835_gpio_write(this->BIN1, LOW);
            bcm2835_gpio_write(this->BIN2, HIGH);
            return 0;
        }

        int right() {
            bcm2835_gpio_write(this->AIN1, LOW);
            bcm2835_gpio_write(this->AIN2, HIGH); 
            bcm2835_gpio_write(this->BIN1, HIGH);
            bcm2835_gpio_write(this->BIN2, LOW);
            return 0;
        }
    
        // def setPWMA(self,value):
        //         self.PA = value
        //         self.PWMA.ChangeDutyCycle(self.PA)

        // def setPWMB(self,value):
        //         self.PB = value
        //         self.PWMB.ChangeDutyCycle(self.PB)

        // def setMotor(self, left, right):
        //         if((right >= 0) and (right <= 100)):
        //                 GPIO.output(self.AIN1,GPIO.HIGH)
        //                 GPIO.output(self.AIN2,GPIO.LOW)
        //                 self.PWMA.ChangeDutyCycle(right)
        //         elif((right < 0) and (right >= -100)):
        //                 GPIO.output(self.AIN1,GPIO.LOW)
        //                 GPIO.output(self.AIN2,GPIO.HIGH)
        //                 self.PWMA.ChangeDutyCycle(0 - right)
        //         if((left >= 0) and (left <= 100)):
        //                 GPIO.output(self.BIN1,GPIO.HIGH)
        //                 GPIO.output(self.BIN2,GPIO.LOW)
        //                 self.PWMB.ChangeDutyCycle(left)
        //         elif((left < 0) and (left >= -100)):
        //                 GPIO.output(self.BIN1,GPIO.LOW)
        //                 GPIO.output(self.BIN2,GPIO.HIGH)
        //                 self.PWMB.ChangeDutyCycle(0 - left)
};

int main() {
    Alphabot alphabot = Alphabot();
    if(alphabot.init()) {
        std::cout << "exited with error code 1";
        return 1;
    }
    alphabot.stop();
    bcm2835_close();
    return 0;
}
