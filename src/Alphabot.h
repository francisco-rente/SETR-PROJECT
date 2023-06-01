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
        int speed_left;
        int speed_right;

    public:
        Alphabot();
        int init();
        int setSpeed(int speed_left, int speed_right);
        int stop();
        int forward();
        int backward();
        int left();
        int right();
        ~Alphabot();
};