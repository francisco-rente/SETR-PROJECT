#define _GNU_SOURCE
#include <sched.h>            /* Definition of SCHED_* constants */
#include <sys/syscall.h>      /* Definition of SYS_* constants */
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <pthread.h>
#include <mutex>
#include <pigpio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>

#define gettid() syscall(__NR_gettid)

#define SCHED_DEADLINE	6

#ifdef __arm__
#define __NR_sched_setattr	380
#define __NR_sched_getattr	381
#endif

#define HIGH 1
#define LOW 0

#define DR 16
#define DL 19

static volatile int done;

struct sched_attr {
    __u32 size;

    __u32 sched_policy;
    __u64 sched_flags;

    /* SCHED_NORMAL, SCHED_BATCH */
    __s32 sched_nice;

    /* SCHED_FIFO, SCHED_RR */
    __u32 sched_priority;

    /* SCHED_DEADLINE (nsec) */
    __u64 sched_runtime;
    __u64 sched_deadline;
    __u64 sched_period;
};

int sched_setattr(pid_t pid,
        const struct sched_attr *attr,
        unsigned int flags)
{
    return syscall(__NR_sched_setattr, pid, attr, flags);
}

int sched_getattr(pid_t pid,
        struct sched_attr *attr,
        unsigned int size,
        unsigned int flags)
{
    return syscall(__NR_sched_getattr, pid, attr, size, flags);
}

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
        Alphabot() {
            this->AIN1 = 12;
            this->AIN2 = 13;
            this->ENA = 6;
            this->BIN1 = 20;
            this->BIN2 = 21;
            this->ENB = 26;
            this->speed_left = 10;
            this->speed_right = 10;
        }

        int init(){
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

        int setSpeed(int speed_left, int speed_right) {
            this->speed_left = speed_left;
            this->speed_right = speed_right;
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

        int forward() {
            gpioPWM(this->ENA, this->speed_left);
            gpioPWM(this->ENB, this->speed_right);
            gpioWrite(this->AIN1, LOW);
            gpioWrite(this->AIN2, HIGH); 
            gpioWrite(this->BIN1, LOW);
            gpioWrite(this->BIN2, HIGH);
            return 0;
        }

        int backward() {
            gpioPWM(this->ENA, this->speed_left);
            gpioPWM(this->ENB, this->speed_right);
            gpioWrite(this->AIN1, HIGH);
            gpioWrite(this->AIN2, LOW); 
            gpioWrite(this->BIN1, HIGH);
            gpioWrite(this->BIN2, LOW);
            return 0;
        }

        int left() {
            gpioPWM(this->ENA, this->speed_left);
            gpioPWM(this->ENB, this->speed_right);
            gpioWrite(this->AIN1, HIGH);
            gpioWrite(this->AIN2, LOW); 
            gpioWrite(this->BIN1, LOW);
            gpioWrite(this->BIN2, HIGH);
            return 0;
        }

        int right() {
            gpioPWM(this->ENA, this->speed_left);
            gpioPWM(this->ENB, this->speed_right);
            gpioWrite(this->AIN1, LOW);
            gpioWrite(this->AIN2, HIGH); 
            gpioWrite(this->BIN1, HIGH);
            gpioWrite(this->BIN2, LOW);
            return 0;
        }

        ~Alphabot(){
            this->stop();
        }
};

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

// ---------------------------------------------------------------

#define MOVEMENT_SPEED 20

struct task_params {
    int runtime;
    int period;
    int deadline;
    void (*function)(void);
};


void *run_deadline(void *parameters) {
    struct sched_attr attr;
    int ret;
    unsigned int flags = 0;

    struct task_params *params = (struct task_params *)parameters;

    printf("deadline thread started [%ld]\n", gettid());

    attr.size = sizeof(attr);
    attr.sched_flags = 0;
    attr.sched_nice = 0;
    attr.sched_priority = 0;

    /* This creates a 10ms/30ms reservation */
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = params->runtime;
    attr.sched_period = params->period;
    attr.sched_deadline = params->deadline;

    ret = sched_setattr(0, &attr, flags);
    if (ret < 0) {
        done = 0;
        perror("sched_setattr");
        exit(-1);
    }

    params->function();

    printf("deadline thread dies [%ld]\n", gettid());
    return NULL;
}

static volatile bool near_object;
static volatile int marker_id = -1;

enum direction {
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD,
    STOP
};

struct movement_params {
    enum direction direction;
    int speed_left;
    int speed_right;
};

static volatile movement_params movement;
static pthread_mutex_t movement_mutex;


void proximity_task() {
    gpioSetPullUpDown(DR, PI_PUD_UP);
    gpioSetPullUpDown(DL, PI_PUD_UP);

    int x = 0;
    while (!done) {
        // timespec time1, time2;
        // int temp;
        // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

        near_object = 1-(gpioRead(DR) && gpioRead(DL));

        // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
        // std::cout<<diff(time1,time2).tv_sec<<","<<diff(time1,time2).tv_nsec<<std::endl;
        sched_yield();
    }
}

static cv::VideoCapture inputVideo;

void camera_task() {

    cv::Mat cameraMatrix, distCoeffs;
    float markerLength = 0.05;
    
    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_6X6_250));

    cv::Mat image;

    std::cout << "Starting camera loop" << std::endl;

    while (!done) {
        // timespec time1, time2;
        // int temp;
        // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

        marker_id = -1;
        inputVideo.grab();
        inputVideo >> image;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams);

        // If at least one marker detected
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            
            int minimum_id = 999999;
            for(int i = 0; i < ids.size(); ++i) {
                if(ids[i] < minimum_id) {
                    minimum_id = ids[i];
                }
            }
            marker_id = minimum_id;
        }
        
        // imshow("Display window", image);
        // char key = (char) cv::waitKey(1);
        // if (key == 27)
        //     break;

        // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
        // std::cout<<diff(time1,time2).tv_sec<<","<<diff(time1,time2).tv_nsec<<std::endl;
        sched_yield();
    }
}

enum CoordinatorState {
    ROTATING_STATE,
    MOVING_FORWARD_STATE,
    STOP_STATE,
};

void move(direction dir, int speed) {
    pthread_mutex_lock(&movement_mutex);
    movement.direction = dir;
    movement.speed_left = speed;
    movement.speed_right = speed;
    pthread_mutex_unlock(&movement_mutex);
}

void unbalanced_move(direction dir, int speed_left, int speed_right) {
    pthread_mutex_lock(&movement_mutex);
    movement.direction = dir;
    movement.speed_left = speed_left;
    movement.speed_right = speed_right;
    pthread_mutex_unlock(&movement_mutex);
}

void coordinator_task() {
    int counter = 0;
    int current_instruction = -1;

    enum CoordinatorState state = ROTATING_STATE;

    while (!done) {
        // std::cout << "coordinator, current_instruction="  << current_instruction << ", counter=" << counter << std::endl;
        
        // timespec time1, time2;
        // int temp;
        // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

        if(near_object == 1) {
            move(STOP, 0);
        } else {
            if(current_instruction == -1 && marker_id >= 0 && marker_id <= 5) {
                current_instruction = marker_id;
                counter = 1000;
            }

            if(current_instruction == 0) {
                move(FORWARD, MOVEMENT_SPEED);
            }
            if(current_instruction == 1) {
                move(BACKWARD, MOVEMENT_SPEED);
            }
            if(current_instruction == 2) {
                move(LEFT, MOVEMENT_SPEED);
            }
            if(current_instruction == 3) {
                move(RIGHT, MOVEMENT_SPEED);
            }
            if(current_instruction == 4) {
                unbalanced_move(FORWARD, 20, 50);
            }
            if(current_instruction == 5) {
                unbalanced_move(BACKWARD, 20, 50);
            }

            if(counter <= 0) {
                counter = -1;
                current_instruction = -1;
                move(STOP, 0);
            }
        }
        counter--;

        // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
        // std::cout<<diff(time1,time2).tv_sec<<","<<diff(time1,time2).tv_nsec<<std::endl;

        // std::cout << "coordinator yield" << std::endl;
        sched_yield();
    }
}

void motor_task() {
   
    Alphabot alphabot = Alphabot();
    if(alphabot.init()) {
        std::cout << "exited with error code 1";
        return;
    }

    while (!done) {

        // timespec time1, time2;
        // int temp;
        // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);


        // std::cout << "motor" << std::endl;
        pthread_mutex_lock(&movement_mutex);
        
        // std::cout << movement.speed_left  << "," << movement.speed_right << std::endl;
        alphabot.setSpeed(movement.speed_left, movement.speed_right);

        //std::cout << "moving in " << movement.direction << " with speed=" << movement.speed << std::endl;

        switch (movement.direction) {
            case FORWARD:
                alphabot.forward();
                break;
            case BACKWARD:
                alphabot.backward();
                break;
            case LEFT:
                alphabot.left();
                break;
            case RIGHT:
                alphabot.right();
                break;
            case STOP:
                alphabot.stop();
                break;
        }
        // int ms = 1000;
        // gpioDelay(5 * 1000);
    
        pthread_mutex_unlock(&movement_mutex);


        // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
        // std::cout<<diff(time1,time2).tv_sec<<","<<diff(time1,time2).tv_nsec<<std::endl;
        sched_yield();
    }
}

task_params* get_task_params(int runtime, int period, int deadline, void (*function)(void)) {
    struct task_params *params = (struct task_params *)malloc(sizeof(struct task_params));
    params->runtime = runtime;
    params->period = period;
    params->deadline = deadline;
    params->function = function;
    return params;
}


int main (int argc, char **argv)
{

    if (gpioInitialise() < 0)
    {
        return 1;
    }
    
    inputVideo.open(0);
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    inputVideo.set(cv::CAP_PROP_FPS, 90);

    printf("main thread [%ld]\n", gettid());

    int ms = 1000000;
    int us = 1000;

    pthread_mutexattr_t movement_mutex_attr;
    pthread_mutexattr_setprotocol(&movement_mutex_attr, PTHREAD_PRIO_INHERIT);
    pthread_mutexattr_init(&movement_mutex_attr);
    pthread_mutex_init(&movement_mutex, &movement_mutex_attr);

    // Distance detection
    pthread_t thread1;
    struct task_params *params1 = get_task_params(150*us, 2 * ms, 2 * ms, proximity_task);
    pthread_create(&thread1, NULL, run_deadline, (void *) params1);

    // Camera 
    pthread_t thread2;                                      
    struct task_params *params2 = get_task_params((int) 45*ms, (int) 50*ms , (int) 50*ms, camera_task);
    pthread_create(&thread2, NULL, run_deadline, (void *) params2);

    // Coordinator
    pthread_t thread3;
    struct task_params *params3 = get_task_params(150*us, 2 * ms, 2 * ms, coordinator_task);
    pthread_create(&thread3, NULL, run_deadline, (void *) params3);

    // Motor control
    pthread_t thread4;
    struct task_params *params4 = get_task_params(150*us, 2 * ms, 2 * ms, motor_task);
    pthread_create(&thread4, NULL, run_deadline, (void *) params4);


    sleep(50);

    done = 1;


    pthread_join(thread1, NULL);
    // std::cout << "Joined 1" << std::endl;

    pthread_join(thread2, NULL);
    //std::cout << "Joined 2" << std::endl;

    pthread_join(thread3, NULL);
    //std::cout << "Joined 3" << std::endl;
    
    pthread_join(thread4, NULL);
    //std::cout << "Joined 4" << std::endl;

    pthread_mutexattr_destroy(&movement_mutex_attr);
    //std::cout << "Destroyed mutex attr" << std::endl;

    pthread_mutex_destroy(&movement_mutex);
    //std::cout << "Destroyed mutex" << std::endl;

    gpioTerminate();

    printf("main dies [%ld]\n", gettid());
    return 0;
}





// void do_useful_computation(void) {
//     /* do periodic computation, with execution time enforcement */
//     while (1) {
//         // do_the_computation();
//         std::cout << "Doing computation" << std::endl;
//         /*
//         * Notify the scheduler the end of the computation
//         * This syscall will block until the next replenishment
//         */
//         sched_yield();
//     }
// }

// int main (int argc, char **argv) {
//     int ret;
//     int flags = 0;
//     struct sched_attr attr;
//     memset(&attr, 0, sizeof(attr));
//     attr.size = sizeof(attr);
//     /* This creates a 200ms / 1s reservation */
//     attr.sched_policy = SCHED_DEADLINE;
//     attr.sched_runtime = 200000000; /*200 ms*/
//     attr.sched_deadline = attr.sched_period = 1000000000; /*1 s*/
//     ret = sched_setattr(0, &attr, flags);
//     if (ret < 0) {
//         perror("sched_setattr failed to set the priorities");
//         exit(-1);
//     }
//     do_useful_computation();
//     exit(0);
// }


