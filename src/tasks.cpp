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
#include <sys/mman.h>
#include <cmath>

#include "Realtime.h"
#include "Alphabot.h"

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

#define LOG_TIME 1

#define RES_WIDTH 320
#define RES_HEIGHT 240

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

#define MOVEMENT_SPEED 30

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
static volatile int center_distance = 0;

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
static pthread_mutex_t camera_mutex;


inline void print_time() {
    timespec time1; 
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
    std::cout << time1.tv_sec << "," << time1.tv_nsec << std::endl;
}


void proximity_task() {
    #ifdef LOG_TIME
        timespec time;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time);
        fprintf(stdout, "st_p,%ld,%ld,%ld\n", time.tv_sec, time.tv_nsec, 0);
    #endif

    mlockall(MCL_CURRENT | MCL_FUTURE);

    gpioSetPullUpDown(DR, PI_PUD_UP);
    gpioSetPullUpDown(DL, PI_PUD_UP);

    while (!done) {
        timespec time1, time2;
        
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

        near_object = 1-(gpioRead(DR) && gpioRead(DL));

        #ifdef LOG_TIME
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
            fprintf(stdout, "p,%ld,%ld,%ld\n", time1.tv_sec, time1.tv_nsec, diff(time1,time2).tv_nsec);
        #endif

        sched_yield();
    }
}

static cv::VideoCapture inputVideo;

void camera_task() {
    #ifdef LOG_TIME
        timespec time;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time);
        fprintf(stdout, "st_c,%ld,%ld,%ld\n", time.tv_sec, time.tv_nsec, 0);
    #endif

    mlockall(MCL_CURRENT | MCL_FUTURE);

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
        timespec time1, time2;
        
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

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
            int maximum_area = 0;
            int minimum_id_idx = 0;
            int correct_center_distance = 0;
            cv::Point2f markerCenter;
            for(int i = 0; i < ids.size(); ++i) {
                if(ids[i] <= minimum_id) {
                    minimum_id = ids[i];
                    minimum_id_idx = i;

                    if(minimum_id == 6) {
                        // Calculate center point of each marker
                        markerCenter = (corners[minimum_id_idx][0] + corners[minimum_id_idx][1] + corners[minimum_id_idx][2] + corners[minimum_id_idx][3]) / 4;
                        correct_center_distance = markerCenter.x - (int) RES_WIDTH/2;
                        // Calculate marker area
                        int area = cv::contourArea(corners[minimum_id_idx]);
                        if(area > maximum_area) {
                            maximum_area = area;
                            marker_id = minimum_id;
                        }
                    }
                }
            }
            pthread_mutex_lock(&camera_mutex);
            marker_id = minimum_id;
            center_distance = correct_center_distance;
            pthread_mutex_unlock(&camera_mutex);
        }

        #ifdef LOG_TIME
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
            fprintf(stdout, "c,%ld,%ld,%ld\n", time1.tv_sec, time1.tv_nsec, diff(time1,time2).tv_nsec);
        #endif
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
    #ifdef LOG_TIME
        timespec time;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time);
        fprintf(stdout, "st_o,%ld,%ld,%ld\n", time.tv_sec, time.tv_nsec, 0);
    #endif
    mlockall(MCL_CURRENT | MCL_FUTURE);

    int counter = 0;
    int current_instruction = -1;

    enum CoordinatorState state = ROTATING_STATE;

    while (!done) {
        timespec time1, time2;
        
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

        pthread_mutex_lock(&camera_mutex);
        int new_marker_id = marker_id;
        int new_center_distance = center_distance;
        pthread_mutex_unlock(&camera_mutex);

        if(near_object == 1) {
            move(STOP, 0);
        } else {
            if(current_instruction == -1 && new_marker_id >= 0 && new_marker_id <= 5) {
                current_instruction = new_marker_id;
                counter = 1000;
            }
            if((current_instruction == -1 || current_instruction == 6) && new_marker_id == 6) {
                current_instruction = new_marker_id;
                counter = 200;
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
            if(current_instruction == 6) {
                if(new_center_distance < -10) {
                    unbalanced_move(FORWARD, 30, 35);
                }
                else if(new_center_distance > 10) {
                    unbalanced_move(FORWARD, 35, 30);
                } else {
                    move(FORWARD, MOVEMENT_SPEED);
                }
            }

            if(counter <= 0) {
                counter = -1;
                current_instruction = -1;
                move(STOP, 0);
            }
        }
        counter--;

        #ifdef LOG_TIME
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
            fprintf(stdout, "o,%ld,%ld,%ld\n", time1.tv_sec, time1.tv_nsec, diff(time1,time2).tv_nsec);
        #endif

        sched_yield();
    }
}




void motor_task() {
    #ifdef LOG_TIME
        timespec time;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time);
        fprintf(stdout, "st_m,%ld,%ld,%ld\n", time.tv_sec, time.tv_nsec, 0);
    #endif
    mlockall(MCL_CURRENT | MCL_FUTURE);

    Alphabot alphabot = Alphabot();
    if(alphabot.init()) {
        std::cout << "exited with error code 1";
        return;
    }

    while (!done) {

        timespec time1, time2;
        
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

        pthread_mutex_lock(&movement_mutex);
        alphabot.setSpeed(movement.speed_left, movement.speed_right);

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
    
        pthread_mutex_unlock(&movement_mutex);


        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
        #ifdef LOG_TIME
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
            fprintf(stdout, "m,%ld,%ld,%ld\n", time1.tv_sec, time1.tv_nsec, diff(time1,time2).tv_nsec);
        #endif

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


int main ()
{

    if (gpioInitialise() < 0)
    {
        return 1;
    }

    Realtime::setup();
    
    inputVideo.open(0);

    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, RES_WIDTH);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, RES_HEIGHT);
    // inputVideo.set(cv::CAP_PROP_FPS, 45); // Config A
    inputVideo.set(cv::CAP_PROP_FPS, 25); // Config B

    printf("main thread [%ld]\n", gettid());

    int ms = 1000000;
    int us = 1000;

    pthread_mutexattr_t movement_mutex_attr;
    pthread_mutexattr_setprotocol(&movement_mutex_attr, PTHREAD_PRIO_INHERIT);
    pthread_mutexattr_init(&movement_mutex_attr);
    pthread_mutex_init(&movement_mutex, &movement_mutex_attr);

    pthread_mutexattr_t camera_mutex_attr;
    pthread_mutexattr_setprotocol(&camera_mutex_attr, PTHREAD_PRIO_INHERIT);
    pthread_mutexattr_init(&camera_mutex_attr);
    pthread_mutex_init(&camera_mutex, &camera_mutex_attr);

    pthread_attr_t init_attr;
    pthread_attr_init(&init_attr);
    pthread_attr_setscope(&init_attr, PTHREAD_SCOPE_SYSTEM);


    // ---------------- Config A ----------------
    // Distance detection
    // pthread_t thread1;
    // struct task_params *params1 = get_task_params(80*us, 7 * ms, 7 * ms, proximity_task);
    // pthread_create(&thread1, NULL, run_deadline, (void *) params1);

    // // Camera 
    // pthread_t thread2;                                      
    // struct task_params *params2 = get_task_params((int) 20*ms, (int) 22*ms , (int) 22*ms, camera_task);
    // pthread_create(&thread2, NULL, run_deadline, (void *) params2);

    // // Coordinator
    // pthread_t thread3;
    // struct task_params *params3 = get_task_params(80*us,  7 * ms,  7 * ms, coordinator_task);
    // pthread_create(&thread3, NULL, run_deadline, (void *) params3);

    // // Motor control
    // pthread_t thread4;
    // struct task_params *params4 = get_task_params(150*us,  7 * ms,  7 * ms, motor_task);
    // pthread_create(&thread4, NULL, run_deadline, (void *) params4);

    // ---------------- Config B ----------------
    // Distance detection
    pthread_t thread1;
    struct task_params *params1 = get_task_params((int) 200*us , 7 * ms, 7 * ms, proximity_task);
    pthread_create(&thread1, NULL, run_deadline, (void *) params1);

    // Camera 
    pthread_t thread2;                                      
    struct task_params *params2 = get_task_params((int) 35*ms, (int) 42*ms , (int) 42*ms, camera_task);
    pthread_create(&thread2, NULL, run_deadline, (void *) params2);

    // Coordinator
    pthread_t thread3;
    struct task_params *params3 = get_task_params((int) 150*us,  7 * ms,  7 * ms, coordinator_task);
    pthread_create(&thread3, NULL, run_deadline, (void *) params3);

    // Motor control
    pthread_t thread4;
    struct task_params *params4 = get_task_params((int) 600*us,  7 * ms,  7 * ms, motor_task);
    pthread_create(&thread4, NULL, run_deadline, (void *) params4);


    sleep(60);

    done = 1;


    pthread_join(thread1, NULL);
    std::cout << "Joined 1" << std::endl;

    pthread_join(thread2, NULL);
    std::cout << "Joined 2" << std::endl;

    pthread_join(thread3, NULL);
    std::cout << "Joined 3" << std::endl;
    
    pthread_join(thread4, NULL);
    std::cout << "Joined 4" << std::endl;

    pthread_mutexattr_destroy(&movement_mutex_attr);
    //std::cout << "Destroyed mutex attr" << std::endl;

    pthread_mutex_destroy(&movement_mutex);
    //std::cout << "Destroyed mutex" << std::endl;

    gpioTerminate();

    printf("main dies [%ld]\n", gettid());
    return 0;
}

