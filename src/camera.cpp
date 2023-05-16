#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>


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

        int forward() {
            gpioPWM(this->ENA, this->speed);
            gpioPWM(this->ENB, this->speed);
            gpioWrite(this->AIN1, LOW);
            gpioWrite(this->AIN2, HIGH); 
            gpioWrite(this->BIN1, LOW);
            gpioWrite(this->BIN2, HIGH);
            return 0;
        }

        int backward() {
            gpioPWM(this->ENA, this->speed);
            gpioPWM(this->ENB, this->speed);
            gpioWrite(this->AIN1, HIGH);
            gpioWrite(this->AIN2, LOW); 
            gpioWrite(this->BIN1, HIGH);
            gpioWrite(this->BIN2, LOW);
            return 0;
        }

        int left(int turnSpeed) {
            gpioPWM(this->ENA, turnSpeed);
            gpioPWM(this->ENB, turnSpeed);
            gpioWrite(this->AIN1, HIGH);
            gpioWrite(this->AIN2, LOW); 
            gpioWrite(this->BIN1, LOW);
            gpioWrite(this->BIN2, HIGH);
            return 0;
        }

        int right(int turnSpeed) {
            gpioPWM(this->ENA, turnSpeed);
            gpioPWM(this->ENB, turnSpeed);
            gpioWrite(this->AIN1, LOW);
            gpioWrite(this->AIN2, HIGH); 
            gpioWrite(this->BIN1, HIGH);
            gpioWrite(this->BIN2, LOW);
            return 0;
        }
};


// float MoveTowards(float current, float target, float maxDelta) {
//     float distance = target - current;
//     // return current 
//     if (std::abs(target - current) *  <= maxDelta) {
//         // The target is closer than the maximum delta, so just return the target
//         return target;
//     } else {
//         // Move towards the target by the maximum delta
//         return current + std::copysign(maxDelta, target - current);
//     }
// } 


float calculateSpeed(float distance){
    int maxSpeed = 40;
    float speed = 40 * distance / 320;
    if(speed > maxSpeed) return maxSpeed;
    else if(speed < 4) return 0;
    else return speed;
}





int main(){
    // Alphabot alphabot = Alphabot();
    // if(alphabot.init()) {
    //     std::cout << "exited with error code 1";
    //     return 1;
    // }
    // alphabot.setSpeed(5);


    cv::VideoCapture inputVideo(0);

    // 1296x972
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1296);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 972);
    inputVideo.set(cv::CAP_PROP_FPS, 40);
    inputVideo.set(cv::CAP_PROP_MODE, 4);

    cv::Mat cameraMatrix, distCoeffs;
    float markerLength = 0.05;
    // You can read camera parameters from tutorial_camera_params.yml
    // readCameraParameters(cameraParamsFilename, cameraMatrix, distCoeffs); // This function is implemented in aruco_samples_utility.hpp
    
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
    // cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
    // Create aruco detector
    // cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // cv::Ptr<cv::aruco::Detector> detector = cv::aruco::Detector::create();

    cv::Mat image;
    while (inputVideo.grab()) {
        inputVideo.retrieve(image);

        // std::vector<int> ids;
        // std::vector<std::vector<cv::Point2f>> corners;
        // cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams);

        // int marker0Index = -1;
        // int marker1Index = -1;

        // // If at least one marker detected
        // if (ids.size() > 0) {
        //     cv::aruco::drawDetectedMarkers(image, corners, ids);
            
        //     // Print corners of each marker
        //     for (int i = 0; i < corners.size(); i++) {
        //         if(ids[i] <= 1){
        //             if(ids[i] == 0){
        //                 marker0Index = i;
        //             }
        //             else{
        //                 marker1Index = i;
        //             }
        //         }
                
        //     }
        // }
        
        // // If both markers detected
        // if(marker0Index != -1 && marker1Index != -1){
        //     // Calculate center point of each marker
        //     cv::Point2f marker0Center = (corners[marker0Index][0] + corners[marker0Index][1] + corners[marker0Index][2] + corners[marker0Index][3]) / 4;
        //     cv::Point2f marker1Center = (corners[marker1Index][0] + corners[marker1Index][1] + corners[marker1Index][2] + corners[marker1Index][3]) / 4;
            

        //     // Calculate middle point between markers
        //     float middlePoint = (marker0Center.x + marker1Center.x) / 2;
        //     std::cout << "Middle point: " << middlePoint << "\n";
        //     std::cout << "Move: " << middlePoint - 320 << "\n";


        //     // draw vertical line with middle point
        //     cv::line(image, cv::Point(middlePoint, 0), cv::Point(middlePoint, 480), cv::Scalar(0, 0, 255), 2);

        //     int move = middlePoint - 320;
        
        //     // int speed = (int) calculateSpeed(abs(move));
        //     // std::cout << "SPEED" << speed << '\n'; 
        //     // int time = 100000;

        //     // if(speed > 0){
        //     //     if(move > 0){
        //     //         alphabot.right(speed);
        //     //     }
        //     //     else{
        //     //         alphabot.left(speed);
        //     //     }
        //     //     gpioDelay(time);
        //     //     alphabot.stop();
        //     // }


        //     // int time = 100000;
        //     // if(move <= 20 && move >= -20) {
        //     //     alphabot.stop();
        //     //     gpioDelay(time);
        //     // }
        //     // else if(move < 0) {
        //     //     alphabot.left(speed);
        //     //     gpioDelay(time);
        //     //     alphabot.stop();
        //     // } else {
        //     //     alphabot.right(speed);
        //     //     gpioDelay(time);
        //     //     alphabot.stop();
        //     // }
        // }
        imshow("Display window", image);
        // // Show resulting image and close window
        // //cv::imshow("out", imageCopy);
        char key = (char) cv::waitKey(1);
        if (key == 27)
            break;
    }
}