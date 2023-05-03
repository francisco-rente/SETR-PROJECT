#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

int main(){
    cv::VideoCapture inputVideo(0);

    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    inputVideo.set(cv::CAP_PROP_FPS, 60);

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
        inputVideo >> image;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams);

        // If at least one marker detected
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            // int nMarkers = corners.size();
            // std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

            // Calculate pose for each marker
            // std::cout << "solvePnP" << std::endl;
            // for (int i = 0; i < nMarkers; i++) {
            //     solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            // }

            // // Draw axis for each marker
            // std::cout << "drawFrameAxes" << std::endl;
            // for(unsigned int i = 0; i < ids.size(); i++) {
            //     cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            // }
        }
        imshow("Display window", image);
        // // Show resulting image and close window
        // //cv::imshow("out", imageCopy);
        char key = (char) cv::waitKey(1);
        if (key == 27)
            break;
    }
}