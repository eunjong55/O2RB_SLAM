#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;
using namespace cv;
/* track on off */
unsigned int HD;
unsigned int goodmatch;
unsigned int DN;
unsigned int MapN;
unsigned int frame_counter;

int main(int argc, char **argv)
{
    string path_to_vocabulary = "/home/oooony/바탕화면/jiwon/fisheye_ORB_SLAM/Vocabulary/ORBvoc.txt";
    string path_to_settings = "/home/oooony/바탕화면/jiwon/fisheye_ORB_SLAM/Examples/Monocular/vadas_cam_params.yaml";

    VideoCapture cap("/home/oooony/바탕화면/dataset/Vadas/AVB_20190702151249_00002_ch01.h264");
    // VideoCapture cap("/home/oooony/바탕화면/dataset/Vadas/AVB_20180829141553_00002_ch01.h264");
    // VideoCapture cap("/home/oooony/Dataset/vadas_fisheye_parking_dataset/P-001/ch01.mp4");

    double delay = 1000.0 / cap.get(CV_CAP_PROP_FPS);

    int start = 35;
    cap.set(CV_CAP_PROP_POS_MSEC, (double)start * 1000);

    Mat mask = imread("/home/oooony/바탕화면/jiwon/fisheye_ORB_SLAM/Examples/Monocular/mask_fisheye.png", IMREAD_GRAYSCALE);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::MONOCULAR, true);

    std::vector<cv::Mat> LUT;
    cv::Mat LUT_x = SLAM.mMap1;
    cv::Mat LUT_y = SLAM.mMap2;
    cv::Mat LUT_x_inv = SLAM.mMap1_inv;
    cv::Mat LUT_y_inv = SLAM.mMap2_inv;
    LUT.push_back(LUT_x);
    LUT.push_back(LUT_y);
    LUT.push_back(LUT_x_inv);
    LUT.push_back(LUT_y_inv);

    // Main loop
    cv::Mat im;

    while (true)
    {
        // Read image from file
        cap >> im; 
        cap.grab();

        if (im.empty())
        {
            cerr << endl
                 << "Failed to read camera" << endl;
            return 1;
        }
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, mask, delay, LUT);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}