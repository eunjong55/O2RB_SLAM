#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

#include <sys/time.h>

using namespace std;
using namespace cv;
/* track on off */
unsigned int HD;
unsigned int goodmatch;
unsigned int DN;
unsigned int MapN;
unsigned int frame_counter;
unsigned int frame_lost_counter;
unsigned int frame_track_started;

int main(int argc, char **argv)
{
    string path_to_vocabulary = "../../Vocabulary/ORBvoc.txt";
    string path_to_settings = "/home/misoyuri/Desktop/Data/Squence/vadas_cam_params.yaml";

    VideoCapture cap("/home/misoyuri/Downloads/AVB_20210910133755_00004_ch01.mp4");
    double delay = 1000.0 / cap.get(CV_CAP_PROP_FPS);

    int start = 15;
    cap.set(CV_CAP_PROP_POS_MSEC, (double)start * 1000);

    Mat mask = imread("/home/misoyuri/Desktop/Data/Squence/NewVadasFishMask.png", IMREAD_GRAYSCALE);

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
    int cnt = 0;
    while (true)
    {
        // struct timeval start = {};
        // gettimeofday(&start, NULL);

        // Read image from file
        cap >> im; 
        cnt++;
        if (im.empty())
        {
            cerr << endl
                 << "Failed to read camera" << endl;
            return 1;
        }
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, mask, cnt, LUT);

        // struct timeval end = {};
        // gettimeofday(&end, NULL);

        // double time = (end.tv_sec - start.tv_sec)*1000.0  + ( end.tv_usec -start.tv_usec) /1000.0;
        // FILE * fp_ = fopen("/home/cgv/Desktop/github/O2RB_time1.csv", "a");
        // fprintf(fp_, "%lf, %lf\n", cap.get(CV_CAP_PROP_POS_FRAMES), time);
        // fclose(fp_);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}