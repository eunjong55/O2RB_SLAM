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

int main(int argc, char **argv)
{
    string path_to_vocabulary = "/home/cgv/Desktop/github/O2RB_SLAM/Vocabulary/ORBvoc.txt";
    string path_to_settings = "/home/cgv/Desktop/github/O2RB_SLAM/Examples/Monocular/vadas_cam_params.yaml";
    // string path_to_settings = "/home/cgv/Desktop/dataset/fish_640_480.yaml";

    VideoCapture cap("/home/cgv/Desktop/dataset/vadas/2019.mp4");
    // VideoCapture cap(0);

    double delay = 1000.0 / cap.get(CV_CAP_PROP_FPS);
    // double delay = 33;

    int start = 0;
    cap.set(CV_CAP_PROP_POS_MSEC, (double)start * 1000);

    Mat mask = imread("/home/cgv/Desktop/github/O2RB_SLAM/Examples/Monocular/mask_fisheye.png", IMREAD_GRAYSCALE);
    Mat cube_mask = imread("/home/cgv/Desktop/CubemapSLAM-master/Masks/vadas_gray_cubemap_front_mask_650.png", IMREAD_GRAYSCALE);

    // Mat mask = imread("/home/cgv/Desktop/dataset/osuk_3rd_floor_mask_mask.jpg", IMREAD_GRAYSCALE);

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

    int width = CamModelGeneral::GetCamera()->GetCubeFaceWidth(), height = CamModelGeneral::GetCamera()->GetCubeFaceHeight();
    Mat cubemapImg(height * 3, width * 3, CV_8U, Scalar::all(0));

    // Main loop
    cv::Mat im;
    for(int i=0; i<200; i++) cap >> im;
    while (true)
    {
        struct timeval start = {};
        gettimeofday(&start, NULL);

        // Read image from file
        cap >> im; 
        if (im.empty())
        {
            cerr << endl
                 << "Failed to read camera" << endl;
            return 1;
        }

        cv::cvtColor(im,im,CV_BGR2GRAY);
        SLAM.CvtFisheyeToCubeMap_reverseQuery_withInterpolation(cubemapImg, im, INTER_LINEAR);

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(cube_mask, cubemapImg, im, mask, delay, LUT);

        struct timeval end = {};
        gettimeofday(&end, NULL);

        double time = (end.tv_sec - start.tv_sec)*1000.0  + ( end.tv_usec -start.tv_usec) /1000.0;
        FILE * fp_ = fopen("/home/cgv/Desktop/github/O2RB_time1.csv", "a");
        //fprintf(fp_, "%lf, %lf\n", cap.get(CV_CAP_PROP_POS_FRAMES), time);
        fclose(fp_);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}