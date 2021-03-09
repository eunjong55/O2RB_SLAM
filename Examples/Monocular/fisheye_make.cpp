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

int main(int argc, char **argv)
{
    string path_to_vocabulary = "/home/oooony/바탕화면/jiwon/fisheye_ORB_SLAM/Vocabulary/ORBvoc.txt";
    string path_to_settings = "/home/oooony/바탕화면/jiwon/fisheye_ORB_SLAM/Examples/Monocular/vadas_fisheye.yaml";


    //omni_static_obj.avi
    //omni_move.avi
    //omni_static.avi

    VideoCapture cap("/home/oooony/바탕화면/dataset/collected_data/omni_static.mp4");

    double delay = 1000.0 / cap.get(CV_CAP_PROP_FPS);

    int start = 35;
    cap.set(CV_CAP_PROP_POS_MSEC, (double)start * 1000);

    Mat mask = imread("/home/oooony/바탕화면/jiwon/fisheye_ORB_SLAM/Examples/Monocular/mask_fisheye.png", IMREAD_GRAYSCALE);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::MONOCULAR, true);

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
        SLAM.TrackMonocular(im, mask, delay);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}