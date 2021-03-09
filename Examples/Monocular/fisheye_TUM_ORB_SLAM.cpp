#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;
using namespace cv;
unsigned int HD;
unsigned int goodmatch;
unsigned int DN;
unsigned int MapN;
void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    string path_to_vocabulary = "/home/oooony/바탕화면/jiwon/fisheye_ORB_SLAM/Vocabulary/ORBvoc.txt";
    string path_to_settings = "/home/oooony/바탕화면/jiwon/fisheye_ORB_SLAM/Examples/Monocular/vadas_fisheye.yaml";


    //  /home/oooony/바탕화면/dataset/sequence_11

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages("/home/oooony/바탕화면/dataset/sequence_17/images/", vstrImageFilenames, vTimestamps);
    Mat mask = imread("/home/oooony/바탕화면/dataset/sequence_17/mask_fisheye.png", IMREAD_GRAYSCALE);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::MONOCULAR, true);

    // Main loop
    cv::Mat im;
    int ni =0;
    while (true)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (im.empty())
        {
            cerr << endl
                 << "Failed to read camera" << endl;
            return 1;
        }
        if( ni ==  1600)
        {
            cout << "frame end" << endl;
            break;
        }
        
        int delay = 33;
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, mask, delay);
        ni++;
        

    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}


void LoadImages(const string& strFile, vector<string>& vstrImageFilenames, vector<double>& vTimestamps)
{
	int num_imgs = 1600;
	string file_name;
	vstrImageFilenames.resize(num_imgs);

	for (int i = 0; i < num_imgs; i++)
	{
		stringstream ss;
		ss << setfill('0') << setw(5) << i;
		vstrImageFilenames[i] = strFile + ss.str() + ".jpg";
		vTimestamps.push_back(33);
	}
}

