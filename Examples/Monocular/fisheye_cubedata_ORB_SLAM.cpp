#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <time.h>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;
using namespace cv;
unsigned int frame_counter;
extern float res_time;
unsigned int HD;
unsigned int goodmatch;
unsigned int DN;
unsigned int MapN;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    string path_to_vocabulary = "/home/oooony/바탕화면/jiwon/fisheye_ORB_SLAM/Vocabulary/ORBvoc.txt";
    string path_to_settings = "/home/oooony/바탕화면/dataset/Config/front_cam_params.yaml";

    // std::string fisheyeImgPath = "/home/oooony/바탕화면/dataset/parkinglot_front/";
    // std::ifstream fin("/home/oooony/바탕화면/dataset/parkinglot_front/front_images.lst");

    ///parkinglot_front
    ///loop2_front
    std::string fisheyeImgPath = "/home/oooony/바탕화면/dataset/loop2_front/";       ///
    std::ifstream fin("/home/oooony/바탕화면/dataset/loop2_front/front_images.lst"); ///

    // Retrieve paths to images
    std::vector<std::string> fisheyeImgNames;
    std::vector<double> vTimestamps;
    std::string line;
    while (std::getline(fin, line))
    {
        fisheyeImgNames.push_back(line);
        size_t p = line.find_last_of("_");
        std::stringstream ss(line.substr(0, p));
        double ts;
        ss >> ts;
        vTimestamps.push_back(ts);
    }

    Mat mask = imread("/home/oooony/바탕화면/dataset/mask/loop2_front_mask.png", IMREAD_GRAYSCALE);
    if (!mask.data)
    {
        cout << "err no mask!" << endl;
        exit(0);
    }

    const int imageCnt = fisheyeImgNames.size();
    std::cout << "find " << imageCnt << " images" << std::endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(imageCnt);

    string filepath_time = "/home/oooony/바탕화면/LFOV_time.txt";
    ofstream writeFile(filepath_time.data());

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
    unsigned int avg_cnt = 0;
    unsigned int avg_cntP = 0;
    unsigned int avg_hd = 0;
    unsigned int avg_DN = 0;

    cv::Mat im;
    int ni = 0;
    while (true)
    {
        avg_cnt++;
        //if(ni == imageCnt)
        if (ni == 200)
            break;

        std::string fisheyeImgName = fisheyeImgPath + fisheyeImgNames[ni];
        // Read image from file
        im = cv::imread(fisheyeImgName, CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (!im.data)
        {
            std::cout << "fail to read image in " << fisheyeImgName << std::endl;
            break;
        }
        if (ni == fisheyeImgNames.size())
        {
            cout << "frame end" << endl;
            break;
        }

        int delay = 33;

        /* time */
        clock_t start1, end1;
        float res1, res2;

        start1 = clock();
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, mask, delay, LUT);
        end1 = clock();
        res1 = (float)(end1 - start1) / CLOCKS_PER_SEC;

        avg_cntP += goodmatch;
        avg_hd += HD;
        avg_DN += DN;

        vTimesTrack[ni] = res1;
        std::cout << std::setprecision(10);
        // cout << "total time:: " << res1 << endl;// 1frame time
        // cout << "res_time::     " << res_time << endl;

        ni++;

        writeFile.precision(7);
        writeFile.setf(ios::fixed);
        // writeFile << std::setprecision(12);
        // writeFile << res_time << endl;   //projection only time
        // writeFile <<  res1 << endl; // 1frame time
    }
    writeFile.close();

    avg_cntP /= avg_cnt;
    avg_hd /= avg_cnt;
    avg_DN /= avg_cnt;
    cout << "  HD,mathces :: " << avg_hd << " , " << avg_cntP << " , " << avg_DN << endl;

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int idx = 0; idx < imageCnt; idx++)
    {
        totaltime += vTimesTrack[idx];
    }
    cout << "-------all" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[imageCnt / 2] << endl;
    cout << "mean tracking time: " << totaltime / imageCnt << endl;
    std::cout << "find " << imageCnt << " images" << std::endl;
    std::cout << "image in " << fisheyeImgPath << std::endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());
    int ts = 0;
    while (!f.eof())
    {
        string s;
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            string sRGB;
            vTimestamps.push_back(ts);
            ts++;
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}