#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <cassert>
#include <string>
#include <vector>
#include <time.h>

#include <opencv2/core/core.hpp>

#include "System.h"

using namespace std;
using namespace cv;
unsigned int frame_counter;
unsigned int frame_lost_counter;
unsigned int frame_track_started;

unsigned int HD;
unsigned int goodmatch;
unsigned int DN;
unsigned int MapN;
extern float res_time;

// #define INDOOR_STATIC
// #define INDOOR_DYNAMIC
// #define OUTDOOR_STATIC
// #define OUTDOOR_STATIC2
// #define OUTDOOR_ROTATION
#define OUTDOOR_LARGE_LOOP

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    frame_counter = 0;
    frame_lost_counter = 0;
    string path_to_vocabulary = "../../Vocabulary/ORBvoc.txt";
    string path_to_settings = "../../Config/lafida_cam0_params.yaml";

    #ifdef INDOOR_STATIC
    ///indoor_static
    std::string fisheyeImgPath = "/home/misoyuri/Desktop/Data/Squence/Lafida/indoor_static/imgs/cam0/"; ///
    std::ifstream fin("/home/misoyuri/Desktop/Data/Squence/Lafida/indoor_static/images_and_timestamps.txt"); ///
    // std::string perfSavingPath("/home/misoyuri/Desktop/Time_Check/o2rb_perf.txt"); ///
    std::string perfSavingPath("/home/misoyuri/Desktop/Data/Test_Log/indoor_static/o2rb/o2rb_perf.txt"); ///
    std::string filepath_time = "/home/misoyuri/Desktop/Data/Test_Log/indoor_static/o2rb/LFOV_time.txt";
    std::string trajSavingPath("/home/misoyuri/Desktop/Data/Test_Log/indoor_static/o2rb/o2rb_KeyFrameTrajectory.txt");
    #endif

    #ifdef INDOOR_DYNAMIC
    ///indoor_dynamic
    std::string fisheyeImgPath = "/home/misoyuri/Desktop/Data/Squence/Lafida/indoor_dynamic/imgs/cam0/"; ///
    std::ifstream fin("/home/misoyuri/Desktop/Data/Squence/Lafida/indoor_dynamic/images_and_timestamps.txt"); ///
    // std::string perfSavingPath("/home/misoyuri/Desktop/Time_Check/o2rb_perf.txt"); ///
    std::string perfSavingPath("/home/misoyuri/Desktop/Data/Log/indoor_dynamic/o2rb/o2rb_perf.txt"); ///
    std::string filepath_time = "/home/misoyuri/Desktop/Data/Test_Log/indoor_dynamic/o2rb/LFOV_time.txt";
    std::string trajSavingPath("/home/misoyuri/Desktop/Data/Test_Log/indoor_dynamic/o2rb/o2rb_KeyFrameTrajectory.txt");
    #endif
    
    #ifdef OUTDOOR_STATIC
    ///outdoor_static     
    std::string fisheyeImgPath = "/home/misoyuri/Desktop/Data/Squence/Lafida/outdoor_static/imgs/cam0/"; ///
    std::ifstream fin("/home/misoyuri/Desktop/Data/Squence/Lafida/outdoor_static/images_and_timestamps.txt"); ///
    // std::string perfSavingPath("/home/misoyuri/Desktop/Time_Check/o2rb_perf.txt"); ///
    std::string perfSavingPath("/home/misoyuri/Desktop/Data/Test_Log/outdoor_static/o2rb/o2rb_perf.txt"); ///
    std::string filepath_time = "/home/misoyuri/Desktop/Data/Test_Log/outdoor_static/o2rb/LFOV_time.txt";
    std::string trajSavingPath("/home/misoyuri/Desktop/Data/Test_Log/outdoor_static/o2rb/o2rb_KeyFrameTrajectory.txt");
    #endif

    #ifdef OUTDOOR_STATIC2
    ///outdoor_static2
    std::string fisheyeImgPath = "/home/misoyuri/Desktop/Data/Squence/Lafida/outdoor_static2/imgs/cam0/"; ///
    std::ifstream fin("/home/misoyuri/Desktop/Data/Squence/Lafida/outdoor_static2/images_and_timestamps.txt"); ///
    // std::string perfSavingPath("/home/misoyuri/Desktop/Time_Check/o2rb_perf.txt"); ///
    std::string perfSavingPath("/home/misoyuri/Desktop/Data/Test_Log/outdoor_static2/o2rb/o2rb_perf.txt"); ///
    std::string filepath_time = "/home/misoyuri/Desktop/Data/Test_Log/outdoor_static2/o2rb/LFOV_time.txt";
    std::string trajSavingPath("/home/misoyuri/Desktop/Data/Test_Log/outdoor_static2/o2rb/o2rb_KeyFrameTrajectory.txt");
    #endif

    #ifdef OUTDOOR_ROTATION
    ///outdoor_rotation
    std::string fisheyeImgPath = "/home/misoyuri/Desktop/Data/Squence/Lafida/outdoor_rotation/imgs/cam0/"; ///
    std::ifstream fin("/home/misoyuri/Desktop/Data/Squence/Lafida/outdoor_rotation/images_and_timestamps.txt"); ///
    // std::string perfSavingPath("/home/misoyuri/Desktop/Time_Check/o2rb_perf.txt"); ///
    std::string perfSavingPath("/home/misoyuri/Desktop/Data/Test_Log/outdoor_rotation/o2rb/o2rb_perf.txt"); ///
    std::string filepath_time = "/home/misoyuri/Desktop/Data/Test_Log/outdoor_rotation/o2rb/LFOV_time.txt";
    std::string trajSavingPath("/home/misoyuri/Desktop/Data/Test_Log/outdoor_rotation/o2rb/o2rb_KeyFrameTrajectory.txt");
    #endif

    #ifdef OUTDOOR_LARGE_LOOP
    ///outdoor_large_loop
    std::string fisheyeImgPath = "/home/misoyuri/Desktop/Data/Squence/Lafida/outdoor_large_loop/imgs/cam0/"; ///
    std::ifstream fin("/home/misoyuri/Desktop/Data/Squence/Lafida/outdoor_large_loop/images_and_timestamps.txt"); ///
    // std::string perfSavingPath("/home/misoyuri/Desktop/Time_Check/o2rb_perf.txt"); ///
    std::string perfSavingPath("/home/misoyuri/Desktop/Data/Test_Log/outdoor_large_loop/o2rb/o2rb_perf.txt"); ///
    std::string filepath_time = "/home/misoyuri/Desktop/Data/Test_Log/outdoor_large_loop/o2rb/LFOV_time.txt";
    std::string trajSavingPath("/home/misoyuri/Desktop/Data/Test_Log/outdoor_large_loop/o2rb/o2rb_KeyFrameTrajectory.txt");
    #endif

    // Retrieve paths to images
    std::vector<std::string> fisheyeImgNames;
    std::vector<double> vTimestamps;
    std::string line;
    while (std::getline(fin, line))
    {
        std::stringstream ss(line);
        double ts;
        ss >> ts;
        std::string name;
        ss >> name;
        size_t p = name.find_last_of("/");
        name = name.substr(p + 1, name.length());

        vTimestamps.push_back(ts);
        fisheyeImgNames.push_back(name);
    }

    Mat mask = imread("/home/misoyuri/Desktop/Data/Squence/Lafida/lafida_mask_fish.png", IMREAD_GRAYSCALE);
    const int imageCnt = fisheyeImgNames.size();
    std::cout << "find " << imageCnt << " images" << std::endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(imageCnt);

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
    cv::Mat im;

    unsigned int avg_cnt = 0;
    unsigned int avg_cntP = 0;
    unsigned int avg_hd = 0;
    unsigned int avg_DN = 0;

    int ni = 0;
    while (true) {
        /* time */
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        avg_cnt++;
        //if (ni == 200)
        //     break;

        std::string fisheyeImgName = fisheyeImgPath + fisheyeImgNames[ni];
        // Read image from file
        im = cv::imread(fisheyeImgName, CV_LOAD_IMAGE_UNCHANGED);

        if (im.empty())
        {
            cerr << endl
                 << "Failed to read camera" << endl;
            break;
        }

        if (ni == fisheyeImgNames.size())
        {
            cout << "frame end" << endl;
            break;
        }
        

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, mask, vTimestamps[ni], LUT);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        vTimesTrack[ni] = ttrack;

        avg_cntP += goodmatch;
        avg_hd += HD;
        avg_DN += DN;

        std::cout << std::setprecision(8);
        // cout << "total time:: " << res1 << endl;
        // cout << "res_time::     " << res_time << endl;

        ni++;

        writeFile.precision(7);
        writeFile.setf(ios::fixed);
        // writeFile << std::setprecision(12);
        writeFile << res_time << endl;
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

    std::ofstream f;

    f.open(perfSavingPath.c_str());
    f << fixed;
    f << "-------" << endl
      << endl;
    f << "median tracking time: " << vTimesTrack[imageCnt / 2] << endl;
    f << "mean tracking time: " << totaltime / imageCnt << endl;
    f << "tracking frames/ total frames: " << frame_counter << "/ " << imageCnt << " " << static_cast<float>(frame_counter) / imageCnt << std::endl;
    f << "lost frames/ total frames: " << frame_lost_counter << "/ " << imageCnt << " " << static_cast<float>(frame_lost_counter) / imageCnt << std::endl;
    f << "track started frame: " << frame_track_started << std::endl;
    
    f.close();

    // Save camera trajectory
    if (trajSavingPath.empty())
        trajSavingPath = std::string("KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM(trajSavingPath);

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