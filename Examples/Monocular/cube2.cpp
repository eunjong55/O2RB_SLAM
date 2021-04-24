// #include <iostream>
// #include <algorithm>
// #include <fstream>
// #include <chrono>
// #include <iomanip>
// #include <cassert>
// #include <string>
// #include <vector>
// #include <time.h>

// #include <opencv2/core/core.hpp>

// #include "System.h"

// using namespace std;
// using namespace cv;
// unsigned int frame_counter;
// unsigned int HD;
// unsigned int goodmatch;
// unsigned int DN;
// unsigned int MapN;
// extern float res_time;

// void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

// int main(int argc, char **argv)
// {
//     frame_counter = 0;
//     string path_to_vocabulary = "/home/misoyuri/cubeSlam/CubemapSLAM/Vocabulary/ORBvoc.txt";
//     string path_to_settings = "../../Config/lafida_cam0_params.yaml";

//     // std::string fisheyeImgPath = "/home/oooony/바탕화면/dataset/outdoor_rotation/imgs/cam0/";
//     // std::ifstream fin("/home/oooony/바탕화면/dataset/outdoor_rotation/images_and_timestamps.txt");

//     ///indoor_static
//     ///indoor_dynamic
//     ///outdoor_static
//     // std::string fisheyeImgPath = "/home/misoyuri/Desktop/Data/indoor_dynamic/imgs/cam0/";      ///
//     // std::ifstream fin("/home/misoyuri/Desktop/Data/indoor_dynamic/images_and_timestamps.txt"); ///
//     std::string fisheyeImgPath = "/home/misoyuri/Desktop/Data/Cube_loop2_front/";
//     std::ifstream fin("/home/misoyuri/Desktop/Data/Cube_loop2_front/front_images.lst");
//     std::string perfSavingPath("/home/misoyuri/Desktop/Data/indoor_dynamic/data_fish.txt");        ///

//     //  /home/oooony/바탕화면/dataset/sequence_11

//     // Retrieve paths to images
//     std::vector<std::string> fisheyeImgNames;
//     std::vector<double> vTimestamps;
//     std::string line;
//     while (std::getline(fin, line))
//     {
//         std::stringstream ss(line);
//         double ts;
//         ss >> ts;
//         std::string name;
//         ss >> name;
//         size_t p = name.find_last_of("/");
//         name = name.substr(p + 1, name.length());

//         vTimestamps.push_back(ts);
//         fisheyeImgNames.push_back(name);
//     }

//     Mat mask = imread("/home/misoyuri/Desktop/Data/HGU_fish/lafida_mask.png ", IMREAD_GRAYSCALE);
//     const int imageCnt = fisheyeImgNames.size();
//     std::cout << "find " << imageCnt << " images" << std::endl;

//     // Vector for tracking time statistics
//     vector<float> vTimesTrack;
//     vTimesTrack.resize(imageCnt);

//     string filepath_time = "/home/misoyuri/Desktop/Data/indoor_dynamic/LFOV_time.txt";
//     ofstream writeFile(filepath_time.data());

//     // Create SLAM system. It initializes all system threads and gets ready to process frames.
//     ORB_SLAM2::System SLAM(path_to_vocabulary, path_to_settings, ORB_SLAM2::System::MONOCULAR, true);

//     std::vector<cv::Mat> LUT;
//     cv::Mat LUT_x = SLAM.mMap1;
//     cv::Mat LUT_y = SLAM.mMap2;
//     cv::Mat LUT_x_inv = SLAM.mMap1_inv;
//     cv::Mat LUT_y_inv = SLAM.mMap2_inv;
//     LUT.push_back(LUT_x);
//     LUT.push_back(LUT_y);
//     LUT.push_back(LUT_x_inv);
//     LUT.push_back(LUT_y_inv);

//     // Main loop
//     cv::Mat im;

//     unsigned int avg_cnt = 0;
//     unsigned int avg_cntP = 0;
//     unsigned int avg_hd = 0;
//     unsigned int avg_DN = 0;

//     int ni = 0;
//     while (true)
//     {
//         avg_cnt++;
//         //if (ni == 200)
//         //     break;

//         std::string fisheyeImgName = fisheyeImgPath + fisheyeImgNames[ni];
//         // Read image from file
//         im = cv::imread(fisheyeImgName, CV_LOAD_IMAGE_UNCHANGED);

//         if (im.empty())
//         {
//             cerr << endl
//                  << "Failed to read camera" << endl;
//             break;
//         }
//         if (ni == fisheyeImgNames.size())
//         {
//             cout << "frame end" << endl;
//             break;
//         }
        

//         /* time */
//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//         // Pass the image to the SLAM system
//         SLAM.TrackMonocular(im, mask, vTimestamps[ni], LUT);
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//         double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
//         vTimesTrack[ni] = ttrack;

//         avg_cntP += goodmatch;
//         avg_hd += HD;
//         avg_DN += DN;

//         std::cout << std::setprecision(8);
//         // cout << "total time:: " << res1 << endl;
//         // cout << "res_time::     " << res_time << endl;

//         ni++;

//         writeFile.precision(7);
//         writeFile.setf(ios::fixed);
//         // writeFile << std::setprecision(12);
//         writeFile << res_time << endl;
//     }
//     writeFile.close();

//     avg_cntP /= avg_cnt;
//     avg_hd /= avg_cnt;
//     avg_DN /= avg_cnt;
//     cout << "  HD,mathces :: " << avg_hd << " , " << avg_cntP << " , " << avg_DN << endl;

//     // Stop all threads
//     SLAM.Shutdown();

//     // Tracking time statistics
//     sort(vTimesTrack.begin(), vTimesTrack.end());
//     float totaltime = 0;
//     for (int idx = 0; idx < imageCnt; idx++)
//     {
//         totaltime += vTimesTrack[idx];
//     }
//     cout << "-------all" << endl
//          << endl;
//     cout << "median tracking time: " << vTimesTrack[imageCnt / 2] << endl;
//     cout << "mean tracking time: " << totaltime / imageCnt << endl;
//     std::cout << "find " << imageCnt << " images" << std::endl;
//     std::cout << "image in " << fisheyeImgPath << std::endl;

//     std::ofstream f;

//     f.open(perfSavingPath.c_str());
//     f << fixed;
//     f << "-------" << endl
//       << endl;
//     f << "median tracking time: " << vTimesTrack[imageCnt / 2] << endl;
//     f << "mean tracking time: " << totaltime / imageCnt << endl;
//     f << "tracking frames/ total frames: " << frame_counter << "/ " << imageCnt << " " << static_cast<float>(frame_counter) / imageCnt << std::endl;
//     f.close();

//     // Save camera trajectory
//     std::string trajSavingPath("/home/misoyuri/Desktop/Data/indoor_dynamic/lafida_KeyFrameTrajectory.txt");
//     if (trajSavingPath.empty())
//         trajSavingPath = std::string("KeyFrameTrajectory.txt");
//     SLAM.SaveKeyFrameTrajectoryTUM(trajSavingPath);

//     return 0;
// }

// void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
// {
//     ifstream f;
//     f.open(strFile.c_str());
//     int ts = 0;
//     while (!f.eof())
//     {
//         string s;
//         getline(f, s);
//         if (!s.empty())
//         {
//             stringstream ss;
//             ss << s;
//             string sRGB;
//             vTimestamps.push_back(ts);
//             ts++;
//             ss >> sRGB;
//             vstrImageFilenames.push_back(sRGB);
//         }
//     }
// }


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <sys/time.h>

#include "System.h"
#define NODEBUG

using namespace std;
using namespace cv;

using namespace std;
using namespace cv;
unsigned int HD;
unsigned int goodmatch;
unsigned int DN;
unsigned int MapN;
unsigned int frame_counter;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());
    int ts = 0;
    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
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

int main(int argc, char **argv)
{
    //read in params
    std::string vocabularyFilePath("/home/cgv/Desktop/github/O2RB_SLAM/Vocabulary/ORBvoc.txt");
    std::string settingFilePath("/home/cgv/Desktop/github/CubemapSLAM/Config/front_cam_params.yaml");
    ORB_SLAM2::System SLAM(vocabularyFilePath, settingFilePath, ORB_SLAM2::System::MONOCULAR, true);

    
    //get image list
    // std::string fisheyeImgPath("/home/misoyuri/Downloads/parkinglot_front/");
    std::string fisheyeImgPath("/home/cgv/Desktop/loop2_front/");
    if(fisheyeImgPath[fisheyeImgPath.length()-1] != '/')
        fisheyeImgPath += "/";
    // std::string fisheyeImgListPath("/home/misoyuri/Downloads/parkinglot_front/front_images.lst");
    std::string fisheyeImgListPath("/home/cgv/Desktop/loop2_front/front_images.lst");



    std::string fishMaskPath("/home/cgv/Desktop/Cube_loop2_front/loop2_front_mask.png");
    // std::string trajSavingPath("/home/misoyuri/Desktop/Data/KeyFrameTrajectory_Using_LUT_1.txt");
    std::string trajSavingPath("");

    cv::Mat fishMask = cv::imread(fishMaskPath.c_str(), cv::IMREAD_GRAYSCALE);
    // cv::Mat cubeMask = imread("/home/misoyuri/Desktop/Data/Vadas/vadas_gray_cubemap_front_mask_650.png", IMREAD_GRAYSCALE);

    //when image mask is not needed, a grayscale image with all pixel as 255 will be created
    if(fishMaskPath.empty())
    {
        std::cout << "fail to read the mask: " << fishMaskPath << std::endl;
        //exit(EXIT_FAILURE);
    }

    std::ifstream fin(fisheyeImgListPath);
    std::vector<std::string> fisheyeImgNames;
    std::vector<double> vTimestamps;
    std::string line;
    while(std::getline(fin, line))
    {
        fisheyeImgNames.push_back(line); 
        size_t p = line.find_last_of("_");
        std::stringstream ss(line.substr(0, p));
        double ts;
        ss >> ts;
        vTimestamps.push_back(ts);
    }

    int offset = 0;
    int width = CamModelGeneral::GetCamera()->GetCubeFaceWidth(), height = CamModelGeneral::GetCamera()->GetCubeFaceHeight();
    cv::Mat cubemapImg(height * 3, width * 3, CV_8U, cv::Scalar::all(0));
    cv::Mat cubemapImg_front = cubemapImg.rowRange(height, 2 * height).colRange(width, 2 * width);
    cv::Mat cubemapImg_left = cubemapImg.rowRange(height, 2 * height).colRange(0+offset, width+offset);
    cv::Mat cubemapImg_right = cubemapImg.rowRange(height, 2 * height).colRange(2 * width-offset, 3 * width-offset);
    cv::Mat cubemapImg_upper = cubemapImg.rowRange(0+offset, height+offset).colRange(width, 2 * width);
    cv::Mat cubemapImg_lower = cubemapImg.rowRange(2 * height-offset, 3 * height-offset).colRange(width, 2 * width);
    
    const int imageCnt = fisheyeImgNames.size();
    std::cout << "find " << imageCnt << " images" << std::endl;

    // Vector for tracking time statistics
    // vector<float> vTimesTrack;
    // vTimesTrack.resize(imageCnt);

    std::vector<cv::Mat> origImages;

    /////////////
    std::vector<cv::Mat> LUT;
    cv::Mat LUT_x = SLAM.mMap1;
    cv::Mat LUT_y = SLAM.mMap2;
    cv::Mat LUT_x_inv = SLAM.mMap1_inv;
    cv::Mat LUT_y_inv = SLAM.mMap2_inv;
    LUT.push_back(LUT_x);
    LUT.push_back(LUT_y);
    LUT.push_back(LUT_x_inv);
    LUT.push_back(LUT_y_inv);
    /////////////



    int nStep = 1;
    for(int idx = 0; idx < imageCnt; ++idx)
    {
        if(idx % nStep != 0)
            continue;
        
        struct timeval start = {};
        gettimeofday(&start, NULL);

        //read in images
        std::string fisheyeImgName = fisheyeImgPath + fisheyeImgNames[idx];
        cv::Mat fisheyeImg = cv::imread(fisheyeImgName.c_str(), cv::IMREAD_GRAYSCALE);
        // cv::Mat mmask = cv::Mat::ones(fisheyeImg.size(), 0);
        // std::cout << "reading image " << fisheyeImgNames[idx] << std::endl;
        if(!fisheyeImg.data)
        {
            std::cout << "fail to read image in " << fisheyeImgName << std::endl;
            exit(EXIT_FAILURE);
        }
        SLAM.TrackMonocular(fisheyeImg, fishMask, idx, LUT);
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        //track cubemap

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // vTimesTrack[idx]=ttrack;
        
        struct timeval end = {};
        gettimeofday(&end, NULL);
        
        double time = end.tv_sec * 1000.0 + end.tv_usec / 1000.0 - start.tv_sec * 1000.0 - start.tv_usec / 1000.0;


        // FILE * fp_ = fopen("/home/misoyuri/Desktop/Slam_log/time_O2RB_SprayMask02.csv", "a");
        // fprintf(fp_, "%d, %lf\n", idx, time); // saving time as a msec.
        // fclose(fp_);
    }
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    // sort(vTimesTrack.begin(),vTimesTrack.end());
    // float totaltime = 0;
    // for(int idx=0; idx<imageCnt; idx++)
    // {
    //     totaltime+=vTimesTrack[idx];
    // }
    // cout << "-------" << endl << endl;
    // cout << "median tracking time: " << vTimesTrack[imageCnt/2] << endl;
    // cout << "mean tracking time: " << totaltime/imageCnt << endl;

    // Save camera trajectory

    //SLAM.SaveKeyFrameTrajectoryTUM("/home/misoyuri/Desktop/Slam_log/KeyFrameTrajectory_O2RB_SprayMask02.txt");
}
