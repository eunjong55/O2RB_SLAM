/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //read in params
    int nrpol = fsSettings["Camera.nrpol"];
    int nrinvpol = fsSettings["Camera.nrinvpol"];

    cv::Mat_<double> poly = cv::Mat::zeros(5, 1, CV_64F);
    for (int i = 0; i < nrpol; ++i)
        poly.at<double>(i, 0) = fsSettings["Camera.a" + std::to_string(i)];
    cv::Mat_<double> invpoly = cv::Mat::zeros(12, 1, CV_64F);
    for (int i = 0; i < nrinvpol; ++i)
        invpoly.at<double>(i, 0) = fsSettings["Camera.pol" + std::to_string(i)];

    int Iw = (int)fsSettings["Camera.Iw"];
    int Ih = (int)fsSettings["Camera.Ih"];

    double cdeu0v0[5] = {fsSettings["Camera.c"], fsSettings["Camera.d"], fsSettings["Camera.e"],
                         fsSettings["Camera.u0"], fsSettings["Camera.v0"]};

    //cubemap params
    int nFaceH = fsSettings["CubeFace.h"];
    int nFaceW = fsSettings["CubeFace.w"];
    double fx = static_cast<double>(nFaceW) / 2, fy = static_cast<double>(nFaceH) / 2;
    double cx = static_cast<double>(nFaceW) / 2, cy = static_cast<double>(nFaceH) / 2;

    double camFov = fsSettings["Camera.fov"];

    //Set camera model
    // CamModelGeneral::GetCamera()->SetCamParams();
    CamModelGeneral::GetCamera()->SetCamParams(cdeu0v0, poly, invpoly, Iw, Ih, fx, fy, cx, cy, nFaceW, nFaceH, camFov);
    std::cout << "finish creating general camera model" << std::endl;

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    //Create mapping from cubemap to fisheye
    CreateUndistortRectifyMap();
    CreateUndistortRectifyMap_inv();

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;


    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    // //Initialize the Loop Closing thread and launch
    // mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    // mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    // mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    // mpLocalMapper->SetLoopCloser(mpLoopCloser);

    // mpLoopCloser->SetTracker(mpTracker);
    // mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

//convert fisheye image to cubemap
void System::CvtFisheyeToCubeMap_reverseQuery_withInterpolation(cv::Mat &cubemapImg, const cv::Mat &fisheyeImg, 
                int interpolation, int borderType, const cv::Scalar& borderValue)
{
    /* time */
    const int offset = 0;
    const int width = CamModelGeneral::GetCamera()->GetCubeFaceWidth(), height = CamModelGeneral::GetCamera()->GetCubeFaceHeight();

    //front
    cv::Mat cubemapImg_front = cubemapImg.rowRange(height, 2 * height).colRange(width, 2 * width);
    cv::Mat mMap1_front = mMap1.rowRange(height, 2 * height).colRange(width, 2 * width);
    cv::Mat mMap2_front = mMap2.rowRange(height, 2 * height).colRange(width, 2 * width);
    cv::remap(fisheyeImg, cubemapImg_front, mMap1_front, mMap2_front, interpolation, borderType, borderValue);

    //left
    cv::Mat cubemapImg_left = cubemapImg.rowRange(height, 2 * height).colRange(0+offset, width+offset);
    cv::Mat mMap1_left = mMap1.rowRange(height, 2 * height).colRange(0+offset, width+offset);
    cv::Mat mMap2_left = mMap2.rowRange(height, 2 * height).colRange(0+offset, width+offset);
    cv::remap(fisheyeImg, cubemapImg_left, mMap1_left, mMap2_left, interpolation, borderType, borderValue);

    //right
    cv::Mat cubemapImg_right = cubemapImg.rowRange(height, 2 * height).colRange(2 * width-offset, 3 * width-offset);
    cv::Mat mMap1_right = mMap1.rowRange(height, 2 * height).colRange(2 * width-offset, 3 * width-offset);
    cv::Mat mMap2_right = mMap2.rowRange(height, 2 * height).colRange(2 * width-offset, 3 * width-offset);
    cv::remap(fisheyeImg, cubemapImg_right, mMap1_right, mMap2_right, interpolation, borderType, borderValue);

    //upper
    cv::Mat cubemapImg_upper = cubemapImg.rowRange(0+offset, height+offset).colRange(width, 2 * width);
    cv::Mat mMap1_upper = mMap1.rowRange(0+offset, height+offset).colRange(width, 2 * width);
    cv::Mat mMap2_upper = mMap2.rowRange(0+offset, height+offset).colRange(width, 2 * width);
    cv::remap(fisheyeImg, cubemapImg_upper, mMap1_upper, mMap2_upper, interpolation, borderType, borderValue);

    //lower
    cv::Mat cubemapImg_lower = cubemapImg.rowRange(2 * height-offset, 3 * height-offset).colRange(width, 2 * width);
    cv::Mat mMap1_lower = mMap1.rowRange(2 * height-offset, 3 * height-offset).colRange(width, 2 * width);
    cv::Mat mMap2_lower = mMap2.rowRange(2 * height-offset, 3 * height-offset).colRange(width, 2 * width);
    cv::remap(fisheyeImg, cubemapImg_lower, mMap1_lower, mMap2_lower, interpolation, borderType, borderValue);

    //interpolation with cv::remap for each face
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if (mSensor != MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}


// Proccess the given monocular frame with mask 
cv::Mat System::TrackMonocular(const cv::Mat &im, const cv::Mat &mask, const double &timestamp, std::vector<cv::Mat> LUT)
{

    if (mSensor != MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }
    
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im, mask, timestamp, LUT);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    
    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    // mpLoopCloser->RequestFinish();

    mpViewer->OutputTrackingSummary();
    
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() /*|| !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA()*/)
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved! lafida!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

	void System::CreateUndistortRectifyMap()
	{
		int width3 = CamModelGeneral::GetCamera()->GetCubeFaceWidth() * 3, height3 = CamModelGeneral::GetCamera()->GetCubeFaceHeight() * 3;
		//create map for u(mMap1) and v(mMap2)
		mMap1.create(height3, width3, CV_32F);
		mMap2.create(height3, width3, CV_32F);
		mMap1.setTo(cv::Scalar::all(0));
		mMap2.setTo(cv::Scalar::all(0));

		int Iw = CamModelGeneral::GetCamera()->GetFisheyeWidth(), Ih = CamModelGeneral::GetCamera()->GetFisheyeHeight();
		for (int y = 0; y < height3; ++y)
		{
			for (int x = 0; x < width3; ++x)
			{
				double u, v;
				CamModelGeneral::GetCamera()->CubemapToFisheye(u, v, static_cast<double>(x), static_cast<double>(y));
				// on some face but doesn't map to a fisheye valid region
				if (u < 0 || v < 0 || u >= Iw || v >= Ih)
					continue;
				mMap1.at<float>(y, x) = static_cast<float>(u);
				mMap2.at<float>(y, x) = static_cast<float>(v);
				// cout << "cube: " << x << " , " << y << " fish: " << u << " , " << v << endl;
			}
		}
	}

	void System::CreateUndistortRectifyMap_inv()
	{
		int width = CamModelGeneral::GetCamera()->GetFisheyeWidth(), height = CamModelGeneral::GetCamera()->GetFisheyeHeight();
		//create map for u(mMap1) and v(mMap2)
		mMap1_inv.create(height, width, CV_32F);
		mMap2_inv.create(height, width, CV_32F);
		mMap1_inv.setTo(cv::Scalar::all(0));
		mMap2_inv.setTo(cv::Scalar::all(0));

		int Cw = CamModelGeneral::GetCamera()->GetCubeFaceWidth()*3, Ch = CamModelGeneral::GetCamera()->GetCubeFaceHeight()*3;
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				float u, v;
				CamModelGeneral::GetCamera()->FisheyeToCubemap(static_cast<double>(x), static_cast<double>(y), u, v);
				// on some face but doesn't map to a fisheye valid region
				if (u < 0 || v < 0 || u >= Cw || v >= Ch)
					continue;
				mMap1_inv.at<float>(y, x) = static_cast<float>(u);
				mMap2_inv.at<float>(y, x) = static_cast<float>(v);
				// cout << "cube: " << x << " , " << y << " fish: " << u << " , " << v << endl;
			}
		}
	}

} //namespace ORB_SLAM
