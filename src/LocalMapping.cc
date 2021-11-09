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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>
#include <unistd.h>

// #define loc_test

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    // mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{
    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Edit 2021.11.03. SeokUn ( Time Check )
            // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // BoW conversion and insertion in Map  ///KeyFrame Insertion
            ProcessNewKeyFrame(); ///

            #ifdef loc_test
            cout << "Before vector<MapPoint*>:: " << mpMap->GetAllMapPoints().size() << endl;
            #endif
            // Check recent MapPoints   ///Recent Map Points Culling
            MapPointCulling();///
            #ifdef loc_test
            cout << "After vector<MapPoint*>:: " << mpMap->GetAllMapPoints().size() << endl << endl;
            #endif

            #ifdef loc_test
            cout << "before CreateNewMapPoints: " << mpMap->GetAllMapPoints().size() << endl;
            #endif
            // Triangulate new MapPoints    ///New Map Point Creation
            CreateNewMapPoints();
            #ifdef loc_test
            cout << "after CreateNewMapPoints: " << mpMap->GetAllMapPoints().size() << endl << endl;
            #endif

            if(!CheckNewKeyFrames())
            {
                #ifdef loc_test
                cout << "before SearchInNeighbors: " << mpMap->GetAllMapPoints().size() << endl;
                #endif
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors(); ///
                #ifdef loc_test
                cout << "after SearchInNeighbors: " << mpMap->GetAllMapPoints().size() << endl << endl;
                #endif
            }

            mbAbortBA = false;

            // cout << "mpMap->KeyFramesInMap():: " << mpMap->KeyFramesInMap()<< endl;
            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                if(mpMap->KeyFramesInMap()>2)
                {
                    #ifdef loc_test
                    cout << "LM: Local BA Start" << mpMap->GetAllMapPoints().size()<< endl;
                    #endif
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);
                    #ifdef loc_test
                    cout << "LM: Local BA End" << mpMap->GetAllMapPoints().size()<< endl <<endl;
                    #endif
                }

                // Check redundant local Keyframes  ///local Keyframe culling
                // KeyFrameCulling();///
            }

            // mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

            // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            // double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

            // FILE * fp_ = fopen("/home/misoyuri/Desktop/Time_Check/O2RB_Mapping.csv", "a");
            // fprintf(fp_, "%lu, %lf\n", mpCurrentKeyFrame->mnId, ttrack);
            // fclose(fp_);

        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling()
{
    // cout << mlpRecentAddedMapPoints.size() << endl;
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs = 2;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

/* sphere */
int LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return nnew;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);


        const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
        const float ratioBaselineDepth = baseline/medianDepthKF2;

        if(ratioBaselineDepth<0.01)
            continue;

        // Compute Fundamental Matrix
        // cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);
        cv::Mat E12 = ComputeE21_sh(mpCurrentKeyFrame,pKF2);/* sphere */

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;


         // Edit 2021.11.03. SeokUn ( Time Check )

        // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        
        // matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false); 
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,E12,vMatchedIndices,false); /* sphere */

        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        // FILE * fp_ = fopen("/home/misoyuri/Desktop/Time_Check/O2RB_traiangle.csv", "a");
        // fprintf(fp_, "%lu, %lf\n", mpCurrentKeyFrame->mnId, ttrack);
        // fclose(fp_);

        ///

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));


        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;


            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeys[idx1];
            const cv::KeyPoint &kp2 = pKF2->mvKeys[idx2];
            const cv::Point3f &kp1_sh = mpCurrentKeyFrame->mvKeysSp[idx1];
            const cv::Point3f &kp2_sh = pKF2->mvKeysSp[idx2];


            // Check parallax between rays
            cv::Mat xn1_sh = (cv::Mat_<float>(3,1) << kp1_sh.x, kp1_sh.y, kp1_sh.z);
            cv::Mat xn2_sh = (cv::Mat_<float>(3,1) << kp2_sh.x, kp2_sh.y, kp2_sh.z);

            cv::Mat ray1 = Rwc1*xn1_sh;
            cv::Mat ray2 = Rwc2*xn2_sh;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));
            float cosParallaxStereo = cosParallaxRays+1;
            /* sphere */

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && cosParallaxRays<0.9998)
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = kp1_sh.x*(Tcw1.row(1)+Tcw1.row(2)) - (kp1_sh.y+kp1_sh.z)*Tcw1.row(0);
                A.row(1) = kp1_sh.y*(Tcw1.row(0)+Tcw1.row(2)) - (kp1_sh.x+kp1_sh.z)*Tcw1.row(1);
                A.row(2) = kp2_sh.x*(Tcw2.row(1)+Tcw2.row(2)) - (kp2_sh.y+kp2_sh.z)*Tcw2.row(0);
                A.row(3) = kp2_sh.y*(Tcw2.row(0)+Tcw2.row(2)) - (kp2_sh.x+kp2_sh.z)*Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();
            // cout << "x3D:: " << x3D << endl;


            /* sphere Check triangulation in front of cameras */
            cv::Mat x3Dc1 = Rcw1*x3D+tcw1;
            float d1 = x3Dc1.at<float>(2) / cv::norm(x3Dc1);
            if(d1<=CamModelGeneral::GetCamera()->GetCosFovTh())
                continue;

            cv::Mat x3Dc2 = Rcw2*x3D+tcw2;
            float d2 = x3Dc2.at<float>(2) / cv::norm(x3Dc2);
            if(d2<=CamModelGeneral::GetCamera()->GetCosFovTh())
                continue;
            /* sphere */


            /* sphere reprojection error in first keyframe */
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1_sh = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1_sh = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float z1_sh = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            const float norm_xyz1_sh = sqrt( x1_sh*x1_sh + y1_sh*y1_sh + z1_sh*z1_sh );

            float u1_sh = x1_sh / norm_xyz1_sh *CamModelGeneral::GetCamera()->radius;
            float v1_sh = y1_sh / norm_xyz1_sh *CamModelGeneral::GetCamera()->radius;
            float w1_sh = z1_sh / norm_xyz1_sh *CamModelGeneral::GetCamera()->radius;

            cv::Vec2f uv1_isin_;
            CamModelGeneral::GetCamera()->WorldToImg(cv::Vec3d(u1_sh,v1_sh,w1_sh), uv1_isin_);
                
            float errX1 = uv1_isin_(0) - kp1.pt.x;
            float errY1 = uv1_isin_(1) - kp1.pt.y;
            if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                continue;


            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2_sh = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2_sh = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float z2_sh = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            const float norm_xyz2_sh = sqrt( x2_sh*x2_sh + y2_sh*y2_sh + z2_sh*z2_sh );

            float u2_sh = x2_sh / norm_xyz2_sh *CamModelGeneral::GetCamera()->radius;
            float v2_sh = y2_sh / norm_xyz2_sh *CamModelGeneral::GetCamera()->radius;
            float w2_sh = z2_sh / norm_xyz2_sh *CamModelGeneral::GetCamera()->radius;

            cv::Vec2f uv2_isin_;
            CamModelGeneral::GetCamera()->WorldToImg(cv::Vec3d(u2_sh,v2_sh,w2_sh), uv2_isin_);
                
            float errX2 = uv2_isin_(0) - kp2.pt.x;
            float errY2 = uv2_isin_(1) - kp2.pt.y;
            if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                continue;
            /* sphere */


            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;


            /* track on off */
            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }

    }
    return nnew;
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        // cout << "Fuse target start" << endl;
        matcher.Fuse(pKFi,vpMapPointMatches);/* sphere */
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

/* sphere model */
cv::Mat LocalMapping::ComputeE21_sh(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    // const cv::Mat &K1 = pKF1->mK;
    // const cv::Mat &K2 = pKF2->mK;


    return t12x*R12;//E = [t]xR
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeys[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeys[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
