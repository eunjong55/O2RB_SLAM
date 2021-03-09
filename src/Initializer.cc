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

#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"
#include "ORBmatcher.h"

#include <thread>

namespace ORB_SLAM2
{

Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
    mK = ReferenceFrame.mK.clone();

    mvKeys1 = ReferenceFrame.mvKeys;
    mvKeysSp1 = ReferenceFrame.mvKeysSp;

    mSigma = sigma;
    mSigma2 = sigma * sigma;
    mMaxIterations = iterations;
}

bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    mvKeys2 = CurrentFrame.mvKeys;
    mvKeysSp2 = CurrentFrame.mvKeysSp;

    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    mvbMatched1.resize(mvKeys1.size());
    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (vMatches12[i] >= 0)
        {
            mvMatches12.push_back(make_pair(i, vMatches12[i]));
            mvbMatched1[i] = true;
        }
        else
            mvbMatched1[i] = false;
    }

    const int N = mvMatches12.size();

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for (int i = 0; i < N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    mvSets = vector<vector<size_t>>(mMaxIterations, vector<size_t>(8, 0));

    DUtils::Random::SeedRandOnce(0);

    for (int it = 0; it < mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for (size_t j = 0; j < 8; j++)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    vector<bool> vbMatchesInliersE;
    cv::Mat E;
    float score;

    FindEssential(vbMatchesInliersE, score, E);

    // cout<<"score : "<<score<<endl;

    return ReconstructE(vbMatchesInliersE, E, mK, R21, t21, vP3D, vbTriangulated, 1.0, 50);
}

/* sphere */
// Find essential matrix from normalized sphere correspondece 
void Initializer::FindEssential(vector<bool> &vbMatchesInliers, float &score, cv::Mat &E21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size(); 

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N, false);

    // Iteration variables
    vector<cv::Point3f> vPn1i(8);
    vector<cv::Point3f> vPn2i(8);
    cv::Mat E;
    vector<bool> vbCurrentInliers(N, false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int it = 0; it < mMaxIterations; it++)
    {
        // Select a minimum sets
        for (int j = 0; j < 8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = mvKeysSp1[mvMatches12[idx].first];
            vPn2i[j] = mvKeysSp2[mvMatches12[idx].second];
        }

        cv::Mat E = ComputeE21(vPn1i, vPn2i);

        currentScore = CheckEssential_(E, vbCurrentInliers, mSigma);

        if (currentScore > score)
        {
            E21 = E.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

/* sphere */
// compute Essential matrix from normalized spherical coordinate 
cv::Mat Initializer::ComputeE21(const vector<cv::Point3f> &vP1, const vector<cv::Point3f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N, 9, CV_32F);

    for (int i = 0; i < N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float w1 = vP1[i].z;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;
        const float w2 = vP2[i].z;

        A.at<float>(i, 0) = u2 * u1;
        A.at<float>(i, 1) = u2 * v1;
        A.at<float>(i, 2) = u2 * w1;
        A.at<float>(i, 3) = v2 * u1;
        A.at<float>(i, 4) = v2 * v1;
        A.at<float>(i, 5) = v2 * w1;
        A.at<float>(i, 6) = u1 * w2;
        A.at<float>(i, 7) = v1 * w2;
        A.at<float>(i, 8) = w1 * w2;
    }

    cv::Mat u, w, vt;
    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);
    cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2) = 0;

    return u * cv::Mat::diag(w) * vt;
}


float Initializer::CheckEssential(const cv::Mat &E21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float e11 = E21.at<float>(0, 0);
    const float e12 = E21.at<float>(0, 1);
    const float e13 = E21.at<float>(0, 2);
    const float e21 = E21.at<float>(1, 0);
    const float e22 = E21.at<float>(1, 1);
    const float e23 = E21.at<float>(1, 2);
    const float e31 = E21.at<float>(2, 0);
    const float e32 = E21.at<float>(2, 1);
    const float e33 = E21.at<float>(2, 2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 3.841;
    const float thScore = 5.991;

    for (int i = 0; i < N; i++)
    {
        bool bIn = true;

        const float u1 = mvKeysSp1[mvMatches12[i].first].x;
        const float v1 = mvKeysSp1[mvMatches12[i].first].y;
        const float w1 = mvKeysSp1[mvMatches12[i].first].z;

        const float u2 = mvKeysSp2[mvMatches12[i].second].x;
        const float v2 = mvKeysSp2[mvMatches12[i].second].y;
        const float w2 = mvKeysSp2[mvMatches12[i].second].z;

        // Reprojection error in second image
        // l2=E21x1=(a2,b2,c2)

        const float a2 = e11 * u1 + e12 * v1 + e13 * w1;
        const float b2 = e21 * u1 + e22 * v1 + e23 * w1;
        const float c2 = e31 * u1 + e32 * v1 + e33 * w1;
        const float d2 = -(a2*u1+b2*v1+c2*w1);

        // const float num2 = a2*u2+b2*v2+c2*w2;
        const float num2 = a2*u2+b2*v2+c2*w2+d2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2+c2*c2);

        float unitVectorSigma = sigma * CamModelGeneral::GetCamera()->GetSigma(mvKeys2[mvMatches12[i].second]);
        float invSigmaSquare = 1.0f/(unitVectorSigma*unitVectorSigma);

        const float chiSquare1 = squareDist1 * invSigmaSquare;

        if (chiSquare1 > th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tE21=(a1,b1,c1)

        const float a1 = e11 * u2 + e21 * v2 + e31 * w2;
        const float b1 = e12 * u2 + e22 * v2 + e32 * w2;
        const float c1 = e13 * u2 + e23 * v2 + e33 * w2;
        const float d1 = -(a1*u2+b1*v2+c1*w2);

        // const float num1 = a1*u1+b1*v1+c1*w1;
        const float num1 = a1*u1+b1*v1+c1*w1+d1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1+c1*c1);

        unitVectorSigma = sigma * CamModelGeneral::GetCamera()->GetSigma(mvKeys2[mvMatches12[i].second]);
        invSigmaSquare = 1.0f/(unitVectorSigma*unitVectorSigma);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if (bIn)
        {
            vbMatchesInliers[i] = true;
        }
        else
            vbMatchesInliers[i] = false;
    }

    return score;
}

// : Moon
float Initializer::CheckEssential_(const cv::Mat &E21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float e11 = E21.at<float>(0, 0);
    const float e12 = E21.at<float>(0, 1);
    const float e13 = E21.at<float>(0, 2);
    const float e21 = E21.at<float>(1, 0);
    const float e22 = E21.at<float>(1, 1);
    const float e23 = E21.at<float>(1, 2);
    const float e31 = E21.at<float>(2, 0);
    const float e32 = E21.at<float>(2, 1);
    const float e33 = E21.at<float>(2, 2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 3.841;
    const float thScore = 5.991;

    for (int i = 0; i < N; i++)
    {
        bool bIn = true;

        const cv::Vec3f &kpRay1 = mvKeysSp1[mvMatches12[i].first];
        const cv::Vec3f &kpRay2 = mvKeysSp2[mvMatches12[i].second];

        const float x1 = kpRay1(0);
        const float y1 = kpRay1(1);
        const float z1 = kpRay1(2);

        const float x2 = kpRay2(0);
        const float y2 = kpRay2(1);
        const float z2 = kpRay2(2);

        const float a2 = e11*x1+e12*y1+e13*z1;
        const float b2 = e21*x1+e22*y1+e23*z1;
        const float c2 = e31*x1+e32*y1+e33*z1;

        const float num2 = a2*x2+b2*y2+c2*z2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2+c2*c2);
        float unitVectorSigma = sigma * CamModelGeneral::GetCamera()->GetSigma(mvKeys2[mvMatches12[i].second]);
        float invSigmaSquare = 1.0f/(unitVectorSigma*unitVectorSigma);

        const float chiSquare1 = squareDist1 * invSigmaSquare;


        if (chiSquare1 > th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tE21=(a1,b1,c1)
        const float a1 = e11*x2+e21*y2+e31*z2;
        const float b1 = e12*x2+e22*y2+e32*z2;
        const float c1 = e13*x2+e23*y2+e33*z2;

        const float num1 = a1*x1+b1*y1+c1*z1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1+c1*c1);

        unitVectorSigma = sigma * CamModelGeneral::GetCamera()->GetSigma(mvKeys2[mvMatches12[i].second]);
        invSigmaSquare = 1.0f/(unitVectorSigma*unitVectorSigma);

        const float chiSquare2 = squareDist2 * invSigmaSquare;

        if (chiSquare2 > th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if (bIn)
        {
            vbMatchesInliers[i] = true;
        }
        else
            vbMatchesInliers[i] = false;
    }

    return score;
}

// ReconstructE for fisheye 
bool Initializer::ReconstructE(vector<bool> &vbMatchesInliers, cv::Mat &E21, cv::Mat &K,
                               cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N = 0;
    for (size_t i = 0, iend = vbMatchesInliers.size(); i < iend; i++)
        if (vbMatchesInliers[i])
            N++;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21, R1, R2, t);

    cv::Mat t1 = t;
    cv::Mat t2 = -t;

    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1, vbTriangulated2, vbTriangulated3, vbTriangulated4;
    float parallax1, parallax2, parallax3, parallax4;

    double th = 4.0;

    int nGood1 = CheckRT_(R1, t1, mvKeys1,mvKeys2, mvKeysSp1, mvKeysSp2, mvMatches12, vbMatchesInliers, vP3D1, th, vbTriangulated1, parallax1);
    int nGood2 = CheckRT_(R2, t1, mvKeys1,mvKeys2, mvKeysSp1, mvKeysSp2, mvMatches12, vbMatchesInliers, vP3D2, th, vbTriangulated2, parallax2);
    int nGood3 = CheckRT_(R1, t2, mvKeys1,mvKeys2, mvKeysSp1, mvKeysSp2, mvMatches12, vbMatchesInliers, vP3D3, th, vbTriangulated3, parallax3);
    int nGood4 = CheckRT_(R2, t2, mvKeys1,mvKeys2, mvKeysSp1, mvKeysSp2, mvMatches12, vbMatchesInliers, vP3D4, th, vbTriangulated4, parallax4);

    // int nGood1 = CheckRT(R1, t1, mvKeysSp1, mvKeysSp2, mvMatches12, vbMatchesInliers, vP3D1, th, vbTriangulated1, parallax1);
    // int nGood2 = CheckRT(R2, t1, mvKeysSp1, mvKeysSp2, mvMatches12, vbMatchesInliers, vP3D2, th, vbTriangulated2, parallax2);
    // int nGood3 = CheckRT(R1, t2, mvKeysSp1, mvKeysSp2, mvMatches12, vbMatchesInliers, vP3D3, th, vbTriangulated3, parallax3);
    // int nGood4 = CheckRT(R2, t2, mvKeysSp1, mvKeysSp2, mvMatches12, vbMatchesInliers, vP3D4, th, vbTriangulated4, parallax4);

    // cout << "N : " << N << endl;
    // cout << "1 : " << nGood1 << endl;
    // cout << "2 : " << nGood2 << endl;
    // cout << "3 : " << nGood3 << endl;
    // cout << "4 : " << nGood4 << endl;
    // cout << endl;

    int maxGood = max(nGood1, max(nGood2, max(nGood3, nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(0.9 * N), minTriangulated);

    int nsimilar = 0;
    if (nGood1 > 0.7 * maxGood)
        nsimilar++;
    if (nGood2 > 0.7 * maxGood)
        nsimilar++;
    if (nGood3 > 0.7 * maxGood)
        nsimilar++;
    if (nGood4 > 0.7 * maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if (maxGood < nMinGood || nsimilar > 1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    if (maxGood == nGood1)
    {
        if (parallax1 > minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }
    else if (maxGood == nGood2)
    {
        if (parallax2 > minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }
    else if (maxGood == nGood3)
    {
        if (parallax3 > minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }
    else if (maxGood == nGood4)
    {
        if (parallax4 > minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }
    
    return false;
}

void Initializer::Triangulate(const cv::Point3f &z1, const cv::Point3f &z2, const cv::Mat &R, const cv::Mat &t, cv::Mat &x3D)
{
    // //Adapted vector-based triangulation method 
    // cv::Mat A(4,4,CV_32F);
    // const float &x1_ = z1.x, &y1_ = z1.y, &z1_ = z1.z;
    // const float &x2_ = z2.x, &y2_ = z2.y, &z2_ = z2.z;
    // A.row(0) = x1_*(R.row(1)+R.row(2)) - (y1_+z1_)*R.row(0);
    // A.row(1) = y1_*(R.row(0)+R.row(2)) - (x1_+z1_)*R.row(1);
    // A.row(2) = x2_*(t.row(1)+t.row(2)) - (y2_+z2_)*t.row(0);
    // A.row(3) = y2_*(t.row(0)+t.row(2)) - (x2_+z2_)*t.row(1);

    // cv::Mat u,w,vt;
    // cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    // x3D = vt.row(3).t();
    // x3D = x3D.rowRange(0,3)/x3D.at<float>(3);



    //cout << "triangulation test::::::::::::::::: " << endl;
    cv::Mat C = -1 * R.t() * t;

    cv::Mat kp1_sh_world = (cv::Mat_<float>(3,1) << z1.x, z1.y, z1.z);//cam2world
    cv::Mat kp1_sh_world_ow1 = kp1_sh_world;

    cv::Mat xn2_sh = (cv::Mat_<float>(3,1) << z2.x, z2.y, z2.z); //(cv::Mat_<float>(3,1) << kp2_sh.x / sh_norm * rad, kp2_sh.y / sh_norm * rad, kp2_sh.z / sh_norm * rad);
    cv::Mat kp2_sh_world = R.inv()*xn2_sh + C;
    cv::Mat kp2_sh_world_ow2 = kp2_sh_world - C;

    float norm_kp1_sh_w = sqrt(kp1_sh_world_ow1.at<float>(0)*kp1_sh_world_ow1.at<float>(0) + kp1_sh_world_ow1.at<float>(1)*kp1_sh_world_ow1.at<float>(1) + kp1_sh_world_ow1.at<float>(2)*kp1_sh_world_ow1.at<float>(2));
    float norm_kp2_sh_w = sqrt(kp2_sh_world_ow2.at<float>(0)*kp2_sh_world_ow2.at<float>(0) + kp2_sh_world_ow2.at<float>(1)*kp2_sh_world_ow2.at<float>(1) + kp2_sh_world_ow2.at<float>(2)*kp2_sh_world_ow2.at<float>(2));

    cv::Mat z1_temp = cv::Mat::zeros(cv::Size(1, 3), CV_32F);
    z1_temp.at<float>(0, 0) = kp1_sh_world_ow1.at<float>(0) / norm_kp1_sh_w;
    z1_temp.at<float>(1, 0) = kp1_sh_world_ow1.at<float>(1) / norm_kp1_sh_w;
    z1_temp.at<float>(2, 0) = kp1_sh_world_ow1.at<float>(2) / norm_kp1_sh_w;
    //cout << "z1_temp = " << endl << " " << z1_temp << endl << endl;

    cv::Mat temp_1 = z1_temp;
    //cout << "temp = " << endl << " " << temp << endl << endl;

    cv::Mat z2_temp = cv::Mat::zeros(cv::Size(1, 3), CV_32F);
    z2_temp.at<float>(0, 0) = kp2_sh_world_ow2.at<float>(0) / norm_kp2_sh_w;
    z2_temp.at<float>(1, 0) = kp2_sh_world_ow2.at<float>(1) / norm_kp2_sh_w;
    z2_temp.at<float>(2, 0) = kp2_sh_world_ow2.at<float>(2) / norm_kp2_sh_w;
    //cout << "z2_temp = " << endl << " " << z2_temp << endl << endl;

    cv::Mat temp_2 = -z2_temp;
    //cout << "temp = " << endl << " " << temp << endl << endl;

    cv::Mat A = cv::Mat::zeros(cv::Size(2, 3), CV_32F);
    A.at<float>(0, 0) = temp_1.at<float>(0, 0);
    A.at<float>(1, 0) = temp_1.at<float>(1, 0);
    A.at<float>(2, 0) = temp_1.at<float>(2, 0);
    A.at<float>(0, 1) = temp_2.at<float>(0, 0);
    A.at<float>(1, 1) = temp_2.at<float>(1, 0);
    A.at<float>(2, 1) = temp_2.at<float>(2, 0);
    //cout << "A = " << endl << " " << A << endl << endl;

    cv::Mat A_pinv;
    cv::invert(A, A_pinv, cv::DECOMP_SVD);
    cv::Mat param = A_pinv * (C);
    x3D = ((param.at<float>(0, 0) * z1_temp) + (param.at<float>(1, 0) * z2_temp + C)) / 2;

}


void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for (int i = 0; i < N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX / N;
    meanY = meanY / N;

    float meanDevX = 0;
    float meanDevY = 0;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX / N;
    meanDevY = meanDevY / N;

    float sX = 1.0 / meanDevX;
    float sY = 1.0 / meanDevY;

    for (int i = 0; i < N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    T = cv::Mat::eye(3, 3, CV_32F);
    T.at<float>(0, 0) = sX;
    T.at<float>(1, 1) = sY;
    T.at<float>(0, 2) = -meanX * sX;
    T.at<float>(1, 2) = -meanY * sY;
}

// CheckRT for fisheye 
int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::Point3f> &vKeysSp1, const vector<cv::Point3f> &vKeysSp2,
                         const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                         vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    vbGood = vector<bool>(vKeysSp1.size(), false);

    vP3D.resize(vKeysSp1.size());
    vector<float> vCosParallax;
    vCosParallax.reserve(vKeysSp1.size());

    cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32F);
    cv::Mat O2 = -R.t() * t; // center  

    int nGood = 0;
    
    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (!vbMatchesInliers[i])
            continue;

        cv::Mat p3dC1;

        Triangulate(vKeysSp1[vMatches12[i].first], vKeysSp2[vMatches12[i].second], R, t, p3dC1);

        if (!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first] = false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2) / (dist1 * dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if((p3dC1.at<float>(2)/dist1)<=CamModelGeneral::GetCamera()->GetCosFovTh() && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R * p3dC1 + t;

        if((p3dC1.at<float>(2)/dist1)<=CamModelGeneral::GetCamera()->GetCosFovTh() && cosParallax<0.99998)
            continue;

        // Check reprojection error in first image
        cv::Mat rpj1 = p3dC1 / dist1 * CamModelGeneral::GetCamera()->radius;

        float squareError1 = (vKeysSp1[vMatches12[i].first].x - rpj1.at<float>(0, 0)) * (vKeysSp1[vMatches12[i].first].x - rpj1.at<float>(0, 0)) +
                             (vKeysSp1[vMatches12[i].first].y - rpj1.at<float>(1, 0)) * (vKeysSp1[vMatches12[i].first].y - rpj1.at<float>(1, 0)) +
                             (vKeysSp1[vMatches12[i].first].z - rpj1.at<float>(2, 0)) * (vKeysSp1[vMatches12[i].first].z - rpj1.at<float>(2, 0));

        float unitVectorSigma = CamModelGeneral::GetCamera()->GetSigma(mvKeys1[mvMatches12[i].first]);

        if (squareError1 > th2 * unitVectorSigma * unitVectorSigma)
            continue;

        // Check reprojection error in second image
        cv::Mat rpj2 = p3dC2 / dist2 * CamModelGeneral::GetCamera()->radius;
        float squareError2 = (vKeysSp2[vMatches12[i].second].x - rpj2.at<float>(0, 0)) * (vKeysSp2[vMatches12[i].second].x - rpj2.at<float>(0, 0)) +
                             (vKeysSp2[vMatches12[i].second].y - rpj2.at<float>(1, 0)) * (vKeysSp2[vMatches12[i].second].y - rpj2.at<float>(1, 0)) +
                             (vKeysSp2[vMatches12[i].second].z - rpj2.at<float>(2, 0)) * (vKeysSp2[vMatches12[i].second].z - rpj2.at<float>(2, 0));

        // cout << "repjoj err1 "  << squareError1 << endl;
        // cout << "repjoj err2 "  << squareError2 << endl;

        unitVectorSigma = CamModelGeneral::GetCamera()->GetSigma(mvKeys2[mvMatches12[i].second]);

        if (squareError2 > th2 * unitVectorSigma * unitVectorSigma)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0), p3dC1.at<float>(1), p3dC1.at<float>(2));
        nGood++;

        if (cosParallax < 0.99998)
            vbGood[vMatches12[i].first] = true;
    }

    if (nGood > 0)
    {
        sort(vCosParallax.begin(), vCosParallax.end());

        size_t idx = min(50, int(vCosParallax.size() - 1));
        parallax = acos(vCosParallax[idx]) * 180 / CV_PI;
    }
    else
        parallax = 0;

    return nGood;
}


int Initializer::CheckRT_(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                        const vector<cv::Point3f> &vKeysSp1, const vector<cv::Point3f> &vKeysSp2,
                         const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                         vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    vbGood = vector<bool>(vKeysSp1.size(), false);
    vP3D.resize(vKeysSp1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeysSp1.size());

    cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32F);
    cv::Mat O2 = -R.t() * t; // center  

    int nGood = 0;
    
    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (!vbMatchesInliers[i])
            continue;

        const cv::Vec3f &ray1 = vKeysSp1[vMatches12[i].first];
        const cv::Vec3f &ray2 = vKeysSp2[vMatches12[i].second];

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

        cv::Mat p3dC1;

        Triangulate(ray1, ray2, R, t, p3dC1);

        if (!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first] = false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2) / (dist1 * dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if((p3dC1.at<float>(2)/dist1)<=CamModelGeneral::GetCamera()->GetCosFovTh() && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R * p3dC1 + t;

        if((p3dC1.at<float>(2)/dist1)<=CamModelGeneral::GetCamera()->GetCosFovTh() && cosParallax<0.99998)
            continue;

        // Check reprojection error in first image
        cv::Vec2f ray1_img;
        CamModelGeneral::GetCamera()->WorldToImg(cv::Vec3d(p3dC1.at<float>(0), p3dC1.at<float>(1), p3dC1.at<float>(2)), ray1_img);

        float squareError1 = (ray1_img(0)-kp1.pt.x)*(ray1_img(0)-kp1.pt.x)+(ray1_img(1)-kp1.pt.y)*(ray1_img(1)-kp1.pt.y);
        float unitVectorSigma = CamModelGeneral::GetCamera()->GetSigma(mvKeys1[mvMatches12[i].first]);
        if (squareError1 > th2 * unitVectorSigma * unitVectorSigma)
            continue;



        // Check reprojection error in second image
        cv::Vec2f ray2_img;
        CamModelGeneral::GetCamera()->WorldToImg(cv::Vec3d(p3dC2.at<float>(0), p3dC2.at<float>(1), p3dC2.at<float>(2)), ray2_img);

        float squareError2 = (ray2_img(0)-kp2.pt.x)*(ray2_img(0)-kp2.pt.x)+(ray2_img(1)-kp2.pt.y)*(ray2_img(1)-kp2.pt.y);
        unitVectorSigma = CamModelGeneral::GetCamera()->GetSigma(mvKeys2[mvMatches12[i].second]);
        if (squareError2 > th2 * unitVectorSigma * unitVectorSigma)
            continue;



        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0), p3dC1.at<float>(1), p3dC1.at<float>(2));
        nGood++;

        if (cosParallax < 0.99998)
            vbGood[vMatches12[i].first] = true;
    }

    if (nGood > 0)
    {
        sort(vCosParallax.begin(), vCosParallax.end());

        size_t idx = min(50, int(vCosParallax.size() - 1));
        parallax = acos(vCosParallax[idx]) * 180 / CV_PI;
    }
    else
        parallax = 0;

    return nGood;
}

void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}

} // namespace ORB_SLAM2
