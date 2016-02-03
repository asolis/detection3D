/**************************************************************************************************
 **************************************************************************************************
 
     BSD 3-Clause License (https://www.tldrlegal.com/l/bsd3)
     
     Copyright (c) 2014 Andrés Solís Montero <http://www.solism.ca>, All rights reserved.
     
     
     Redistribution and use in source and binary forms, with or without modification,
     are permitted provided that the following conditions are met:
     
     1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.
     3. Neither the name of the copyright holder nor the names of its contributors
        may be used to endorse or promote products derived from this software
        without specific prior written permission.
     
     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
     AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
     IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
     LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
     THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
     OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
     OF THE POSSIBILITY OF SUCH DAMAGE.
 
 **************************************************************************************************
 **************************************************************************************************/

#ifndef detection3D_utils_h
#define detection3D_utils_h


#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "hest.h"

using namespace std;
using namespace cv;


class Prosac
{
public:
    
    static Mat findHomography(const vector< Point2f >& modelPoints,
                              const vector< Point2f >& rtPoints,
                              float            maxDist    = 3.0,
                              Mat             &inliers    = (Mat&)noArray(),
                              unsigned         maxIter    = 2000,
                              float            confidence = 0.995,
                              unsigned         minInliers = 4U)
    {
        Mat H;
        float distSq = maxDist*maxDist;
        float* src = (float*)&modelPoints[0];
        float* dst = (float*)&rtPoints[0];
        float Harr[3*4];
        
        if (inliers.empty())
            inliers = Mat(1, (int)modelPoints.size(), CV_8UC1);
        
        hestRefC(src,
                 dst,
                 (unsigned int)rtPoints.size(),
                 maxDist,
                 maxIter,
                 maxIter,
                 confidence,
                 minInliers > 4U ? minInliers : 4U,
                 Harr);
        
        double HarrD[3*4];
        for (int i = 0; i < 12; ++i)
            HarrD[i] = (double)Harr[i];
        
        H = Mat(3, 3, CV_64FC1, HarrD, 4*sizeof(double)).clone();
        
        for(int i = 0; i < rtPoints.size(); ++i)
        {
            /* Backproject */
            float x = src[i*2], y = src[i*2+1];
            float X = dst[i*2], Y = dst[i*2+1];
            
            float reprojX=Harr[0]*x + Harr[1]*y + Harr[2]; //  ( X_1 )     ( H_11 H_12    H_13  ) (x_1)
            float reprojY=Harr[4]*x + Harr[5]*y + Harr[6]; //  ( X_2 )  =  ( H_21 H_22    H_23  ) (x_2)
            float reprojZ=Harr[8]*x + Harr[9]*y + Harr[10];//  ( X_3 )     ( H_31 H_32 H_33=1.0 ) (x_3 = 1.0)
            
            //reproj is in homogeneous coordinates. To bring back to "regular" coordinates, divide by Z.
            reprojX/=reprojZ;
            reprojY/=reprojZ;
            
            //Compute distance
            reprojX-=X;
            reprojY-=Y;
            reprojX*=reprojX;
            reprojY*=reprojY;
            float reprojDist = reprojX+reprojY;
            
            /* ... */
            inliers.at<uchar>(0, i) = reprojDist <= distSq;
        }
        
        return H;
        
    }
    
};


#endif
