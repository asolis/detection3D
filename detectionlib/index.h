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
#ifndef __detection3D__index__
#define __detection3D__index__

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

using namespace std;
using namespace cv;

class Index
{
private:
    static const vector<vector<int>> rotateXs();
    static const vector<vector<int>> rotateYs();
    static const vector<int> createUMax();
    static const vector<int> u_max;
public:
    static const int SIZE = 13;
    static const int ROTATION_PATCH_SIZE = 31;
    static const float xPos[SIZE];
    static const float yPos[SIZE];
    
    static const vector<vector<int>> xRotated;
    static const vector<vector<int>> yRotated;
    
    static uint16_t getDescIndex(const Mat& img, const vector<uint16_t> &positions);
    static void getDescIndices(const Mat& descriptors, vector<uint16_t> &indxs, const vector<uint16_t> &positions);
    static void getDescPyIndices(const vector<Mat> &descriptors, vector<vector<uint16_t>> &indxs, const vector<uint16_t> &positions);
    
    static vector<int> getBitHistogram(const Mat &descriptors);
    
    static void indexHistogram(vector<uint16_t> &idxs);
    static vector<uint16_t> createPos();

    static float getKeyPointOrientation(const Mat &img, const KeyPoint &pnt);
};


#endif /* defined(__detection3D__index__) */
