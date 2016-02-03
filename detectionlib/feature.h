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

#ifndef __detection3D__train__
#define __detection3D__train__

#include <iostream>
#include <string>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "brief.h"

using namespace std;
using namespace cv;

struct Feat
{
private:
    static string names[9];
public:
    enum Code {ORB, FAST, MSER, SIFT, SURF, STAR, GRIDFAST, GRIDORB, RFAST};
    
    static string getName(Feat::Code code);
    static Ptr<FeatureDetector>  create(Feat::Code code);
};


struct Desc
{
private:
    static string names[6];
public:
    enum Code {ORB, BRIEF, SIFT, SURF, FREAK, BRISK};
    
    static string getName(Desc::Code code);
    static Ptr<DescriptorExtractor> create(Desc::Code code);
};


struct Space
{
public:
    double min;
    double max;
    double step;
    Space(double _min, double _max, double _step):
    min(_min), max(_max), step(_step){}
};



struct StablePoint
{
public:
    Point pt;
    vector<KeyPoint> viewPointPts;
    vector<int>       imageNumber;
    vector<uint16_t>      indices;
    Mat    viewPointPtDescriptors;
    
    StablePoint():
    pt(Point(0,0)), viewPointPtDescriptors(0, 0, CV_8UC1){}
    
    StablePoint(Point _pt, KeyPoint view, int image, Mat &desc, uint16_t index):
    pt(_pt), viewPointPtDescriptors(0, desc.cols, CV_8UC1)
    {
        viewPointPts.push_back(view);
        imageNumber.push_back(image);
        viewPointPtDescriptors.push_back(desc);
        indices.push_back(index);
    }
};

struct SimpleParams
{
public:
    Space roll;
    Space scale;

    int    numberOfPointsPerViewPoints;
    SimpleParams(Space roll,
                 Space scale,
                 int nOfPts):
        roll(roll), scale(scale),
    numberOfPointsPerViewPoints(nOfPts){}
};

struct ViewParams
{
public:
    
    double fov;
    Space  yaw; //Vertical Axis
    Space  pitch; //Horizontal Axis
    Space  roll;  //Depth Axis
    Space  scale;
    int    numberOfViewPoints;
    int    numberOfPointsPerViewPoints;
    Feat::Code detector;
    Desc::Code extractor;
    
    ViewParams(double _fov,
               Space _yaw,
               Space _pitch,
               Space _roll,
               Space _scale,
               int _nOfViewPoints,
               int _nOfPointsPerView,
               Feat::Code _detector,
               Desc::Code _extractor)
    :fov(_fov), yaw(_yaw), pitch(_pitch), roll(_roll), scale(_scale),
    numberOfViewPoints(_nOfViewPoints),
    numberOfPointsPerViewPoints(_nOfPointsPerView),
    detector(_detector),
    extractor(_extractor){}
};

#endif /* defined(__detection3D__train__) */


