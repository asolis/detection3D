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
#include "kalman.h"


Kalman::Kalman(int numPts):
filters(numPts), initialized(false)
{
    initialize(numPts);
}
Kalman::Kalman(vector<Point2f> &pts):
filters(pts.size()), initialized(false)
{
    initialize((int)pts.size());
    initState(pts);
}

void Kalman::initialize(int numPts)
{
    for (size_t i = 0; i < numPts; ++i) {
        KalmanFilter KF(4, 2, 0);
        //A matrix
        KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,
                                                     0,1,0,1,
                                                     0,0,1,0,
                                                     0,0,0,1);
        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
        setIdentity(KF.errorCovPost, Scalar::all(.1));
        filters[i] = (KF);
    }
}
void Kalman::initState(vector<Point2f> &pts)
{
    CV_Assert(filters.size() == pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
    {
        filters[i].statePre.at<float>(0) = pts[i].x;
        filters[i].statePre.at<float>(1) = pts[i].y;
        filters[i].statePre.at<float>(2) = 1;
        filters[i].statePre.at<float>(3) = 1;
    }
    initialized = true;
}
void Kalman::reset()
{
    initialized = false;
}

vector<Point2f> Kalman::correct(vector<Point2f> &pts)
{
    CV_Assert(filters.size() == pts.size());
    
    if (!initialized)
    {
        initState(pts);
        return pts;
    }
    
    vector<Point2f> estimation(pts.size());
    
    Mat measurement(2, 1,CV_32F);
    for (size_t i = 0; i < filters.size(); i++)
    {
        filters[i].predict();
        measurement.at<float>(0) = pts[i].x;
        measurement.at<float>(1) = pts[i].y;
        
        Mat estimated = filters[i].correct(measurement);
        
        estimation[i] = Point(estimated.at<float>(0),
                              estimated.at<float>(1));
        
    }
    return estimation;
}

vector<Point2f> Kalman::predict()
{
    vector<Point2f> prediction(filters.size(),Point2f(0,0));
    if (!initialized)
        return prediction;
        
    Mat predict(2, 1,CV_32F);
    for (size_t i = 0; i < filters.size(); i++)
    {
        predict = filters[i].predict();
        
        
        
        prediction[i] = Point(predict.at<float>(0),
                              predict.at<float>(1));
        
    }
    return prediction;
}