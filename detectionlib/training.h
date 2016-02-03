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

#ifndef __detection3D__training__
#define __detection3D__training__

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "utils.h"
#include "feature.h"
#include "bitcount.h"
#include "index.h"

using namespace cv;
using namespace std;

class Train
{
public:

    static int fastTrainFromReference(Mat &image, SimpleParams &params,
                                       vector<vector<KeyPoint>> &kps,
                                       vector<Mat> &descs,
                                       vector<vector<Point2f>> &corn);
    
    static void trainFromReferenceImage(Mat &image,
                                        ViewParams &params,
                                        vector<vector<StablePoint> > &database);
    
    static vector<uint16_t> computeDescIndices(vector<vector<StablePoint>> &database, int size);
    
    static void computeIndices(vector<vector<StablePoint>> &database, vector<uint16_t> &indices);

    
    static Ptr<FeatureDetector>     create(Feat::Code code);
    static Ptr<DescriptorExtractor> create(Desc::Code code);
    static string featureName(Feat::Code code);
    static string descriptorName(Desc::Code code);
    static int  hammingDistance(const Mat &desc1, const Mat &desc2);
    static const int minDistance2ReferencePoint;
    static const int minDistance2ReferencePointSqr;
    static const int HALF_PATCH_WIDTH;
    static const int SIGMA_BLUR;
    
    
private:
    static void rotateCorners(vector<Point2f> &pts,
                              vector<Point2f> &out,
                              Point2f center,
                              float angle,
                              float scale);
    
    static vector<Point2f> rotateImage(Mat &image,
                            Mat &out,
                            float angle,
                            float scale);
    
    static bool sortByProbNearToDot5(std::pair<int,float> &first,
                                     std::pair<int,float> &second);
    
    static double rad2Deg(double rad);
    static double deg2Rad(double deg);
    static void   warpMatrix(Size sz,
                             double yaw,
                             double pitch,
                             double roll,
                             double scale,
                             double fovy,
                             Mat &M,
                             vector<Point2f>* corners);
    static void warpImage(const Mat &src,
                          double yaw,
                          double pitch,
                          double roll,
                          double scale,
                          double fovy,
                          Mat &dst,
                          Mat &M,
                          vector<Point2f> &corners);
    static void selectPoints(Mat &image,
                             Mat &M,
                             vector<Point2f> &corners,
                             Size refImgSize,
                             Ptr<FeatureDetector> &detector,
                             Ptr<DescriptorExtractor> &extractor,
                             vector<KeyPoint> &kps,
                             vector<uint16_t> &indxs,
                             Mat &desc);
    static void selectMostStableKeypoints(const vector<vector<KeyPoint>> &keypoints,
                                          const vector<vector<uint16_t>> &indices,
                                          const vector<Mat> &descriptors,
                                          const vector<Mat>&transfMatrix,
                                          const vector<vector<Point2f> >& corners,
                                          int maxPointsPerView,
                                          Size refImgSize,
                                          Ptr<FeatureDetector> &detector,
                                          Ptr<DescriptorExtractor> &extractor,
                                          vector<StablePoint> &bestPoints);
    static bool keyPointOrderingByResponse(const KeyPoint& k1,
                                           const KeyPoint& k2);
    static bool compareStablePoints(const StablePoint &x1,
                                    const StablePoint &x2);
    static Mat  keyPoint2Mat(const vector<KeyPoint>& keypoints);
    static vector<Point> mat2Points(const Mat& stub);
    static void projectKeypoints(const vector<KeyPoint> &original,
                                 const MatExpr M,
                                 vector<Point> &transformedPoints);
    static void filterKeyPoints(Size targetImgSize,
                                const vector<Point2f> &corners,
                                vector<KeyPoint> &keypoints,
                                vector<Point> &pointsTransformed);
    static int getNearbyPoint(const Point& p, const vector<vector<int> > &grid);
    
    /**
     * Rotate an image around its center. It will cut out the borders
     * of the image while rotating it.
     */
    static void rotateImage(cv::Mat& src, double angle, cv::Mat& dst, Mat &R);
    
    /**
     * Rotate an image around its center. I will not cut out the borders of the image
     * while rotating. It will create a black frame around the image. 
     */
    static void rotateAtCenter(cv::Mat& src, double angle, cv::Mat& dst, Mat &R);
    
};



#endif /* defined(__detection3D__training__) */
