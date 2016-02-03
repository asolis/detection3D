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

#include "objectdetector.h"

ObjectDetector::ObjectDetector(const vector<string> &images, Feat::Code feat, Desc::Code desc) :
classifiers(images.size()), filter(4), percent(0), faces(images.size())
{
    
    Space yaw(-30, 30, 60);
    Space pitch(-30,30,60);
    Space roll(-10,10,20);
    Space scale(1*pow(.8,7), 1.0, .8);
    //Space scale(1*pow(.8,3), 1.0, .8);
    
    double fov = 37;
    int numViewPoints = 1000;
    int numKeyPointsPerViewPoint = 70;
    
    
    ViewParams params(fov,
                      yaw,
                      pitch,
                      roll,
                      scale,
                      numViewPoints,
                      numKeyPointsPerViewPoint,
                      feat,
                      desc);
    
    featDetector = Train::create(feat);
    descExtractor= Train::create(desc);
    
    countMissingFrames = MAX_MISS_FRAMES;
    
    for (int i = 0; i < classifiers.size(); i++)
    {
        stringstream ss;
        ss << images[i];
        ss << ".database";
        string modelFilename = ss.str();
        
        FILE * exists = fopen(modelFilename.c_str(), "r");
        if ( !exists )
        {
            Mat referenceImage = imread(images[i], CV_LOAD_IMAGE_GRAYSCALE);
            
            int m = max(referenceImage.rows, referenceImage.cols);
            if ( m > 600 )
            {
                float r = (float) 640.f/m;
                resize(referenceImage, referenceImage,
                       Size(referenceImage.cols * r, referenceImage.rows * r));
            }
            
            vector<vector<StablePoint> > database;
            Train::trainFromReferenceImage(referenceImage, params, database);

            vector<uint16_t> index = Train::computeDescIndices(database, INDEX_SIZE);
            classifiers[i].setIndexPositions(index);
            Train::computeIndices(database, index);
            
            
            classifiers[i].initialize(referenceImage.size(),database, feat, desc );
            classifiers[i].saveModel(modelFilename);
            database.clear();
        }
        else
        {
            fclose(exists);
            classifiers[i].loadModel(modelFilename);
            
        }
    }
    

}

void toGray(const Mat &frame, Mat &gray)
{
    if (frame.channels() > 1)
        cvtColor(frame, gray, CV_BGR2GRAY);
    else
        frame.copyTo(gray);
}

void toRGB(const Mat &frame, Mat &rgb)
{
    if (frame.channels() == 1)
        cvtColor(frame, rgb, CV_GRAY2RGB);
    else
        frame.copyTo(rgb);
}


void ObjectDetector::operator()(const size_t frame_number, const Mat &frameInput, Mat &frameOut)
{
    
//    clock_t A = clock();
    vector<Point2f> corners;
    Mat frame;
    toRGB(frameInput, frameOut);
    toGray(frameInput, frame);
    
    Mat mask = Mat::ones(frame.size(), CV_8UC1);
//    vector<Point> cc;
//    cc.push_back(rect.tl());
//    cc.push_back(rect.tl() + Point(rect.width, 0));
//    cc.push_back(rect.br());
//    cc.push_back(rect.tl() + Point(0,rect.height));
//    fillConvexPoly(mask, &cc[0], (int)cc.size(), 255, 8 , 0);
    
    
    int levels = 3;
    GaussianBlur(frame, frame, Size(3,3), 3, 3);
    vector<Mat> frames;
    vector<Mat> masks;
    buildPyramid(frame, frames, levels);
//    buildPyramid(mask, masks, levels);
    vector<vector<KeyPoint> > keypoints;
    vector<vector<uint16_t> > indices;
    vector<Mat> descriptors;
//    clock_t B = clock();
//    featDetector->detect(frames, keypoints);
    featDetector->detect(frames, keypoints, masks);
//    clock_t C = clock();
    descExtractor->compute(frames, keypoints, descriptors);
//    clock_t D = clock();
    
    

    int idx = 0, maxInliers = 0;
    bool possibleObjectView = false;
    vector<Point2f> oPts, sPts;
    for (int i = 0; i < classifiers.size(); i++)
    {
        vector<DMatch> matches;
        vector<Point2f> objPts;
        vector<Point2f> scnPts;
        
        vector<vector<uint16_t> > idxs;
        Index::getDescPyIndices(descriptors, idxs, classifiers[i].getIndexPositions());
        classifiers[i].match(keypoints, descriptors, idxs, objPts, scnPts, matches);
        
       // classifiers[i].match(keypoints, descriptors, indices, objPts, scnPts, matches);
        //8 Points needed for fundamental matrix computation
        if (matches.size() < 8)
            continue;
        int inliers = FernUtils::numInliers(objPts, scnPts);

        if (inliers > maxInliers)
        {
            maxInliers = inliers;
            idx = i;
            oPts = objPts;
            sPts = scnPts;
            possibleObjectView = true;
        }
    }
//    clock_t E = clock();
    Scalar purple(255,0,255);
    Scalar red(0,0,255);
    Scalar blue(255,0,0);
    Scalar green(0,255,0);
    Point2f shift(0,0);
    vector<Point2f> fc;
    
    if (possibleObjectView)
    {

        Mat F, H;
        vector<Point2f> oFInliners, sFInliers, corners, fCorners;
        vector<Vec3f> eLines;
        const vector<Point2f> &clCorners = classifiers[idx].getCorners();
        FernUtils::computeFundamentalMatrix(oPts, sPts, clCorners, F, oFInliners, sFInliers, eLines);
        int realInliers = FernUtils::computeHomography(oFInliners, sFInliers, clCorners, H, corners);
        
        if (realInliers > 4)
        {
            if (!faces[idx])
            {
                faces[idx] = true;
                cout << faces.size() << " " << 100/faces.size() << endl;
                percent += 100.0 / faces.size();
            }
            
            FernUtils::refineCornersWithEpipolarLines(eLines, corners, fCorners);
            corners = filter.correct(fCorners);
            lastKnowCorners  = corners;
            FernUtils::drawObject(frameOut, corners, purple, blue, shift, percent);
            FernUtils::drawCorners(frameOut, corners, green, shift);
            rect = boundingRect(corners);
            //rectangle(frameOut, rect.tl(), rect.br(), blue, 4);
            stringstream ss;
            ss << "Object View Detected: " << (idx + 1);
            putText(frameOut, ss.str() , Point(40,30), FONT_HERSHEY_DUPLEX, 1, blue);
            rect = boundingRect(corners);
            countMissingFrames = 0;
        }
        else
        {
            countMissingFrames++;
            if (countMissingFrames < MAX_MISS_FRAMES)
            {
                corners = filter.predict();
                rect = boundingRect(corners);
                //rectangle(frameOut, rect.tl(), rect.br(), blue, 4);
                FernUtils::drawObject(frameOut, corners, purple, blue, shift, percent);
                FernUtils::drawCorners(frameOut, corners, green, shift);
            }
            else
                rect = Rect(0,0,frame.cols, frame.rows);
        }
    }
    if (lastKnowCorners.size() == 4)
        filter.correct(lastKnowCorners);
    
//    clock_t FE = clock();

//    printf("Pre: %.2f (%.2f) Detect: %.2f (%.2f) Extract: %.2f (%.2f) Match: %.2f (%.2f) R+K: %.2f (%.2f)\n",
//           double(B-A)/CLOCKS_PER_SEC,
//           1.0/(double(B-A)/CLOCKS_PER_SEC),
//           double(C-B)/CLOCKS_PER_SEC,
//           1.0/(double(C-B)/CLOCKS_PER_SEC),
//           double(D-C)/CLOCKS_PER_SEC,
//           1.0/(double(D-C)/CLOCKS_PER_SEC),
//           double(E-D)/CLOCKS_PER_SEC,
//           1.0/(double(E-D)/CLOCKS_PER_SEC),
//           double(FE-E)/CLOCKS_PER_SEC,
//           1.0/(double(FE-E)/CLOCKS_PER_SEC));
    
    frames.clear();
    masks.clear();
    keypoints.clear();
    indices.clear();
    descriptors.clear();
}











