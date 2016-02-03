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

#ifndef __detection3D__objectdetector__
#define __detection3D__objectdetector__


#include <iostream>
#include <string>
#include <sstream>
#include "viva.h"
#include "classifier.h"
#include "kalman.h"

using namespace viva;

class ObjectDetector : public  ProcessFrame
{
private:
    const int MAX_MISS_FRAMES = 5;
    const int INDEX_SIZE = 13;
    
    Ptr<FeatureDetector>     featDetector;
    Ptr<DescriptorExtractor> descExtractor;
    vector<Classifier>   classifiers;

    int countMissingFrames;
    Rect    rect;
    Kalman filter;
    
    vector<bool>               faces;
    float percent;
    
    
    vector<Point2f> lastKnowCorners;
    
public:
    /*
     * Uses a reference image to generate
     * the model. The image is a orthographic pic.
     * @param image Path to the image.
     *              If the path of image concatenated with ".database"
     *              exists, it will try to load the model from file.
     *              e.g.: If image = "pic.jpg"
     *              it will try to find "pic.jpg.database" in the same folder
     *              to load the model. If it's not found it will train
     *              and create the model from image "pic.jpg" and generate
     *              file "pic.jpg"
     *
     */
    ObjectDetector(const vector<string> &images, Feat::Code feat, Desc::Code desc);
    
    void operator()(const size_t frameN, const Mat &frame, Mat &output);
    
};


#endif /* defined(__detection3D__objectdetector__) */
