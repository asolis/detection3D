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

#ifndef __detection3D__taylor__
#define __detection3D__taylor__

#include <iostream>
#include "training.h"
#include "bitcount.h"
#include "ferns.h"
/*
 *  Data struct used to hold a Feature of the model.
 *  idx is and an unique id of the feature in a model.
 *  x, y the position on the target image
 *  a Descriptor associated to this point (x,y)
 *  The Descriptor is a Mat(1,SizeOfDescriptorInBytes, CV_8U)
 *
 */
struct FernFeat
{
    uint32_t idx;
    uint16_t x,y;
    Mat descriptor;
};

class Classifier
{
public:
    /*
     *  Feature Detector code used when generating
     *  the database.
     *  See training.{h|cpp} for possible codes
     */
    Feat::Code detectorCode;
    /*
     *  Descriptor Extractor code used when generating 
     *  the database.
     *  See training.{h|cpp} for possible codes
     */
    Desc::Code extractorCode;
    
    /*
     * Constructor
     */
    Classifier();
    
    /*
     *  Initialize a model/database from a list of most stable points.
     *  @param targetSize The size of the image used to train the model (i.e in pixels)
     *  @param database A list of the most stable points extracted from the model image.
     *  @param feat  Code of the feature detector to use to generate the model.
     *  @param desc  Code of the descriptor extractor to use to generate the model.
     */
    void initialize(const Size targetSize,
                    const vector<vector<StablePoint> > &database,
                    Feat::Code feat,
                    Desc::Code desc);
    
    /*
     *  Loads a model from a file.
     */
    void loadModel(const string &modelFilename);
    /*
     *  Saves model to a file
     */
    void saveModel(const string &modelFilename);
    
    
    
    /*
     *  Searches for a target in a pyramid of pts extracted from a 
     *  downsampling pyramid image.
     *  Uses the information of the most stable keypoints, their descriptors and
     *  their indices to locate the target.
     *  @param pts list of keypoints.
     *              pts[0] represent keypoints extracted on original image
     *              pts[1] extracted from downsampling (i.e. 0.5 scale factor) the previous level
     *              pts[i] extracted from downsampling .5 of previous level
     *  @param desc  desc[i] the descriptor of pts[i]
     *               each row j of desc[i] (i.e row[i][j]) is the descriptor of pst[i][j]
     *  @param indices indeces of all keypoints.
     *  @param kpts_obj outputs the points from the database that matches pts
     *  @param kpts_frame outputs the points that matches kpts_obj
     *  @param matches outputs the correspondances between kpts_obj and kpts_frame only used for debbuging 
     */
    void match(const vector< vector<KeyPoint>> &pts,
               const vector< Mat> &desc,
               const vector< vector<uint16_t>>  &indices,
               vector<Point2f> &kpts_obj,
               vector<Point2f> &kpts_frame,
               vector<DMatch> &matches);
    
    /* Prints a short description of the classifier*/
    string description();
    void getKeyPointAndDescriptors(vector<KeyPoint> &pts, Mat &desc);
    void setIndexPositions(vector<uint16_t> &idxs);
    const vector<uint16_t> &getIndexPositions();
    const vector<Point2f> &getCorners()
    {
        return corners;
    }
    
private:
    bool write(const string &filename);
    void read(const string &filename);
    void createFeatureFromStablePoint(const StablePoint &sp);
    int  computeDistance(const Mat &desc1, const Mat &desc2);
    void createCorners();
    
    const float STABILITY_OF_BITS = .50;
    
    Size               imgRefSize;
    vector<Point2f>    corners;
    vector<FernFeat> featureTable;
	vector<uint16_t>   lookUpTable[FernUtils::INDEX_MAX_VALUE];
    vector<uint16_t>   indexPositions;
    
};


#endif /* defined(__detection3D__taylor__) */
