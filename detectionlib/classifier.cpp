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

#include "classifier.h"
#include <map>
#include <utility>
#include <algorithm>

/*
 *  util function to order two pairs. 
 */
bool comparePairs( const  std::pair<uint16_t, uint32_t> &l,
                  const  std::pair<uint16_t, uint32_t> &r)
{
    return l.second > r. second;
}

/*
 *  Create a database entry (i.e. Feature) from the stable point.
 *  it merges all the descriptors from the stable points (i.e. computed from different views)
 *  to create an unique descriptor.
 */
void Classifier::createFeatureFromStablePoint(const StablePoint &sp)
{
    std::map<uint16_t,uint32_t> indexHistogram;
    uint16_t descSize = sp.viewPointPtDescriptors.cols * 8;
    vector<int> bitHistogram(descSize, 0);
    
    int totalDesc = sp.viewPointPtDescriptors.rows;
    int colSize   = sp.viewPointPtDescriptors.cols;
    
    Mat finalDescriptor = Mat::zeros(1, colSize, CV_8UC1);
    for (int d = 0 ; d < totalDesc; ++d)
    {
        Mat desc = sp.viewPointPtDescriptors.row(d);
        indexHistogram[sp.indices[d]]++;
        
        for (int col = 0; col < colSize; ++col)
            for (int bit = 7; bit >= 0; --bit)
            {
                uchar tmp = (1 << bit);
                if ((desc.at<uchar>(0, col) & tmp) > 0)
                {
                    bitHistogram[ col * 8 + (7 - bit)]++;
                }
            }
    }
    
    for (int col = 0; col < colSize; ++col)
        for (int bit = 7; bit >= 0; --bit)
        {
            uchar tmp = (1 << bit);
            if (bitHistogram[ col * 8 + (7 - bit)] >
                STABILITY_OF_BITS * totalDesc)
            {
                finalDescriptor.at<uchar>(0, col) |= tmp;
            }
        }
    
    vector< std::pair<uint16_t, uint32_t> > mPairs;
    float sum = 0;
    std::map<uint16_t,uint32_t>::iterator mapIt;
    for (mapIt = indexHistogram.begin(); mapIt != indexHistogram.end(); ++mapIt)
    {
        mPairs.push_back(*mapIt);
        sum += mapIt->second;
    }
    sort(mPairs.begin(), mPairs.end(), comparePairs);
    
    float tmpSum = 0;
    for (int i = 0; ( i < mPairs.size() && (tmpSum/sum < .75)) ; ++i)
    {
        uint16_t index = mPairs[i].first;
        uint32_t value = mPairs[i].second;
        lookUpTable[index].push_back((uint32_t)featureTable.size());
        tmpSum += value;
    }
    
    FernFeat _feat;
    _feat.idx = (uint32_t)featureTable.size();
    _feat.descriptor = finalDescriptor.clone();
    _feat.x = sp.pt.x;
    _feat.y = sp.pt.y;
    featureTable.push_back(_feat);
}

Classifier::Classifier():
imgRefSize(Size(0,0)),
detectorCode(Feat::FAST),
extractorCode(Desc::BRIEF),
corners(),
featureTable()
{
    for(int i = 0; i < FernUtils::INDEX_MAX_VALUE; ++i)
        lookUpTable[i] = vector<uint16_t>();
};

/*
 *  Initializes Classifier using the list of most stable keypoints.
 */
void Classifier::initialize(const Size img,
                                  const vector<vector<StablePoint> > &database,
                                  Feat::Code    feat,
                                  Desc::Code desc)
{
    imgRefSize    = img;
    detectorCode  = feat;
    extractorCode = desc;
    createCorners();
    
    for(int v = 0; v < database.size(); ++v)
        for (int sp = 0; sp < database[v].size(); ++sp)
        {
            createFeatureFromStablePoint(database[v][sp]);
        }
}

/*
 *  Saves the binary model to the filename path.
 */
void Classifier::saveModel(const string &modelFilename)
{
    write(modelFilename);
}

/*
 *  Loads the binary model into the
 *  instance of the object.
 */
void Classifier::loadModel(const string &modelFilename)
{
    read(modelFilename);
}


/*
 *  Computes the Hamming Distance between two binary descriptors.
 *  It uses a lookup table f(x)=> number_of_bits. Where x is a 8bit number
 *  (0 to 255).
 */
int Classifier::computeDistance(const Mat &desc1, const Mat &desc2)
{
    Mat _xor = Mat::zeros(1, desc1.cols, CV_8UC1);
    int hamming = 0;
    for (size_t i = 0 ; i < desc1.cols; ++i)
    {
        _xor.data[i] = desc1.data[i] ^ + desc2.data[i];
        hamming += BitCount::bits_in_char[_xor.data[i]];
    }
    return hamming;
}

/*
 *  Private method to create the targets corners position
 *  from the width and height. 
 *  The order of the points are Top Left, Top Right, Bottom Right, Bottom Left.
 */
void Classifier::createCorners()
{
    corners.clear();
    corners.push_back(Point2f(0,imgRefSize.height));//TL
    corners.push_back(Point2f(imgRefSize.width,imgRefSize.height));//TR
    corners.push_back(Point2f(imgRefSize.width,0));//BR
    corners.push_back(Point2f(0,0)); //BL
}


/*
 *  Searchs for the target using the keypoints, descriptors, and indices.
 *  pts[i] is a list of keypoints selected at pyramid level i of the image.
 *          Each image in the pyramid is half the size of the level before.
 *          At level 0 is the original image.
 *          Pts are scaled according of the level they are detected in.
 *  desc[i] is a Mat with each row representing a descriptor.
 *          The row j of the mat represents the descriptor of keypoint at index j;
 *          desc[i].row(j) is the descriptor of point pts[i][j]
 *  index[i]list of keypoint indices.
 *          index[i][j] = index of keypoint pts[i][j]
 *
 *  @param pts List of keypoint extracted at each level of the pyramid level.
 *             The first level is the original image.
 *  @param desc List of descriptors for each keypoints extracted at each level.
 *              It's represented as a matrix where the rows are the keypoint index
 *              and cols are the size of descriptor.
 *  @param indices
 *
 */
void Classifier::match(const vector< vector<KeyPoint>> &pts,
                           const vector< Mat> &desc,
                           const vector< vector<uint16_t>>  &indices,
                           vector<Point2f> &kpts_obj,
                           vector<Point2f> &kpts_frame,
                           vector<DMatch> &matches)
{
    vector<Point2f>  rtPts;
    vector<DMatch>   _match;
    
    for (size_t level = 0; level < pts.size() ; level++)
    {
        const vector<KeyPoint> &viewPts  = pts[level];
        const vector<uint16_t>  &indexPts = indices[level];
        const Mat &viewDesc = desc[level];
        
        for (size_t index = 0; index < viewPts.size(); ++index)
        {
            unsigned int minDist = 0x7FFFFFFF;
            int minDistIdx = 0;
            
            size_t possibleMatches = lookUpTable[indexPts[index]].size();
            for (size_t p = 0; p < possibleMatches; ++p)
            {
                int idx = lookUpTable[indexPts[index]][p];
                FernFeat _f = featureTable[idx];
                Mat desc = viewDesc.row((int)index);
                int distance = computeDistance(_f.descriptor, desc);
                
                if (distance < minDist)
                {
                    minDist = distance;
                    minDistIdx = idx;
                }
            }
            
            if (minDist < FernUtils::BINARY_THRESHOLD)
            {
                _match.push_back(DMatch((int)rtPts.size(),
                                        minDistIdx,
                                        minDist));
                rtPts.push_back( viewPts[index].pt * (1 << level) );
            }
        }
        if (rtPts.size() > FernUtils::MAX_POINTS)
            break;
    }
    sort(_match.begin(), _match.end(), FernUtils::sortByQueryId);
    
    for( int index = 0 ; index < (int)(_match.size() - 1); ++index)
    {
        DMatch &match = _match[index];
        DMatch &nMatch= _match[index + 1];
        if (match.trainIdx != nMatch.trainIdx)
        {
            matches.push_back(match);
        }
    }
    sort(matches.begin(), matches.end(), FernUtils::sortByDistance);
    
    for( int index = 0 ; index < matches.size(); ++index)
    {
        DMatch &match = matches[index];
        kpts_frame.push_back( rtPts[match.queryIdx]);
        kpts_obj.push_back(Point2f(featureTable[match.trainIdx].x,
                                    featureTable[match.trainIdx].y));
            
        match.queryIdx = (int)kpts_obj.size() - 1;
        match.trainIdx = match.queryIdx;
    }
        
    rtPts.clear();
    _match.clear();
}






/**
 *  Prints a description test describing 
 *  the Feature Extractor and Descriptor detector used to create the 
 *  model.
 *
 **/
string Classifier::description()
{
    stringstream ss;
    ss << "Taylor: Feature [" <<
          Train::featureName(detectorCode) <<
          "] Descriptor: "  <<
          Train::descriptorName(extractorCode) << "]";
    
    return ss.str();
}

/**
 *  Writes a binary model from the filename.
 *  @param filename string. Path to the binary file containing the model
 *
 *  Format:
 *  4 bytes - Value of detectorCode used to generate the model
 *  4 bytes - Value of extractorCode used to generate the model
 *  4 bytes - Size in bytes of the descriptor used
 *  8 bytes (4bytes, 4bytes) - Size of the target image used to train.
 *  8 bytes - Length in bytes of the feature table
 *  .........Table of Features (consecutive Features)
 *      A Feature is:
 *          X bytes, Descriptor bytes. The amount of X is the Size in bytes of
 *                   the descriptor.
 *          2 bytes, 'x' coordiate of the feature.
 *          2 bytes, 'y' coordinate of the feature.
 *          4 bytes, id of the feature.
 *  ..........Lookup Table from 0 to 8192
 *  index0: 8 bytes - Size in bytes of the first vector (So)
 *  index0: So * 4 bytes to fillup the vector
 *  index1: 8 bytes (S1)
 *  index1: S1 * 4 bytes
 *  index2: 8 bytes (S2)
 *  index2: S2 * 4 bytes
 *  .
 *  .
 *  .
 *
 **/
bool Classifier::write(const string &filename)
{
    FILE* dfile = fopen(filename.c_str(), "wb");
    printf("Saving model in %s \n", filename.c_str());
    
    fwrite(&detectorCode, sizeof(uint32_t), 1, dfile);
    fwrite(&extractorCode, sizeof(uint32_t), 1, dfile);
    uint32_t dSize;
    switch (featureTable[0].descriptor.depth())
    {
        case CV_8U:
            dSize = featureTable[0].descriptor.cols;
            break;
        case CV_32F:
            dSize = featureTable[0].descriptor.cols * sizeof(float);
            break;
        default:
            dSize = featureTable[0].descriptor.cols;
            break;
    }
    fwrite(&dSize, sizeof(uint32_t),1, dfile);
    fwrite(&imgRefSize, sizeof(Size), 1, dfile);
    uint16_t amountOfElements = featureTable.size();
    fwrite(&amountOfElements, sizeof(uint16_t), 1, dfile);
    for ( uint16_t e = 0; e < amountOfElements; ++e)
    {
        fwrite(featureTable[e].descriptor.data, sizeof(uchar), dSize, dfile);
        fwrite(&featureTable[e].x, sizeof(uint16_t), 1, dfile);
        fwrite(&featureTable[e].y, sizeof(uint16_t), 1, dfile);
        fwrite(&featureTable[e].idx, sizeof(uint32_t), 1, dfile);
        
    }
    
    uint16_t indices = 0;
    for (uint16_t i = 0; i < 8192; i++)
    {
        if (lookUpTable[i].size() > 0)
            indices++;
    }
    fwrite(&indices, sizeof(uint16_t), 1, dfile);
    
    //8192 = 2^13

    for (uint16_t i = 0; i < 8192; i++)
    {
        uint16_t numberOfElements = lookUpTable[i].size();
        if (numberOfElements > 0)
        {
            fwrite(&i, sizeof(uint16_t), 1, dfile);
            fwrite(&numberOfElements, sizeof(uint16_t), 1, dfile);
            fwrite(&lookUpTable[i][0], sizeof(uint16_t), numberOfElements, dfile);
        }
    }
    
    
    uint16_t index = indexPositions.size();
    fwrite(&index, sizeof(uint16_t), 1, dfile);
    fwrite(&indexPositions[0], sizeof(uint16_t), index, dfile);
    fclose(dfile);
    return true;
}

const vector<uint16_t> &Classifier::getIndexPositions()
{
    return indexPositions;
}


void Classifier::setIndexPositions(vector<uint16_t> &idxs)
{
    indexPositions = idxs;
}
/**
 *  Reads a binary model from the filename.
 *  @param filename string. Path to the binary file containing the model
 *  
 *  Format:
 *  4 bytes - Value of detectorCode used to generate the model
 *  4 bytes - Value of extractorCode used to generate the model
 *  4 bytes - Size in bytes of the descriptor used
 *  8 bytes (4bytes, 4bytes) - Size of the target image used to train. 
 *  8 byes - Length in bytes of the feature table
 *  .........Table of Features (consecutive Features)
 *      A Feature is:
 *          X bytes, Descriptor bytes. The amount of X is the Size in bytes of 
 *                   the descriptor. 
 *          2 bytes, 'x' coordiate of the feature.
 *          2 bytes, 'y' coordinate of the feature.
 *          4 bytes, id of the feature.
 *  ..........Lookup Table from 0 to 8192
 *  index0: 8 bytes - Size in bytes of the first vector (So)
 *  index0: So * 4 bytes to fillup the vector
 *  index1: 8 bytes (S1)
 *  index1: S1 * 4 bytes
 *  index2: 8 bytes (S2)
 *  index2: S2 * 4 bytes
 *  .
 *  .
 *  .
 *
 **/
void Classifier::read(const string &filename)
{
    printf("Loding model %s \n", filename.c_str());
    FILE * dfile = fopen(filename.c_str(), "rb");
    if ( !dfile )
        return;
    
    fread(&detectorCode, sizeof(uint32_t), 1, dfile);
    fread(&extractorCode, sizeof(uint32_t), 1, dfile);
    uint32_t dSize;
    fread(&dSize, sizeof(uint32_t),1, dfile);
    fread(&imgRefSize, sizeof(Size), 1, dfile);
 
    uint16_t amountOfElements;
    fread(&amountOfElements, sizeof(uint16_t), 1, dfile);
    featureTable.resize(amountOfElements);
    for ( uint16_t e = 0; e < amountOfElements; ++e)
    {
        featureTable[e].descriptor = Mat(1, dSize, CV_8U);
        fread(featureTable[e].descriptor.data, sizeof(uchar), dSize, dfile);
        fread(&featureTable[e].x, sizeof(uint16_t), 1, dfile);
        fread(&featureTable[e].y, sizeof(uint16_t), 1, dfile);
        fread(&featureTable[e].idx, sizeof(uint32_t), 1, dfile);
        
    }
    uint16_t numberOfIndxs;
    fread(&numberOfIndxs, sizeof(uint16_t), 1, dfile);
    
    //8192 = 2^13
    for (uint64_t i = 0; i < numberOfIndxs; i++)
    {
        
        uint16_t numberOfElements, index;
        fread(&index, sizeof(uint16_t), 1, dfile);
        fread(&numberOfElements, sizeof(uint16_t), 1, dfile);
        lookUpTable[index].resize(numberOfElements);
        size_t elements = fread(&lookUpTable[index][0],
                                sizeof(uint16_t), numberOfElements, dfile);
        if (elements != numberOfElements)
            printf("Error: Could not read look up table\n");
        
    }
    uint16_t index;
    fread(&index, sizeof(uint16_t), 1, dfile);
    indexPositions.resize(index);
    fread(&indexPositions[0], sizeof(uint16_t), index, dfile);
    
    createCorners();
    fclose(dfile);
}


void Classifier::getKeyPointAndDescriptors(vector<KeyPoint> &pts, Mat &desc)
{
    desc = Mat::zeros(0, featureTable[0].descriptor.cols, CV_8UC1);
    for (int i = 0 ; i < featureTable.size(); ++i)
    {
        pts.push_back(KeyPoint(featureTable[i].x, featureTable[i].y, 4));
        desc.push_back(featureTable[i].descriptor);
    }
}


