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
#include "index.h"

const float Index::yPos[] = {-5.5,-5.5,-3.5,-3.5,-1.5,-1.5,0.5,1.5,1.5,3.5,3.5,5.5,5.5};
const float Index::xPos[] = {-5.5,5.5,-1.5,1.5,-3.5,3.5,0.5,-3.5,3.5,-1.5,1.5,-5.5,5.5};

void Index::indexHistogram(vector<uint16_t> &idxs)
{
    vector<int> histogram(8192,0);
    for (int i = 0; i < idxs.size(); ++i) {
        histogram[idxs[i]]++;
    }
    int count = 0;
    for (int i = 0; i < 8192; ++i) {
        if (histogram[i])
            count++;
    }
    cout << count  <<endl;
    for (int i = 0; i < 8192; ++i) {
        if (histogram[i])
            cout << histogram[i] << " , " ;
    }
    cout << endl;
    
}


vector<int> Index::getBitHistogram(const Mat &descriptors)
{
    int size = descriptors.cols * CHAR_BIT;
    vector<int> tmp(size, 0);
    for (int r = 0; r < descriptors.rows; ++r)
    {
        Mat row =  descriptors.row(r);
        uchar *data = row.data;
        for (int i = 0; i < size; ++i)
        {
           uchar mask = 1 << i % CHAR_BIT;
           if (data[i / CHAR_BIT] & mask)
               tmp[i]++;
        }
    }
    return tmp;
}


vector<uint16_t> Index::createPos()
{
    vector<uint16_t> rng;
    int max = 255;
    bool exist[max];
    for (int i = 0; i < max; i++)
    {
        exist[i] = false;
    }
    
    while (rng.size() < SIZE)
    {
        int n = rand() % max;
        if (exist[n])
            continue;
        else
        {
            exist[n] = true;
            rng.push_back(n);
        }
    }
    sort(rng.begin(), rng.end());
    return rng;
    
}
const vector<vector<int>> Index::rotateXs()
{
    vector<vector<int>> data;
    for (size_t a = 0; a < 360; ++a)
    {
        float angle = degreesToRadians(a);
        vector<int> _tmp;
        for (size_t i = 0; i < SIZE; ++i)
        {
            _tmp.push_back((xPos[i] * cos(angle) + yPos[i] * sin(angle)));
//            _tmp.push_back((xPos[i] * cos(angle) - yPos[i] * sin(angle)));
        }
        data.push_back(_tmp);
    }
    return data;
}
const vector<vector<int>> Index::rotateYs()
{
    vector<vector<int>> data;
    for (size_t a = 0; a < 360; ++a)
    {
        float angle = degreesToRadians(a);
        vector<int> _tmp;
        for (size_t i = 0; i < SIZE; ++i)
        {
            //Reserved because image coord are reversed as natural Euclidean system
             _tmp.push_back(-(-xPos[i] * sin(angle) + yPos[i] * cos(angle)));
            // _tmp.push_back(xPos[i] * sin(angle) + yPos[i] * cos(angle));
        }
        data.push_back(_tmp);
    }
    return data;
}
void Index::getDescIndices(const Mat& descriptors, vector<uint16_t> &indxs, const vector<uint16_t> &positions)
{
    for (int t = 0; t < descriptors.rows; ++t)
    {
        Mat row = descriptors.row(t);
        indxs.push_back(getDescIndex(row, positions));
    }
}
void Index::getDescPyIndices(const vector<Mat> &descriptors, vector<vector<uint16_t>> &indxs, const vector<uint16_t> &positions)
{
    for (int i = 0; i < descriptors.size(); ++i)
    {
        vector<uint16_t> tmp;
        getDescIndices(descriptors[i], tmp, positions);
        indxs.push_back(tmp);
    }
}

uint16_t Index::getDescIndex(const Mat& img, const vector<uint16_t> &positions)
{
    uchar *data = img.data;
    uint16_t index = 0;
    int size = (int)positions.size();
    for (int i = 0; i < size; i++)
    {
        uchar mask = 1 << positions[i] % CHAR_BIT;
        
        index |= (data[positions[i] / CHAR_BIT ] & mask)? 1 << i: 0;
    }
    return index;
}




const vector<int> Index::createUMax()
{
    int patchSize = ROTATION_PATCH_SIZE;
    // Make sure we forget about what is too close to the boundary
    //edge_threshold_ = std::max(edge_threshold_, patch_size_/2 + kKernelWidth / 2 + 2);
    
    // pre-compute the end of a row in a circular patch
    int halfPatchSize = patchSize / 2;
    vector<int> u_max(halfPatchSize + 2);
    
    int v, v0, vmax = cvFloor(halfPatchSize * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(halfPatchSize * sqrt(2.f) / 2);
    for (v = 0; v <= vmax; ++v)
        u_max[v] = cvRound(sqrt((double)halfPatchSize * halfPatchSize - v * v));
    
    // Make sure we are symmetric
    for (v = halfPatchSize, v0 = 0; v >= vmin; --v)
    {
        while (u_max[v0] == u_max[v0 + 1])
            ++v0;
        u_max[v] = v0;
        ++v0;
    }
    return u_max;
}

float Index::getKeyPointOrientation(const Mat& image, const KeyPoint &pnt)
{
    int half_k = ROTATION_PATCH_SIZE/2;
    Point2f pt = pnt.pt;
    
    int m_01 = 0, m_10 = 0;
    
    const uchar* center = image.ptr<uchar> (cvRound(pt.y), cvRound(pt.x));
    
    // Treat the center line differently, v=0
    for (int u = -half_k; u <= half_k; ++u)
        m_10 += u * center[u];
    
    // Go line by line in the circular patch
    int step = (int)image.step1();
    for (int v = 1; v <= half_k; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }
    
    return fastAtan2((float)m_01, (float)m_10);
}

const vector<vector<int>> Index::xRotated = Index::rotateXs();
const vector<vector<int>> Index::yRotated = Index::rotateYs();
const vector<int> Index::u_max = Index::createUMax();
