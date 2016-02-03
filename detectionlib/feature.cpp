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
#include "feature.h"
#include "rfast.h"
string Desc::names[6] = {"ORB", "BRIEF", "SIFT", "SURF", "FREAK", "BRISK"};
string Feat::names[9] = {"ORB", "FAST", "MSERF", "SIFT", "SURF", "STAR", "GRIDFAST", "GRIDORB", "RFAST"};

string Feat::getName(Feat::Code code)
{
    return names[code];
}

Ptr<FeatureDetector>  Feat::create(Feat::Code code)
{
    switch (code)
    {
        case Feat::ORB:
            return new cv::OrbFeatureDetector(1000);
            break;
        case Feat::FAST:
            return new cv::FastFeatureDetector();
            break;
        case Feat::MSER:
            return new cv::MserFeatureDetector();
            break;
        case Feat::SIFT:
            return new cv::SiftFeatureDetector();
            break;
        case Feat::SURF:
            return new cv::SurfFeatureDetector();
            break;
        case Feat::GRIDFAST:
            return new GridAdaptedFeatureDetector(new FastFeatureDetector(10), 1000, 4, 4);
            break;
        case Feat::GRIDORB:
            return new GridAdaptedFeatureDetector(new OrbFeatureDetector(700), 700, 4, 4);
            break;
        case Feat::RFAST:
            return new RFastFeatureDetector();
            break;

        default:
            return new cv::FastFeatureDetector();
            break;
    }
    
}

string Desc::getName(Desc::Code code)
{
    return names[code];
}
Ptr<DescriptorExtractor> Desc::create(Desc::Code code)
{
    switch (code)
    {
        case Desc::ORB:
            return new cv::OrbDescriptorExtractor();
            break;
        case Desc::BRIEF:
            //return  new cv::BriefDescriptorExtractor();
            return new Brief();
            break;
        case Desc::SIFT:
            return new cv::SiftDescriptorExtractor();
            break;
        case Desc::SURF:
            return new cv::SurfDescriptorExtractor();
            break;
        case Desc::FREAK:
            return DescriptorExtractor::create("FREAK");
        case Desc::BRISK:
            return DescriptorExtractor::create("BRISK");
        default:
            return new cv::BriefDescriptorExtractor();
            break;
    }
}