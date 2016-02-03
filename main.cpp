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


using namespace viva;
/*
 *  3D object recognition and localization using an uncalibrated camera
 */
int main(int argc, const char * argv[])
{
    vector<string> images;
    //images.push_back("model.bmp");
    
    Feat::Code feat = Feat::RFAST;
    Desc::Code desc = Desc::BRIEF;
    images.push_back("cup.orot.r/02.jpg");
    images.push_back("cup.orot.r/06.jpg");
    images.push_back("cup.orot.r/10.jpg");
    images.push_back("cup.orot.r/14.jpg");
    images.push_back("cup.orot.r/18.jpg");
    

//    Feat::Code feat = Feat::FAST;
//    Desc::Code desc = Desc::BRIEF;
//    images.push_back("cup.nrot/02.jpg");
//    images.push_back("cup.nrot/06.jpg");
//    images.push_back("cup.nrot/10.jpg");
//    images.push_back("cup.nrot/14.jpg");
//    images.push_back("cup.nrot/18.jpg");

    
//    Feat::Code feat = Feat::FAST;
//    Desc::Code desc = Desc::BRIEF;
//    images.push_back("car01/01.jpg");
//    images.push_back("car01/02.jpg");
//    images.push_back("car01/03.jpg");
//    images.push_back("car01/04.jpg");
//    images.push_back("car01/05.jpg");
//    images.push_back("car01/06.jpg");
    
//    Feat::Code feat = Feat::FAST;
//    Desc::Code desc = Desc::BRIEF;
//    images.push_back("car02/01.jpg");
//    images.push_back("car02/02.jpg");
//    images.push_back("car02/03.jpg");
//    images.push_back("car02/04.jpg");
//    images.push_back("car02/05.jpg");
//    images.push_back("car02/06.jpg");
//    images.push_back("car02/07.jpg");
//    images.push_back("car02/08.jpg");
    
//    Feat::Code feat = Feat::FAST;
//    Desc::Code desc = Desc::BRIEF;
//    images.push_back("car03/01.jpg");
//    images.push_back("car03/02.jpg");
//    images.push_back("car03/03.jpg");
//    images.push_back("car03/04.jpg");
//    images.push_back("car03/05.jpg");
//    images.push_back("car03/06.jpg");
//    images.push_back("car03/07.jpg");
//    images.push_back("car03/08.jpg");
//    images.push_back("car03/09.jpg");


//    Feat::Code feat = Feat::FAST;
//    Desc::Code desc = Desc::BRIEF;
//    images.push_back("toy/01.jpg");
//    images.push_back("toy/02.jpg");
//    images.push_back("toy/03.jpg");
//    images.push_back("toy/04.jpg");
//    images.push_back("toy/05.jpg");
//    images.push_back("toy/06.jpg");
//    images.push_back("toy/07.jpg");
    

    Ptr<Input> input = new CameraInput(0, Size(640,480));
    Ptr<ProcessFrame> process = new ObjectDetector(images, feat, desc);
    Ptr<Output> output = new VideoOutput("output.avi", Size(640,480));
    
  
    Processor processor;
    processor.setInput(input);
    processor.setProcess(process);
    processor.setOutput(output);
    processor.run();
    
    return 0;
}