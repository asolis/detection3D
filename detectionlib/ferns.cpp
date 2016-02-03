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
#include "ferns.h"
#include <limits>



const char FernUtils::bits[256][8] = {
    {4,12,20,28,36,44,52,60},
    {4,12,20,28,36,44,52,56},
    {4,12,20,28,36,44,48,60},
    {4,12,20,28,36,44,48,56},
    {4,12,20,28,36,40,52,60},
    {4,12,20,28,36,40,52,56},
    {4,12,20,28,36,40,48,60},
    {4,12,20,28,36,40,48,56},
    {4,12,20,28,32,44,52,60},
    {4,12,20,28,32,44,52,56},
    {4,12,20,28,32,44,48,60},
    {4,12,20,28,32,44,48,56},
    {4,12,20,28,32,40,52,60},
    {4,12,20,28,32,40,52,56},
    {4,12,20,28,32,40,48,60},
    {4,12,20,28,32,40,48,56},
    {4,12,20,24,36,44,52,60},
    {4,12,20,24,36,44,52,56},
    {4,12,20,24,36,44,48,60},
    {4,12,20,24,36,44,48,56},
    {4,12,20,24,36,40,52,60},
    {4,12,20,24,36,40,52,56},
    {4,12,20,24,36,40,48,60},
    {4,12,20,24,36,40,48,56},
    {4,12,20,24,32,44,52,60},
    {4,12,20,24,32,44,52,56},
    {4,12,20,24,32,44,48,60},
    {4,12,20,24,32,44,48,56},
    {4,12,20,24,32,40,52,60},
    {4,12,20,24,32,40,52,56},
    {4,12,20,24,32,40,48,60},
    {4,12,20,24,32,40,48,56},
    {4,12,16,28,36,44,52,60},
    {4,12,16,28,36,44,52,56},
    {4,12,16,28,36,44,48,60},
    {4,12,16,28,36,44,48,56},
    {4,12,16,28,36,40,52,60},
    {4,12,16,28,36,40,52,56},
    {4,12,16,28,36,40,48,60},
    {4,12,16,28,36,40,48,56},
    {4,12,16,28,32,44,52,60},
    {4,12,16,28,32,44,52,56},
    {4,12,16,28,32,44,48,60},
    {4,12,16,28,32,44,48,56},
    {4,12,16,28,32,40,52,60},
    {4,12,16,28,32,40,52,56},
    {4,12,16,28,32,40,48,60},
    {4,12,16,28,32,40,48,56},
    {4,12,16,24,36,44,52,60},
    {4,12,16,24,36,44,52,56},
    {4,12,16,24,36,44,48,60},
    {4,12,16,24,36,44,48,56},
    {4,12,16,24,36,40,52,60},
    {4,12,16,24,36,40,52,56},
    {4,12,16,24,36,40,48,60},
    {4,12,16,24,36,40,48,56},
    {4,12,16,24,32,44,52,60},
    {4,12,16,24,32,44,52,56},
    {4,12,16,24,32,44,48,60},
    {4,12,16,24,32,44,48,56},
    {4,12,16,24,32,40,52,60},
    {4,12,16,24,32,40,52,56},
    {4,12,16,24,32,40,48,60},
    {4,12,16,24,32,40,48,56},
    {4,8,20,28,36,44,52,60},
    {4,8,20,28,36,44,52,56},
    {4,8,20,28,36,44,48,60},
    {4,8,20,28,36,44,48,56},
    {4,8,20,28,36,40,52,60},
    {4,8,20,28,36,40,52,56},
    {4,8,20,28,36,40,48,60},
    {4,8,20,28,36,40,48,56},
    {4,8,20,28,32,44,52,60},
    {4,8,20,28,32,44,52,56},
    {4,8,20,28,32,44,48,60},
    {4,8,20,28,32,44,48,56},
    {4,8,20,28,32,40,52,60},
    {4,8,20,28,32,40,52,56},
    {4,8,20,28,32,40,48,60},
    {4,8,20,28,32,40,48,56},
    {4,8,20,24,36,44,52,60},
    {4,8,20,24,36,44,52,56},
    {4,8,20,24,36,44,48,60},
    {4,8,20,24,36,44,48,56},
    {4,8,20,24,36,40,52,60},
    {4,8,20,24,36,40,52,56},
    {4,8,20,24,36,40,48,60},
    {4,8,20,24,36,40,48,56},
    {4,8,20,24,32,44,52,60},
    {4,8,20,24,32,44,52,56},
    {4,8,20,24,32,44,48,60},
    {4,8,20,24,32,44,48,56},
    {4,8,20,24,32,40,52,60},
    {4,8,20,24,32,40,52,56},
    {4,8,20,24,32,40,48,60},
    {4,8,20,24,32,40,48,56},
    {4,8,16,28,36,44,52,60},
    {4,8,16,28,36,44,52,56},
    {4,8,16,28,36,44,48,60},
    {4,8,16,28,36,44,48,56},
    {4,8,16,28,36,40,52,60},
    {4,8,16,28,36,40,52,56},
    {4,8,16,28,36,40,48,60},
    {4,8,16,28,36,40,48,56},
    {4,8,16,28,32,44,52,60},
    {4,8,16,28,32,44,52,56},
    {4,8,16,28,32,44,48,60},
    {4,8,16,28,32,44,48,56},
    {4,8,16,28,32,40,52,60},
    {4,8,16,28,32,40,52,56},
    {4,8,16,28,32,40,48,60},
    {4,8,16,28,32,40,48,56},
    {4,8,16,24,36,44,52,60},
    {4,8,16,24,36,44,52,56},
    {4,8,16,24,36,44,48,60},
    {4,8,16,24,36,44,48,56},
    {4,8,16,24,36,40,52,60},
    {4,8,16,24,36,40,52,56},
    {4,8,16,24,36,40,48,60},
    {4,8,16,24,36,40,48,56},
    {4,8,16,24,32,44,52,60},
    {4,8,16,24,32,44,52,56},
    {4,8,16,24,32,44,48,60},
    {4,8,16,24,32,44,48,56},
    {4,8,16,24,32,40,52,60},
    {4,8,16,24,32,40,52,56},
    {4,8,16,24,32,40,48,60},
    {4,8,16,24,32,40,48,56},
    {0,12,20,28,36,44,52,60},
    {0,12,20,28,36,44,52,56},
    {0,12,20,28,36,44,48,60},
    {0,12,20,28,36,44,48,56},
    {0,12,20,28,36,40,52,60},
    {0,12,20,28,36,40,52,56},
    {0,12,20,28,36,40,48,60},
    {0,12,20,28,36,40,48,56},
    {0,12,20,28,32,44,52,60},
    {0,12,20,28,32,44,52,56},
    {0,12,20,28,32,44,48,60},
    {0,12,20,28,32,44,48,56},
    {0,12,20,28,32,40,52,60},
    {0,12,20,28,32,40,52,56},
    {0,12,20,28,32,40,48,60},
    {0,12,20,28,32,40,48,56},
    {0,12,20,24,36,44,52,60},
    {0,12,20,24,36,44,52,56},
    {0,12,20,24,36,44,48,60},
    {0,12,20,24,36,44,48,56},
    {0,12,20,24,36,40,52,60},
    {0,12,20,24,36,40,52,56},
    {0,12,20,24,36,40,48,60},
    {0,12,20,24,36,40,48,56},
    {0,12,20,24,32,44,52,60},
    {0,12,20,24,32,44,52,56},
    {0,12,20,24,32,44,48,60},
    {0,12,20,24,32,44,48,56},
    {0,12,20,24,32,40,52,60},
    {0,12,20,24,32,40,52,56},
    {0,12,20,24,32,40,48,60},
    {0,12,20,24,32,40,48,56},
    {0,12,16,28,36,44,52,60},
    {0,12,16,28,36,44,52,56},
    {0,12,16,28,36,44,48,60},
    {0,12,16,28,36,44,48,56},
    {0,12,16,28,36,40,52,60},
    {0,12,16,28,36,40,52,56},
    {0,12,16,28,36,40,48,60},
    {0,12,16,28,36,40,48,56},
    {0,12,16,28,32,44,52,60},
    {0,12,16,28,32,44,52,56},
    {0,12,16,28,32,44,48,60},
    {0,12,16,28,32,44,48,56},
    {0,12,16,28,32,40,52,60},
    {0,12,16,28,32,40,52,56},
    {0,12,16,28,32,40,48,60},
    {0,12,16,28,32,40,48,56},
    {0,12,16,24,36,44,52,60},
    {0,12,16,24,36,44,52,56},
    {0,12,16,24,36,44,48,60},
    {0,12,16,24,36,44,48,56},
    {0,12,16,24,36,40,52,60},
    {0,12,16,24,36,40,52,56},
    {0,12,16,24,36,40,48,60},
    {0,12,16,24,36,40,48,56},
    {0,12,16,24,32,44,52,60},
    {0,12,16,24,32,44,52,56},
    {0,12,16,24,32,44,48,60},
    {0,12,16,24,32,44,48,56},
    {0,12,16,24,32,40,52,60},
    {0,12,16,24,32,40,52,56},
    {0,12,16,24,32,40,48,60},
    {0,12,16,24,32,40,48,56},
    {0,8,20,28,36,44,52,60},
    {0,8,20,28,36,44,52,56},
    {0,8,20,28,36,44,48,60},
    {0,8,20,28,36,44,48,56},
    {0,8,20,28,36,40,52,60},
    {0,8,20,28,36,40,52,56},
    {0,8,20,28,36,40,48,60},
    {0,8,20,28,36,40,48,56},
    {0,8,20,28,32,44,52,60},
    {0,8,20,28,32,44,52,56},
    {0,8,20,28,32,44,48,60},
    {0,8,20,28,32,44,48,56},
    {0,8,20,28,32,40,52,60},
    {0,8,20,28,32,40,52,56},
    {0,8,20,28,32,40,48,60},
    {0,8,20,28,32,40,48,56},
    {0,8,20,24,36,44,52,60},
    {0,8,20,24,36,44,52,56},
    {0,8,20,24,36,44,48,60},
    {0,8,20,24,36,44,48,56},
    {0,8,20,24,36,40,52,60},
    {0,8,20,24,36,40,52,56},
    {0,8,20,24,36,40,48,60},
    {0,8,20,24,36,40,48,56},
    {0,8,20,24,32,44,52,60},
    {0,8,20,24,32,44,52,56},
    {0,8,20,24,32,44,48,60},
    {0,8,20,24,32,44,48,56},
    {0,8,20,24,32,40,52,60},
    {0,8,20,24,32,40,52,56},
    {0,8,20,24,32,40,48,60},
    {0,8,20,24,32,40,48,56},
    {0,8,16,28,36,44,52,60},
    {0,8,16,28,36,44,52,56},
    {0,8,16,28,36,44,48,60},
    {0,8,16,28,36,44,48,56},
    {0,8,16,28,36,40,52,60},
    {0,8,16,28,36,40,52,56},
    {0,8,16,28,36,40,48,60},
    {0,8,16,28,36,40,48,56},
    {0,8,16,28,32,44,52,60},
    {0,8,16,28,32,44,52,56},
    {0,8,16,28,32,44,48,60},
    {0,8,16,28,32,44,48,56},
    {0,8,16,28,32,40,52,60},
    {0,8,16,28,32,40,52,56},
    {0,8,16,28,32,40,48,60},
    {0,8,16,28,32,40,48,56},
    {0,8,16,24,36,44,52,60},
    {0,8,16,24,36,44,52,56},
    {0,8,16,24,36,44,48,60},
    {0,8,16,24,36,44,48,56},
    {0,8,16,24,36,40,52,60},
    {0,8,16,24,36,40,52,56},
    {0,8,16,24,36,40,48,60},
    {0,8,16,24,36,40,48,56},
    {0,8,16,24,32,44,52,60},
    {0,8,16,24,32,44,52,56},
    {0,8,16,24,32,44,48,60},
    {0,8,16,24,32,44,48,56},
    {0,8,16,24,32,40,52,60},
    {0,8,16,24,32,40,52,56},
    {0,8,16,24,32,40,48,60},
    {0,8,16,24,32,40,48,56}
};
const uint64_t FernUtils::dict[256] = {
    0xF0F0F0F0F0F0F0F0,
    0x0FF0F0F0F0F0F0F0,
    0xF00FF0F0F0F0F0F0,
    0x0F0FF0F0F0F0F0F0,
    0xF0F00FF0F0F0F0F0,
    0x0FF00FF0F0F0F0F0,
    0xF00F0FF0F0F0F0F0,
    0x0F0F0FF0F0F0F0F0,
    0xF0F0F00FF0F0F0F0,
    0x0FF0F00FF0F0F0F0,
    0xF00FF00FF0F0F0F0,
    0x0F0FF00FF0F0F0F0,
    0xF0F00F0FF0F0F0F0,
    0x0FF00F0FF0F0F0F0,
    0xF00F0F0FF0F0F0F0,
    0x0F0F0F0FF0F0F0F0,
    0xF0F0F0F00FF0F0F0,
    0x0FF0F0F00FF0F0F0,
    0xF00FF0F00FF0F0F0,
    0x0F0FF0F00FF0F0F0,
    0xF0F00FF00FF0F0F0,
    0x0FF00FF00FF0F0F0,
    0xF00F0FF00FF0F0F0,
    0x0F0F0FF00FF0F0F0,
    0xF0F0F00F0FF0F0F0,
    0x0FF0F00F0FF0F0F0,
    0xF00FF00F0FF0F0F0,
    0x0F0FF00F0FF0F0F0,
    0xF0F00F0F0FF0F0F0,
    0x0FF00F0F0FF0F0F0,
    0xF00F0F0F0FF0F0F0,
    0x0F0F0F0F0FF0F0F0,
    0xF0F0F0F0F00FF0F0,
    0x0FF0F0F0F00FF0F0,
    0xF00FF0F0F00FF0F0,
    0x0F0FF0F0F00FF0F0,
    0xF0F00FF0F00FF0F0,
    0x0FF00FF0F00FF0F0,
    0xF00F0FF0F00FF0F0,
    0x0F0F0FF0F00FF0F0,
    0xF0F0F00FF00FF0F0,
    0x0FF0F00FF00FF0F0,
    0xF00FF00FF00FF0F0,
    0x0F0FF00FF00FF0F0,
    0xF0F00F0FF00FF0F0,
    0x0FF00F0FF00FF0F0,
    0xF00F0F0FF00FF0F0,
    0x0F0F0F0FF00FF0F0,
    0xF0F0F0F00F0FF0F0,
    0x0FF0F0F00F0FF0F0,
    0xF00FF0F00F0FF0F0,
    0x0F0FF0F00F0FF0F0,
    0xF0F00FF00F0FF0F0,
    0x0FF00FF00F0FF0F0,
    0xF00F0FF00F0FF0F0,
    0x0F0F0FF00F0FF0F0,
    0xF0F0F00F0F0FF0F0,
    0x0FF0F00F0F0FF0F0,
    0xF00FF00F0F0FF0F0,
    0x0F0FF00F0F0FF0F0,
    0xF0F00F0F0F0FF0F0,
    0x0FF00F0F0F0FF0F0,
    0xF00F0F0F0F0FF0F0,
    0x0F0F0F0F0F0FF0F0,
    0xF0F0F0F0F0F00FF0,
    0x0FF0F0F0F0F00FF0,
    0xF00FF0F0F0F00FF0,
    0x0F0FF0F0F0F00FF0,
    0xF0F00FF0F0F00FF0,
    0x0FF00FF0F0F00FF0,
    0xF00F0FF0F0F00FF0,
    0x0F0F0FF0F0F00FF0,
    0xF0F0F00FF0F00FF0,
    0x0FF0F00FF0F00FF0,
    0xF00FF00FF0F00FF0,
    0x0F0FF00FF0F00FF0,
    0xF0F00F0FF0F00FF0,
    0x0FF00F0FF0F00FF0,
    0xF00F0F0FF0F00FF0,
    0x0F0F0F0FF0F00FF0,
    0xF0F0F0F00FF00FF0,
    0x0FF0F0F00FF00FF0,
    0xF00FF0F00FF00FF0,
    0x0F0FF0F00FF00FF0,
    0xF0F00FF00FF00FF0,
    0x0FF00FF00FF00FF0,
    0xF00F0FF00FF00FF0,
    0x0F0F0FF00FF00FF0,
    0xF0F0F00F0FF00FF0,
    0x0FF0F00F0FF00FF0,
    0xF00FF00F0FF00FF0,
    0x0F0FF00F0FF00FF0,
    0xF0F00F0F0FF00FF0,
    0x0FF00F0F0FF00FF0,
    0xF00F0F0F0FF00FF0,
    0x0F0F0F0F0FF00FF0,
    0xF0F0F0F0F00F0FF0,
    0x0FF0F0F0F00F0FF0,
    0xF00FF0F0F00F0FF0,
    0x0F0FF0F0F00F0FF0,
    0xF0F00FF0F00F0FF0,
    0x0FF00FF0F00F0FF0,
    0xF00F0FF0F00F0FF0,
    0x0F0F0FF0F00F0FF0,
    0xF0F0F00FF00F0FF0,
    0x0FF0F00FF00F0FF0,
    0xF00FF00FF00F0FF0,
    0x0F0FF00FF00F0FF0,
    0xF0F00F0FF00F0FF0,
    0x0FF00F0FF00F0FF0,
    0xF00F0F0FF00F0FF0,
    0x0F0F0F0FF00F0FF0,
    0xF0F0F0F00F0F0FF0,
    0x0FF0F0F00F0F0FF0,
    0xF00FF0F00F0F0FF0,
    0x0F0FF0F00F0F0FF0,
    0xF0F00FF00F0F0FF0,
    0x0FF00FF00F0F0FF0,
    0xF00F0FF00F0F0FF0,
    0x0F0F0FF00F0F0FF0,
    0xF0F0F00F0F0F0FF0,
    0x0FF0F00F0F0F0FF0,
    0xF00FF00F0F0F0FF0,
    0x0F0FF00F0F0F0FF0,
    0xF0F00F0F0F0F0FF0,
    0x0FF00F0F0F0F0FF0,
    0xF00F0F0F0F0F0FF0,
    0x0F0F0F0F0F0F0FF0,
    0xF0F0F0F0F0F0F00F,
    0x0FF0F0F0F0F0F00F,
    0xF00FF0F0F0F0F00F,
    0x0F0FF0F0F0F0F00F,
    0xF0F00FF0F0F0F00F,
    0x0FF00FF0F0F0F00F,
    0xF00F0FF0F0F0F00F,
    0x0F0F0FF0F0F0F00F,
    0xF0F0F00FF0F0F00F,
    0x0FF0F00FF0F0F00F,
    0xF00FF00FF0F0F00F,
    0x0F0FF00FF0F0F00F,
    0xF0F00F0FF0F0F00F,
    0x0FF00F0FF0F0F00F,
    0xF00F0F0FF0F0F00F,
    0x0F0F0F0FF0F0F00F,
    0xF0F0F0F00FF0F00F,
    0x0FF0F0F00FF0F00F,
    0xF00FF0F00FF0F00F,
    0x0F0FF0F00FF0F00F,
    0xF0F00FF00FF0F00F,
    0x0FF00FF00FF0F00F,
    0xF00F0FF00FF0F00F,
    0x0F0F0FF00FF0F00F,
    0xF0F0F00F0FF0F00F,
    0x0FF0F00F0FF0F00F,
    0xF00FF00F0FF0F00F,
    0x0F0FF00F0FF0F00F,
    0xF0F00F0F0FF0F00F,
    0x0FF00F0F0FF0F00F,
    0xF00F0F0F0FF0F00F,
    0x0F0F0F0F0FF0F00F,
    0xF0F0F0F0F00FF00F,
    0x0FF0F0F0F00FF00F,
    0xF00FF0F0F00FF00F,
    0x0F0FF0F0F00FF00F,
    0xF0F00FF0F00FF00F,
    0x0FF00FF0F00FF00F,
    0xF00F0FF0F00FF00F,
    0x0F0F0FF0F00FF00F,
    0xF0F0F00FF00FF00F,
    0x0FF0F00FF00FF00F,
    0xF00FF00FF00FF00F,
    0x0F0FF00FF00FF00F,
    0xF0F00F0FF00FF00F,
    0x0FF00F0FF00FF00F,
    0xF00F0F0FF00FF00F,
    0x0F0F0F0FF00FF00F,
    0xF0F0F0F00F0FF00F,
    0x0FF0F0F00F0FF00F,
    0xF00FF0F00F0FF00F,
    0x0F0FF0F00F0FF00F,
    0xF0F00FF00F0FF00F,
    0x0FF00FF00F0FF00F,
    0xF00F0FF00F0FF00F,
    0x0F0F0FF00F0FF00F,
    0xF0F0F00F0F0FF00F,
    0x0FF0F00F0F0FF00F,
    0xF00FF00F0F0FF00F,
    0x0F0FF00F0F0FF00F,
    0xF0F00F0F0F0FF00F,
    0x0FF00F0F0F0FF00F,
    0xF00F0F0F0F0FF00F,
    0x0F0F0F0F0F0FF00F,
    0xF0F0F0F0F0F00F0F,
    0x0FF0F0F0F0F00F0F,
    0xF00FF0F0F0F00F0F,
    0x0F0FF0F0F0F00F0F,
    0xF0F00FF0F0F00F0F,
    0x0FF00FF0F0F00F0F,
    0xF00F0FF0F0F00F0F,
    0x0F0F0FF0F0F00F0F,
    0xF0F0F00FF0F00F0F,
    0x0FF0F00FF0F00F0F,
    0xF00FF00FF0F00F0F,
    0x0F0FF00FF0F00F0F,
    0xF0F00F0FF0F00F0F,
    0x0FF00F0FF0F00F0F,
    0xF00F0F0FF0F00F0F,
    0x0F0F0F0FF0F00F0F,
    0xF0F0F0F00FF00F0F,
    0x0FF0F0F00FF00F0F,
    0xF00FF0F00FF00F0F,
    0x0F0FF0F00FF00F0F,
    0xF0F00FF00FF00F0F,
    0x0FF00FF00FF00F0F,
    0xF00F0FF00FF00F0F,
    0x0F0F0FF00FF00F0F,
    0xF0F0F00F0FF00F0F,
    0x0FF0F00F0FF00F0F,
    0xF00FF00F0FF00F0F,
    0x0F0FF00F0FF00F0F,
    0xF0F00F0F0FF00F0F,
    0x0FF00F0F0FF00F0F,
    0xF00F0F0F0FF00F0F,
    0x0F0F0F0F0FF00F0F,
    0xF0F0F0F0F00F0F0F,
    0x0FF0F0F0F00F0F0F,
    0xF00FF0F0F00F0F0F,
    0x0F0FF0F0F00F0F0F,
    0xF0F00FF0F00F0F0F,
    0x0FF00FF0F00F0F0F,
    0xF00F0FF0F00F0F0F,
    0x0F0F0FF0F00F0F0F,
    0xF0F0F00FF00F0F0F,
    0x0FF0F00FF00F0F0F,
    0xF00FF00FF00F0F0F,
    0x0F0FF00FF00F0F0F,
    0xF0F00F0FF00F0F0F,
    0x0FF00F0FF00F0F0F,
    0xF00F0F0FF00F0F0F,
    0x0F0F0F0FF00F0F0F,
    0xF0F0F0F00F0F0F0F,
    0x0FF0F0F00F0F0F0F,
    0xF00FF0F00F0F0F0F,
    0x0F0FF0F00F0F0F0F,
    0xF0F00FF00F0F0F0F,
    0x0FF00FF00F0F0F0F,
    0xF00F0FF00F0F0F0F,
    0x0F0F0FF00F0F0F0F,
    0xF0F0F00F0F0F0F0F,
    0x0FF0F00F0F0F0F0F,
    0xF00FF00F0F0F0F0F,
    0x0F0FF00F0F0F0F0F,
    0xF0F00F0F0F0F0F0F,
    0x0FF00F0F0F0F0F0F,
    0xF00F0F0F0F0F0F0F,
    0x0F0F0F0F0F0F0F0F
};

/*
 *  util function to order two pairs.
 */
bool compareIndexPairs( const  std::pair<uint16_t, uint32_t> &l,
                       const  std::pair<uint16_t, uint32_t> &r)
{
    return l.second > r. second;
}




/*
 *  Util function to check the homography
 *  From "Multiple View Geometry"
 *  If the determinant of the top-left 2x2 matrix is > 0 the transformation is orientation
 *  preserving. Else is <0 it's orientation reversing (bad homography).
 */
bool FernUtils::goodHomography(const cv::Mat &H)
{
    const double *Hr0 = H.ptr<double>(0);
    const double *Hr1 = H.ptr<double>(1);
    const double *Hr2 = H.ptr<double>(2);
    
    const double det = Hr0[0] * Hr1[1] - Hr1[0] * Hr0[1];
    if (det < 0)
        return false;
    
    const double N1 = sqrt(Hr0[0] * Hr0[0] + Hr1[0] * Hr1[0]);
    if (N1 > 4 || N1 < 0.1)
        return false;
    
    const double N2 = sqrt(Hr0[1] * Hr0[1] + Hr1[1] * Hr1[1]);
    if (N2 > 4 || N2 < 0.1)
        return false;
    
    const double N3 = sqrt(Hr2[0] * Hr2[0] + Hr2[1] * Hr2[1]);
    if (N3 > 0.002)
        return false;
    
    return true;
}

int FernUtils::numInliers(const vector<Point2f> &obj_pts,
                          const vector<Point2f> &scn_pts)
{
    int amountInliers = 0;
    if (obj_pts.size() > MIN_NUM_MATCHES)
    {
        Mat inliers;
        Mat H = Prosac::findHomography(obj_pts, scn_pts, 3, inliers);
        amountInliers = countNonZero(inliers);
        amountInliers = (FernUtils::goodHomography(H))? amountInliers : 0;
    }
    return amountInliers;
}

int FernUtils::computeHomography(const vector<Point2f> &obj_pts,
                                 const vector<Point2f> &scn_pts,
                                 const vector<Point2f> &corners,
                                 Mat &H,
                                 vector<Point2f> &dstCorners)
{
    int amountInliers = 0;
    if (obj_pts.size() > MIN_NUM_MATCHES)
    {
        Mat inliers;
        //H = Prosac::findHomography(obj_pts, scn_pts, 3, inliers);
        H = findHomography(obj_pts, scn_pts, CV_RANSAC, 3, inliers);
        amountInliers = countNonZero(inliers);
        perspectiveTransform(corners, dstCorners, H);
        bool convex = isContourConvex(dstCorners);
        bool goodH  = FernUtils::goodHomography(H);
        amountInliers = (convex & goodH)? amountInliers : 0;
    }
#ifdef STATS
    cout << "Homography inliers :" << amountInliers << " Matches: " << obj_pts.size() << endl;
#endif
    return amountInliers;
}


bool FernUtils::sortByQueryId(const DMatch &left, const DMatch &right)
{
    if (left.trainIdx == right.trainIdx)
    {
        return left.distance > right.distance;
    }
    else return left.trainIdx < right.trainIdx;
}
bool FernUtils::sortByDistance(const DMatch &left, const DMatch &right)
{
    return left.distance < right.distance;
}

Point FernUtils::pointOnALine(const Vec3f &line, const Point2f &pt)
{
    float a = line[0];
    float b = line[1];
    float c = line[2];
    
    if (a == 0)
        return Point(cvRound(pt.x), cvRound(-c)/b);
    if (b == 0)
        return Point(cvRound(-c)/a, cvRound(pt.y));
    
    float slope = line[1]/line[0];
    float c2 = pt.y - slope * pt.x;
    
    float x = (c2 + c/b) / ( -(a/b) - (b/a));
    float y = slope * x + c2;
    
    return Point(cvRound(x), cvRound(y));
}

Point FernUtils::twoLinesIntersection(const Point2f &p1,
                                      const Point2f &p2,
                                      const Point2f &p3,
                                      const Point2f &p4)
{
    Point inter(-1,-1);
    double det = (p1.x - p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x);
    if (det != 0)
    {
        double x =  (p1.x * p2.y - p1.y * p2.x) * ( p3.x - p4.x) -
        (p3.x * p4.y - p3.y * p4.x) * ( p1.x - p2.x);
        x /= det;
        double y =  (p1.x * p2.y - p1.y * p2.x) * ( p3.y - p4.y) -
        (p3.x * p4.y - p3.y * p4.x) * ( p1.y - p2.y);
        y /= det;
        
        inter.x = x;
        inter.y = y;
    }
    return inter;
}

static void crossProdut(const Vec3f &a, const Vec3f &b, Vec3f &pt)
{
    
    pt[0] =    a[1] * b[2] - a[2] * b[1];
    pt[1] =    a[2] * b[0] - a[0] * b[2];
    pt[2] =    a[0] * b[1] - a[1] * b[0];
}

double FernUtils::radiusOfInscribedCircled(const Point center, const vector<Point2f> &corners)
{
    Vec3f line1 = getLine(corners[0], corners[1]);
    Vec3f line2 = getLine(corners[1], corners[2]);
    Vec3f line3 = getLine(corners[2], corners[3]);
    Vec3f line4 = getLine(corners[3], corners[0]);
    Point2f centerF(center.x, center.y);
    Point cLine1 = pointOnALine(line1, centerF);
    Point cLine2 = pointOnALine(line2, centerF);
    Point cLine3 = pointOnALine(line3, centerF);
    Point cLine4 = pointOnALine(line4, centerF);
    double d1 = norm(center - cLine1);
    double d2 = norm(center - cLine2);
    double d3 = norm(center - cLine3);
    double d4 = norm(center - cLine4);
    return  min(min(min(d1, d2),d3), d4);
}

Vec3f FernUtils::getLine(const Point2f &p1, const Point2f &p2)
{
    double den = (p2.x - p1.x);
    
    double A =  (den == 0) ? 1 : - (p2.y - p1.y)/den;
    double B =  (den == 0) ? 0 : 1;
    double C =  (den == 0) ? - p1.x :  - ( B * p1.y + A * p1.x);
    return Vec3f( A, B, C );
}

int FernUtils::computeFundamentalMatrix(const vector<Point2f> &obj_pts,
                                        const vector<Point2f> &scn_pts,
                                        const vector<Point2f> &corners,
                                        Mat &F,
                                        vector<Point2f> &obj_inliers,
                                        vector<Point2f> &scn_inliers,
                                        vector<Vec3f> &cLines)
{
    vector<uchar> outliers;
    F = findFundamentalMat(obj_pts, scn_pts, outliers);
    
    int count = 0;
    for (int i = 0; i < outliers.size(); ++i )
    {
        if (outliers[i] == 0)
            continue;
        else
        {
            obj_inliers.push_back(obj_pts[i]);
            scn_inliers.push_back(scn_pts[i]);
            count++;
        }
    }
#ifdef STATS
    cout << "Fundamental Inliers :" << count << " Matches: " << obj_pts.size() << endl;
#endif
    computeCorrespondEpilines(corners, 1, F, cLines);
    return count;
}



void FernUtils::drawCorners(Mat &frameOut,
                        const vector<Point2f> &corners,
                        Scalar &color,
                        Point2f &shift,
                        int thickness)
{
    line( frameOut, corners[0] + shift, corners[1] + shift, color, thickness );
    line( frameOut, corners[1] + shift, corners[2] + shift, color, thickness );
    line( frameOut, corners[2] + shift, corners[3] + shift, color, thickness );
    line( frameOut, corners[3] + shift, corners[0] + shift, color, thickness );
}

void FernUtils::drawPoints(Mat &frameOut,
                       const vector<Point2f> &corners,
                       Scalar &color,
                       Point2f &shift,
                       int thickness)
{
    line( frameOut, corners[0] + shift, corners[0] + shift, color, thickness );
    line( frameOut, corners[1] + shift, corners[1] + shift, color, thickness );
    line( frameOut, corners[2] + shift, corners[2] + shift, color, thickness );
    line( frameOut, corners[3] + shift, corners[3] + shift, color, thickness );
}

void FernUtils::drawELines(Mat &frameOut,
                       const vector<Vec3f> &eLines,
                       Scalar &color,
                       Point2f &shift,
                       int width )
{
    for (size_t i = 0; i < eLines.size(); i++)
    {
        const Vec3f &l = eLines[i];
        Point2f one(0, -l[2]/l[1]);
        Point2f two(width, -(l[2] + l[0] * width)/l[1]);
        line(frameOut, one + shift, two + shift, color);
    }
}
void FernUtils::refineCornersWithEpipolarLines(const vector<Vec3f> &eLines,
                                           const vector<Point2f> &corners,
                                           vector<Point2f> &refined)
{
    
    for (int i = 0; i < eLines.size(); ++i)
    {
        const Vec3f &line = eLines[i];
        const Point2f &pt = corners[i];
        Point rPt = FernUtils::pointOnALine(line, pt);
        refined.push_back(rPt);
    }
}


void FernUtils::drawObject(Mat &frameOut,
                 const vector<Point2f> &corners,
                 Scalar &color,
                 Scalar &topColor,
                 Point2f &shift,
                 float percent,
                 int thickness,
                 float alpha)
{
    Point2f center  = FernUtils::twoLinesIntersection(corners[0], corners[2], corners[1], corners[3]);
    double minR   = FernUtils::radiusOfInscribedCircled(center, corners);
    Vec3f topLine = FernUtils::getLine(corners[2], corners[3]);
    Point2f top   = FernUtils::pointOnALine(topLine, center);
    circle(frameOut, center + shift , minR, color,thickness/2);
    line(frameOut, center + shift, top + shift, topColor, thickness);
    drawPercentCircle(frameOut, center+ shift, minR,percent, alpha);
}


void FernUtils::drawPercentCircle(Mat &frameOut,
                              const Point2f &center,
                              int radius,
                              float percent,
                              float alpha)
{
    int x,y,r2;
    r2 = radius * radius;
    Vec3b color;
    color[0] = 255;
    color[1] = 0;
    color[2] = 255;
    uchar *data;
    if (center.x < 0 || center.y < 0)
        return;
    for (y = 0; y < radius; ++y)
    {
        for (x = y; x < radius; ++x)
        {
            if ( (x * x) + (y * y) < r2 )
            {
                if (percent >= 12.5 * 0)
                {
                    if (center.y - x > frameOut.rows)
                        continue;
                    data = frameOut.ptr<uchar>(center.y - x , center.x + y); //1
                    data[0] = color[0]*alpha + data[0]*(1-alpha);
                    data[1] = color[1]*alpha + data[1]*(1-alpha);
                    data[2] = color[2]*alpha + data[2]*(1-alpha);
                }
                if (percent >= 12.5 * 1)
                {

                    if (center.y - y > frameOut.rows)
                        continue;
                    data = frameOut.ptr<uchar>(center.y - y , center.x + x); //2
                    data[0] = color[0]*alpha + data[0]*(1-alpha);
                    data[1] = color[1]*alpha + data[1]*(1-alpha);
                    data[2] = color[2]*alpha + data[2]*(1-alpha);
                }
                if (percent >= 12.5 * 2)
                {
                    if (center.y + y > frameOut.rows)
                        continue;
                    data = frameOut.ptr<uchar>(center.y + y , center.x + x); //3
                    data[0] = color[0]*alpha + data[0]*(1-alpha);
                    data[1] = color[1]*alpha + data[1]*(1-alpha);
                    data[2] = color[2]*alpha + data[2]*(1-alpha);
                }
                if (percent >= 12.5 * 3)
                {
                    if (center.y + x > frameOut.rows)
                        continue;
                    data = frameOut.ptr<uchar>(center.y + x , center.x + y); //4
                    data[0] = color[0]*alpha + data[0]*(1-alpha);
                    data[1] = color[1]*alpha + data[1]*(1-alpha);
                    data[2] = color[2]*alpha + data[2]*(1-alpha);
                }
                if (percent >= 12.5 * 4)
                {
                    if (center.y + x > frameOut.rows)
                        continue;
                    data = frameOut.ptr<uchar>(center.y + x , center.x - y); //5
                    data[0] = color[0]*alpha + data[0]*(1-alpha);
                    data[1] = color[1]*alpha + data[1]*(1-alpha);
                    data[2] = color[2]*alpha + data[2]*(1-alpha);
                }
                if (percent >= 12.5 * 5)
                {
                    if (center.y + y > frameOut.rows)
                        continue;
                    data = frameOut.ptr<uchar>(center.y + y , center.x - x); //6
                    data[0] = color[0]*alpha + data[0]*(1-alpha);
                    data[1] = color[1]*alpha + data[1]*(1-alpha);
                    data[2] = color[2]*alpha + data[2]*(1-alpha);
                }
                if (percent >= 12.5 * 6)
                {
                    if (center.y - y > frameOut.rows)
                        continue;
                    data = frameOut.ptr<uchar>(center.y - y , center.x + -x); //7
                    data[0] = color[0]*alpha + data[0]*(1-alpha);
                    data[1] = color[1]*alpha + data[1]*(1-alpha);
                    data[2] = color[2]*alpha + data[2]*(1-alpha);
                }
                if (percent >= 12.5 * 7)
                {
                    if (center.y - x > frameOut.rows)
                        continue;
                    data = frameOut.ptr<uchar>(center.y - x , center.x + -y); //8
                    data[0] = color[0]*alpha + data[0]*(1-alpha);
                    data[1] = color[1]*alpha + data[1]*(1-alpha);
                    data[2] = color[2]*alpha + data[2]*(1-alpha);
                }
            }
        }
    }
}
