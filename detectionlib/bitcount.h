/**************************************************************************************************
 **************************************************************************************************
 
     BSD 3-Clause License (https://www.tldrlegal.com/l/bsd3)
     
     Copyright (c) 2013 Andrés Solís Montero <http://www.solism.ca>, All rights reserved.
     
     
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

#ifndef __FFlow__bitcount__
#define __FFlow__bitcount__

#include <iostream>


class BitCount
{
public:
    /*
     * Iterative bit counting. O(b) where b number of bits (int = 32) (long = 64)
     */
    static int countU(unsigned int  n)
    {
        int count = 0;
        while(n)
        {
            count += (int)(n & 0x1u);
            n >>= 1;
        }
        return count;
    }
    
    static int countL(unsigned long n)
    {
        int count = 0;
        while(n)
        {
            count += (int)(n & 0x1l);
            n >>= 1;
        }
        return count;
    }
    
    /*
     * Sparse ones. O(b1) where b1 is the number of 1's bit. 
     * Sparse Ones and Dense Ones were first described by Peter Wegner in 
     *"A Technique for Counting Ones in a Binary Computer", 
     * Communications of the ACM, Volume 3 (1960) Number 5, page 322.
     */
    static int countSOL(unsigned int n)
    {
        int count = 0;
        for (count = 0; n; ++count)
            n &= (n - 1);
        return count;
    }
    static int countSOL(unsigned long n)
    {
        int count = 0;
        while (n)
        {
            count++;
            n &= (n - 1);
        }
        return count;
    }

    static int countDOL(unsigned int n)
    {
        int count = 8 * sizeof(unsigned int);
        n ^= (unsigned int) - 1;
        while(n)
        {
            count --;
            n &= (n-1);
        }
        return count;
    }
    static int countDOL(unsigned long n)
    {
        int count = 8 * sizeof(unsigned long);
        n ^= (unsigned long) - 1;
        while(n)
        {
            count --;
            n &= (n-1);
        }
        return count;
    }

    /*
     * Pre computed look up table for 8bits and 16bits.
     * O(C) constant but space is compromised. for 8bits is 256 bytes of memory.
     * and for 16 bits is 64KB of 
     */
    static const char bits_in_char[256];
    static const char bits_in_char16[0x1u << 16];
    
    static int countPRE8U(unsigned int n)
    {
        return  bits_in_char[n & 0xffu] +
                bits_in_char[(n >> 8)  & 0xffu] +
                bits_in_char[(n >> 16) & 0xffu] +
                bits_in_char[(n >> 24) & 0xffu];
    }
    static int countPRE8L(unsigned long n)
    {
        return  bits_in_char[n & 0xffl] +
                bits_in_char[(n >> 8)  & 0xffl] +
                bits_in_char[(n >> 16) & 0xffl] +
                bits_in_char[(n >> 24) & 0xffl] +
                bits_in_char[(n >> 32) & 0xffl] +
                bits_in_char[(n >> 40) & 0xffl] +
                bits_in_char[(n >> 48) & 0xffl] +
                bits_in_char[(n >> 56) & 0xffl];
    }

    static int countPRE16U(unsigned int n)
    {
        return  bits_in_char16[n & 0xffffu] +
                bits_in_char16[(n >> 16) & 0xffffu];
    }
    static int countPRE16L(unsigned long n)
    {
        return  bits_in_char16[n & 0xffffl] +
                bits_in_char16[(n >> 16) & 0xffffl] +
                bits_in_char16[(n >> 32) & 0xffffl]+
                bits_in_char16[(n >> 48) & 0xffffl] ;
    }

    
    /*
     * Parallel Bit Counting. Counting the number of bits every each pair of bits. Then 4bits, 8bits,
     * 16bits.... O(C) constant and space is reduced from look up tables. 
     */
      
    //01010101 01010101 01010101 01010101   (unsigned long)(-1)/3     2^(2^0) + 1
    #define MASK_55555555 (((unsigned int)(-1))/3)
    //00110011 00110011 00110011 00110011   (unsigned long)(-1)/5     2^(2^1) + 1
    #define MASK_33333333 (((unsigned int)(-1))/5)
    //00001111 00001111 00001111 00001111   (unsigned long)(-1)/17    2^(2^2) + 1
    #define MASK_0F0F0F0F (((unsigned int)(-1))/17)
    //11111111 00000000 11111111 00000000   (unsigned long)(-1)/257   2^(2^3) + 1
    #define MASK_00FF00FF (((unsigned int)(-1))/257)
    //00000000 00000000 11111111 11111111   (unsigned long)(-1)/65537 2^(2^4) + 1
    #define MASK_0000FFFF (((unsigned int)(-1))/65537)
    
    //01010101 01010101 01010101 01010101
    #define MASKL_55555555 (((unsigned long)(-1))/3)
    //00110011 00110011 00110011 00110011
    #define MASKL_33333333 (((unsigned long)(-1))/5)
    //00001111 00001111 00001111 00001111
    #define MASKL_0F0F0F0F (((unsigned long)(-1))/17)
    //11111111 00000000 11111111 00000000
    #define MASKL_00FF00FF (((unsigned long)(-1))/257)
    //00000000 00000000 11111111 11111111
    #define MASKL_0000FFFF (((unsigned long)(-1))/65537)
    //11111111 11111111 11111111 11111111
    #define MASKL_FFFFFFFF ((unsigned long)(-1))
    
    static int countParallelU(unsigned int n)
    {
        //for (int i = 0; i < 5; ++i)
        //  n = (n & mask[i]) + (( n >> shift[i]) & mask[i]);
        n = (n & MASK_55555555) + (( n >> 1) & MASK_55555555);
        n = (n & MASK_33333333) + (( n >> 2) & MASK_33333333);
        n = (n & MASK_0F0F0F0F) + (( n >> 4) & MASK_0F0F0F0F);
        n = (n & MASK_00FF00FF) + (( n >> 8) & MASK_00FF00FF);
        n = (n & MASK_0000FFFF) + (( n >> 16)& MASK_0000FFFF);
        return n;
    }
    static int countParallelL(unsigned long n)
    {
        //for (int i = 0; i < 6; ++i)
        //  n = (n & maskL[i]) + (( n >> shift[i]) & maskL[i]);
        n = (n & MASKL_55555555) + (( n >> 1) & MASKL_55555555);
        n = (n & MASKL_33333333) + (( n >> 2) & MASKL_33333333);
        n = (n & MASKL_0F0F0F0F) + (( n >> 4) & MASKL_0F0F0F0F);
        n = (n & MASKL_00FF00FF) + (( n >> 8) & MASKL_00FF00FF);
        n = (n & MASKL_0000FFFF) + (( n >> 16)& MASKL_0000FFFF);
        n = (n & MASKL_FFFFFFFF) + (( n >> 32)& MASKL_FFFFFFFF);
        return (int)n;
    }
    
    /*
     * Nifty Parallel.
     * According to Don Knuth (The Art of Computer Programming Vol IV, p 11), in the first textbook on
     * programming, The Preparation of Programs for an Electronic Digital Computer by Wilkes, Wheeler and 
     * Gill (1957, reprinted 1984), pages 191--193 presented Nifty Parallel Count by D B Gillies and J C P 
     * Miller.
     */
    static int countNiftyU(unsigned int n)
    {
        n = (n & MASK_55555555) + (( n >> 1) & MASK_55555555);
        n = (n & MASK_33333333) + (( n >> 2) & MASK_33333333);
        n = (n & MASK_0F0F0F0F) + (( n >> 4) & MASK_0F0F0F0F);
        return n % 255;
    }
    

    static int countNiftyL(unsigned long n)
    {
        n = (n & MASKL_55555555) + (( n >> 1) & MASKL_55555555);
        n = (n & MASKL_33333333) + (( n >> 2) & MASKL_33333333);
        n = (n & MASKL_0F0F0F0F) + (( n >> 4) & MASKL_0F0F0F0F);
        return (int)(n % 255);
    }
    
    /* 
     *  MIT HAKMEN -only works for 32 bits-
     */
    static int countHAKMEMU(unsigned int n)
    {
        unsigned int tmp;
        tmp = n - ((n >> 1) & 033333333333)
                - ((n >> 2) & 011111111111);
        return ((tmp + (tmp >> 3)) & 030707070707) % 63;
    }

};

#endif /* defined(__FFlow__bitcount__) */
