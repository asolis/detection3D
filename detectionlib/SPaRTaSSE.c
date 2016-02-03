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
/* Includes */
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <limits.h>
#include <math.h>
#include <emmintrin.h>

/* Defines */
#define MEM_ALIGN               32
#define HSIZE                   (3*4*sizeof(float))
#define MIN_DELTA_CHNG          0.1
#define REL_CHNG(a, b)          (fabs((a) - (b))/(a))
#define CHNG_SIGNIFICANT(a, b)  (REL_CHNG(a, b) > MIN_DELTA_CHNG)



/* Data Structures */
typedef struct Flt2DPt{float x,y;} Flt2DPt;
typedef struct{
	Flt2DPt*  src;
	Flt2DPt*  dst;
	unsigned  N;
	float     maxD;
	unsigned  maxI;
	unsigned  rConvg;
	double    cfd;
	unsigned  minInl;
	float*    finalH;
	
	unsigned  i;        /* Iteration Number */
	unsigned  phNum;    /* Phase Number */
	unsigned  phEndI;   /* Phase End Iteration */
	double    phEndFpI; /* Phase floating-point End Iteration */
	unsigned  phMax;    /* Termination phase number */
	unsigned  bestInl;  /* Best    number of inliers */
	unsigned  inl;      /* Current number of inliers */
	unsigned* smpl;     /* Sample */
	float*    H;        /* Current homography */
	float*    bestH;    /* Best homography */
	Flt2DPt*  pkdPts;   /* Packed points */
	
	/* SPRT */
	double    t_M;      /* t_M */
	double    m_S;      /* m_S */
	double    epsilon;  /* Epsilon */
	double    delta;    /* delta */
	double    A;        /* SPRT Threshold */
	unsigned  N_tested; /* Number of points tested */
	int       good;     /* Good/bad flag */
	double    lambdaAccept;
	double    lambdaReject;
	double    lambdaTBL[16];
} PROSAC_HEST;



/* Prototypes */
static inline void*  almalloc(size_t nBytes);
static inline void   alfree(void* ptr);

static inline int    sacPhaseEndReached(PROSAC_HEST* p);
static inline void   sacGoToNextPhase(PROSAC_HEST* p);
static inline void   sacGetPROSACSample(PROSAC_HEST* p);
static inline int    sacIsSampleDegenerate(PROSAC_HEST* p);
static inline void   sacGenerateModel(PROSAC_HEST* p);
static inline int    sacIsModelDegenerate(PROSAC_HEST* p);
static inline void   sacEvaluateModelSPRT(PROSAC_HEST* p);
static inline void   sacUpdateSPRT(PROSAC_HEST* p);
static inline void   sacDesignSPRTTest(PROSAC_HEST* p);
static inline int    sacIsBestModel(PROSAC_HEST* p);
static inline int    sacIsBestModelGoodEnough(PROSAC_HEST* p);
static inline void   sacSaveModel(PROSAC_HEST* p);
static inline void   sacUpdateBounds(PROSAC_HEST* p);

static inline double sacInitPEndFpI(const unsigned ransacConvg,
                                    const unsigned n,
                                    const unsigned m);
static inline void   sacRndSmpl(unsigned  sampleSize,
                                unsigned* currentSample,
                                unsigned  dataSetSize);
static inline unsigned sacCalcIterBound(double   confidence,
                                        double   inlierRate,
                                        unsigned sampleSize,
                                        unsigned maxIterBound);
static void hFuncRefC(float* packedPoints, float* H);



/* Functions */

/**
 * @brief hEstSSE
 * @param src
 * @param dst
 * @param n
 * @param maxD
 * @param maxI
 * @param rConvg
 * @param cfd
 * @param minInl
 * @param finalH
 * @return 
 */

int hestSSE(float*   src,
            float*   dst,
            unsigned N,
            float    maxD,
            unsigned maxI,
            unsigned rConvg,
            double   cfd,
            unsigned minInl,
            float*   finalH){
	PROSAC_HEST  prosacState;
	PROSAC_HEST* p = &prosacState;
	
	p->src    = (Flt2DPt*)src;
	p->dst    = (Flt2DPt*)dst;
	p->N      = N;
	p->maxD   = maxD;
	p->maxI   = maxI;
	p->rConvg = rConvg;
	p->cfd    = cfd;
	p->minInl = minInl;
	p->finalH = finalH;
	
	
	p->phNum     = 4;
	p->phEndI    = 1;
	p->phEndFpI  = sacInitPEndFpI(p->rConvg, p->N, 4);
	p->phMax     = p->N;
	p->bestInl   = 0;
	p->inl       = 0;
	p->smpl      = almalloc(4*sizeof(*p->smpl));
	p->H         = almalloc(HSIZE);
	p->bestH     = almalloc(HSIZE);
	p->pkdPts    = almalloc(8*sizeof(*p->pkdPts));
	memset(p->bestH, 0, HSIZE);
	p->N_tested  = 0;
	p->good      = 1;
	p->t_M       = 25;
	p->m_S       = 1;
	p->epsilon   = 0.1;
	p->delta     = 0.01;
	sacDesignSPRTTest(p);
	
	
	/**
	 * PROSAC Loop
	 */
	
	for(p->i=0;p->i<p->maxI;p->i++){
		if(sacPhaseEndReached(p)){
			sacGoToNextPhase(p);
		}
		
		sacGetPROSACSample(p);
		if(sacIsSampleDegenerate(p)){
			continue;
		}
		
		sacGenerateModel(p);
		if(sacIsModelDegenerate(p)){
			continue;
		}
		
		sacEvaluateModelSPRT(p);
		sacUpdateSPRT(p);
		if(sacIsBestModel(p)){
			sacSaveModel(p);
			sacUpdateBounds(p);
		}
	}
	
	if(!sacIsBestModelGoodEnough(p)){
		memset(p->bestH, 0, HSIZE);
	}
	p->finalH && memcpy(p->finalH, p->bestH, HSIZE);
	
	alfree(p->smpl);
	alfree(p->H);
	alfree(p->bestH);
	alfree(p->pkdPts);
	return sacIsBestModelGoodEnough(p);
}


/**
 * 
 */

static inline void*  almalloc(size_t nBytes){
	unsigned char* ptr = malloc(MEM_ALIGN + nBytes);
	unsigned char* adj = (unsigned char*)(((uintptr_t)(ptr+MEM_ALIGN))&((uintptr_t)(-MEM_ALIGN)));
	ptrdiff_t diff = adj - ptr;
	adj[-1] = diff - 1;
	return adj;
}

/**
 * 
 */

static inline void   alfree(void* ptr){
	unsigned char* cptr = ptr;
	free(cptr - (ptrdiff_t)cptr[-1] - 1);
}

static inline int    sacPhaseEndReached(PROSAC_HEST* p){
	return p->i == p->phEndI && p->phNum < p->phMax;
}

static inline void   sacGoToNextPhase(PROSAC_HEST* p){
	double next;
	unsigned  m = 4;
	
	p->phNum++;
	next        = (p->phEndFpI * p->phNum)/(p->phNum - m);
	p->phEndI  += ceil(next - p->phEndFpI);
	p->phEndFpI = next;
}

static inline void   sacGetPROSACSample(PROSAC_HEST* p){
	if(p->i > p->phEndI){
		sacRndSmpl(4, p->smpl, p->phMax);
	}else{
		sacRndSmpl(3, p->smpl, p->phNum-1);
		p->smpl[3] = p->phNum-1;
	}
}

static inline int    sacIsSampleDegenerate(PROSAC_HEST* p){
	unsigned i0 = p->smpl[0], i1 = p->smpl[1], i2 = p->smpl[2], i3 = p->smpl[3];
	
	/**
	 * Pack the matches selected by the SAC algorithm.
	 * Must be packed  points[0:7]  = {srcx0, srcy0, srcx1, srcy1, srcx2, srcy2, srcx3, srcy3}
	 *                 points[8:15] = {dstx0, dsty0, dstx1, dsty1, dstx2, dsty2, dstx3, dsty3}
	 * Gather 4 points into the vector
	 */
	
	__m128 src10 = _mm_loadl_pi(src10, (__m64*)&p->src[i0]);
	src10        = _mm_loadh_pi(src10, (__m64*)&p->src[i1]);
	__m128 src32 = _mm_loadl_pi(src32, (__m64*)&p->src[i2]);
	src32        = _mm_loadh_pi(src32, (__m64*)&p->src[i3]);
	__m128 dst10 = _mm_loadl_pi(dst10, (__m64*)&p->dst[i0]);
	dst10        = _mm_loadh_pi(dst10, (__m64*)&p->dst[i1]);
	__m128 dst32 = _mm_loadl_pi(dst32, (__m64*)&p->dst[i2]);
	dst32        = _mm_loadh_pi(dst32, (__m64*)&p->dst[i3]);
	
	
	/**
	 * If the matches' source points have common x and y coordinates, abort.
	 */
	
	/**
	 * Check:
	 * packedPoints[0].x == packedPoints[2].x
	 * packedPoints[0].y == packedPoints[2].y
	 * packedPoints[1].x == packedPoints[3].x
	 * packedPoints[1].y == packedPoints[3].y
	 */
	
	__m128 chkEq0 = _mm_cmpeq_ps(src10, src32);
	
	/**
	 * Check:
	 * packedPoints[1].x == packedPoints[2].x
	 * packedPoints[1].y == packedPoints[2].y
	 * packedPoints[0].x == packedPoints[3].x
	 * packedPoints[0].y == packedPoints[3].y
	 */
	
	__m128 chkEq1 = _mm_cmpeq_ps(_mm_shuffle_ps(src10, src10, _MM_SHUFFLE(1, 0, 3, 2)), src32);
	
	/**
	 * Check:
	 * packedPoints[0].x == packedPoints[1].x
	 * packedPoints[0].y == packedPoints[1].y
	 * packedPoints[2].x == packedPoints[3].x
	 * packedPoints[2].y == packedPoints[3].y
	 */
	
	__m128 chkEq2 = _mm_cmpeq_ps(_mm_shuffle_ps(src10, src32, _MM_SHUFFLE(1, 0, 1, 0)),
	                             _mm_shuffle_ps(src10, src32, _MM_SHUFFLE(3, 2, 3, 2)));
	
	/* Verify */
	if(_mm_movemask_ps(_mm_or_ps(chkEq0, _mm_or_ps(chkEq1, chkEq2)))){
		return 1;
	}
	
	/* If the matches do not satisfy the strong geometric constraint, abort. */
	
	/**
	 * p6420x   = (p6.x, p4.x, p2.x, p0.x)
	 * p6420y   = (p6.y, p4.y, p2.y, p0.y)
	 * p7531x   = (p7.x, p5.x, p3.x, p1.x)
	 * p7531y   = (p7.y, p5.y, p3.y, p1.y)
	 * crosssd0 = p6420y - p7531y                     = (cross2d0, cross0d0, cross2s0, cross0s0)
	 * crosssd1 = p7531x - p6420x                     = (cross2d1, cross0d1, cross2s1, cross0s1)
	 * crosssd2 = p6420x * p7531y  -  p6420y * p7531x = (cross2d2, cross0d2, cross2s2, cross0s2)
	 * 
	 * shufcrosssd0 = (cross0d0, cross2d0, cross0s0, cross2s0)
	 * shufcrosssd1 = (cross0d1, cross2d1, cross0s1, cross2s1)
	 * shufcrosssd2 = (cross0d2, cross2d2, cross0s2, cross2s2)
	 * 
	 * dotsd0   = shufcrosssd0 * p6420x +
	 *            shufcrosssd1 * p6420y + 
	 *            shufcrosssd2
	 *          = (dotd0, dotd2, dots0, dots2)
	 * dotsd1   = shufcrosssd0 * p7531x +
	 *            shufcrosssd1 * p7531y + 
	 *            shufcrosssd2
	 *          = (dotd1, dotd3, dots1, dots3)
	 * 
	 * dots     = shufps(dotsd0, dotsd1, _MM_SHUFFLE(1, 0, 1, 0))
	 * dotd     = shufps(dotsd0, dotsd1, _MM_SHUFFLE(3, 2, 3, 2))
	 *            movmaskps(dots ^ dotd)
	 */
	
	__m128 p3210x       = _mm_shuffle_ps(src10,  src32,  _MM_SHUFFLE(2, 0, 2, 0));
	__m128 p3210y       = _mm_shuffle_ps(src10,  src32,  _MM_SHUFFLE(3, 1, 3, 1));
	__m128 p7654x       = _mm_shuffle_ps(dst10,  dst32,  _MM_SHUFFLE(2, 0, 2, 0));
	__m128 p7654y       = _mm_shuffle_ps(dst10,  dst32,  _MM_SHUFFLE(3, 1, 3, 1));
	__m128 p6420x       = _mm_shuffle_ps(p3210x, p7654x, _MM_SHUFFLE(2, 0, 2, 0));
	__m128 p6420y       = _mm_shuffle_ps(p3210y, p7654y, _MM_SHUFFLE(2, 0, 2, 0));
	__m128 p7531x       = _mm_shuffle_ps(p3210x, p7654x, _MM_SHUFFLE(3, 1, 3, 1));
	__m128 p7531y       = _mm_shuffle_ps(p3210y, p7654y, _MM_SHUFFLE(3, 1, 3, 1));
	
	__m128 crosssd0     = _mm_sub_ps(p6420y, p7531y);
	__m128 crosssd1     = _mm_sub_ps(p7531x, p6420x);
	__m128 crosssd2     = _mm_sub_ps(_mm_mul_ps(p6420x, p7531y), _mm_mul_ps(p6420y, p7531x));
	
	__m128 shufcrosssd0 = _mm_shuffle_ps(crosssd0, crosssd0, _MM_SHUFFLE(2, 3, 0, 1));
	__m128 shufcrosssd1 = _mm_shuffle_ps(crosssd1, crosssd1, _MM_SHUFFLE(2, 3, 0, 1));
	__m128 shufcrosssd2 = _mm_shuffle_ps(crosssd2, crosssd2, _MM_SHUFFLE(2, 3, 0, 1));
	
	__m128 dotsd0       = _mm_add_ps(_mm_add_ps(_mm_mul_ps(shufcrosssd0, p6420x),
	                                            _mm_mul_ps(shufcrosssd1, p6420y)),
	                                 shufcrosssd2);
	__m128 dotsd1       = _mm_add_ps(_mm_add_ps(_mm_mul_ps(shufcrosssd0, p7531x),
	                                            _mm_mul_ps(shufcrosssd1, p7531y)),
	                                 shufcrosssd2);
	
	__m128 dots         = _mm_shuffle_ps(dotsd0, dotsd1, _MM_SHUFFLE(0, 1, 0, 1));
	__m128 dotd         = _mm_shuffle_ps(dotsd0, dotsd1, _MM_SHUFFLE(2, 3, 2, 3));
	
	//if(_mm_movemask_ps(_mm_cmpge_ps(_mm_setzero_ps(), _mm_mul_ps(dots, dotd)))){
	if(_mm_movemask_epi8(_mm_cmplt_epi32(_mm_xor_si128(_mm_cvtps_epi32(dots), _mm_cvtps_epi32(dotd)), _mm_setzero_si128()))){
		return 1;
	}
	
	
	/* Otherwise, proceed with evaluation */
	_mm_store_ps((float*)&p->pkdPts[0], src10);
	_mm_store_ps((float*)&p->pkdPts[2], src32);
	_mm_store_ps((float*)&p->pkdPts[4], dst10);
	_mm_store_ps((float*)&p->pkdPts[6], dst32);
	
	return 0;
}

static inline void   sacGenerateModel(PROSAC_HEST* p){
	hFuncRefC((float*)p->pkdPts, p->H);
}

static inline int    sacIsModelDegenerate(PROSAC_HEST* p){
	float* H = p->H;
	float f=H[0]+H[1]+H[2]+H[4]+H[5]+H[6]+H[8]+H[9];
	//return isnan(f);
	return f!=f;
}

static inline void   sacEvaluateModelSPRT(PROSAC_HEST* p){
	unsigned i;
	unsigned isInlier;
	double   lambda       = 1.0;
	double   lambdaReject = ((1.0 - p->delta) / (1.0 - p->epsilon));
	double   lambdaAccept = ((   p->delta   ) / (    p->epsilon  ));
	float    distSq = p->maxD*p->maxD;
	float*   src = (float*)p->src;
	float*   dst = (float*)p->dst;
	float*   H   = p->H;
	
	
	p->inl      = 0;
	p->N_tested = 0;
	p->good     = 1;
	
	
	/* VECTOR */
	const __m128 distSqV=_mm_set1_ps(distSq);
	
	const __m128 H00=_mm_set1_ps(H[0]);
	const __m128 H01=_mm_set1_ps(H[1]);
	const __m128 H02=_mm_set1_ps(H[2]);
	const __m128 H10=_mm_set1_ps(H[4]);
	const __m128 H11=_mm_set1_ps(H[5]);
	const __m128 H12=_mm_set1_ps(H[6]);
	const __m128 H20=_mm_set1_ps(H[8]);
	const __m128 H21=_mm_set1_ps(H[9]);
	const __m128 H22=_mm_set1_ps(H[10]);
	
	for(i=0;i<(p->N-3) && p->good;i+=4){
		/* Backproject */
		__m128 x, y, X, Y, inter0, inter1, inter2, inter3;
		x=_mm_load_ps(src+2*i);
		y=_mm_load_ps(src+2*i+4);
		X=_mm_load_ps(dst+2*i);
		Y=_mm_load_ps(dst+2*i+4);
		
		inter0=_mm_unpacklo_ps(x,y);// y1 y0 x1 x0
		inter1=_mm_unpackhi_ps(x,y);// y3 y2 x3 x2
		inter2=_mm_unpacklo_ps(X,Y);// Y1 Y0 X1 X0
		inter3=_mm_unpackhi_ps(X,Y);// Y3 Y2 X3 X2
		
		x=_mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(inter0), _mm_castps_pd(inter1)));
		y=_mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(inter0), _mm_castps_pd(inter1)));
		X=_mm_castpd_ps(_mm_unpacklo_pd(_mm_castps_pd(inter2), _mm_castps_pd(inter3)));
		Y=_mm_castpd_ps(_mm_unpackhi_pd(_mm_castps_pd(inter2), _mm_castps_pd(inter3)));
		
		__m128 reprojX = _mm_add_ps(_mm_add_ps(_mm_mul_ps(H00, x), _mm_mul_ps(H01, y)), H02);
		__m128 reprojY = _mm_add_ps(_mm_add_ps(_mm_mul_ps(H10, x), _mm_mul_ps(H11, y)), H12);
		__m128 reprojZ = _mm_add_ps(_mm_add_ps(_mm_mul_ps(H20, x), _mm_mul_ps(H21, y)), H22);
		
		__m128 recipZ = _mm_rcp_ps(reprojZ);
		reprojX = _mm_mul_ps(reprojX, recipZ);
		reprojY = _mm_mul_ps(reprojY, recipZ);
		//reprojX = _mm_div_ps(reprojX, reprojZ);
		//reprojY = _mm_div_ps(reprojY, reprojZ);
		
		reprojX = _mm_sub_ps(reprojX, X);
		reprojY = _mm_sub_ps(reprojY, Y);
		
		reprojX = _mm_mul_ps(reprojX, reprojX);
		reprojY = _mm_mul_ps(reprojY, reprojY);
		
		__m128 reprojDistV = _mm_add_ps(reprojX, reprojY);
		
		__m128 cmp = _mm_cmple_ps(reprojDistV, distSqV);
		int msk = _mm_movemask_ps(cmp);
		
		/* ... */
		/*                   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15*/
		unsigned bitCnt[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
		p->inl     += bitCnt[msk];
		
		
		/* SPRT */
		lambda *= p->lambdaTBL[msk];
		p->good = lambda <= p->A;
		/* If !p->good, the threshold A was exceeded, so we're rejecting */
	}
	
	/* SCALAR */
	for(;i<p->N && p->good;i++){
		/* Backproject */
		float x=src[i*2],y=src[i*2+1];
		float X=dst[i*2],Y=dst[i*2+1];
		
		float reprojX=H[0]*x+H[1]*y+H[2]; //  ( X_1 )     ( H_11 H_12    H_13  ) (x_1)
		float reprojY=H[4]*x+H[5]*y+H[6]; //  ( X_2 )  =  ( H_21 H_22    H_23  ) (x_2)
		float reprojZ=H[8]*x+H[9]*y+H[10];//  ( X_3 )     ( H_31 H_32 H_33=1.0 ) (x_3 = 1.0)
		
		//reproj is in homogeneous coordinates. To bring back to "regular" coordinates, divide by Z.
		reprojX/=reprojZ;
		reprojY/=reprojZ;
		
		//Compute distance
		reprojX-=X;
		reprojY-=Y;
		reprojX*=reprojX;
		reprojY*=reprojY;
		float reprojDist = reprojX+reprojY;
		
		/* ... */
		isInlier    = reprojDist <= distSq;
		p->inl     += isInlier;
		
		
		/* SPRT */
		lambda *= isInlier ? lambdaAccept : lambdaReject;
		p->good = lambda <= p->A;
		/* If !p->good, the threshold A was exceeded, so we're rejecting */
	}
	
	
	p->N_tested = i;
}

/**
 * Update either the delta or epsilon SPRT parameters, depending on the events
 * that transpired in the previous evaluation.
 * 
 * If a "good" model that is also the best was encountered, update epsilon,
 * since
 */

static inline void   sacUpdateSPRT(PROSAC_HEST* p){
	if(p->good){
		if(sacIsBestModel(p)){
			p->epsilon = (double)p->inl/p->N;
			sacDesignSPRTTest(p);
		}
	}else{
		double newDelta = (double)p->inl/p->N_tested;
		
		if(newDelta > 0 && CHNG_SIGNIFICANT(p->delta, newDelta)){
			p->delta = newDelta;
			sacDesignSPRTTest(p);
		}
	}
}

/**
 * Numerically compute threshold A from the estimated delta, epsilon, t_M and
 * m_S values.
 * 
 * Epsilon:  Denotes the probability that a randomly chosen data point is
 *           consistent with a good model.
 * Delta:    Denotes the probability that a randomly chosen data point is
 *           consistent with a bad model.
 * t_M:      Time needed to instantiate a model hypotheses given a sample.
 *           (Computing model parameters from a sample takes the same time
 *            as verification of t_M data points)
 * m_S:      The number of models that are verified per sample.
 */

static inline double designSPRTTest(double delta, double epsilon, double t_M, double m_S){
	double An, C, K, prevAn;
	unsigned i;
	
	/**
	 * Randomized RANSAC with Sequential Probability Ratio Test, ICCV 2005
	 * Eq (2)
	 */
	
	C = (1-delta)  *  log((1-delta)/(1-epsilon)) +
	    delta      *  log(  delta  /  epsilon  );
	
	/**
	 * Randomized RANSAC with Sequential Probability Ratio Test, ICCV 2005
	 * Eq (6)
	 * K = K_1/K_2 + 1 = (t_M*C)/m_S + 1
	 */
	
	K = t_M*C/m_S + 1;
	
	/**
	 * Randomized RANSAC with Sequential Probability Ratio Test, ICCV 2005
	 * Paragraph below Eq (6)
	 * 
	 * A* = lim_{n -> infty} A_n, where
	 *     A_0     = K1/K2 + 1             and
	 *     A_{n+1} = K1/K2 + 1 + log(A_n)
	 * The series converges fast, typically within four iterations.
	 */
	
	An = K;
	i  = 0;
	
	do{
		prevAn = An;
		An = K + log(An);
	}while((An-prevAn > 1.5e-8)  &&  (++i < 10));
	
	/**
	 * Return A = An_stopping, with n_stopping < 10
	 */
	
	return An;
}

/**
 * Design the SPRT test. Shorthand for
 *     A = sprt(delta, epsilon, t_M, m_S);
 */

static inline void   sacDesignSPRTTest(PROSAC_HEST* p){
	p->A = designSPRTTest(p->delta, p->epsilon, p->t_M, p->m_S);
	p->lambdaReject = ((1.0 - p->delta) / (1.0 - p->epsilon));
	p->lambdaAccept = ((   p->delta   ) / (    p->epsilon  ));
	
	double a0r4 = p->lambdaReject*p->lambdaReject*p->lambdaReject*p->lambdaReject;
	double a1r3 = p->lambdaAccept*p->lambdaReject*p->lambdaReject*p->lambdaReject;
	double a2r2 = p->lambdaAccept*p->lambdaAccept*p->lambdaReject*p->lambdaReject;
	double a3r1 = p->lambdaAccept*p->lambdaAccept*p->lambdaAccept*p->lambdaReject;
	double a4r0 = p->lambdaAccept*p->lambdaAccept*p->lambdaAccept*p->lambdaAccept;
	
	p->lambdaTBL[ 0] = a0r4;
	p->lambdaTBL[ 1] = p->lambdaTBL[ 2] = p->lambdaTBL[ 4] = p->lambdaTBL[ 8] = a1r3;
	p->lambdaTBL[ 3] = p->lambdaTBL[ 5] = p->lambdaTBL[ 6] = p->lambdaTBL[ 9] = p->lambdaTBL[10] = p->lambdaTBL[12] = a2r2;
	p->lambdaTBL[ 7] = p->lambdaTBL[11] = p->lambdaTBL[13] = p->lambdaTBL[14] = a3r1;
	p->lambdaTBL[15] = a4r0;
}

/**
 * Return whether the current model is the best model so far.
 */

static inline int    sacIsBestModel(PROSAC_HEST* p){
	return p->inl > p->bestInl;
}

/**
 * Returns whether the current-best model is good enough to be an
 * acceptable best model, by checking whether it meets the minimum
 * number of inliers.
 */

static inline int    sacIsBestModelGoodEnough(PROSAC_HEST* p){
	return p->bestInl >= p->minInl;
}

/**
 * 
 */

static inline void   sacSaveModel(PROSAC_HEST* p){
	p->bestInl = p->inl;
	memcpy(p->bestH, p->H, HSIZE);
}

/**
 * 
 */

static inline void   sacUpdateBounds(PROSAC_HEST* p){
	p->maxI = sacCalcIterBound(p->cfd,
	                           (double)p->bestInl/p->N,
	                           4,
	                           p->maxI);
}


/**
 * Compute the real-valued number of samples per phase, given the RANSAC convergence speed,
 * data set size and sample size.
 */

static inline double sacInitPEndFpI(const unsigned ransacConvg,
                                 const unsigned n,
                                 const unsigned m){
	double numer=1, denom=1;
	
	unsigned i;
	for(i=0;i<m;i++){
		numer *= m-i;
		denom *= n-i;
	}
	
	return ransacConvg*numer/denom;
}

/**
 * Choose, without repetition, sampleSize integers in the range [0, numDataPoints).
 */

static inline void sacRndSmpl(unsigned  sampleSize,
                           unsigned* currentSample,
                           unsigned  dataSetSize){
	/**
	 * If sampleSize is very close to dataSetSize, we use selection sampling.
	 * Otherwise we use the naive sampling technique wherein we select random
	 * indexes until sampleSize of them are distinct.
	 */
	
	if(sampleSize*2>dataSetSize){
		/**
		 * Selection Sampling:
		 * 
		 * Algorithm S (Selection sampling technique). To select n records at random from a set of N, where 0 < n ≤ N.
		 * S1. [Initialize.] Set t ← 0, m ← 0. (During this algorithm, m represents the number of records selected so far,
		 *                                      and t is the total number of input records that we have dealt with.)
		 * S2. [Generate U.] Generate a random number U, uniformly distributed between zero and one.
		 * S3. [Test.] If (N – t)U ≥ n – m, go to step S5.
		 * S4. [Select.] Select the next record for the sample, and increase m and t by 1. If m < n, go to step S2;
		 *               otherwise the sample is complete and the algorithm terminates.
		 * S5. [Skip.] Skip the next record (do not include it in the sample), increase t by 1, and go back to step S2.
		 */
		
		unsigned m=0,t=0;
		
		for(m=0;m<sampleSize;t++){
			double U=((double)random())/INT_MAX;
			if((dataSetSize-t)*U < (sampleSize-m)){
				currentSample[m++]=t;
			}
		}
	}else{
		/**
		 * Naive sampling technique. Generate indexes until sampleSize of them are distinct.
		 */
		
		unsigned i, j;
		for(i=0;i<sampleSize;i++){
			int inList;
			
			do{
				currentSample[i]=dataSetSize*((double)random())/INT_MAX;
				
				inList=0;
				for(j=0;j<i;j++){
					if(currentSample[i] == currentSample[j]){
						inList=1;
						break;
					}
				}
			}while(inList);
		}
	}
}

/**
 * Estimate the number of iterations required based on the requested confidence,
 * proportion of inliers in the best model so far and sample size.
 * 
 * Clamp return value at maxIterationBound.
 */

static inline unsigned sacCalcIterBound(double   confidence,
                                        double   inlierRate,
                                        unsigned sampleSize,
                                        unsigned maxIterBound){
	unsigned retVal;
	
	/**
	 * Formula chosen from http://en.wikipedia.org/wiki/RANSAC#The_parameters :
	 * 
	 * \[ k = \frac{\log{(1-confidence)}}{\log{(1-inlierRate**sampleSize)}} \]
	 */
	
	double atLeastOneOutlierProbability = 1.-pow(inlierRate, (double)sampleSize);
	
	/**
	 * There are two special cases: When argument to log() is 0 and when it is 1.
	 * Each has a special meaning.
	 */
	
	if(atLeastOneOutlierProbability==1.){
		/**
		 * A certainty of picking at least one outlier means that we will need
		 * an infinite amount of iterations in order to find a correct solution.
		 */
		
		retVal = maxIterBound;
	}else if(atLeastOneOutlierProbability==0.){
		/**
		 * The certainty of NOT picking an outlier means that no more iterations
		 * are needed to find a solution.
		 */
		
		retVal = 0;
	}else{
		/**
		 * Since 1-confidence (the probability of the model being based on at
		 * least one outlier in the data) is equal to
		 * (1-inlierRate**sampleSize)**numIterations (the probability of picking
		 * at least one outlier in numIterations samples), we can isolate
		 * numIterations (the return value) into
		 */
		
		retVal = ceil(log(1.-confidence)/log(atLeastOneOutlierProbability));
	}
	
	/**
	 * Clamp to maxIterationBound.
	 */
	
	return retVal <= maxIterBound ? retVal : maxIterBound;
}



static void hFuncRefC(float* packedPoints,/* Source (four x,y float coordinates) points followed by
                                             destination (four x,y float coordinates) points, aligned by 32 bytes */
                      float* H){          /* Homography (three 16-byte aligned rows of 3 floats) */
	float x0=*packedPoints++;
	float y0=*packedPoints++;
	float x1=*packedPoints++;
	float y1=*packedPoints++;
	float x2=*packedPoints++;
	float y2=*packedPoints++;
	float x3=*packedPoints++;
	float y3=*packedPoints++;
	float X0=*packedPoints++;
	float Y0=*packedPoints++;
	float X1=*packedPoints++;
	float Y1=*packedPoints++;
	float X2=*packedPoints++;
	float Y2=*packedPoints++;
	float X3=*packedPoints++;
	float Y3=*packedPoints++;
	
	float x0X0=x0*X0, x1X1=x1*X1, x2X2=x2*X2, x3X3=x3*X3;
	float x0Y0=x0*Y0, x1Y1=x1*Y1, x2Y2=x2*Y2, x3Y3=x3*Y3;
	float y0X0=y0*X0, y1X1=y1*X1, y2X2=y2*X2, y3X3=y3*X3;
	float y0Y0=y0*Y0, y1Y1=y1*Y1, y2Y2=y2*Y2, y3Y3=y3*Y3;
	
	
	/**
	 *  [0]   [1] Hidden   Prec
	 *  x0    y0    1       x1
	 *  x1    y1    1       x1
	 *  x2    y2    1       x1
	 *  x3    y3    1       x1
	 * 
	 * Eliminate ones in column 2 and 5.
	 * R(0)-=R(2)
	 * R(1)-=R(2)
	 * R(3)-=R(2)
	 * 
	 *  [0]   [1] Hidden   Prec
	 * x0-x2 y0-y2  0       x1+1
	 * x1-x2 y1-y2  0       x1+1
	 *  x2    y2    1       x1
	 * x3-x2 y3-y2  0       x1+1
	 * 
	 * Eliminate column 0 of rows 1 and 3
	 * R(1)=(x0-x2)*R(1)-(x1-x2)*R(0),     y1'=(y1-y2)(x0-x2)-(x1-x2)(y0-y2)
	 * R(3)=(x0-x2)*R(3)-(x3-x2)*R(0),     y3'=(y3-y2)(x0-x2)-(x3-x2)(y0-y2)
	 * 
	 *  [0]   [1] Hidden   Prec
	 * x0-x2 y0-y2  0      x1+1
	 *   0    y1'   0      x2+3
	 *  x2    y2    1       x1
	 *   0    y3'   0      x2+3
	 * 
	 * Eliminate column 1 of rows 0 and 3
	 * R(3)=y1'*R(3)-y3'*R(1)
	 * R(0)=y1'*R(0)-(y0-y2)*R(1)
	 * 
	 *  [0]   [1] Hidden   Prec
	 *  x0'    0    0      x3+5
	 *   0    y1'   0      x2+3
	 *  x2    y2    1       x1
	 *   0     0    0      x4+7
	 * 
	 * Eliminate columns 0 and 1 of row 2
	 * R(0)/=x0'
	 * R(1)/=y1'
	 * R(2)-= (x2*R(0) + y2*R(1))
	 * 
	 *  [0]   [1] Hidden   Prec
	 *   1     0    0      x6+10
	 *   0     1    0      x4+6
	 *   0     0    1      x4+7
	 *   0     0    0      x4+7
	 */
	
	//Eliminate ones in column 2 and 5.
	//R(0)-=R(2)
	//R(1)-=R(2)
	//R(3)-=R(2)
	float minor[4][2] = {{x0-x2,y0-y2},
						 {x1-x2,y1-y2},
						 {x2   ,y2   },
						 {x3-x2,y3-y2}};
	float major[8][3] = {{x2X2-x0X0,y2X2-y0X0,(X0-X2)},
						 {x2X2-x1X1,y2X2-y1X1,(X1-X2)},
						 {-x2X2    ,-y2X2    ,(X2   )},
						 {x2X2-x3X3,y2X2-y3X3,(X3-X2)},
						 {x2Y2-x0Y0,y2Y2-y0Y0,(Y0-Y2)},
						 {x2Y2-x1Y1,y2Y2-y1Y1,(Y1-Y2)},
						 {-x2Y2    ,-y2Y2    ,(Y2   )},
						 {x2Y2-x3Y3,y2Y2-y3Y3,(Y3-Y2)}};
	
	//int i;
	//for(i=0;i<8;i++) major[i][2]=-major[i][2];
	//Eliminate column 0 of rows 1 and 3
	//R(1)=(x0-x2)*R(1)-(x1-x2)*R(0),     y1'=(y1-y2)(x0-x2)-(x1-x2)(y0-y2)
	//R(3)=(x0-x2)*R(3)-(x3-x2)*R(0),     y3'=(y3-y2)(x0-x2)-(x3-x2)(y0-y2)
	float scalar1=minor[0][0], scalar2=minor[1][0];
	minor[1][1]=minor[1][1]*scalar1-minor[0][1]*scalar2;
	
	major[1][0]=major[1][0]*scalar1-major[0][0]*scalar2;
	major[1][1]=major[1][1]*scalar1-major[0][1]*scalar2;
	major[1][2]=major[1][2]*scalar1-major[0][2]*scalar2;
	
	major[5][0]=major[5][0]*scalar1-major[4][0]*scalar2;
	major[5][1]=major[5][1]*scalar1-major[4][1]*scalar2;
	major[5][2]=major[5][2]*scalar1-major[4][2]*scalar2;
	
	scalar2=minor[3][0];
	minor[3][1]=minor[3][1]*scalar1-minor[0][1]*scalar2;
	
	major[3][0]=major[3][0]*scalar1-major[0][0]*scalar2;
	major[3][1]=major[3][1]*scalar1-major[0][1]*scalar2;
	major[3][2]=major[3][2]*scalar1-major[0][2]*scalar2;
	
	major[7][0]=major[7][0]*scalar1-major[4][0]*scalar2;
	major[7][1]=major[7][1]*scalar1-major[4][1]*scalar2;
	major[7][2]=major[7][2]*scalar1-major[4][2]*scalar2;
	
	//Eliminate column 1 of rows 0 and 3
	//R(3)=y1'*R(3)-y3'*R(1)
	//R(0)=y1'*R(0)-(y0-y2)*R(1)
	scalar1=minor[1][1];scalar2=minor[3][1];
	major[3][0]=major[3][0]*scalar1-major[1][0]*scalar2;
	major[3][1]=major[3][1]*scalar1-major[1][1]*scalar2;
	major[3][2]=major[3][2]*scalar1-major[1][2]*scalar2;
	
	major[7][0]=major[7][0]*scalar1-major[5][0]*scalar2;
	major[7][1]=major[7][1]*scalar1-major[5][1]*scalar2;
	major[7][2]=major[7][2]*scalar1-major[5][2]*scalar2;
	
	scalar2=minor[0][1];
	minor[0][0]=minor[0][0]*scalar1-minor[1][0]*scalar2;
	
	major[0][0]=major[0][0]*scalar1-major[1][0]*scalar2;
	major[0][1]=major[0][1]*scalar1-major[1][1]*scalar2;
	major[0][2]=major[0][2]*scalar1-major[1][2]*scalar2;
	
	major[4][0]=major[4][0]*scalar1-major[5][0]*scalar2;
	major[4][1]=major[4][1]*scalar1-major[5][1]*scalar2;
	major[4][2]=major[4][2]*scalar1-major[5][2]*scalar2;
	
	//Eliminate columns 0 and 1 of row 2
	//R(0)/=x0'
	//R(1)/=y1'
	//R(2)-= (x2*R(0) + y2*R(1))
	scalar1=minor[0][0];
	major[0][0]/=scalar1;
	major[0][1]/=scalar1;
	major[0][2]/=scalar1;
	major[4][0]/=scalar1;
	major[4][1]/=scalar1;
	major[4][2]/=scalar1;
	
	scalar1=minor[1][1];
	major[1][0]/=scalar1;
	major[1][1]/=scalar1;
	major[1][2]/=scalar1;
	major[5][0]/=scalar1;
	major[5][1]/=scalar1;
	major[5][2]/=scalar1;
	
	
	scalar1=minor[2][0];scalar2=minor[2][1];
	major[2][0]-=major[0][0]*scalar1+major[1][0]*scalar2;
	major[2][1]-=major[0][1]*scalar1+major[1][1]*scalar2;
	major[2][2]-=major[0][2]*scalar1+major[1][2]*scalar2;
	
	major[6][0]-=major[4][0]*scalar1+major[5][0]*scalar2;
	major[6][1]-=major[4][1]*scalar1+major[5][1]*scalar2;
	major[6][2]-=major[4][2]*scalar1+major[5][2]*scalar2;
	
	//Only major matters now. R(3) and R(7) correspond to the hollowed-out rows.
	scalar1=major[7][0];
	major[7][1]/=scalar1;
	major[7][2]/=scalar1;
	
	scalar1=major[0][0];major[0][1]-=scalar1*major[7][1];major[0][2]-=scalar1*major[7][2];
	scalar1=major[1][0];major[1][1]-=scalar1*major[7][1];major[1][2]-=scalar1*major[7][2];
	scalar1=major[2][0];major[2][1]-=scalar1*major[7][1];major[2][2]-=scalar1*major[7][2];
	scalar1=major[3][0];major[3][1]-=scalar1*major[7][1];major[3][2]-=scalar1*major[7][2];
	scalar1=major[4][0];major[4][1]-=scalar1*major[7][1];major[4][2]-=scalar1*major[7][2];
	scalar1=major[5][0];major[5][1]-=scalar1*major[7][1];major[5][2]-=scalar1*major[7][2];
	scalar1=major[6][0];major[6][1]-=scalar1*major[7][1];major[6][2]-=scalar1*major[7][2];
	
	
	//One column left (Two in fact, but the last one is the homography)
	scalar1=major[3][1];
	
	major[3][2]/=scalar1;
	scalar1=major[0][1];major[0][2]-=scalar1*major[3][2];
	scalar1=major[1][1];major[1][2]-=scalar1*major[3][2];
	scalar1=major[2][1];major[2][2]-=scalar1*major[3][2];
	scalar1=major[4][1];major[4][2]-=scalar1*major[3][2];
	scalar1=major[5][1];major[5][2]-=scalar1*major[3][2];
	scalar1=major[6][1];major[6][2]-=scalar1*major[3][2];
	scalar1=major[7][1];major[7][2]-=scalar1*major[3][2];
	
	
	//Homography is done.
	H[0]=major[0][2];
	H[1]=major[1][2];
	H[2]=major[2][2];
	
	H[4]=major[4][2];
	H[5]=major[5][2];
	H[6]=major[6][2];
	
	H[8]=major[7][2];
	H[9]=major[3][2];
	H[10]=1.0;
}

