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
#include "training.h"
#include <sstream>

const int Train::minDistance2ReferencePoint    = 2;
const int Train::minDistance2ReferencePointSqr = minDistance2ReferencePoint *
                                          minDistance2ReferencePoint;
const int Train::HALF_PATCH_WIDTH = 15;
const int Train::SIGMA_BLUR = 15;


double Train::rad2Deg(double rad)
{
    return rad*(180/M_PI);
}
double Train::deg2Rad(double deg)
{
    return deg*(M_PI/180);
}

void Train::warpImage(const Mat &src,
                      double yaw,
                      double pitch,
                      double roll,
                      double scale,
                      double fovy,
                      Mat &dst,
                      Mat &M,
                      vector<Point2f> &corners)
{
	//Warp Image
	//Half of vertical field of view
	double halfFovy = fovy * 0.5;
	//Compute d
	double d = hypot(src.cols, src.rows);
	//Compute side length of square
	double sideLength = scale * d / cos(deg2Rad(halfFovy));
	//Compute warp matrix and set vector of corners
	warpMatrix(src.size(), yaw, pitch, roll, scale, fovy, M, &corners);
	//Perform actual warp to finish the method
	warpPerspective(src,dst,M,Size(sideLength,sideLength));
}



bool Train::keyPointOrderingByResponse(const KeyPoint& k1, const KeyPoint& k2)
{
    return k1.response > k2.response;
}
bool Train::compareStablePoints(const StablePoint &x1, const StablePoint &x2)
{
    return x1.viewPointPts.size() > x2.viewPointPts.size();
}




void Train::warpMatrix(Size sz,
                       double yaw,
                       double pitch,
                       double roll,
                       double scale,
                       double fovy,
                       Mat &M,
                       vector<Point2f>* corners)
{
	double st=sin(deg2Rad(roll));
	double ct=cos(deg2Rad(roll));
	double sp=sin(deg2Rad(pitch));
	double cp=cos(deg2Rad(pitch));
	double sg=sin(deg2Rad(yaw));
	double cg=cos(deg2Rad(yaw));
    
	double halfFovy=fovy*0.5;
	double d=hypot(sz.width,sz.height);
	double sideLength=scale*d/cos(deg2Rad(halfFovy));
	double h=d/(2.0*sin(deg2Rad(halfFovy)));
	double n=h-(d/2.0);
	double f=h+(d/2.0);
    
    
	Mat F=Mat(4,4,CV_64FC1);
	Mat Rroll=Mat::eye(4,4,CV_64FC1);
	Mat Rpitch=Mat::eye(4,4,CV_64FC1);
	Mat Ryaw=Mat::eye(4,4,CV_64FC1);
    
	Mat T=Mat::eye(4,4,CV_64FC1);
	Mat P=Mat::zeros(4,4,CV_64FC1);
    
    
	Rroll.at<double>(0,0)=Rroll.at<double>(1,1)=ct;
	Rroll.at<double>(0,1)=-st;Rroll.at<double>(1,0)=st;
    
	Rpitch.at<double>(1,1)=Rpitch.at<double>(2,2)=cp;
	Rpitch.at<double>(1,2)=-sp;Rpitch.at<double>(2,1)=sp;
    
	Ryaw.at<double>(0,0)=Ryaw.at<double>(2,2)=cg;
	Ryaw.at<double>(0,2)=sg;Ryaw.at<double>(2,0)=sg;
    
    
	T.at<double>(2,3)=-h;
    
	P.at<double>(0,0)=P.at<double>(1,1)=1.0/tan(deg2Rad(halfFovy));
	P.at<double>(2,2)=-(f+n)/(f-n);
	P.at<double>(2,3)=-(2.0*f*n)/(f-n);
	P.at<double>(3,2)=-1.0;
    
	F=P*T*Rpitch*Rroll*Ryaw;
    
	double ptsIn [4*3];
	double ptsOut[4*3];
	double halfW=sz.width/2, halfH=sz.height/2;
	ptsIn[0]=-halfW;ptsIn[ 1]= halfH;
	ptsIn[3]= halfW;ptsIn[ 4]= halfH;
	ptsIn[6]= halfW;ptsIn[ 7]=-halfH;
	ptsIn[9]=-halfW;ptsIn[10]=-halfH;
	ptsIn[2]=ptsIn[5]=ptsIn[8]=ptsIn[11]=0;
	Mat ptsInMat(1,4,CV_64FC3,ptsIn);Mat ptsOutMat(1,4,CV_64FC3,ptsOut);
	perspectiveTransform(ptsInMat,ptsOutMat,F);
    
	Point2f ptsInPt2f[4];Point2f ptsOutPt2f[4];
	for(int i=0;i<4;i++)
	{
		Point2f ptIn(ptsIn[i*3+0],ptsIn[i*3+1]);
		Point2f ptOut(ptsOut[i*3+0],ptsOut[i*3+1]);
		ptsInPt2f[i]=ptIn+Point2f(halfW,halfH);
		ptsOutPt2f[i]=(ptOut+Point2f(1,1))*(sideLength*0.5);
	}
	M=getPerspectiveTransform(ptsInPt2f,ptsOutPt2f);
    
	if(corners!=NULL)
	{
		corners->clear();
		corners->push_back(ptsOutPt2f[0]);
		corners->push_back(ptsOutPt2f[1]);
		corners->push_back(ptsOutPt2f[2]);
		corners->push_back(ptsOutPt2f[3]);
	}
}

Mat Train::keyPoint2Mat(const vector< KeyPoint >& keypoints)
{
    Mat stub((int)keypoints.size(),1,CV_32FC2);
    for ( unsigned int i=0; i<keypoints.size(); i++ )
        stub.at<Vec2f>(i,0)=keypoints[i].pt;
    return stub;
}

vector<Point> Train::mat2Points(const Mat& stub)
{
    vector<Point> points(stub.rows);
    for ( int i=0; i<stub.rows; i++ )
    {
        Point2f pnt = stub.at<Vec2f>(i,0);
        points[i] = Point(pnt.x, pnt.y);
    }
    return points;
}

void Train::projectKeypoints(const vector<KeyPoint> &original, const MatExpr M, vector<Point> &transformedPoints)
{
    Mat keypointMatIn = keyPoint2Mat(original);
    Mat keypointMatOut;
    perspectiveTransform(keypointMatIn, keypointMatOut, M);
    transformedPoints = mat2Points(keypointMatOut);
}

void Train::rotateCorners(vector<Point2f> &pts, vector<Point2f> &out,  Point2f center, float angle, float scale)
{
    
    angle = angle * 0.0174532925;
    double s = sin(angle);
    double c = cos(angle);
    
    for (int i = 0 ; i < pts.size(); i++)
    {
        float x = pts[i].x;
        float y = pts[i].y;
        out[i].x = (c*(x-center.x)*scale - s*(y-center.y)*scale) + center.x;
        out[i].y = (s*(x-center.x)*scale + c*(y-center.y)*scale) + center.y;
    }
    
}

vector<Point2f> Train::rotateImage(Mat &image,
                        Mat &out,
                        float angle,
                        float scale)
{
    int borderSize = 40;
    int iRows = image.rows >> 1;
    int iCols = image.cols >> 1;
    int maxRC = max(iRows, iCols);
    int maxSize = (int)(sqrt(pow(maxRC, 2.0) * 2) + borderSize + 2) << 1;
    Mat tmp = Mat::zeros(maxSize, maxSize, image.type());
    int nRows = maxSize >> 1;
    int nCols = maxSize >> 1;
    Rect middle(Point( nCols - iCols,
                      nRows - iRows),
                Size(image.cols,
                     image.rows));
    
    vector<Point2f> corners;
    Point tl = middle.tl();
    Point br = middle.br();
    corners.push_back(Point2f(tl.x, tl.y));
    corners.push_back(Point2f(tl.x, tl.y) + Point2f(image.cols, 0));
    corners.push_back(Point2f(br.x, br.y));
    corners.push_back(Point2f(tl.x, tl.y) + Point2f(0, image.rows));
    vector<Point2f> outCorners(corners.size());
    
    image.copyTo(tmp(middle));
    
    Point2f center(nRows, nCols);
    Mat rot = getRotationMatrix2D(center, -angle, scale);
    warpAffine(tmp, out, rot,Size(maxSize, maxSize));
    rotateCorners(corners, outCorners, center, angle, scale     );
      
    return outCorners;
    
}


int chooseFASTThreshold(const Mat &img, const int lowerBound, const int upperBound)
{
    static vector<cv::KeyPoint> kpts;
    
    int left = 0;
    int right = 255;
    int currentThreshold = 128;
    int currentScore = 256;
   
    while (currentScore < lowerBound || currentScore > upperBound)
    {
        FAST(img, kpts, currentThreshold, true);
        currentScore = (int)kpts.size();
        
        if (lowerBound > currentScore)
        {
            // we look for a lower threshold to increase the number of corners:
            right = currentThreshold;
            currentThreshold = (currentThreshold + left) >> 1;
            if (right == currentThreshold)
                break;
        } else
        {
            // we look for a higher threshold to decrease the number of corners:
            left = currentThreshold;
            currentThreshold = (currentThreshold + right) >> 1;
            if (left == currentThreshold)
                break;
        }
    }

    return currentThreshold;
}





int  Train::fastTrainFromReference(Mat &image, SimpleParams &params,
                                   vector<vector<KeyPoint>> &kps,
                                   vector<Mat> &descs,
                                   vector<vector<Point2f>> &corn)
{
    
    Ptr<DescriptorExtractor> extractor = Train::create(Desc::BRIEF);

    int threshold = chooseFASTThreshold(image, 200, 255);
    
    for (double scale = params.scale.max; scale > params.scale.min; scale*=params.scale.step)
    {
        for (double roll = params.roll.min; roll < params.roll.max; roll+=params.roll.step)
        {
            Mat rotated, descriptor;
            vector<Point2f> cr = rotateImage(image, rotated, roll, scale);
            vector<KeyPoint> keypoints;
            FAST(rotated, keypoints, threshold, true);
            extractor->compute(rotated, keypoints, descriptor);
            kps.push_back(keypoints);
            descs.push_back(descriptor);
            corn.push_back(cr);
        }
    }
    return threshold;
}

bool Train::sortByProbNearToDot5(std::pair<int,float> &first, std::pair<int,float> &second)
{
    float f = std::abs(.5 - first.second);
    float s = std::abs(.5 - second.second);
    return f < s;
}

void Train::computeIndices(vector<vector<StablePoint>> &database, vector<uint16_t> &indices)
{
    //For all list of stable points
    for (int i = 0; i < database.size(); ++i)
    {
        //Go through all the stable points
        for (int j = 0; j < database[i].size(); ++j)
        {
            Mat descs = database[i][j].viewPointPtDescriptors;
            //All the descriptors associated to a point
            for (int r = 0; r < descs.rows; ++r)
            {
                Mat desc = descs.row(r);
                database[i][j].indices[r] = Index::getDescIndex(desc, indices);
            }
        }
    }
}

vector<uint16_t> Train::computeDescIndices(vector<vector<StablePoint>> &database, int size)
{
    std::map<int,float> bitHistograms;
    int totalDescriptors = 0;
    //For all list of stable points
    for (int i = 0; i < database.size(); ++i)
    {
        //Go through all the stable points
        for (int j = 0; j < database[i].size(); ++j)
        {
            Mat descs = database[i][j].viewPointPtDescriptors;
            //All the descriptors associated to a point
            for (int r = 0; r < descs.rows; ++r)
            {
                uchar *row = descs.ptr<uchar>(r);
                //For every single col value in the descriptor
                for (int c = 0; c < descs.cols ; c++)
                {
                    //Take the bits positions
                    for (int bit = 7; bit >= 0; --bit)
                    {
                        uchar tmp = (1 << bit);
                        if ((row[c] & tmp) > 0)
                        {
                            bitHistograms[ c * 8 + (7 - bit)]++;
                        }
                    }
                }
                totalDescriptors++;
            }
        }
    }
    typedef std::map<int,float>::iterator iter;
    vector< pair<int,float> > result;
    for (iter it = bitHistograms.begin(); it != bitHistograms.end(); ++it)
    {
        it->second /= (float)totalDescriptors;
        result.push_back(pair<int,float>(it->first, it->second));
    }
    sort(result.begin(), result.end(), Train::sortByProbNearToDot5);
    vector<uint16_t> indices;
    int i = 0;
    typedef std::vector<pair<int,float>>::iterator iter2;
    for (iter2 it = result.begin(); it != result.end() && i < size; ++it, ++i)
    {
        indices.push_back(it->first);
    }
    sort(indices.begin(), indices.end());
    return indices;
}


void Train::trainFromReferenceImage(Mat &image, ViewParams &params, vector<vector<StablePoint> > &database)
{
    CV_Assert(image.channels() == 1);

    RNG rng(0xFFFFFFFF);
    int viewPointBin = 0, totalBins = 0;
    totalBins = ((int) floor(1.0 + abs( log(params.scale.min / params.scale.max) / log (params.scale.step)))) *
                ((int) ceil(abs((params.yaw.max - params.yaw.min) / params.yaw.step))) *
                ((int) ceil(abs((params.pitch.max - params.pitch.min) / params.pitch.step))) *
                ((int) ceil(abs((params.roll.max - params.roll.min) / params.roll.step)));
    
    double rYaw, rPitch, rRoll;
    
    Ptr<FeatureDetector> detector = Train::create(params.detector);
    Ptr<DescriptorExtractor> extractor = Train::create(params.extractor);
    
    for ( double yaw = params.yaw.min; yaw < params.yaw.max; yaw += params.yaw.step )
	{
		for ( double pitch = params.pitch.min; pitch < params.pitch.max; pitch += params.pitch.step )
		{
			for ( double roll = params.roll.min; roll < params.roll.max; roll += params.roll.step )
			{
				for ( double scale = params.scale.max; scale >= params.scale.min; scale *= params.scale.step )
				{
                    viewPointBin++;
                    printf("Training %d / %d\n", viewPointBin, totalBins);
                    
                    vector<Mat> /*images, */projMatrices;
                    
                    vector<vector<KeyPoint>> keyPoints;
                    vector<vector<uint16_t>> indxs;
                    vector<Mat> descriptors;
					vector<vector<Point2f> > vecCorners;
                    
					for ( int viewPoint = 0; viewPoint  < params.numberOfViewPoints ; viewPoint++ )
					{
						Mat warpBlurred, warped, projecMatrix;
						
                        vector<Point2f> corners;
                        rPitch = rng.uniform(pitch, pitch + params.pitch.step);
                        rYaw   = rng.uniform(yaw,yaw + params.yaw.step);
                        rRoll  = rng.uniform(roll, roll + params.roll.step);
                        
						warpImage(image, rYaw, rPitch ,rRoll,
                                  scale, params.fov, warped, projecMatrix, corners);

                        warped.copyTo(warpBlurred);
                        //warped.copyTo(warpBlurred);
						GaussianBlur(warped, warpBlurred,
                                     Size(3,3), SIGMA_BLUR, SIGMA_BLUR); //scale*sigma_blur

                        
                        vector<KeyPoint> kps;
                        vector<uint16_t> idx;
                        Mat desc;
                        selectPoints(warpBlurred,
                                     projecMatrix,
                                     corners,
                                     image.size(),
                                     detector,
                                     extractor,
                                     kps,
                                     idx,
                                     desc);
                        
                        keyPoints.push_back(kps);
                        descriptors.push_back(desc);
                        indxs.push_back(idx);
						projMatrices.push_back(projecMatrix);
						vecCorners.push_back(corners);
					}
                    
                    vector<StablePoint> viewStablePts;
                    
                    selectMostStableKeypoints(keyPoints,
                                              indxs,
                                              descriptors,
                                              projMatrices,
                                              vecCorners,
                                              params.numberOfPointsPerViewPoints,
                                              image.size(),
                                              detector,
                                              extractor,
                                              viewStablePts);
                    
                    database.push_back(viewStablePts);

					//free memory
                    keyPoints.clear();
                    indxs.clear();
                    descriptors.clear();
                    vecCorners.clear();
					projMatrices.clear();
                }
            }
        }
    }
//    printf("Learning best index positions\n");
//    vector<int> indices = computeDescIndices(database, Index::SIZE);
//    printf("Computing index based on positions\n");
//    computeIndices(database, indices);
}

void Train::filterKeyPoints(Size targetImgSize,
                            const vector<Point2f> &corners,
                            vector<KeyPoint> &keypoints,
                            vector<Point> &pointsTransformed)
{
    vector<Point> newPointsTransformed;
    newPointsTransformed.reserve(pointsTransformed.size());
    vector<KeyPoint> newKeypoints;
    newKeypoints.reserve(keypoints.size());
    
    for ( unsigned int i=0; i<keypoints.size(); i++ )
    {
        Point& p = pointsTransformed[i];
        KeyPoint& k = keypoints[i];
        //reject if transformed point is too close to the edge
        if (p.x < HALF_PATCH_WIDTH ||
            p.y < HALF_PATCH_WIDTH ||
            p.x >= targetImgSize.width - HALF_PATCH_WIDTH ||
            p.y >= targetImgSize.height - HALF_PATCH_WIDTH )
            continue;
        
        //reject if keypoint is too close to edge of the warped image
        if (pointPolygonTest(corners, Point2f(k.pt.x-HALF_PATCH_WIDTH, k.pt.y-HALF_PATCH_WIDTH), false) < 0 ||
            pointPolygonTest(corners, Point2f(k.pt.x-HALF_PATCH_WIDTH, k.pt.y+HALF_PATCH_WIDTH), false) < 0 ||
            pointPolygonTest(corners, Point2f(k.pt.x+HALF_PATCH_WIDTH, k.pt.y-HALF_PATCH_WIDTH), false) < 0 ||
            pointPolygonTest(corners, Point2f(k.pt.x+HALF_PATCH_WIDTH, k.pt.y+HALF_PATCH_WIDTH), false) < 0 )
            continue;
        
        newPointsTransformed.push_back(p);
        newKeypoints.push_back(k);
    }
    //replace old points with new points
    pointsTransformed.clear();
    keypoints.clear();
    pointsTransformed = newPointsTransformed;
    keypoints = newKeypoints;
    
    CV_Assert(keypoints.size() == pointsTransformed.size());
    return;
}

int Train::getNearbyPoint(const Point& p, const vector<vector<int> > &grid)
{
    int i,j, idx = grid[p.y][p.x];
    // Set initial threshold to one more than the acceptable threshold
    int dist, minDist = minDistance2ReferencePointSqr + 1;
    //the center point was already an existing point, so return its index
    if (idx != -1)
        return idx;
    
    /* Changed because the limits were being re-evaluated each time with the stdmin, and because
     * the distance is evaluated as distance squared the results are always integral, so floating
     * point is not needed.
     */
    
    int xmin,ymin,xmax,ymax;
    xmin = std::max(p.x - minDistance2ReferencePoint,0);
    ymin = std::max(p.y - minDistance2ReferencePoint,0);
    xmax = std::min(p.x + minDistance2ReferencePoint, (int)grid.size()-1);
    ymax = std::min(p.y + minDistance2ReferencePoint, (int)grid[0].size()-1);
    
    for ( i = ymin; i <= ymax; i++)
    {
        for ( j = xmin; j <= xmax; j++)
        {
            dist = (p.y-i)*(p.y-i)+(p.x-j)*(p.x-j);
            
            if ( ( dist < minDist ) && ( grid[i][j]!=-1 ) )
            {
                minDist = dist; idx = grid[i][j];
            }
        }
    }
    return idx;
}

string Train::featureName(Feat::Code code)
{
    return Feat::getName(code);
}
string Train::descriptorName(Desc::Code code)
{
    return Desc::getName(code);
}

Ptr<FeatureDetector>  Train::create(Feat::Code code)
{
    return Feat::create(code);
}
Ptr<DescriptorExtractor> Train::create(Desc::Code code)
{
    return Desc::create(code);
}


/*
 *  Computes the Hamming Distance between two binary descriptors.
 *  It uses a lookup table f(x)=> number_of_bits. Where x is a 8bit number
 *  (0 to 255).
 */
int Train::hammingDistance(const Mat &desc1, const Mat &desc2)
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





void Train::selectPoints(Mat &image,
                         Mat &M,
                         vector<Point2f> &corners,
                         Size refImgSize,
                         Ptr<FeatureDetector> &detector,
                         Ptr<DescriptorExtractor> &extractor,
                         vector<KeyPoint> &kps,
                         vector<uint16_t> &indxs,
                         Mat &desc)
{
    
    unsigned int MAX_KEYPOINTS = 1000;
    detector->detect(image, kps);
    vector<Point> bpKps;
    sort(kps.begin(), kps.end(), keyPointOrderingByResponse);
    kps.resize(std::min((unsigned int)MAX_KEYPOINTS,
                        (unsigned int)kps.size()));
    projectKeypoints(kps, M.inv(), bpKps);
    filterKeyPoints(refImgSize, corners, kps, bpKps);
    extractor->compute(image, kps, desc);
    projectKeypoints(kps, M.inv(), bpKps);
    indxs.resize(kps.size(), 0);
}


void Train::selectMostStableKeypoints(const vector<vector<KeyPoint>>& keypoints,
                                      const vector<vector<uint16_t>> &indices,
                                      const vector<Mat> &descriptors,
									  const vector<Mat>&transfMatrix,
								      const vector<vector<Point2f> >& corners,
								      int maxPointsPerView,
                                      Size refImgSize,
                                      Ptr<FeatureDetector> &detector,
                                      Ptr<DescriptorExtractor> &extractor,
                                      vector<StablePoint> &bestPoints)
{
    
	vector<vector<int> > grid(refImgSize.height,
                        vector<int> (refImgSize.width, -1));
    
	for ( int i = 0; i < keypoints.size(); ++i )
	{
		const vector<KeyPoint> &viewKeyPoints = keypoints[i];
		vector<Point> backProjPoints;
        projectKeypoints(viewKeyPoints,transfMatrix[i].inv(), backProjPoints);
		const Mat &viewDescriptors = descriptors[i];
        const vector<uint16_t> idxs = indices[i];
		//find the points and update them
		for ( int j = 0; j < backProjPoints.size(); j++ )
		{
			Point &p    = backProjPoints[j];
			const KeyPoint &k = viewKeyPoints[j];
			Mat d       = viewDescriptors.row(j);
            const uint16_t index = idxs[j];
            
			int idx = getNearbyPoint(p, grid);
			//no nearby model found
			if ( idx == -1 )
			{
				//add new point
				bestPoints.push_back(StablePoint(p, k, i, d, index));
				idx = (int)bestPoints.size() - 1;
				grid[p.y][p.x] = idx;
			}
			//update existing point
			else
			{
				bestPoints[idx].viewPointPts.push_back(k);
				bestPoints[idx].imageNumber.push_back(i);
                bestPoints[idx].viewPointPtDescriptors.push_back(d);
                bestPoints[idx].indices.push_back(index);
				continue;
			}
		}
	}
    
    
	/*
	 * We now sort bestPoints based on the number of votes for each refPnt and select the TOP-N points
	 */
    
	sort(bestPoints.begin(), bestPoints.end(), compareStablePoints);
	bestPoints.resize(std::min((unsigned int)maxPointsPerView,
                               (unsigned int)bestPoints.size()));
    
	//bestPoints.clear();
	for ( unsigned int i=0; i<grid.size(); i++ )
		grid[i].clear();
	grid.clear();
	return;
}

/**
 * Rotate an image
 */
void Train::rotateImage(cv::Mat& src, double angle, cv::Mat& dst, Mat &R)
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(len/2., len/2.);
    R = cv::getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, R, cv::Size(len, len));
}

/**
 * Rotate an image
 */
void Train::rotateAtCenter(cv::Mat& src, double angle, cv::Mat& dst, Mat &R)
{
    
    float a  = src.cols / 2.0;
    float b  = src.rows / 2.0;
    float c  = sqrt( a * a + b * b);
    
    int topFrame   = ceil(c - b);
    int rightFrame = ceil(c - a);
    
    Size newSize(src.rows + (2 * topFrame),
                 src.cols + (2 * rightFrame));
    
    Mat canvas = Mat::zeros(newSize,src.type());
    Mat area   = canvas(Rect(topFrame,rightFrame, src.cols, src.rows));
    src.copyTo(area);
    
    rotateImage(canvas, angle, dst, R);
}
