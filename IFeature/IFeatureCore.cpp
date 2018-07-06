// IFeatureCore.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "IFeature.h"
#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "CxRectangle.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


#define RANSREPROJTHRESHOLD 7

using namespace std;
using namespace cv;


int calcFeatureValueCore(IplImage * pImg)
{
	if(!pImg)
		return -1;

	vector<KeyPoint> keypoints1;
	keypoints1.clear();
	try
	{
		cv::initModule_nonfree();
		bool isWarpPerspective = 0;
		char buf[5];
		memset(buf, '\0', 5);
		buf[0] = 83;
		buf[1] = 73;
		buf[2] = 70;
		buf[3] = 84;
		string s = buf;
		Ptr<FeatureDetector> detector = FeatureDetector::create( s);
		Mat imgm(pImg);	
		detector->detect( imgm, keypoints1 );
	}
	catch (...)
	{
		return -2;
	}	
	return keypoints1.size();
}

void simpleMatchingCore( Ptr<DescriptorMatcher>& descriptorMatcher,
					const Mat& descriptors1, const Mat& descriptors2,
					vector<DMatch>& matches12 )
{
	vector<DMatch> matches;
	descriptorMatcher->match( descriptors1, descriptors2, matches12 );
}

void crossCheckMatchingCore( Ptr<DescriptorMatcher>& descriptorMatcher,
						const Mat& descriptors1, const Mat& descriptors2,
						vector<DMatch>& filteredMatches12, int knn=1 )
{
	filteredMatches12.clear();
	vector<vector<DMatch> > matches12, matches21;
	descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
	descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );
	if(matches21.size() < 1) return;
	for( size_t m = 0; m < matches12.size(); m++ )
	{
		bool findCrossCheck = false;
		for( size_t fk = 0; fk < matches12[m].size(); fk++ )
		{
			DMatch forward = matches12[m][fk];

			for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
			{
				DMatch backward = matches21[forward.trainIdx][bk];
				if( backward.trainIdx == forward.queryIdx )
				{
					filteredMatches12.push_back(forward);
					findCrossCheck = true;
					break;
				}
			}
			if( findCrossCheck ) break;
		}
	}
}


void warpPerspectiveRandCore( const Mat& src, Mat& dst, Mat& H, RNG& rng )
{
	H.create(3, 3, CV_32FC1);
	H.at<float>(0,0) = rng.uniform( 0.8f, 1.2f);
	H.at<float>(0,1) = rng.uniform(-0.1f, 0.1f);
	H.at<float>(0,2) = rng.uniform(-0.1f, 0.1f)*src.cols;
	H.at<float>(1,0) = rng.uniform(-0.1f, 0.1f);
	H.at<float>(1,1) = rng.uniform( 0.8f, 1.2f);
	H.at<float>(1,2) = rng.uniform(-0.1f, 0.1f)*src.rows;
	H.at<float>(2,0) = rng.uniform( -1e-4f, 1e-4f);
	H.at<float>(2,1) = rng.uniform( -1e-4f, 1e-4f);
	H.at<float>(2,2) = rng.uniform( 0.8f, 1.2f);

	warpPerspective( src, dst, H, src.size() );
}


int doIterationCore(/* const Mat& img1, */Mat& img2, bool isWarpPerspective,
				vector<KeyPoint>& keypoints1, const Mat& descriptors1, vector<KeyPoint>& keypoints2, 
				Ptr<FeatureDetector>& detector, Ptr<DescriptorExtractor>& descriptorExtractor,
				Ptr<DescriptorMatcher>& descriptorMatcher, double ransacReprojThreshold, PKeyArea pKA)
{
	Mat H12;
	Mat descriptors2;
	descriptorExtractor->compute( img2, keypoints2, descriptors2 );

	vector<DMatch> filteredMatches;
	crossCheckMatchingCore( descriptorMatcher, descriptors1, descriptors2, filteredMatches, 1 );

	if(filteredMatches.size() < 6) 
		return 0;

	vector<int> queryIdxs( filteredMatches.size() ), trainIdxs( filteredMatches.size() );
	for( size_t i = 0; i < filteredMatches.size(); i++ )
	{
		queryIdxs[i] = filteredMatches[i].queryIdx;
		trainIdxs[i] = filteredMatches[i].trainIdx;
	}

	vector<Point2f> points1; KeyPoint::convert(keypoints1, points1, queryIdxs);
	vector<Point2f> points2; KeyPoint::convert(keypoints2, points2, trainIdxs);
	if( !isWarpPerspective && ransacReprojThreshold >= 0 )
	{
		H12 = findHomography( Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold );
	}

	if( !H12.empty() ) // filter outliers
	{
		vector<char> matchesMask( filteredMatches.size(), 0 );
		Mat points1t; 
		perspectiveTransform(Mat(points1), points1t, H12);
		double maxInlierDist = ransacReprojThreshold < 0 ? RANSREPROJTHRESHOLD : ransacReprojThreshold;
		for( size_t i1 = 0; i1 < points1.size(); i1++ )
		{
			if( norm(points2[i1] - points1t.at<Point2f>((int)i1, 0)) <= maxInlierDist ) // inlier
					matchesMask[i1] = 1;
		}
		return countNonZero(matchesMask);
	}
	else
	{
		return 0;
	}
}

void RotateImageCore(IplImage *src,IplImage *dst,CvPoint center,float angle,float factor)
{
	//以点center为旋转中心，对src旋转angle度并缩放factor倍。
	float m[6];
	CvMat mat=cvMat(2,3,CV_32FC1,m);
	m[0] = (float)(factor*cos(-angle*CV_PI/180.));
	m[1] = (float)(factor*sin(-angle*CV_PI/180.));
	m[2] = center.x;
	m[3] = -m[1];
	m[4] = m[0];
	m[5] = center.y;
	cvSetZero(dst);
	cvGetQuadrangleSubPix(src,dst,&mat);
}

//imag1 : search zone
//img2: feature zone
int compareFeatureValueNewCore(IplImage *img1, IplImage *img2, double dvalue[5], PMFEATUREX q, int index)
{
	dvalue[0] = 0.0;
	dvalue[1] = 0.0;
	if(!img1 || !img2)
	{
		return -1;
	}
	try
	{

		bool isWarpPerspective = 0;
		double ransacReprojThreshold = RANSREPROJTHRESHOLD;

		char buf[5];
		memset(buf, '\0', 5);
		buf[0] = 83;
		buf[1] = 73;
		buf[2] = 70;
		buf[3] = 84;
		string s = buf;

		Ptr<FeatureDetector> detector = FeatureDetector::create( s);
		Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create( s);//"SURF");
		char buf2[7];
		memset(buf2, '\0', 7);
		buf2[0] = 70;
		buf2[1] = 108;
		buf2[2] = 97;
		buf2[3] = 110;
		buf2[4] = 110;
		buf2[5] = 66;
		s = buf2;
		s+="ased";
		Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create( s);//"FlannBased");//"BruteForce");

		Mat imgm1(img2);
		Mat imgm2(img1);

		//一些特征匹配低，是由于模板清晰，而搜索区间模糊导致的;对imgm2 进行锐化处理;
		/////
		cv::Mat gaussiarImage;
		cv::GaussianBlur(imgm2, gaussiarImage, cv::Size(0, 0), 3);
		cv::addWeighted(imgm2, 1.5, gaussiarImage, -0.5, 0, gaussiarImage);
		imgm2 = gaussiarImage;

		if (0)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("D:\\Image%04d%02d%02d_%02d%02d%02d_%03d_锐化图.jpg",  st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			//cvSaveImage((LPSTR)(LPCTSTR)sFileName, grey_Mat);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, gaussiarImage);
		}

		/////

		vector<KeyPoint> keypoints1;
		keypoints1 = q->keypointsArray[index];
		dvalue[0] = keypoints1.size();

		if(keypoints1.size() < 8) 
			return -3;

		Mat descriptors1;
		descriptors1 = q->descriptorsArray[index];	


		//增加二值图片特征点计算;
		//cv::Mat image_bin_search;
		//int blockSize = 25;
		////int constValue = 70;
		//int constValue = 10;
		////cv::adaptiveThreshold(imgm2, image_bin_search, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
		//cv::adaptiveThreshold(imgm2, image_bin_search, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blockSize, constValue);
		//vector<cv::KeyPoint> keypoints_binImageSearch;
		//detector->detect( image_bin_search, keypoints_binImageSearch);
		//int num_search_feature = 0;
		//num_search_feature = keypoints_binImageSearch.size();
		//int num_template_feature = 0;
		//num_template_feature = q->nums_pointsIntersts[index];
		//dvalue[3] = num_template_feature;
		//dvalue[4] = num_search_feature;

		//if(num_search_feature - num_template_feature  > 150)
		//{
		//	return -2;
		//}
		//if(num_search_feature < num_template_feature)
		//{
		//	return -3;
		//}

		vector<KeyPoint> keypoints2;
		detector->detect(imgm2, keypoints2);
		dvalue[1] = keypoints2.size();

		if(keypoints2.size() < 8) 
			return -4;
		
		//if((int)dvalue[1] < (int)dvalue[0] - 22)
		//	return -5;
		
		//BOOL bForceCompute = q->pFeature->bFroceCompute;
		//if( (bForceCompute == FALSE) && abs((int)dvalue[1] - (int)dvalue[0]) > 125)
		//	return -6;
		
		dvalue[2] = doIterationCore(/* imgm1, */imgm2, isWarpPerspective, keypoints1, descriptors1,
			keypoints2, detector, descriptorExtractor, descriptorMatcher, ransacReprojThreshold, (q->pFeature->ka)+index);
	}
	catch (...)
	{
		return -2;
	}
	return 0;
}


//旋转图像内容不变，尺寸相应变大
IplImage* rotateImagexCore(IplImage* img, double degree, CvPoint2D32f center, const CvPoint i_pt[4], CvPoint r_pt[4])
{
	double angle = degree  * CV_PI / 180.; // 弧度  
	double a = sin(angle), b = cos(angle); 
	int width = img->width;  
	int height = img->height;  
	int width_rotate= int(height * fabs(a) + width * fabs(b));  
	int height_rotate=int(width * fabs(a) + height * fabs(b));  
	//旋转数组map
	// [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]
	// [ m3  m4  m5 ] ===>  [ A21  A22   b2 ]
	float map[6];
	CvMat map_matrix = cvMat(2, 3, CV_32F, map);  
	// 旋转中心
	//CvPoint2D32f center = cvPoint2D32f(width / 2, height / 2);  
	cv2DRotationMatrix(center, degree, 1.0, &map_matrix); 

	map[2] += (width_rotate - width) / 2;  
	map[5] += (height_rotate - height) / 2; 


	//计算旋转后对应点的坐标值
	for( int i = 0; i < 4; i++ )  
	{  
		double x = i_pt[i].x, y = i_pt[i].y;  
		double X = (map[0]*x + map[1]*y + map[2]);  
		double Y = (map[3]*x + map[4]*y + map[5]);  
		r_pt[i].x=cvRound(X);
		r_pt[i].y=cvRound(Y);
	}

	IplImage *img_rotate = cvCreateImage(cvSize(width_rotate, height_rotate), 8, img->nChannels); 
	//对图像做仿射变换
	//CV_WARP_FILL_OUTLIERS - 填充所有输出图像的象素。
	//如果部分象素落在输入图像的边界外，那么它们的值设定为 fillval.
	//CV_WARP_INVERSE_MAP - 指定 map_matrix 是输出图像到输入图像的反变换，
	cvWarpAffine(img, img_rotate, &map_matrix, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS, cvScalarAll(0)); 
	return img_rotate;
}

int BCalcAngleCore(IplImage* img, double degree, CvPoint2D32f center, const CvPoint i_pt[4])
{
	CvPoint r_pt[4];
	IplImage* pImgDst = rotateImagexCore(img, degree, center, i_pt, r_pt);

	CxRectangle xr(r_pt);
	CvRect I_rect;
	CvPoint pt1, pt2;
	pt1 = xr.GetTopLeftPoint();
	pt2 = xr.GetButtomRightPoint();
	I_rect.x = abs(pt1.x);
	I_rect.y = abs(pt1.y);
	I_rect.width = pt2.x - abs(pt1.x);
	I_rect.height = pt2.y - abs(pt1.y);

	cvSetImageROI(pImgDst, I_rect); //设置ROI区域 
	IplImage* RImg = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, img->nChannels); 
	cvCopy(pImgDst, RImg); 
	cvResetImageROI(pImgDst); 
	cvReleaseImage(&pImgDst); 
	if(RImg->width < RImg->height)  //说明图像是垂直的
	{
		cvResetImageROI(RImg);
		cvReleaseImage(&RImg); 
		return TRUE;
	}
	cvResetImageROI(RImg); 
	cvReleaseImage(&RImg); 
	return FALSE;
}

IplImage* capImageAndCalcAngleCore(IplImage *pImg, const CvBox2D rect, const CvRect ROI_rect, CvPoint i_pt[4], CvPoint2D32f &center, double &dAngle)
{
	CvPoint2D32f rect_pts0[4]; 
	cvBoxPoints(rect, rect_pts0);   

	int npts = 4;       
	CvPoint rect_pts[4], *pt = rect_pts;
	for (int i=0; i<4; i++) 
	{  
		rect_pts[i]= cvPointFrom32f(rect_pts0[i]);
	}

	cvSetImageROI(pImg, ROI_rect); //设置ROI区域 
	IplImage* RoiImg = cvCreateImage(cvSize(ROI_rect.width, ROI_rect.height), 8, pImg->nChannels);
	cvCopy(pImg, RoiImg); 
	cvResetImageROI(pImg); 

	//SYSTEMTIME st;
	//GetSystemTime(&st);
	//CString sFileName;
	//sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_1.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour+8, st.wMinute, st.wSecond, st.wMilliseconds);
	//cvSaveImage((LPSTR)(LPCTSTR)sFileName, RoiImg);

	center.x = rect.center.x - ROI_rect.x;
	center.y = rect.center.y - ROI_rect.y;
	for( int i = 0; i < 4; i++ )  
	{  
		i_pt[i].x = rect_pts[i].x - ROI_rect.x;
		i_pt[i].y = rect_pts[i].y - ROI_rect.y;
	}

	float angle = rect.angle;	
	if(rect_pts[0].x > rect_pts[2].x)
	{
		angle = 90 + rect.angle;
	}				

	BOOL bAngle = BCalcAngleCore(RoiImg, angle, center, i_pt);
	if(bAngle)
		angle = 90 + angle;

	dAngle = angle;
	return RoiImg;
}
/*
void feaReleaseImage(IplImage *pImg)
{
	if(pImg)
		cvReleaseImage(&pImg);
}
*/





