// IFeature.h : main header file for the IFeature DLL
//

#pragma once

#ifndef __AFXWIN_H__
#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


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
#include "commdef.h"
#include "commpub.h"
#include "ModelFile.h"

#include <map>
#include <string>

using namespace std;

typedef struct _Featurex_
{
	MFEATURE* pFeature;
	IplImage* pFImag[8];
	vector<vector<cv::KeyPoint>> keypointsArray;
	vector<cv::Mat> descriptorsArray;
	vector<int> nums_pointsIntersts;                    //����ģ��ͼƬ��ֵ����������Ŀ;
	_Featurex_()
	{
		pFeature = NULL;
		for(int i=0; i<8; i++)
			pFImag[i] = NULL;
		keypointsArray.clear();
		descriptorsArray.clear();
		nums_pointsIntersts.clear();
	}
}MFEATUREX, *PMFEATUREX;
typedef map<string, MFEATUREX*> MAPSTFEATUREX;
typedef map<string, MFEATUREX*>::iterator  ITERST;


typedef struct _VData_
{
	int iFlag;
	int iThis;
	_VData_()
	{
		iFlag = 0;
		iThis = 0;
	}
}VDATA;



typedef struct _DenseData_
{
	int index_pix;
	int dense_val;
	_DenseData_(int in_index_pix, int in_value_dense)
	{
		index_pix = in_index_pix;
		dense_val = in_value_dense;
	}
	_DenseData_()
	{
		index_pix = 0;
		dense_val = 0;
	}
}DENSEDATA;




//Ԥ������ ģ����Ϣ
class CTemplateInfor
{
public:
	void SetImage(cv::Mat inImage)
	{
		m_image = inImage;
	}

	void SetRawImage(cv::Mat rawImage)
	{
		m_image_raw = rawImage;
	}

	void SetSealPoint(POINT pt_seal)
	{
		m_pt.x = pt_seal.x;
		m_pt.y = pt_seal.y;
	}
		
	//ͳ�Ʋü����ܼ��е�ͶӰ;
	//image_type: 
	int CalacMaxDenseLineProj(cv::Mat image, CString path_log, std::vector<int>& result_proj_line, int iReserve)
	{
		cv::Mat image_grey, image_bin;
		cv::cvtColor(image, image_grey, CV_BGR2GRAY);
		int blockSize = 25;
		int constValue = 10;	
		cv::adaptiveThreshold(image_grey, image_bin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
		if (iReserve & 2)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_�ܼ��ж�ֵͼ.jpg", (LPSTR)(LPCTSTR)path_log, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_bin);
		}

		//ͳ�ư�ɫ�����طֲ�;
		const int width = image_bin.cols;
		const int height = image_bin.rows;
		
		//ͼƬ����Ҫ������ͶӰ��
		if(height > width)
		{			
			//����ͶӰ
			int *projectValArry = new int[height];
			memset(projectValArry, 0, sizeof(int)*height);
			//����ÿһ�е�ͼ��Ҷ�ֵ������ÿһ��255��ֵ
			for (int row = 0; row < height; ++row)		
			{
				for (int col = 0; col < width; ++col)
				{
					int perPixelValue = image_bin.at<uchar>(row, col);
					if (perPixelValue == 255)//����Ǻڵװ���
					{
						projectValArry[row]++;
					}
				}
				result_proj_line.push_back(projectValArry[row]);
			}
			delete[] projectValArry;
		}
		else
		{	
			int *projectValArry = new int[width];
			memset(projectValArry, 0, sizeof(int)*width);
			//����ͶӰ
			//����ÿһ�е�ͼ��Ҷ�ֵ������ÿһ��255��ֵ
			for (int col = 0; col < width; ++col)
			{
				for (int row = 0; row < height; ++row)
				{
					int perPixelValue = image_bin.at<uchar>(row, col);
					if (perPixelValue == 255)//����Ǻڵװ���
					{
						projectValArry[col]++;
					}
				}
				result_proj_line.push_back(projectValArry[col]);
			}
			delete[] projectValArry;
		}
		return 0;
	}


	int CalacFeatureProj(int image_type, CString path_log, int iReserve)
	{
		cv::Mat image_grey, image_bin;
		cv::cvtColor(m_image, image_grey, CV_BGR2GRAY);
		
		int blockSize = 25;
		int constValue = 10;
		if(image_type == 0)
		{
			//cv::threshold(image_grey, image_bin, 50, 255, CV_THRESH_BINARY_INV);
			blockSize = 25;
			constValue = 10;
		}
		else
		{
			//cv::threshold(image_grey, image_bin, 50, 255, CV_THRESH_BINARY_INV);
			blockSize = 25;
			constValue = 10;
		}
		cv::adaptiveThreshold(image_grey, image_bin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
		if (iReserve & 2)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_ͶӰ��ֵͼ.jpg", (LPSTR)(LPCTSTR)path_log, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_bin);
		}


		//ͳ�ư�ɫ�����طֲ�;
		const int width = image_bin.cols;
		const int height = image_bin.rows;
		
		if(height > width)
		{
			m_direction_image = 1;
			//����ͶӰ
			int *projectValArry = new int[height];
			memset(projectValArry, 0, sizeof(int)*height);
			//����ÿһ�е�ͼ��Ҷ�ֵ������ÿһ��255��ֵ
			for (int row = 0; row < height; ++row)		
			{
				for (int col = 0; col < width; ++col)
				{
					int perPixelValue = image_bin.at<uchar>(row, col);
					if (perPixelValue == 255)//����Ǻڵװ���
					{
						projectValArry[row]++;
					}
				}
				m_feature_proj.push_back(projectValArry[row]);
			}
			delete[] projectValArry;
		}
		else
		{
			m_direction_image = 0;
			int *projectValArry = new int[width];
			memset(projectValArry, 0, sizeof(int)*width);
			//����ͶӰ
			//����ÿһ�е�ͼ��Ҷ�ֵ������ÿһ��255��ֵ
			for (int col = 0; col < width; ++col)
			{
				for (int row = 0; row < height; ++row)
				{
					int perPixelValue = image_bin.at<uchar>(row, col);
					if (perPixelValue == 255)//����Ǻڵװ���
					{
						projectValArry[col]++;
					}
				}
				m_feature_proj.push_back(projectValArry[col]);
			}
			delete[] projectValArry;
		}
		return 0;
	}

	//�����ģ�壬���ܼ�������
	//�������ܣ� ��������� ����ĸ����� ����ı߳�
	int maxDenseZone(std::vector<cv::Rect>& out_rects, int rect_num, int length_pix);

	void calacError()
	{
		if(m_direction_image == 1)
		{
			m_row_pixel_error = m_image.cols/4;
			//m_rows_num_error= m_image.rows/14;
			m_rows_num_error= m_image.rows/5.0;
		}
		else
		{
			m_row_pixel_error = m_image.rows/4;
			//m_rows_num_error= m_image.cols/14;
			m_rows_num_error= m_image.rows/5.0;
		}
	}
	
public:
	cv::Mat m_image;                          //���ݲ�ɫͼ
	cv::Point m_pt;                           //ӡ�µ�λ��
	std::vector<int> m_feature_proj;          //ͶӰ������;
	int m_direction_image;                    //ģ��ͼ�ķ��� �� 0 �� ���� 1

	cv::Mat m_image_raw;                      //ԭʼͼ

	std::string m_strID;
	//����ÿ��ģ��Ĵ�С��һ�������ֵҲ�ǲ�һ���ĵģ�
	int m_row_pixel_error;
	int m_rows_num_error;

};




extern "C" {
	typedef  int (WINAPI *CVTIMAGE)(const char *pInFile, const char *pSaveFile, int *pArr, DWORD imagetype);
	//typedef  int (WINAPI *CVTIMAGEX)(BYTE *buffer, DWORD size, DWORD inimagetype, const char *pSaveFile, int *pArr, char *buftxt, DWORD outimagetype, char *pTessdata);
	typedef  int (WINAPI *CVTIMAGEX)(BYTE *buffer, DWORD size, DWORD inimagetype, const char *pSaveFile, int *pArr, char *buftxt, DWORD outimagetype, char *pTessdata, char* pathFeaturex);
	typedef  int (WINAPI *CVTSAVEIMAGE)(BYTE *buffer, DWORD size, DWORD inimagetype, const char *pSaveFile, DWORD outimagetype, BYTE bQuality, long xDpi, long yDpi);
	typedef int (WINAPI *CVTSAVEIMAGETOMEM)(BYTE *buffer, DWORD size, DWORD inimagetype, char *pSaveFile, DWORD outimagetype, BYTE bQuality, long xDpi, long yDpi);

	//typedef  int (WINAPI *CVTGENERATESEALCODEBARIMAGETOFILE)(char* szTradeCode, char* szImageFile);	
	//typedef  int (WINAPI *CVTGENERATESEALCODEBARIMAGETOMEM)(char* szTradeCode, BYTE *buffer, DWORD &size);
	typedef int (WINAPI *CVTGENERATESEALCODEBARIMAGETOMEM)(char* szTradeCode, BYTE *buffer, DWORD &size);
	//typedef  int (WINAPI *COMPARESEALCODE)(cv::Mat i_stand_image, cv::Mat i_real_image, std::string& result_sealCode);


	typedef  int (WINAPI *CALCFEATUREVALUE)(IplImage * pImg);
	typedef  int (WINAPI *COMPAREFEATUREVALUE)(IplImage *img1, IplImage *img2, double dvalue[3],  PKeyArea pKA);
	typedef int (WINAPI *COMPAREFEATUREVALUENEW)(IplImage *img1, IplImage *img2, double dvalue[3], PMFEATUREX q, int index);
	typedef  void (WINAPI *FEARELEASEIMAGE)(IplImage *pImg);
	typedef  IplImage* (WINAPI *ROTATEIMAGEX)(IplImage* img, double degree, CvPoint2D32f center, const CvPoint i_pt[4], CvPoint r_pt[4]);
	typedef  IplImage* (WINAPI *CAPIMAGEANDCALCANGLE)(IplImage *pImg, const CvBox2D rect, const CvRect ROI_rect, CvPoint i_pt[4], CvPoint2D32f &center, double &dAngle);

	//��ά���йص�
	//QR_GenerateQRCode
	typedef int (WINAPI *QR_GENERATEQRCODE)(char* szSourceSring, char* out_fileName);
	//qr_recognize
	typedef  int (WINAPI *QR_RECOGNIZE)(cv::Mat in_image, std::string& result_ocr, std::vector<cv::Point>& out_ptQR);
	typedef  int (WINAPI *QR_RECOGNIZEX)(cv::Mat in_image, std::string& result_ocr, std::vector<cv::Point>& out_ptQR);
	//�ɵ���֤��Ľӿ�
	typedef int (*CALL_GenerateVerifyCodeToMemFile)(int InPrintTypeCode, char* InTradeCode,int  FontSize, char* pMemFile,int Max_MemFile,int iFileFormat,char* szDir,bool bPrintLab,int LineCharCount );

}
// CIFeatureApp
// See IFeature.cpp for the implementation of this class
//

enum CHECK_STATE
{
	CHECK_UNKNOW = 0,
	CHECK_BEGIN,
	CHECK_END,
	CHECK_ERR
};

#define  MAX_THR 8  

class CCameraDS;

class CIFeatureApp : public CWinApp
{
public:
	CIFeatureApp();

	// Overrides
public:
	virtual BOOL InitInstance();
	
	DECLARE_MESSAGE_MAP()

public:
	CString m_strPath;
	CString m_strExePath;
	MAPSTFEATUREX m_map;

	//HMODULE m_hCoreModule; 
	HMODULE m_hCvtModule; 
	HMODULE m_hGenVerifyCode;

	HMODULE m_hQRCode;
	HMODULE m_hQRocr;
	CVTIMAGE cvtImage;
	CVTIMAGEX cvtImagex;
	CVTSAVEIMAGE cvtSaveImage; 
	CVTSAVEIMAGETOMEM  cvtSaveImageToMem;
	CVTGENERATESEALCODEBARIMAGETOMEM cvtGenerateSealCodeBarImageToMem;
	//CVTGENERATESEALCODEBARIMAGETOFILE cvtGenerateSealCodeBarImageToFile;
	
	//COMPARESEALCODE compareSealCode;

	//CALCFEATUREVALUE calcFeatureValue;
	//COMPAREFEATUREVALUE compareFeatureValue;
	//COMPAREFEATUREVALUENEW compareFeatureValueNew;
	//ROTATEIMAGEX rotateImagex;
	//CAPIMAGEANDCALCANGLE capImageAndCalcAngle;
	//FEARELEASEIMAGE feaReleaseImage;
	
	QR_GENERATEQRCODE QR_GenerateQRCode;
	QR_RECOGNIZE QR_Recognize;
	QR_RECOGNIZEX QR_Recognizex;
	CALL_GenerateVerifyCodeToMemFile GenerateVerifyCodeToMemFile;


public:
	HANDLE m_hCameraThread;
	HANDLE m_hCameraHandle;
	DWORD m_CameraThreadID;
	DWORD m_WaitTimeout;
	CString m_szCameraName;
	CCameraDS *m_pcap;

public:
	void IniImgConfig(CString in_modelFileName);
private:
	//void IniImgConfig();
	void ReleaseMap();
	CHECK_STATE m_CheckState;

public:
	uchar *m_pData;
	int *m_psize;
	BOOL m_bQuit;
	RCNDATA *m_pRcnData;
	double m_maxSim;

	IplImage* m_pdst[MAX_THR];
	double m_degree;
	int m_iReserve;
	bool m_bFeaRight;
	int m_iArrFlag[MAX_THR];
	VDATA m_vData[MAX_THR];

public:
	int m_iLeft;
	int m_iTop;
	int m_iBottom;
	int m_iRight;
	int m_iCameraIndex;
	int m_iRotate; 
	double m_dFocus;
	double m_dExposure;

	int m_iXScaleplate;
	int m_iYScaleplate;
	int m_iBackupLeft;
	int m_iBackupTop;
	int m_iBackupBottom;
	int m_iBackupRight;
	double m_dBackupFocus;
	double m_dBackupExposure;


	int min_threshold;
	//int match_keyPoint;

	//double keyPointProportion_0_15_m;
	//double keyPointProportion_15_30_m;
	//double keyPointProportion_30_80_m;
	//double keyPointProportion_80_ul_m;

	//int keyPointErrorDown;
	//int keyPointErrorUp;
	//int minPercentPointFirst;
	//int minPercentPointSecond;
	//int minPercentPointThird;

	int Bright;


	FILE *pFileLog;


	//ʶ���ģ��ü�ͼ
	cv::Mat m_image_clipper; 
	//�Ƿ���СƱ��
	bool m_isSmall;
	int VerifyCodeRotate; // 0 ��ʱ����ת 1 ˳ʱ����ת 2 ����Ҫ��ת 3��ά��

	int m_nYzmX;
	int m_nYzmY;
	int m_OffSet_x;
	int m_OffSet_y;
	int m_OffSet_w;
	int m_OffSet_h;
	int m_idebug;

	//��ά����ص�
	int m_QRX;
	int m_QRY;
	int m_QRWidth;

	double m_pixNumPerMm;

	int m_minDistence;

	int m_width_camera;
	int m_height_camera;

	int DeltaXMin;
	int DeltaXMax;
	int DeltaYMin;
	int DeltaYMax;

	double m_scaleVerifyCode;

	int m_A4Width;
	int m_A4Height;

	int m_QRRight;
	int m_QRDown;


	int Percent;
	double m_scaleText;

	int distenceChars;
	int errorNumDarkPixel;
	int keyPointsError;

	CString m_pathModelFile;
	CString m_modelFileNameHead;
	CString m_modelFileNameBranch;
	BOOL m_bVerifyCodeBinPW;
	int m_iDirectionTitle;       //ƾ֤�ķ��� 0:�� 1���� 2:�� 3����
	CString m_IFeatureName;
	CString m_GenVerifyCodeName;
	CString m_CvtImageName;
	CString m_QRocrName;
	int m_CameraType;
	int m_VerifyCodeBinValue;   //��֤�����⴦��ʱ����ֵ��ʹ�õ�ֵ;

public:
	virtual int ExitInstance();
	CHECK_STATE GetCheckState() const
	{
		return m_CheckState;
	}

	void SetCheckState( CHECK_STATE state)
	{
		m_CheckState = state;
	}
	void putLog(char* logMessage);	
	PMFEATUREX GetMapData();

public:
	cv::Rect m_zoneInpad;   //ӡ�������
	cv::Rect m_zonea4;      //Ʊ������
	std::vector<CTemplateInfor> m_templateInforLists;     //Ԥ������ģ����Ϣ;
	int m_errorMaxProj;     //ÿ��ͶӰ�����ƫ��
	int m_errorMaxLine;     //��ƪ�������ƫ��;

	//��ʱʹ��
	CString m_pathPreAdj;
	cv::Point m_ptSealTemplate; //ģ��ӡ�µ�λ��
};

//ÿһ����ĺ����Ա�ǩ���Լ��ھӵĸ���
class CResultLabel
{
public:
	CResultLabel()
	{
		isValid = false;
		num_neighbour = 0;
		isMaxConcentrated = false;
		isProcessedClassify = false;
	}
	bool isValid;
	unsigned int num_neighbour;
	bool isMaxConcentrated;
	bool isProcessedClassify;
};



