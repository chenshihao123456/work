// IFeature.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include <shlwapi.h>
#include <stdlib.h>  
#include <stdio.h>  
#include <time.h>  
#include "CxRectangle.h"
#include "CvxText.h"
#include <locale>
#include <iostream>
#include <algorithm>
#include "strngs.h"
#include "baseapi.h"
#include <iostream>
#include "IFeature.h"
#include "bmp2ipl.h"
#include "vfw.h"
#include "winioctl.h"
#include "strmif.h"
#include "uuids.h"
#include "commpub.h"
#include "CameraDlg.h"
#include "pthread.h"
#include "camerads.h"




#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace cv;
using namespace std;

//#define SMALLVERIFYCODEZONE

#pragma comment(lib,"vfw32.lib")
#pragma comment(lib,"Strmiids.lib") 

//#define TESTDEMO



#define COMPUTE_IMAGEX_XDIM(xsize,bpp) ((bpp)>8 ? ((xsize)*(bpp)+7)/8 :((xsize)+8/(bpp)-1)/(8/(bpp)))



//--------------------------------------
int calcFeatureValueCore(IplImage * pImg);
int compareFeatureValueNewCore(IplImage *img1, IplImage *img2, double dvalue[3], PMFEATUREX q, int index);
IplImage* rotateImagexCore(IplImage* img, double degree, CvPoint2D32f center, const CvPoint i_pt[4], CvPoint r_pt[4]);
IplImage* capImageAndCalcAngleCore(IplImage *pImg, const CvBox2D rect, const CvRect ROI_rect, CvPoint i_pt[4], CvPoint2D32f &center, double &dAngle);
//--------------------------------------

void WINAPI DrawRectAndSave(cv::Mat img, std::vector<cv::Rect> rects);
int WINAPI compareSealCode(cv::Mat i_stand_image, cv::Mat i_real_image, std::string& result_sealCode, int *pMinOper);
int ldistance(const string source, const string target);

int cutVerifyCode(cv::Mat in_image, cv::Mat& out_image, int angle, cv::Rect in_sealCodeZone, int iReserve);
int WINAPI recognizeSealCode(cv::Mat img, std::string &result, std::vector<cv::Rect>& rects, int ocr_type);

int WINAPI compareSealCodeNoType(cv::Mat i_stand_image, cv::Mat i_real_image, std::string& result_sealCode, int *pMinOper);
int WINAPI recognizeSealCodeNoType(cv::Mat img, std::string &result, std::vector<cv::Rect>& rects, int ocr_type);
int WINAPI recognizeSealCodeNoTypeNoStdImg(cv::Mat img, std::string &result, std::string& result_1, std::vector<cv::Rect>& rects, float* pConfidence, int ocr_type);
int removeSpaceEnter(std::string i_str, std::string& o_str);

//int cut_imageROIBlock(cv::Mat inTestImg, cv::Mat inTemplateImg, cv::Mat& outTestROI, cv::Mat& outTemplateROI, int inI, int inJ, int inDebug);
int cut_imageROIBlock(cv::Mat inTestImg, cv::Mat inTemplateImg, cv::Mat& outTestROI, cv::Mat& outTemplateROI, int inRow, int inCol, int inDebug);

int cut_imageROIBlockFeature(cv::Mat inTestImg, cv::Mat inTemplateImg, cv::Mat& outTestROI, cv::Mat& outTemplateROI, int inRow, int inCol, int inDebug);
int calcFeaturePointNum(cv::Mat img);
int RCN_CalacUniteNum(cv::Mat in_image, int in_style_unite, int in_direct_unite, int& out_unite_num, int iReserve);

int RCN_GetCutPictureParagramInner(IplImage *pImg, cv::RotatedRect& rectPoint, cv::Rect& roi, int iReserve);

//int filterContoursFeatures(cv::Mat image_std, cv::Mat image_real, PMFEATURE p, int debug);
int filterContoursFeatures(cv::Mat image_std, cv::Mat image_real, PKeyArea pkey_Area, int searchImagebinValue, int debug);

//针对识别，重控号;
int recognizeImportantCodes(cv::Mat grey, char* fontName, std::string &result, std::vector<cv::Rect>& rects);
void filteredRed(const Mat &inputImage, Mat &resultGray, Mat &resultColor);
void filteredBlack(const Mat &inputImage, Mat &resultGray, Mat &resultColor);

void processImageByDark(cv::Mat image_color, cv::Mat &image_grey, int iReserve);
void processImageByRed(cv::Mat image_color, cv::Mat &image_bin, int iReserve);
void processImageByBlack(cv::Mat image_color, cv::Mat &image_bin, int iReserve);
void processImageByOther(cv::Mat image_color, cv::Mat &image_grey, int iReserve);


pthread_t g_ThreadID[MAX_THR];
pthread_mutex_t g_queue_cs;               //队列锁
static ITERST   g_curIter;                //队列头队列尾初始化   
static int  g_iQuit = 0;
void * process_map(void * pData);         //线程入口函数

pthread_mutex_t g_log_cs;                //算法日志鎖
pthread_mutex_t g_sim_cs;                //相似值鎖

//
//TODO: If this DLL is dynamically linked against the MFC DLLs,
//		any functions exported from this DLL which call into
//		MFC must have the AFX_MANAGE_STATE macro added at the
//		very beginning of the function.
//
//		For example:
//
//		extern "C" BOOL PASCAL EXPORT ExportedFunction()
//		{
//			AFX_MANAGE_STATE(AfxGetStaticModuleState());
//			// normal function body here
//		}
//
//		It is very important that this macro appear in each
//		function, prior to any calls into MFC.  This means that
//		it must appear as the first statement within the 
//		function, even before any object variable declarations
//		as their constructors may generate calls into the MFC
//		DLL.
//
//		Please see MFC Technical Notes 33 and 58 for additional
//		details.
//

// templated version of my_equal so it could work with both char and wchar_t
template<typename charT>
struct my_equal {
	my_equal(const std::locale& loc) : loc_(loc) {}
	bool operator()(charT ch1, charT ch2)
	{
		return std::toupper(ch1, loc_) == std::toupper(ch2, loc_);
	}
private:
	const std::locale& loc_;
};

// find substring (case insensitive)
template<typename T>
int ci_find_substr(const T& str1, const T& str2, const std::locale& loc = std::locale())
{
	T::const_iterator it = std::search(str1.begin(), str1.end(),
		str2.begin(), str2.end(), my_equal<T::value_type>(loc));

	if (it != str1.end())
		return it - str1.begin();
	else
		return -1; // not found
}

// CIFeatureApp

BEGIN_MESSAGE_MAP(CIFeatureApp, CWinApp)
END_MESSAGE_MAP()


// CIFeatureApp construction
CIFeatureApp::CIFeatureApp()
{
	m_map.clear();
	g_curIter = m_map.end();
	memset(m_iArrFlag, 1, sizeof(int)*MAX_THR);
	m_hCvtModule = NULL;
	m_hGenVerifyCode=NULL;
	cvtImage = NULL;
	cvtImagex = NULL;
	m_iLeft = 0;
	m_iTop = 0;
	m_iBottom = 0;
	m_iRight = 0;
	m_dFocus = 80.0;
	m_dExposure = -5.0;
	m_iRotate = 0;
	m_iXScaleplate = 0;
	m_iYScaleplate = 0;
	min_threshold = 80;
	pFileLog = NULL;
	m_pRcnData = NULL;
	m_maxSim = 0;
	Bright = 120;
	m_OffSet_x = 0;
	m_OffSet_y = 0;
	m_OffSet_w = 0;
	m_OffSet_h = 0;
	VerifyCodeRotate = 2;
	m_pixNumPerMm = 1.0;
	m_minDistence = 15;
	m_width_camera = 0;
	m_height_camera = 0;
	DeltaXMin = 15;
	DeltaXMax = 90;
	DeltaYMin = 228;
	DeltaYMax = 328;
	m_scaleVerifyCode = 1.0;
	m_A4Width = 2438;
	m_A4Height = 1752;
	m_QRRight = 132;
	m_QRDown = 427;
	Percent = 10;
	m_scaleText = 1.0;
	distenceChars = 20;
	errorNumDarkPixel = 1500;
	keyPointsError = 30;
	m_idebug = 0;
	m_pathModelFile = "";
	m_modelFileNameHead = "";
	m_modelFileNameBranch = "";
	m_bVerifyCodeBinPW = FALSE;
	m_iDirectionTitle = -1;
	m_CameraType = 0;
	m_errorMaxProj = 300; 
	m_errorMaxLine = 130;
	m_pathPreAdj = "";
	m_pcap = new CCameraDS();
}

// The one and only CIFeatureApp object

CIFeatureApp theApp;


PMFEATUREX CIFeatureApp::GetMapData()
{
	PMFEATUREX p = NULL;
	pthread_mutex_lock(&g_queue_cs);
	if (g_curIter != m_map.end())
	{
		p = (PMFEATUREX)g_curIter->second;
		g_curIter++;
	}
	else
	{
		p = NULL;
	}
	pthread_mutex_unlock(&g_queue_cs);
	return p;
}

void CIFeatureApp::ReleaseMap()
{
	g_iQuit = 1;
	for (int i = 0; i < MAX_THR; i++)
	{
		pthread_join(g_ThreadID[i], NULL);
	}
	pthread_mutex_destroy(&g_queue_cs);
	pthread_mutex_destroy(&g_log_cs);
	pthread_mutex_destroy(&g_sim_cs);
	if (m_pcap)
	{
		delete m_pcap;
		m_pcap = NULL;
	}
	ITERST  iter1, iter2;
	for (iter1 = m_map.begin(); iter1 != m_map.end(); )
	{
		iter2 = iter1;
		iter1++;
		PMFEATUREX p = iter2->second;
		m_map.erase(iter2);
		if (p)
		{
			if (p->pFeature)
			{
				delete p->pFeature;
				p->pFeature = NULL;
			}

			for (int i = 0; i < 8; i++)
			{
				if (p->pFImag[i])
				{
					cvReleaseImage(&p->pFImag[i]);
					p->pFImag[i] = NULL;
				}
			}
		}
		delete p;
	}
	m_map.clear();
}


/************************************************************************/
/* IniImgConfig读取模板配置函数
*/
/************************************************************************/
//
//void CIFeatureApp::IniImgConfig()
//{	
//	if(m_pathModelFile.IsEmpty())
//	{
//		m_pathModelFile = m_strExePath;
//	}
//	CString sPath;
//	//sPath.Format("%s\\Featurex\\*.dat", (LPSTR)(LPCTSTR)m_strExePath);
//	sPath.Format("%s\\Featurex\\*.dat", (LPSTR)(LPCTSTR)m_pathModelFile);
//	CString sDir;
//	//sDir.Format("%s\\Featurex\\", (LPSTR)(LPCTSTR)m_strExePath);
//	sDir.Format("%s\\Featurex\\", (LPSTR)(LPCTSTR)m_pathModelFile);
//	WIN32_FIND_DATA fd;
//	HANDLE hFind = ::FindFirstFile((LPSTR)(LPCTSTR)sPath, &fd);
//
//	cv::initModule_nonfree();
//	Ptr<FeatureDetector> detector = FeatureDetector::create( "SIFT");
//	Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create( "SIFT");//"SURF");
//	if ( hFind != INVALID_HANDLE_VALUE )
//	{
//		do{
//			if ( !(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) )
//			{
//				PMFEATUREX q = new MFEATUREX();
//				PMFEATURE p = new MFEATURE();
//				memset(p, 0, sizeof(MFEATURE));
//
//				q->pFeature = p;
//				CString sfile;
//				sfile.Format("%s%s", (LPSTR)(LPCTSTR)sDir, fd.cFileName);
//				FILE *fp = fopen((LPSTR)(LPCTSTR)sfile,"rb");  
//				if(fp)
//				{
//					size_t t = fread(p, sizeof(MFEATURE), 1, fp);  
//					if(t == 1)
//					{
//						PathRemoveExtension(fd.cFileName);
//						CString ssImg;
//						for(int i=0; i<p->nKeyAreaCount; i++)
//						{
//							//ssImg.Format("%s\\Featurex\\%s\\%02d.jpg", (LPSTR)(LPCTSTR)m_strExePath, p->strModNo, i+1);
//							ssImg.Format("%s\\Featurex\\%s\\%02d.jpg", (LPSTR)(LPCTSTR)m_pathModelFile, p->strModNo, i+1);
//							q->pFImag[i] = cvLoadImage((LPSTR)(LPCTSTR)ssImg, 0);
//
//							//提前算好特征点，以及描述符
//#if 1
//							vector<cv::KeyPoint> keypoints;
//							detector->detect( cv::Mat(q->pFImag[i]), keypoints);
//							q->keypointsArray.push_back(keypoints);
//
//							cv::Mat descriptors;
//							descriptorExtractor->compute( cv::Mat(q->pFImag[i]), keypoints, descriptors);
//							q->descriptorsArray.push_back(descriptors);
//
//#endif
//						}		
//						m_map.insert(pair<string, PMFEATUREX>(fd.cFileName, q));
//					}
//					else
//					{
//						AfxMessageBox(sfile);
//						delete q;
//						delete p;
//					}
//					fclose(fp);
//				}
//				else
//				{
//					delete q;
//					delete p;
//				}					
//			}
//		}while (::FindNextFile(hFind, &fd));
//		::FindClose(hFind);
//	}
//	
//	//int iLen = m_map.size(); 
//	//char szBuf[256];
//	//memset(szBuf, 0, 256);
//	//sprintf(szBuf, "加载模板总数： %d", iLen);
//	//theApp.putLog(szBuf);
//	
//}

void CIFeatureApp::IniImgConfig(CString in_modelFileName)
{
	if (m_pathModelFile.IsEmpty())
	{
		m_pathModelFile = m_strExePath;
	}

	CString fileAbsPath = m_pathModelFile + "/DFJY/" + in_modelFileName;
	//是否有文件;
	int ret_fileExist = PathFileExists(fileAbsPath);
	if (!ret_fileExist)
	{
		return;
	}

	cv::initModule_nonfree();
	Ptr<FeatureDetector> detector = FeatureDetector::create("SIFT");
	Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create("SIFT");//"SURF");

	CModelFile modelFile;
	//int iRet = modelFile.OpenModelFile(m_pathModelFile + "\\Featurex\\DfjyModel.dat");
	int iRet = modelFile.OpenModelFile(fileAbsPath);
	if (iRet != 0)
	{
		return;
	}
	modelFile.ReadModelFile();
	int numModel = modelFile.GetModelCount();

	/*char buf_message[1024];
	memset(buf_message, 0, 1024);
	sprintf(buf_message, "model num: %d", numModel);
	theApp.putLog(buf_message);*/

	ITERSTFILE itBeg = modelFile.m_map.begin();
	ITERSTFILE itEnd = modelFile.m_map.end();
	for (; itBeg != itEnd; itBeg++)
	{
		std::string ModelNo = itBeg->first;
		PMMODELFILE pMModel = itBeg->second;

		//memset(buf_message, 0, 1024);
		//sprintf(buf_message, "modelNO: %s", ModelNo.c_str());
		//theApp.putLog(buf_message);

		PMFEATUREX q = new MFEATUREX();
		PMFEATURE p = new MFEATURE();
		memset(p, 0, sizeof(MFEATURE));
		q->pFeature = p;

		*p = pMModel->pFeature;

		char*  imageAddressBase = pMModel->szIMGData;
		int offSetAddress = 0;
		for (int i = 0; i < p->nKeyAreaCount; i++)
		{
			//memset(buf_message, 0, 1024);
			//sprintf(buf_message, "keyArea: %d", i);
			//theApp.putLog(buf_message);

			//q->pFImag[i] = cvLoadImage((LPSTR)(LPCTSTR)ssImg, 0);
			int lenImage = *((int*)(imageAddressBase + i * 4));
			int imageFeatureAddress = (int)imageAddressBase + MAX_KEYAREA_COUNT * 4 + offSetAddress;

			char bufImage[10240];
			memset(bufImage, 0, 10240);
			memcpy(bufImage, (char*)imageFeatureAddress, lenImage);
			q->pFImag[i] = cvJpeg2Ipl(bufImage, lenImage);
			//提前算好特征点，以及描述符
#if 1
			vector<cv::KeyPoint> keypoints;
			detector->detect(cv::Mat(q->pFImag[i]), keypoints);
			q->keypointsArray.push_back(keypoints);
			cv::Mat descriptors;
			descriptorExtractor->compute(cv::Mat(q->pFImag[i]), keypoints, descriptors);
			q->descriptorsArray.push_back(descriptors);

			////增加二值图特征的计算;
			//cv::Mat image_bin_template;
			////int blockSize = 25;
			////int constValue = 70;			
			//int blockSize = 25;
			//int constValue = 10;
			//cv::adaptiveThreshold(cv::Mat(q->pFImag[i]), image_bin_template, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blockSize, constValue);
			//vector<cv::KeyPoint> keypoints_binImage;
			//detector->detect( image_bin_template, keypoints_binImage);
			//q->nums_pointsIntersts.push_back(keypoints_binImage.size());

			//if(ModelNo == "100000013")
			//{
			//	//cv::imwrite("E:\\01_bin.jpg", image_bin_template);
			//	cv::imwrite("E:\\01_src.jpg", cv::Mat(q->pFImag[i]));
			//}
#endif
			offSetAddress += *((int*)(imageAddressBase + i * 4));
		}
		m_map.insert(pair<string, PMFEATUREX>(ModelNo, q));
	}

	if (theApp.m_idebug == 1)
	{
		int iLen = m_map.size();
		char szBuf[256];
		memset(szBuf, 0, 256);
		sprintf(szBuf, "加载模板总数： %d", iLen);
		theApp.putLog(szBuf);
	}
}



HMODULE  DumpModuleFeature()  
{  
	HMODULE hMoudle = NULL;  
	GetModuleHandleEx(  
		GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,  
		(PCTSTR)DumpModuleFeature,  
		&hMoudle);  
	return hMoudle;
} 

// CIFeatureApp initialization

BOOL CIFeatureApp::InitInstance()
{
	CWinApp::InitInstance();
	m_bQuit = FALSE;
	m_pData = NULL;
	m_psize = NULL;
	m_WaitTimeout = 10;//INFINITE;
	char szDir[1024];
	memset(szDir, '\0', 1024);
	TCHAR szPath[MAX_PATH];


#ifdef TESTDEMO
	GetModuleFileName( GetModuleHandle("IFeature.dll"), szPath, MAX_PATH );
#else
	//GetModuleFileName(GetModuleHandle("ABC.Dfjy.Device.STAMP.IABCFeature.dll"), szPath, MAX_PATH);
	HMODULE hModule;	
	hModule = DumpModuleFeature();
	GetModuleFileName(hModule, szPath, MAX_PATH);

#endif
	GetLongPathName(szPath, szDir, MAX_PATH);
	PathRemoveFileSpec(szDir);		
	char confiLibPath[1024];
	memset(confiLibPath,0,sizeof(confiLibPath));
	sprintf_s(confiLibPath, "%s\\configLib.ini", szDir);

	char pathValue[1024];
	memset(pathValue, 0, 1024);
	int nSizePath = 1000;
	GetPrivateProfileString("LibraryName", "CvtImage", "", pathValue, nSizePath, confiLibPath);
	m_CvtImageName = pathValue;

	memset(pathValue, 0, 1024);
	GetPrivateProfileString("LibraryName", "GenVerifyCode", "", pathValue, nSizePath, confiLibPath);
	m_GenVerifyCodeName = pathValue;

	memset(pathValue, 0, 1024);
	GetPrivateProfileString("LibraryName", "QROcr", "", pathValue, nSizePath, confiLibPath);
	m_QRocrName = pathValue;

	SYSTEMTIME st;
	GetSystemTime(&st);
	CString sFileName;
	sFileName.Format("image%04d%02d%02d", st.wYear, st.wMonth, st.wDay);
	m_strPath.Format("%s\\%s", szDir, (LPSTR)(LPCTSTR)sFileName);

	m_pathPreAdj.Format("%s\\%s", szDir, "预审批结果");

	m_strExePath = szDir;
	CString sdll;
#ifdef TESTDEMO
	sdll.Format("%s\\cvtImage.dll", (LPSTR)(LPCTSTR)m_strExePath);
#else
	//sdll.Format("%s\\ABC.Dfjy.Device.STAMP.cvtABCImage.dll", (LPSTR)(LPCTSTR)m_strExePath);
	sdll.Format("%s\\%s", (LPSTR)(LPCTSTR)m_strExePath, (LPSTR)(LPCTSTR)m_CvtImageName);
#endif
	m_hCvtModule = LoadLibrary((LPSTR)(LPCTSTR)sdll);
	if (m_hCvtModule == NULL)
	{
		DWORD dw = GetLastError();
		CString s;
		s.Format("加载文件失败< %s >", (LPSTR)(LPCTSTR)sdll);
		AfxMessageBox(s);
		return FALSE;
	}
	cvtImage = (CVTIMAGE)::GetProcAddress(m_hCvtModule, "cvtImage");
	cvtImagex = (CVTIMAGEX)::GetProcAddress(m_hCvtModule, "cvtImagex");
	cvtSaveImage = (CVTSAVEIMAGE)::GetProcAddress(m_hCvtModule, "cvtSaveImage");
	cvtSaveImageToMem = (CVTSAVEIMAGETOMEM)::GetProcAddress(m_hCvtModule, "cvtSaveImageToMem");
	if (!cvtImage ||
		!cvtImagex ||
		!cvtSaveImage ||
		!cvtSaveImageToMem)
	{
		CString s;
		s.Format("%s版本错误.", sdll);
		AfxMessageBox(s);
		return FALSE;
	}
	//sdll.Format("%s\\ABC.Dfjy.Device.STAMP.GenABCVerifyCode.dll", (LPSTR)(LPCTSTR)m_strExePath);
	sdll.Format("%s\\%s", (LPSTR)(LPCTSTR)m_strExePath, (LPSTR)(LPCTSTR)m_GenVerifyCodeName);
	m_hGenVerifyCode = LoadLibrary((LPSTR)(LPCTSTR)sdll);
	if (m_hGenVerifyCode == NULL)
	{
		DWORD dw = GetLastError();
		CString s;
		s.Format("加载文件失败< %s >", (LPSTR)(LPCTSTR)sdll);
		AfxMessageBox(s);
		return FALSE;
	}
	cvtGenerateSealCodeBarImageToMem = (CVTGENERATESEALCODEBARIMAGETOMEM)::GetProcAddress(m_hGenVerifyCode, "cvtGenerateSealCodeBarImageToMemIFeature");
	if (!cvtGenerateSealCodeBarImageToMem)
	{
		CString s;
		s.Format("%s版本错误.", sdll);
		AfxMessageBox(s);
		return FALSE;
	}

	/*
	sdll.Format("%s\\QRGenerator.dll", (LPSTR)(LPCTSTR)m_strExePath);
	m_hQRCode = LoadLibrary((LPSTR)(LPCTSTR)sdll);
	if(m_hQRCode == NULL)
	{
	CString s;
	s.Format("加载文件失败< %s >", (LPSTR)(LPCTSTR)sdll);
	AfxMessageBox(s);
	return FALSE;
	}

	QR_GenerateQRCode= (QR_GENERATEQRCODE)::GetProcAddress(m_hQRCode, "QR_GenerateQRCode");
	if(!QR_GenerateQRCode )
	{
	AfxMessageBox("QRGenerator.dll版本错误。");
	return FALSE;
	}
	*/
	//二维码识别部分;
	//sdll.Format("%s\\ABC.Dfjy.Device.STAMP.QRocr.dll", (LPSTR)(LPCTSTR)m_strExePath);
	sdll.Format("%s\\%s", (LPSTR)(LPCTSTR)m_strExePath, (LPSTR)(LPCTSTR)m_QRocrName);
	m_hQRocr = LoadLibrary((LPSTR)(LPCTSTR)sdll);
	if(m_hQRocr == NULL)
	{
		CString s;
		s.Format("加载文件失败< %s >", (LPSTR)(LPCTSTR)sdll);
		AfxMessageBox(s);
		return FALSE;
	}

	QR_Recognize= (QR_RECOGNIZE)::GetProcAddress(m_hQRocr, "QR_Recognize");
	if(!QR_Recognize )
	{
		AfxMessageBox("QRocr.dll版本错误。");
		return FALSE;
	}

	QR_Recognizex= (QR_RECOGNIZEX)::GetProcAddress(m_hQRocr, "QR_Recognizex");
	if(!QR_Recognizex )
	{
		AfxMessageBox("QRocr.dll版本错误。");
		return FALSE;
	}

	CString  sFileName_esm;
	sFileName_esm.Format("%s\\Featurex\\esm.ini", (LPSTR)(LPCTSTR)m_strExePath);
	m_width_camera = GetPrivateProfileInt("Camera", "camera_width", 2592, sFileName_esm);
	m_height_camera = GetPrivateProfileInt("Camera", "camera_height", 1944, sFileName_esm);
	/*
	//char strDir[1024] = {0};
	//char szPath[1024] = {0};

	//GetModuleFileName( GetModuleHandle("GenerateVerifyCode.dll"), strDir, MAX_PATH );
	//GetLongPathName(szPath, strDir, MAX_PATH);
	//PathRemoveFileSpec(strDir);
	//strcat(strDir,"/GenerateVerifyCode.dll");
	sdll.Format("%s\\GenerateVerifyCode.dll", (LPSTR)(LPCTSTR)m_strExePath);
	HMODULE hSealModule = LoadLibrary(sdll);
	if(!hSealModule)
	{
	AfxMessageBox("load generateVerifyCode failed!");
	return FALSE;
	}
	GenerateVerifyCodeToMemFile = (CALL_GenerateVerifyCodeToMemFile)::GetProcAddress(hSealModule, "GenerateVerifyCodeToMemFile");
	if(!GenerateVerifyCodeToMemFile)
	{
	AfxMessageBox("load GenerateVerifyCodeToMemFile failed!");
	return FALSE;
	}
	*/
	//IniImgConfig();
	if (m_hCameraHandle == NULL)
	{
		m_hCameraHandle = ::CreateEvent(NULL, TRUE, FALSE, NULL);
		if (m_hCameraHandle == NULL)
		{
			return FALSE;
		}
	}

	g_curIter = m_map.end();
	//锁和条件变量初始化
	pthread_mutex_init(&g_queue_cs, NULL);
	pthread_mutex_init(&g_log_cs, NULL);
	pthread_mutex_init(&g_sim_cs, NULL);

	//生成服务线程
	for (int i = 0; i < MAX_THR; i++)
	{
		m_vData[i].iFlag = i;
		m_vData[i].iThis = (int)this;
		pthread_create(&g_ThreadID[i], NULL, (void*(*)(void*))process_map, &m_vData[i]);
	}

	return TRUE;
}








/************************************************************************/
/* RCN_GenImageTemplate模板特征值计算函数
pFile:  模板图片文件名称
ivalue: 特征值
iW:     图片宽度
iH:     图片高度
x1, y1  特征区域左上角坐标点
x2, y2  特征区域右下角坐标点
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_GenImageTemplate(char *pFile, int &ivalue, int &iW, int &iH, int x1, int y1, int x2, int y2)
{
	try
	{
		ivalue = 0;
		char szFile[1024];
		memset(szFile, '\0', 1024);
		strcpy(szFile, pFile);
		PathRemoveExtension(szFile);

		char szDir[1024];
		memset(szDir, '\0', 1024);
		strcpy(szDir, pFile);
		PathRemoveFileSpec(szDir);

		char szName[512];
		memset(szName, '\0', 512);
		char *p1 = szFile;
		strncpy_s(szName, p1 + strlen(szDir), strlen(szFile) - strlen(szDir));
		PathRemoveFileSpec(szDir);

		CvRect I_rect;
		I_rect.x = x1;
		I_rect.y = y1;
		I_rect.width = x2 - x1;
		I_rect.height = y2 - y1;
		IplImage* pImg = cvLoadImage(pFile, 3);
		iW = pImg->width;
		iH = pImg->height;
		cvSetImageROI(pImg, I_rect); //设置ROI区域 
		IplImage* RImg = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
		cvCopy(pImg, RImg);
		cvResetImageROI(pImg);
		cvReleaseImage(&pImg);
		CString sFileName;
		sFileName.Format("%s\\Featurex\\%s%s_M.jpg", szDir, szName, szFile);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, RImg);
		if (calcFeatureValueCore)
			ivalue = calcFeatureValueCore(RImg);

		cvResetImageROI(RImg);
		cvReleaseImage(&RImg);
	}
	catch (...)
	{
		return -1;
	}
	return 0;
}

/************************************************************************/
/* RCN_CreateImage创建opencv读取图片数据函数,支持jpg、bmp两种格式
pcImg: 图片数值地址
size:  图片数据大小
*/
/************************************************************************/
__declspec(dllexport)  unsigned long WINAPI RCN_CreateImage(char *pcImg, int size)
{
	if (size <= 0 || !pcImg)
	{
		return -1;
	}
	IplImage *pImage = NULL;
#if 0
	int iUnKnown = 0;
	if (((BYTE)pcImg[0] == 255) && ((BYTE)pcImg[1] == 216) && ((BYTE)pcImg[2] == 255)) //jpg
	{
		pImage = cvJpeg2Ipl(pcImg, size);
	}
	else if (((BYTE)pcImg[0] == 66) && ((BYTE)pcImg[1] == 77)) //bmp
	{
		BMP bt;
		BITMAPFILEHEADER *bf = (BITMAPFILEHEADER *)pcImg;
		memcpy(&bt.bfHeader, bf, sizeof(BITMAPFILEHEADER));
		BITMAPINFOHEADER *bi = (BITMAPINFOHEADER *)(pcImg + sizeof(BITMAPFILEHEADER));
		memcpy(&bt.biHeader, bi, sizeof(BITMAPINFOHEADER));
		bt.bmpData = (unsigned char *)(pcImg + sizeof(BITMAPINFOHEADER) + sizeof(BITMAPFILEHEADER));
		pImage = bt.BMP2Ipl();
		bt.bmpData = NULL;    //不能删除该语句，空间不需要释放。
	}
	else
	{
		AfxMessageBox("该文件类型不支持。");
	}
#endif
	std::vector<char> data;
	for(int i=0; i<size; i++)
	{
		data.push_back(pcImg[i]);
	}
	Mat matrixImg = imdecode(Mat(data), CV_LOAD_IMAGE_COLOR); 
	//pImage = cvCreateImage(cvSize(ROI_rect.width, ROI_rect.height), 8, pImg->nChannels);
	int width_image = matrixImg.size().width;
	int height_image = matrixImg.size().height;
	pImage = cvCreateImage(cvSize(width_image, height_image), 8 , matrixImg.channels());
	cvCopy(&IplImage(matrixImg), pImage); 
	return (unsigned long)pImage;
}


/************************************************************************/
/* RCN_ReleaseImage释放opencv打开图片句柄
dwImg: 图片句柄地址
*/
/************************************************************************/
__declspec(dllexport)  void WINAPI RCN_ReleaseImage(DWORD dwImg)
{
	IplImage *pImage = (IplImage *)dwImg;
	cvReleaseImage(&pImage);
	return;
}


/************************************************************************/
/* FitRotate图片旋转函数
img:       图片句柄
degree:    角度
iChannels: 图片通道数
*/
/************************************************************************/
IplImage* FitRotate(IplImage* img, int degree, int iChannels)
{
	double angle = degree  * CV_PI / 180.; // 弧度    
	double a = sin(angle), b = cos(angle);
	int width = img->width;
	int height = img->height;
	int width_rotate = int(height * fabs(a) + width * fabs(b));
	int height_rotate = int(width * fabs(a) + height * fabs(b));
	//旋转数组map  
	// [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]  
	// [ m3  m4  m5 ] ===>  [ A21  A22   b2 ]  
	float map[6];
	CvMat map_matrix = cvMat(2, 3, CV_32F, map);
	// 旋转中心  
	CvPoint2D32f center = cvPoint2D32f(width / 2, height / 2);
	cv2DRotationMatrix(center, degree, 1.0, &map_matrix);
	map[2] += (width_rotate - width) / 2;
	map[5] += (height_rotate - height) / 2;
	IplImage* img_rotate = cvCreateImage(cvSize(width_rotate, height_rotate), 8, iChannels);
	//对图像做仿射变换  
	//CV_WARP_FILL_OUTLIERS - 填充所有输出图像的象素。  
	//如果部分象素落在输入图像的边界外，那么它们的值设定为 fillval.  
	//CV_WARP_INVERSE_MAP - 指定 map_matrix 是输出图像到输入图像的反变换，  
	cvWarpAffine(img, img_rotate, &map_matrix, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
	return img_rotate;
}


/************************************************************************/
/* FitRotate_cut计算坐标点旋转函数, 旋转图像内容不变，尺寸相应变大，内容不会丢失  以图像中心进行旋转，可以自己改变
degree:      角度
src_corners: 原坐标点
dst_corners: 旋转后坐标点
*/
/************************************************************************/
void FitRotate_cut(RCNDATA *pRcnData, int degree, CvPoint src_corners, CvPoint &dst_corners)
{
	double angle = degree  * CV_PI / 180.; // 弧度    
	double a = sin(angle), b = cos(angle);
	int width = pRcnData->iCutWidth;
	int height = pRcnData->iCutHeight;
	int width_rotate = int(height * fabs(a) + width * fabs(b));
	int height_rotate = int(width * fabs(a) + height * fabs(b));
	//旋转数组map  
	// [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]  
	// [ m3  m4  m5 ] ===>  [ A21  A22   b2 ]  
	float map[6];
	CvMat map_matrix = cvMat(2, 3, CV_32F, map);
	// 旋转中心  
	CvPoint2D32f center = cvPoint2D32f(width / 2, height / 2);
	cv2DRotationMatrix(center, degree, 1.0, &map_matrix);
	map[2] += (width_rotate - width) / 2;
	map[5] += (height_rotate - height) / 2;
	//IplImage* img_rotate = cvCreateImage(cvSize(width_rotate, height_rotate), 8, 3);   
	//对图像做仿射变换  
	//CV_WARP_FILL_OUTLIERS - 填充所有输出图像的象素。  
	//如果部分象素落在输入图像的边界外，那么它们的值设定为 fillval.  
	//CV_WARP_INVERSE_MAP - 指定 map_matrix 是输出图像到输入图像的反变换，  
	//cvWarpAffine( img, img_rotate, &map_matrix, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS, cvScalarAll(0)); 

	//计算旋转后对应点的坐标值
	double x = src_corners.x, y = src_corners.y;
	double X = (map[0] * x + map[1] * y + map[2]);
	double Y = (map[3] * x + map[4] * y + map[5]);
	dst_corners.x = cvRound(X);
	dst_corners.y = cvRound(Y);
}

/************************************************************************/
/* FitRotate_3计算坐标点旋转函数, 旋转图像内容不变，尺寸相应变大，内容不会丢失  以图像中心进行旋转，可以自己改变
img:         图片句柄
degree:      角度
src_corners: 原坐标点
dst_corners: 旋转后坐标点
*/
/************************************************************************/
IplImage* FitRotate_3(IplImage* img, int degree, CvPoint src_corners, CvPoint &dst_corners)
{
	double angle = degree  * CV_PI / 180.; // 弧度    
	double a = sin(angle), b = cos(angle);
	int width = img->width;
	int height = img->height;
	int width_rotate = int(height * fabs(a) + width * fabs(b));
	int height_rotate = int(width * fabs(a) + height * fabs(b));
	//旋转数组map  
	// [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]  
	// [ m3  m4  m5 ] ===>  [ A21  A22   b2 ]  
	float map[6];
	CvMat map_matrix = cvMat(2, 3, CV_32F, map);
	// 旋转中心  
	CvPoint2D32f center = cvPoint2D32f(width / 2, height / 2);
	cv2DRotationMatrix(center, degree, 1.0, &map_matrix);
	map[2] += (width_rotate - width) / 2;
	map[5] += (height_rotate - height) / 2;
	IplImage* img_rotate = cvCreateImage(cvSize(width_rotate, height_rotate), 8, 3);
	//对图像做仿射变换  
	//CV_WARP_FILL_OUTLIERS - 填充所有输出图像的象素。  
	//如果部分象素落在输入图像的边界外，那么它们的值设定为 fillval.  
	//CV_WARP_INVERSE_MAP - 指定 map_matrix 是输出图像到输入图像的反变换，  
	cvWarpAffine(img, img_rotate, &map_matrix, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS, cvScalarAll(0));

	//计算旋转后对应点的坐标值
	double x = src_corners.x, y = src_corners.y;
	double X = (map[0] * x + map[1] * y + map[2]);
	double Y = (map[3] * x + map[4] * y + map[5]);
	dst_corners.x = cvRound(X);
	dst_corners.y = cvRound(Y);
	return img_rotate;
}

double get_avg_gray(IplImage *img)
{
	cv::Mat image = cv::Mat(img, true);
	cv::Mat grey_image;
	if (image.channels() == 3)
	{
		cv::cvtColor(image, grey_image, CV_BGR2GRAY);
	}
	else
	{
		grey_image = image;
	}
	cv::Scalar scalar = cv::mean(grey_image);
	return scalar[0];
}



void myThreshold(cv::Mat image, cv::Mat & out_image, int BlockSize = 30)
{
	for (int i = 0; i < image.rows; )
	{
		int y_size = 0;
		if ((i + BlockSize) < image.rows)
		{
			y_size = BlockSize;
		}
		else
		{
			y_size = image.rows - i;
		}
		for (int j = 0; j < image.cols; )
		{
			//边界保护
			int x_size = 0;

			if ((j + BlockSize) < image.cols)
			{
				x_size = BlockSize;
			}
			else
			{
				x_size = image.cols - j;
			}

			cv::Mat ROI = image(cv::Rect(j, i, x_size, y_size));

			//精细化阈值处理
			IplImage* ipImg = &IplImage(ROI);
			int light = get_avg_gray(ipImg);

			cv::threshold(ROI, ROI, 11 * light / 12, 255, CV_THRESH_BINARY);
			//cv::putText();
			image.copyTo(ROI);
			j += BlockSize;
		}
		i += BlockSize;
	}
	out_image = image.clone();
}



string HashValue(Mat& src)  
{  
	string rst(64,'\0');  
	Mat img;  
	if(src.channels()==3)  
		cvtColor(src,img,CV_BGR2GRAY);  
	else  
		img=src.clone();  
	/*第一步，缩小尺寸。 
	将图片缩小到8x8的尺寸，总共64个像素,去除图片的细节*/  

	resize(img,img,Size(8,8));  
	/* 第二步，简化色彩(Color Reduce)。 
	将缩小后的图片，转为64级灰度。*/  

	uchar *pData;  
	for(int i=0;i<img.rows;i++)  
	{  
		pData = img.ptr<uchar>(i);  
		for(int j=0;j<img.cols;j++)  
		{  
			pData[j]=pData[j]/4;            }  
	}  

	/* 第三步，计算平均值。 
	计算所有64个像素的灰度平均值。*/  
	int average = mean(img).val[0];  

	/* 第四步，比较像素的灰度。 
	将每个像素的灰度，与平均值进行比较。大于或等于平均值记为1,小于平均值记为0*/  
	Mat mask= (img>=(uchar)average);  

	/* 第五步，计算哈希值。*/  
	int index = 0;  
	for(int i=0;i<mask.rows;i++)  
	{  
		pData = mask.ptr<uchar>(i);  
		for(int j=0;j<mask.cols;j++)  
		{  
			if(pData[j]==0)  
				rst[index++]='0';  
			else  
				rst[index++]='1';  
		}  
	}  
	return rst;  
}  



//汉明距离计算  
int HanmingDistance(string &str1,string &str2)  
{  
	if((str1.size()!=64)||(str2.size()!=64))  
		return -1;  
	int difference = 0;  
	for(int i=0;i<64;i++)  
	{  
		if(str1[i]!=str2[i])  
			difference++;  
	}  
	return difference;  
}  



int calacHammingDistence(cv::Mat image_std, cv::Mat image_search)
{
	if(image_std.data == NULL || image_search.data == NULL)
	{
		return -1;
	}
	if(image_std.channels() == 3)
	{
		cv::cvtColor(image_std, image_std, CV_BGR2GRAY);
	}
	if(image_search.channels() == 3)
	{
		cv::cvtColor(image_search, image_search, CV_BGR2GRAY);
	}
	std::string hashValueStr_std;
	hashValueStr_std = HashValue(image_std);
	std::string hashValueStr_search;
	hashValueStr_search = HashValue(image_search);
	int diffImage = 1000;
	diffImage = HanmingDistance(hashValueStr_std, hashValueStr_search);
	if(diffImage > 25)
	{
		return -1;
	}
	return 0;
}


/************************************************************************/
/* CmpFeatureKeyArea比较特征值函数
RImg:       图片句柄
pRcnData:   凭证参数
q:          模板特征参数
iReserve:   控制图片输出
*/
/************************************************************************/
int CmpFeatureKeyArea(IplImage* RImg, RCNDATA *pRcnData, PMFEATUREX q, int iReserve, int id_thread, double* featureSim, float fxScale, float fyScale)
{
	int iRet = -1;
	CvPoint pt;
	CvRect I_rect;
	SYSTEMTIME st;
	CString sFileName;
	PMFEATURE p = q->pFeature;

	if (abs(p->nVoucherW - (RImg->width)*fxScale) > 200)
	{
		return iRet;
	}

	PKeyArea p_tmp_keyArea;
	char buf_message[2048];
	int total_iV1 = 0, total_iV2 = 0;

	//如果多特征，进行多特征匹配,完全匹配上，才认为是该模板！
	bool flag_circle = true;
	for (int i = 0; i < p->nKeyAreaCount; i++)
	{
		//支持的特征数
		if (i == 3)
		{
			break;
		}
		memset(buf_message, 0, sizeof buf_message);
#if 0
		pt.x = p->ka[i].nKeySearchX;
		pt.y = p->ka[i].nKeySearchY;
		int iw, ih;
		iw = p->ka[i].nKeySearchW;
		ih = p->ka[i].nKeySearchH;
		p_tmp_keyArea = (p->ka) + i;
		if ((pt.x + iw) > RImg->width || (pt.y + ih) > RImg->height)
		{
			flag_circle = false;
			break;
		}
#elif 1

		int iw, ih;
		//pt.x = p->ka[i].nKeySearchX - 3;
		//pt.y = p->ka[i].nKeySearchY + 3;		
		//iw = p->ka[i].nKeySearchW + 3;
		//ih = p->ka[i].nKeySearchH + 3;

		pt.x = (p->ka[i].nKeySearchX - 3) / fxScale;
		pt.y = (p->ka[i].nKeySearchY + 3) / fyScale;
		iw = (p->ka[i].nKeySearchW + 3) / fxScale;
		ih = (p->ka[i].nKeySearchH + 3) / fyScale;

		p_tmp_keyArea = (p->ka) + i;
		if (pt.x + iw + 3 > RImg->width)
		{
			/*	memset(buf_message, 0, sizeof buf_message);
			sprintf(buf_message, "%s%d %s%s%d", "异常的裁剪区间x 方向： ",(pt.x + iw + 3) ,p->strModNo, "模板的宽:", RImg->width);
			theApp.putLog(buf_message);*/
			pt.x = RImg->width - iw - 2;
		}

		if (pt.y + ih + 3 > RImg->height)
		{

			/*memset(buf_message, 0, sizeof buf_message);
			sprintf(buf_message, "%s%d%s%s%d", "异常的裁剪区间y方向： ",(pt.y + ih + 3) ,p->strModNo, "模板的宽:", RImg->height);
			theApp.putLog(buf_message);*/

			pt.y = RImg->height - ih - 2;
		}
#endif

		I_rect.x = pt.x;
		I_rect.y = pt.y;
		I_rect.width = iw;
		I_rect.height = ih;

		cvSetImageROI(RImg, I_rect); //设置ROI区域 
		IplImage* TImg = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, RImg->nChannels);
		cvCopy(RImg, TImg);
		cvResetImageROI(RImg);

		IplImage *srcImg = cvCreateImage(cvGetSize(TImg), TImg->depth, 1);
		cvCvtColor(TImg, srcImg, CV_RGB2GRAY);

		if (iReserve & 8)
		{
			GetSystemTime(&st);
			sFileName.Format("%s\\Image%d_%04d%02d%02d_%02d%02d%02d_%03d_识别图_%02d.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, id_thread, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds, i + 1);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, srcImg);
		}

#if 1
		BYTE average_light = p_tmp_keyArea->gray_average;
		BYTE prev_avg_gray = get_avg_gray(srcImg);
		double scale = ((double)average_light - 10) / prev_avg_gray;
		cvConvertScale(srcImg, srcImg, scale);
		if (iReserve & 4)
		{
			GetSystemTime(&st);
			sFileName.Format("%s\\Image%d_%04d%02d%02d_%02d%02d%02d_%03d_识别图_光照调整_%02d.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, id_thread, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds, i + 1);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, srcImg);
		}
#endif
		//缩放特征区间到模板配置的尺寸;		
		CvSize dst_cvsize;
		IplImage *RImg2;
		dst_cvsize.width = srcImg->width*fxScale;   //目标图像宽度  
		dst_cvsize.height = srcImg->height*fyScale; //目标图像高度    
		RImg2 = cvCreateImage(dst_cvsize, srcImg->depth, srcImg->nChannels); //创建一个目标图像   
		cvResize(srcImg, RImg2, CV_INTER_LINEAR); //缩放
		IplImage *BImg = q->pFImag[i];

		if (iReserve & 8)
		{
			GetSystemTime(&st);
			sFileName.Format("%s\\Image%d_%04d%02d%02d_%02d%02d%02d_%03d_标准图_%02d.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, id_thread, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds, i + 1);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		}

		//////用来调试
		int retCmp = strcmp(p->strModNo, "000000010");
		if( retCmp == 0)
		{
			int i_Debug = 0;
			i_Debug++;
			i_Debug++;
		}
		////添加轮廓过滤;
		//DWORD t1_contoursFilter = GetTickCount();
		BOOL bForceCompute = p->bFroceCompute;		
		if(bForceCompute == FALSE)
		{
			int retFilterContour = filterContoursFeatures(cv::Mat(BImg, true), cv::Mat(RImg2, true), p_tmp_keyArea, p->nSearchThresholdValue, retCmp);
			/*DWORD t2_contoursFilter = GetTickCount() - t1_contoursFilter;
			CString messageFilterContours;
			messageFilterContours.Format("过滤轮廓耗时:%d ms", t2_contoursFilter);
			theApp.putLog((LPSTR)(LPCTSTR)messageFilterContours);*/
			if(retFilterContour != 0)
			{
				return retFilterContour;
			}
		}

		//汉明距离判断;
		//int iHammRet = calacHammingDistence(cv::Mat(BImg, true), cv::Mat(RImg2, true));
		//if(iHammRet != 0)
		//{
		//	return iHammRet;
		//}
		//if(compareFeatureValueNewCore)
		//{
		//double dvalue[3]={0.0, 0.0, 0.0};
		double dvalue[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
		//DWORD dwStart = GetTickCount();
		int ii = compareFeatureValueNewCore(RImg2, BImg, dvalue, q, i);

		//char bufTime[256];
		//memset(bufTime, 0, 256);
		//DWORD dwTemp = GetTickCount();
		//sprintf(bufTime, "Time26 = %d(%d) \n", dwTemp, dwTemp - dwStart);
		//OutputDebugString(bufTime);

		int iV1 = 0, iV2 = 0, iV3 = 0, iV4 = 0, iV5 = 0;
		iV1 = dvalue[0];
		iV2 = dvalue[2];
		iV3 = dvalue[1];   //搜索特征區間的點數
		iV4 = dvalue[3];   //模板二值图,特征点数;
		iV5 = dvalue[4];   //搜索区间，特征点数；

		total_iV1 += iV1;
		total_iV2 += iV2;
		double feature_pro = dvalue[2] / dvalue[0];
		if (i == 0)
		{
			*featureSim = feature_pro * 100;
		}
		int feature_sim = p->ka[i].nKeySimilarity;
		if (ii == 0 && theApp.m_idebug & 1)
		{
			//测试日志, 暂时关闭.
			//sprintf(buf_message, "%s%s%s%d%s%4d%s%4d%s%.1f%s%d%s%s%d%s%d%s%d","模板： ", p->strModNo,", 特征编号:",i+1, ", 模板特征点数:", iV1, " ,匹配特征点数: ", iV2, " ,占比:", feature_pro*100, "%, 标准占比:", feature_sim, "%", " 搜索点数: ", iV3, " 模板二值: ", iV4, " 搜索二值: ", iV5);
			sprintf(buf_message, "%s%s%s%d%s%4d%s%4d%s%.1f%s%d%s%s%d", "模板： ", p->strModNo, ", 特征编号:", i + 1, ", 模板特征点数:", iV1, " ,匹配特征点数: ", iV2, " ,占比:", feature_pro * 100, "%, 标准占比:", feature_sim, "%", " 搜索点数: ", iV3);
			theApp.putLog(buf_message);
		}
		int feature_threshold_rate = p->ka[i].featureThreshold;
		if ((feature_pro * 100) < feature_threshold_rate)
		{
			flag_circle = false;
			break;
		}
		//}
		cvReleaseImage(&srcImg);
		cvReleaseImage(&TImg);
		cvReleaseImage(&RImg2);
	}

	if (flag_circle == true)
	{
		iRet = 0;
	}
	else
	{
		iRet = -1;
	}
	return iRet;
}

/************************************************************************/
/* jySmoothImage图片平滑函数
graySrc:    源图片句柄
grayDst:    平滑后图片句柄
*/
/************************************************************************/
int jySmoothImage(IplImage* graySrc, IplImage* grayDst)
{
	IplImage* mean = cvCreateImage(cvGetSize(graySrc), 8, 1);
	cvSmooth(graySrc, mean, CV_MEDIAN, 101, 101, 0, 0);

	int i, j;
	int k;
	double val, val_mean, val_dst;
	float factor = 0.7;
	int step_src = graySrc->widthStep / sizeof(uchar);
	int step_mean = mean->widthStep / sizeof(uchar);
	for (j = 0; j < graySrc->height; j++)
	{
		for (i = 0; i < graySrc->width; i++)
		{
			k = j*step_src + i;
			val = ((uchar*)(graySrc->imageData))[k];  //获取源图像像素点数据
			val_mean = ((uchar*)(mean->imageData))[k];//获取平滑处理后的图像像素点
			val_dst = (int)((127 - val_mean)*factor + val); //计算目标像素值
			//防止数据越界
			if (val_dst < 0)
				val_dst = 0;
			//转换成整数复制
			((uchar*)(grayDst->imageData))[k] = saturate_cast<uchar>(val_dst);
		}
	}
	cvReleaseImage(&mean);
	return 0;
}


/************************************************************************/
/* RecogniseImage2识别函数
img:       图片句柄
degree:    角度
center:    旋转中心点
i_pt[4]:   矩形区域四个角坐标点
iReserve:  控制图片输出
*/
/************************************************************************/
int RecogniseImage2(IplImage* img, double degree, CvPoint2D32f center, const CvPoint i_pt[4], int iReserve)
{
	DWORD dw = GetTickCount();
	int iRet = -1;
	CvPoint r_pt[4];
	IplImage* pImgDst = rotateImagexCore(img, degree, center, i_pt, r_pt);

	SYSTEMTIME st;
	CString sFileName;
	if (iReserve == 3)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, img);
	}

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
	IplImage* pdst = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
	cvCopy(pImgDst, pdst);
	cvResetImageROI(pImgDst);
	cvReleaseImage(&pImgDst);
	if (iReserve == 3)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_旋转图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, pdst);
	}
	return 0;
}

/************************************************************************/
/* RecogniseImage识别函数
img:       图片句柄
degree:    角度
center:    旋转中心点
i_pt[4]:   矩形区域四个角坐标点
pRcnData:  凭证参数
iReserve:  控制图片输出
*/
/************************************************************************/
int RecogniseImage(IplImage* img, double degree, CvPoint2D32f center, const CvPoint i_pt[4], RCNDATA *pRcnData, int iReserve)
{
	theApp.m_bFeaRight = false;
	theApp.m_maxSim = 0;
	theApp.m_degree = degree;
	theApp.m_iReserve = iReserve;
	char bufTime[256];
	//DWORD dwTemp, dwStart = GetTickCount();
	int iRet = -1;
	CvPoint r_pt[4];
	IplImage* pImgDst = rotateImagexCore(img, degree, center, i_pt, r_pt);

	SYSTEMTIME st;
	CString sFileName;
	if (iReserve & 2)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, img);
	}

	/*memset(bufTime, 0, 256);
	dwTemp = GetTickCount();
	sprintf(bufTime, "Time24 = %d(%d) \n", dwTemp, dwTemp - dwStart);*/
	//OutputDebugString(bufTime);

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
	IplImage* pdst = cvCreateImage(cvSize(I_rect.width, I_rect.height), pImgDst->depth, pImgDst->nChannels);
	cvCopy(pImgDst, pdst);
	cvResetImageROI(pImgDst);
	cvReleaseImage(&pImgDst);

	theApp.m_pdst[0] = pdst;
	for (int i = 1; i < MAX_THR; i++)
	{
		theApp.m_pdst[i] = cvCreateImage(cvSize(I_rect.width, I_rect.height), pdst->depth, pdst->nChannels);
		cvCopy(pdst, theApp.m_pdst[i]);
	}

	pRcnData->iCutWidth = pdst->width;
	pRcnData->iCutHeight = pdst->height;
	theApp.m_image_clipper = cv::Mat(pdst, true);
	if (iReserve & 2)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_旋转图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, theApp.m_pdst[0]);
	}

	//memset(bufTime, 0, 256);
	//dwTemp = GetTickCount();
	//sprintf(bufTime, "Time25 = %d(%d) \n", dwTemp, dwTemp - dwStart);
	//OutputDebugString(bufTime);

	//对任务进行分发， 多线程同时处理！
	if (theApp.m_idebug & 1)
		theApp.putLog("开始模板匹配");

	pthread_mutex_lock(&g_queue_cs);
	g_curIter = theApp.m_map.begin();
	memset(theApp.m_iArrFlag, 2, sizeof(int)*MAX_THR);
	pthread_mutex_unlock(&g_queue_cs);
	bool bcnt = false;
	int i = 0;
	//do
	for (;;)
	{

#ifdef _DEBUG
		Sleep(100);
#else
		Sleep(100);
#endif
		bcnt = false;
		for (i = 0; i < MAX_THR; i++)
		{
			if (theApp.m_iArrFlag[i] == 2)
			{
				bcnt = true;
				break;
			}
		}
		if (bcnt)
			continue;

		break;
	}

	if (theApp.m_bFeaRight)
	{
		if (theApp.m_idebug & 1)
			theApp.putLog("模板匹配结束！");
		for (int i = 0; i < MAX_THR; i++)
		{
			cvReleaseImage(&theApp.m_pdst[i]);
		}
		return 0;
	}
	for (int i = 0; i < MAX_THR; i++)
	{
		cvReleaseImage(&theApp.m_pdst[i]);
	}
	if (theApp.m_idebug & 1)
		theApp.putLog("模板匹配结束！");
	return iRet;
}


/************************************************************************/
/* GetAreaMaxContour计算最大联通区域函数
*/
/************************************************************************/
inline int GetAreaMaxContour(CvSeq *contour, CvSeq **Max1, CvSeq **Max2)
{
	double contour_area_temp = 0, contour_area_max = 0;
	CvSeq *area_max_contour = 0, *area_max_contour2 = 0;
	CvSeq *c = 0;
	for (c = contour; c != NULL; c = c->h_next)
	{
		contour_area_temp = fabs(cvContourArea(c, CV_WHOLE_SEQ));
		if (contour_area_temp > contour_area_max)
		{
			contour_area_max = contour_area_temp;
			if (area_max_contour2 == 0)
			{
				area_max_contour2 = c;
			}
			else
			{
				area_max_contour2 = area_max_contour;
			}
			area_max_contour = c;
		}
	}
	*Max1 = area_max_contour;
	*Max2 = area_max_contour2;
	return 0;
}


/************************************************************************/
/* GetSealArea用于印章校准
img:       图片句柄

*/
/************************************************************************/
int GetSealArea(IplImage* img, CvRect &I_rect, Rect &rt, int iReserve)
{
	int iRet = -1;
	cvSetImageROI(img, I_rect); //设置ROI区域 
	IplImage* RImg = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
	cvCopy(img, RImg);
	cvResetImageROI(img);

	SYSTEMTIME st;
	CString sFileName;
	if (iReserve  == 3)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, img);

		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_印章原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, RImg);
	}

	int w = RImg->width;
	int h = RImg->height;
	IplImage* rgb = cvCreateImage(cvSize(w, h), 8, 3);
	cvCvtColor(RImg, rgb, COLOR_BGR2RGB);
	for (int j = 0; j < w; ++j)
	{
		for (int i = 0; i < h; ++i)
		{
			int r = CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 0);
			int g = CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 1);
			int b = CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 2);
			if (!((r - g) > 30 && (r - b) > 30))
			{
				CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 0) = 255;
				CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 1) = 255;
				CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 2) = 255;
			}
		}
	}
	IplImage* bgr = cvCreateImage(cvSize(w, h), 8, 3);
	cvCvtColor(rgb, bgr, COLOR_RGB2BGR);

	if (iReserve == 3)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_印章图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, bgr);
	}

	IplImage* src = cvCreateImage(cvGetSize(RImg), RImg->depth, 1);
	cvCvtColor(bgr, src, CV_RGB2GRAY);
	if (iReserve & 1)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_灰度图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, src);
	}

	cvThreshold(src, src, 200, 255, CV_THRESH_BINARY); //二值化
	if (iReserve & 1)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_二值化图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, src);
	}

	IplImage* dst = cvCreateImage(cvGetSize(RImg), RImg->depth, 1);
	cvErode(src, dst, 0, 1);
	cvDilate(dst, dst, 0, 1);
	if (iReserve & 1)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_膨胀图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, dst);
	}

	//获取轮廓   
	std::vector<std::vector<Point>> contours;
	Mat image(dst, 0);

	//获取轮廓：   
	findContours(image, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	//除去太长或者太短的轮廓   
	int cmin = 250;
	int cmax = 1000;
	std::vector<std::vector<Point>>::const_iterator itc = contours.begin();
	while (itc != contours.end())
	{
		if (itc->size() < cmin || itc->size() > cmax)
			itc = contours.erase(itc);
		else
			++itc;
	}

	//drawContours(image, contours, -1, Scalar(255,255,255), 2);
	//cvNamedWindow("img");    
	//cvShowImage("img",image);   
	//imshow("img",image); 
	//cvWaitKey(0);

	Rect rtValue, rtMax;
	int iIndex = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		rtValue = boundingRect(Mat(contours[i]));
		if (rtValue.height > 450 || rtValue.width > 450)
			continue;

		if ((rtValue.width > rtMax.width) &&
			(rtValue.height > rtMax.height))
		{
			rtMax = rtValue;
			iIndex = i;
		}
	}
	if (contours.size() > 0)
	{
		iRet = 0;
		rt = rtMax;//boundingRect(Mat(contours[iIndex]));  		
	}
	//cvDestroyAllWindows();  
	cvReleaseImage(&src);
	cvReleaseImage(&dst);
	cvReleaseImage(&RImg);
	return iRet;
}


/************************************************************************/
/* RCN_AdjustCorrectionSealArea用于印章校准
pFile:       图片文件路径

*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_AdjustCorrectionSealArea(const char *pFile)
{
	int iRet = -1;
	DWORD dw = GetTickCount();
	IplImage *src, *pImg;
	pImg = cvLoadImage(pFile, 3);
	int w = pImg->width;
	int h = pImg->height;
	IplImage* rgb = cvCreateImage(cvSize(w, h), 8, 3);
	cvCvtColor(pImg, rgb, COLOR_BGR2RGB);
	for (int j = 0; j < w; ++j)
	{
		for (int i = 0; i < h; ++i)
		{
			int r = CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 0);
			int g = CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 1);
			int b = CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 2);
			if (r < 150)//((r > g) && (r > b))
			{
				if (g < b)
				{
					CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 0) = 255;
					CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 1) = 255;
					CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 2) = 255;
				}
				else if (r < 10 && g < 10 && b < 10)
				{
					if (abs(b - r) < 2)
					{
						CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 0) = 255;
						CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 1) = 255;
						CV_IMAGE_ELEM(rgb, uchar, i, j * 3 + 2) = 255;
					}
				}
			}
		}
	}

	IplImage* bgr = cvCreateImage(cvSize(w, h), 8, 3);
	cvCvtColor(rgb, bgr, COLOR_RGB2BGR);
	cvReleaseImage(&rgb);

	IplImage* gray = cvCreateImage(cvGetSize(bgr), bgr->depth, 1);
	cvCvtColor(bgr, gray, CV_RGB2GRAY);
	cvReleaseImage(&bgr);

	IplImage* back = cvCreateImage(cvGetSize(gray), gray->depth, 1);
	cvDilate(gray, back, NULL, 2);
	cvReleaseImage(&gray);

	IplImage* back2 = cvCreateImage(cvGetSize(back), back->depth, 1);
	cvErode(back, back2, 0, 3);
	cvReleaseImage(&back);

	SYSTEMTIME st;
	CString sFileName;
	GetSystemTime(&st);
	sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_去色图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
	cvSaveImage((LPSTR)(LPCTSTR)sFileName, back2);

	//获取轮廓   
	std::vector<std::vector<Point>> contours;
	Mat image(back2, 0);

	//获取轮廓：   
	findContours(image, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	//除去太长或者太短的轮廓   
	int cmin = 250;
	int cmax = 1000;
	std::vector<std::vector<Point>>::const_iterator itc = contours.begin();
	int ii = 0;
	while (itc != contours.end())
	{
		ii = itc->size();
		if (itc->size() < cmin || itc->size() > cmax)
			itc = contours.erase(itc);
		else
			++itc;
	}

	drawContours(image, contours, -1, Scalar(255, 255, 255), 2);

	cvNamedWindow("img");
	cvShowImage("img", back2);
	imshow("img", image);
	cvWaitKey(0);

	Rect rtValue, rtMax;
	int iIndex = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		rtValue = boundingRect(Mat(contours[i]));
		if (rtValue.height > 450 || rtValue.width > 450)
			continue;

		if ((rtValue.width > rtMax.width) &&
			(rtValue.height > rtMax.height))
		{
			rtMax = rtValue;
			iIndex = i;
		}
	}
	if (contours.size() > 0)
	{
		iRet = 0;
	}
	return iRet;
}


/************************************************************************/
/* RCN_CorrectionDeviceCoordinate 印章校准函数
dwImg:                 图片句柄
OutArraySealCenter[3]: 三个印章中心坐标点
iReserve:              控制图片输出
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_CorrectionDeviceCoordinate(DWORD dwImg, POINT OutArraySealCenter[3], int iReserve)
{
	int iRet = -1;
	IplImage* img = (IplImage *)dwImg;

	//上边印章区域
	CvRect rt;
	rt.x = (int)ceil((float)img->width / 4.0);
	rt.y = 60;
	rt.width = img->width / 2;
	rt.height = img->height / 2 - 80;

	Rect rtop, rleft, rright;
	if (GetSealArea(img, rt, rtop, iReserve) != 0)
		return iRet;

	//左边印章区域
	rt.x = 180;
	rt.y = img->height / 2;
	rt.width = img->width / 2 - 220;
	rt.height = img->height / 2 - 80;
	if (GetSealArea(img, rt, rleft, iReserve) != 0)
		return iRet;

	//右边印章区域
	rt.x = img->width / 2 + 80;
	rt.y = img->height / 2;
	rt.width = img->width / 2 - 250;
	rt.height = img->height / 2 - 80;
	if (GetSealArea(img, rt, rright, iReserve) != 0)
		return iRet;

	if ((abs(rtop.width - rleft.width) < 30) &&
		(abs(rtop.width - rright.width) < 30) &&
		(abs(rright.width - rleft.width) < 30) &&
		(abs(rtop.height - rleft.height) < 30) &&
		(abs(rtop.height - rright.height) < 30) &&
		(abs(rright.height - rleft.height) < 30))
	{
		OutArraySealCenter[0].x = (int)ceil((float)img->width / 4.0) + rtop.x + (int)ceil((float)rtop.width / 2.0);
		OutArraySealCenter[0].y = rtop.y + (int)ceil((float)rtop.height / 2.0) + 60;

		OutArraySealCenter[1].x = rleft.x + (int)ceil((float)rleft.width / 2.0) + 180;
		OutArraySealCenter[1].y = (int)ceil((float)img->height / 2.0) + rleft.y + (int)ceil((float)rleft.height / 2.0);

		OutArraySealCenter[2].x = (int)ceil((float)img->width / 2.0) + rright.x + (int)ceil((float)rright.width / 2.0) + 80;
		OutArraySealCenter[2].y = (int)ceil((float)img->height / 2.0) + rright.y + (int)ceil((float)rright.height / 2.0);
		iRet = 0;
	}
	return iRet;
}

float bbCmpOverlap(const CvRect& box1, const CvRect& box2)
{
	if (box1.x > box2.x + box2.width) { return 0.0; }
	if (box1.y > box2.y + box2.height) { return 0.0; }
	if (box1.x + box1.width < box2.x) { return 0.0; }
	if (box1.y + box1.height < box2.y) { return 0.0; }
	float colInt = min(box1.x + box1.width, box2.x + box2.width) - max(box1.x, box2.x);
	float rowInt = min(box1.y + box1.height, box2.y + box2.height) - max(box1.y, box2.y);
	float intersection = colInt * rowInt;
	float area1 = box1.width*box1.height;
	float area2 = box2.width*box2.height;
	return intersection / (area1 + area2 - intersection);
}

/************************************************************************/
/* RCN_CheckRecognise 凭证识别函数
dwImg:              图片句柄
pRcnData:           凭证参数
iImgReversal:       判断图片抬头在上面还是倒立。
iReserve:           控制图片输出
*/
/************************************************************************/
//__declspec(dllexport)  int WINAPI RCN_CheckRecognise(DWORD dwImg, RCNDATA *pRcnData, int &iImgReversal, int iReserve)
//{
//	DWORD dwTemp, dwStart = GetTickCount();
//	int iRet = -1;
//	if(!dwImg || !pRcnData) 
//	{
//		iRet = -2;
//		return iRet;
//	}
//	try
//	{
//		iReserve = theApp.m_idebug;
//		theApp.m_isSmall = false;
//		theApp.m_pRcnData = pRcnData;	
//		IplImage *src, *pImg;    
//		pImg = (IplImage *)dwImg;
//
//		IplImage * pImg_clipper = cvCreateImage(cvSize(pImg->width, pImg->height), pImg->depth, pImg->nChannels);	
//		cvCopy(pImg, pImg_clipper);
//		if(iReserve & 6)
//		{
//			SYSTEMTIME st;
//			GetSystemTime(&st);
//			CString sFileName;
//			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour+8, st.wMinute, st.wSecond, st.wMilliseconds);
//			cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
//		}
//
//		char bufTime[256];
//		memset(bufTime, 0, 256);
//		dwTemp = GetTickCount();
//		sprintf(bufTime, "Time1 = %d(%d) \n", dwTemp, dwTemp - dwStart);
//		OutputDebugString(bufTime);
//#if 1
//		BYTE prev_avg_gray = get_avg_gray(pImg_clipper);
//		double scale = 0;
//		if(prev_avg_gray > 70)
//		{
//			scale = ((double)theApp.Bright)/prev_avg_gray;
//		}
//		else
//		{
//			scale = ((double)70)/prev_avg_gray;
//		}
//		cvConvertScale(pImg_clipper, pImg_clipper, scale);
//		if(iReserve & 6)
//		{
//			SYSTEMTIME st;
//			GetSystemTime(&st);
//			CString sFileName;
//			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图_光照调整.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath,  st.wYear, st.wMonth, st.wDay, st.wHour+8, st.wMinute, st.wSecond, st.wMilliseconds);
//			cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg_clipper);
//		}
//
//		memset(bufTime, 0, 256);
//		dwTemp = GetTickCount();
//		sprintf(bufTime, "Time2 = %d(%d) \n", dwTemp, dwTemp - dwStart);
//		OutputDebugString(bufTime);
//#endif
//		src = cvCreateImage(cvGetSize(pImg_clipper), pImg_clipper->depth, 1);
//		cvCvtColor(pImg_clipper, src, CV_RGB2GRAY);
//		cvReleaseImage(&pImg_clipper);
//		IplImage* grey_auto_threshold = cvCreateImage(cvSize(src->width, src->height), 8, 1);		
//		cvCopy(src, grey_auto_threshold);
//		CvMemStorage* storage = cvCreateMemStorage(0);    
//		CvSeq* contour = 0;    
//
//		//细化
//		if(prev_avg_gray < theApp.Bright)
//		{
//			cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY); //二值化 
//			cvDilate(src, src, 0, 1);
//		}
//		else
//		{
//			cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY);
//		}
//
//		if(iReserve & 6)
//		{
//			SYSTEMTIME st;
//			GetSystemTime(&st);
//			CString sFileName;
//			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_阈值化处理.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath,  st.wYear, st.wMonth, st.wDay, st.wHour+8, st.wMinute, st.wSecond, st.wMilliseconds);
//			cvSaveImage((LPSTR)(LPCTSTR)sFileName, src);
//		}
//
//		//提取轮廓      
//		cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );   //CV_RETR_CCOMP
//		CvSeq *_contour1;
//		CvSeq *_contour2;
//		GetAreaMaxContour(contour, &_contour1, &_contour2);      	
//		if(_contour1 == 0)
//		{
//			cvReleaseMemStorage(&storage);
//			cvReleaseImage(&src);
//			cvReleaseImage(&grey_auto_threshold);
//		}
//
//		CvRect ROI_rect, ROI_rect2;
//		ROI_rect =  cvBoundingRect(_contour1);
//		CvBox2D rect = cvMinAreaRect2(_contour1);
//		if(ROI_rect.width < ((src->width)-300))
//		{
//			theApp.m_isSmall = true;
//			cv::Mat threshold_image;
//			cv::threshold(cv::Mat(grey_auto_threshold), threshold_image, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//			std::vector<std::vector<cv::Point> > contours;
//			std::vector<cv::Vec4i> hierarchy;
//			cv::findContours(threshold_image, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//			int max_contours_index = 0;
//			int max_width = 0;
//			int idx = 0;
//			for (; idx < contours.size(); idx++)
//			{
//				cv::Rect rectangle_char = cv::boundingRect(contours[idx]);
//				int area = rectangle_char.area();
//				if(max_width < area)
//				{
//					max_width = area;
//					max_contours_index = idx;
//				}
//			}
//			cv::Rect roi  = cv::boundingRect(contours[max_contours_index]);
//			ROI_rect = roi;
//			cv::RotatedRect rotat_roi = cv::minAreaRect(contours[max_contours_index]);
//			rect = rotat_roi;
//		}
//
//		memset(bufTime, 0, 256);
//		dwTemp = GetTickCount();
//		sprintf(bufTime, "Time3 = %d(%d) \n", dwTemp, dwTemp - dwStart);
//		OutputDebugString(bufTime);
//
//		CvPoint2D32f center;
//		CvPoint i_pt[4];
//		double angle = 0.0;
//		IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
//		if(RoiImg)
//		{
//			memset(bufTime, 0, 256);
//			dwTemp = GetTickCount();
//			sprintf(bufTime, "Time4 = %d(%d) \n", dwTemp, dwTemp - dwStart);
//			OutputDebugString(bufTime);
//			int it = RecogniseImage(RoiImg, angle, center, i_pt, pRcnData, iReserve);
//			if(it != 0)
//			{
//				memset(bufTime, 0, 256);
//				dwTemp = GetTickCount();
//				sprintf(bufTime, "Time5 = %d(%d) \n", dwTemp, dwTemp - dwStart);
//				OutputDebugString(bufTime);
//
//				double angle1 = 180+angle;
//				it = RecogniseImage(RoiImg, angle1, center, i_pt, pRcnData, iReserve);
//				if(it == 0)
//				{
//					memset(bufTime, 0, 256);
//					dwTemp = GetTickCount();
//					sprintf(bufTime, "Time6 = %d(%d) \n", dwTemp, dwTemp - dwStart);
//					OutputDebugString(bufTime);
//
//					pRcnData->fCenterX = center.x;
//					pRcnData->fCenterY = center.y;
//					pRcnData->iCutX = ROI_rect.x;
//					pRcnData->iCutY = ROI_rect.y;
//					pRcnData->dAngle = angle1;
//					pRcnData->iX = ROI_rect.x + pRcnData->iX;
//					pRcnData->iY = ROI_rect.y + pRcnData->iY;
//					//凭证的方向 0:上 1：下 2:左 3：右
//					if(pRcnData->bDirectionTitle == 3)
//					{
//						pRcnData->dAngleSeal = angle + 270;
//					}
//					else
//					{
//						pRcnData->dAngleSeal = angle1;
//					}
//
//					iRet = 0;
//					iImgReversal = 2;
//					CString ss;
//					ss.Format("名称<%s>, 编号<%s>", pRcnData->szCheckTypeName, pRcnData->szCheckNO);
//					//AfxMessageBox(ss);
//				}
//			}
//			else
//			{
//				memset(bufTime, 0, 256);
//				dwTemp = GetTickCount();
//				sprintf(bufTime, "Time7 = %d(%d) \n", dwTemp, dwTemp - dwStart);
//				OutputDebugString(bufTime);
//
//				iImgReversal = 1;
//				pRcnData->fCenterX = center.x;
//				pRcnData->fCenterY = center.y;
//				pRcnData->iCutX = ROI_rect.x;
//				pRcnData->iCutY = ROI_rect.y;
//				pRcnData->dAngle = angle;
//				pRcnData->iX = ROI_rect.x + pRcnData->iX;
//				pRcnData->iY = ROI_rect.y + pRcnData->iY;
//
//				//凭证的方向 0:上 1：下 2:左 3：右
//				if(pRcnData->bDirectionTitle == 3)
//				{
//					pRcnData->dAngleSeal = angle + 90;
//				}
//				else
//				{
//					pRcnData->dAngleSeal = angle;
//				}
//				CString ss;
//				ss.Format("名称<%s>, 编号<%s>", pRcnData->szCheckTypeName, pRcnData->szCheckNO);
//				//AfxMessageBox(ss);
//				iRet = 0;
//			}
//			cvResetImageROI(RoiImg);
//			cvReleaseImage(&RoiImg);
//		}
//		cvReleaseMemStorage(&storage);
//		cvReleaseImage(&src);
//		cvReleaseImage(&grey_auto_threshold);
//		memset(bufTime, 0, 256);
//		dwTemp = GetTickCount();
//		sprintf(bufTime, "Time8 = %d(%d) \n", dwTemp, dwTemp - dwStart);
//		OutputDebugString(bufTime);
//	}
//	catch (...)
//	{
//		iRet = -3;
//	}
//
//	return iRet;
//}


__declspec(dllexport)  int WINAPI RCN_CheckRecognise(DWORD dwImg, RCNDATA *pRcnData, int &iImgReversal, int iReserve)
{
	//memset(pRcnData->szInputName, 0, 128);
	//memset(pRcnData->szInputNo, 0, 32);
	if (theApp.m_idebug&1)
	{	
		char buf_tmp[1024];
		memset(buf_tmp, 0, 1024);
		sprintf(buf_tmp, "票据名称： %s, 票据号:  %s", pRcnData->szInputName, pRcnData->szInputNo);
		theApp.putLog(buf_tmp);
	}

	DWORD dwTemp, dwStart = GetTickCount();
	int iRet = -1;
	if (!dwImg || !pRcnData)
	{
		iRet = -2;
		return iRet;
	}

	try
	{
		iReserve = theApp.m_idebug;
		theApp.m_isSmall = false;
		theApp.m_pRcnData = pRcnData;
		IplImage *pImg = NULL, *pSrcImg = NULL;
		pSrcImg = (IplImage *)dwImg;
		Mat pic;
		if(pSrcImg->nChannels == 4)
		{
			pic = Mat(pSrcImg, TRUE);
			cv::cvtColor(pic, pic, CV_BGRA2BGR); 
			pImg = &IplImage(pic);
		}
		else
			pImg = pSrcImg;

		if (iReserve & 2)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
		}

		//char bufTime[256];
		//memset(bufTime, 0, 256);
		//dwTemp = GetTickCount();
		//sprintf(bufTime, "Time1 = %d(%d) \n", dwTemp, dwTemp - dwStart);
		//OutputDebugString(bufTime);

		RotatedRect rectPoint;
		cv::Rect roi;
		iRet = RCN_GetCutPictureParagramInner(pImg, rectPoint, roi, iReserve);
		if (iRet != 0)
		{
			if (theApp.m_idebug == 1)
				theApp.putLog("获取内部参数失败");

			return iRet;
		}

		//memset(bufTime, 0, 256);
		//dwTemp = GetTickCount();
		//sprintf(bufTime, "Time3 = %d(%d) \n", dwTemp, dwTemp - dwStart);
		//OutputDebugString(bufTime);

		CvBox2D rect;
		CvRect ROI_rect;
		CvPoint2D32f center;
		CvPoint i_pt[4];
		double angle = 0.0;
		rect = rectPoint;
		ROI_rect = roi;
		if(ROI_rect.height > pImg->height)
			ROI_rect.height = pImg->height - 1;
		if(ROI_rect.width > pImg->width)
			ROI_rect.width = pImg->width - 1;

		IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
		if (RoiImg)
		{
			//memset(bufTime, 0, 256);
			//dwTemp = GetTickCount();
			//sprintf(bufTime, "Time4 = %d(%d) \n", dwTemp, dwTemp - dwStart);
			//OutputDebugString(bufTime);
			int it = RecogniseImage(RoiImg, angle, center, i_pt, pRcnData, iReserve);
			if (it != 0)
			{
				/*memset(bufTime, 0, 256);
				dwTemp = GetTickCount();
				sprintf(bufTime, "Time5 = %d(%d) \n", dwTemp, dwTemp - dwStart);
				OutputDebugString(bufTime);*/

				double angle1 = 180 + angle;
				it = RecogniseImage(RoiImg, angle1, center, i_pt, pRcnData, iReserve);
				if (it == 0)
				{
					/*memset(bufTime, 0, 256);
					dwTemp = GetTickCount();
					sprintf(bufTime, "Time6 = %d(%d) \n", dwTemp, dwTemp - dwStart);
					OutputDebugString(bufTime);*/

					pRcnData->fCenterX = center.x;
					pRcnData->fCenterY = center.y;
					pRcnData->iCutX = ROI_rect.x;
					pRcnData->iCutY = ROI_rect.y;
					pRcnData->dAngle = angle1;
					pRcnData->iX = ROI_rect.x + pRcnData->iX;
					pRcnData->iY = ROI_rect.y + pRcnData->iY;
					//凭证的方向 0:上 1：下 2:左 3：右
					if (pRcnData->bDirectionTitle == 3)
					{
						pRcnData->dAngleSeal = angle + 270;
					}
					else
					{
						pRcnData->dAngleSeal = angle1;
					}

					iRet = 0;
					iImgReversal = 2;
					CString ss;
					ss.Format("名称<%s>, 编号<%s>", pRcnData->szCheckTypeName, pRcnData->szCheckNO);
					//AfxMessageBox(ss);
				}
			}
			else
			{
				/*memset(bufTime, 0, 256);
				dwTemp = GetTickCount();
				sprintf(bufTime, "Time7 = %d(%d) \n", dwTemp, dwTemp - dwStart);
				OutputDebugString(bufTime);*/

				iImgReversal = 1;
				pRcnData->fCenterX = center.x;
				pRcnData->fCenterY = center.y;
				pRcnData->iCutX = ROI_rect.x;
				pRcnData->iCutY = ROI_rect.y;
				pRcnData->dAngle = angle;
				pRcnData->iX = ROI_rect.x + pRcnData->iX;
				pRcnData->iY = ROI_rect.y + pRcnData->iY;

				//凭证的方向 0:上 1：下 2:左 3：右
				if (pRcnData->bDirectionTitle == 3)
				{
					pRcnData->dAngleSeal = angle + 90;
				}
				else
				{
					pRcnData->dAngleSeal = angle;
				}
				CString ss;
				ss.Format("名称<%s>, 编号<%s>", pRcnData->szCheckTypeName, pRcnData->szCheckNO);
				//AfxMessageBox(ss);
				iRet = 0;
			}
			cvResetImageROI(RoiImg);
			cvReleaseImage(&RoiImg);
		}
		//char bufTime[256];
		//memset(bufTime, 0, 256);
		//dwTemp = GetTickCount();
		//sprintf(bufTime, "Time = %d ms \n", (dwTemp - dwStart));
		////OutputDebugString(bufTime);
		//theApp.putLog(bufTime);
		if(iReserve & 1)
		{
			char buf_message_tmp_1[1024];
			memset(buf_message_tmp_1, 0, 1024);
			sprintf(buf_message_tmp_1, "票据类型：%s, 盖章位置： %d, %d. 印章角度:%.3f", pRcnData->szCheckTypeName, pRcnData->iX, pRcnData->iY, pRcnData->dAngleSeal);
			theApp.putLog(buf_message_tmp_1);
		}
	}
	catch (...)
	{
		iRet = -3;
	}

	return iRet;
}


/************************************************************************/
/* RCN_ClipperImage 裁剪票据
dwImg:              图片句柄
iReserve:           控制图片输出
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_ClipperImage(char* input_image, char* out_image, double* out_angle, int iReserve)
{
	int iRet = -1;
	if (strlen(input_image) == 0 || strlen(out_image) == 0)
	{
		return -1;
	}
	try
	{
		DWORD dw = GetTickCount();
		IplImage *src, *pImg;
		pImg = cvLoadImage(input_image, 3);
		if (pImg == NULL)
		{
			return -1;
		}

		if (iReserve == 3)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
		}

#if 1
		BYTE prev_avg_gray = get_avg_gray(pImg);
		double scale = 0;
		if (prev_avg_gray > 70)
		{
			scale = ((double)theApp.Bright) / prev_avg_gray;
		}
		else
		{
			scale = ((double)70) / prev_avg_gray;
		}
		cvConvertScale(pImg, pImg, scale);
		if (iReserve == 3)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图_光照调整.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
		}
#endif

		src = cvCreateImage(cvGetSize(pImg), pImg->depth, 1);
		cvCvtColor(pImg, src, CV_RGB2GRAY);
		IplImage* grey_auto_threshold = cvCreateImage(cvSize(src->width, src->height), 8, 1);
		cvCopy(src, grey_auto_threshold);
		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq* contour = 0;

		//细化
		if (prev_avg_gray < theApp.Bright)
		{
			cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY); //二值化 
			cvDilate(src, src, 0, 1);
		}
		else
		{
			cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		}

		if (iReserve == 3)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_阈值化处理.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, src);
		}

		//提取轮廓      
		cvFindContours(src, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);   //CV_RETR_CCOMP
		CvSeq *_contour1;
		CvSeq *_contour2;
		GetAreaMaxContour(contour, &_contour1, &_contour2);
		if (_contour1 == 0)
		{
			cvReleaseMemStorage(&storage);
			cvReleaseImage(&src);
			cvReleaseImage(&grey_auto_threshold);
		}

		CvRect ROI_rect, ROI_rect2;
		ROI_rect = cvBoundingRect(_contour1);
		CvBox2D rect = cvMinAreaRect2(_contour1);
		if (ROI_rect.width < ((src->width) - 200))
		{
			cv::Mat threshold_image;
			cv::threshold(cv::Mat(grey_auto_threshold), threshold_image, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
			std::vector<std::vector<cv::Point> > contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours(threshold_image, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			int max_contours_index = 0;
			int max_width = 0;
			int idx = 0;
			for (; idx < contours.size(); idx++)
			{
				cv::Rect rectangle_char = cv::boundingRect(contours[idx]);
				int area = rectangle_char.area();
				if (max_width < area)
				{
					max_width = area;
					max_contours_index = idx;
				}
			}
			cv::Rect roi = cv::boundingRect(contours[max_contours_index]);
			ROI_rect = roi;
			cv::RotatedRect rotat_roi = cv::minAreaRect(contours[max_contours_index]);
			rect = rotat_roi;
		}
		CvPoint2D32f center;
		CvPoint i_pt[4];
		double angle = 0.0;
		IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
		*out_angle = angle;
		if (RoiImg)
		{
			DWORD dw = GetTickCount();
			iRet = 0;
			CvPoint r_pt[4];
			IplImage* pImgDst = rotateImagexCore(RoiImg, angle, center, i_pt, r_pt);
			SYSTEMTIME st;
			CString sFileName;
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
			IplImage* tmpImgDst;
			tmpImgDst = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
			cvCopy(pImgDst, tmpImgDst);
			cvResetImageROI(pImgDst);
			cvReleaseImage(&pImgDst);
			cvSaveImage(out_image, tmpImgDst);
			cvResetImageROI(RoiImg);
			cvReleaseImage(&RoiImg);
			cvReleaseImage(&tmpImgDst);
		}
		cvReleaseMemStorage(&storage);
		cvReleaseImage(&src);
		cvReleaseImage(&grey_auto_threshold);
	}
	catch (...)
	{
		iRet = -3;
	}
	return iRet;
}



/************************************************************************/
/* RCN_CloseCamera 关闭相机
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_CloseCamera()
{
	if (theApp.m_hCameraThread)
	{
		theApp.m_bQuit = TRUE;
		while (theApp.m_hCameraThread)
		{
			Sleep(10);
		}
	}
	return 0;
}


void CIFeatureApp::putLog(char* logMessage)
{
	//return;
	if (logMessage == NULL || strlen(logMessage) == 0)
	{
		return;
	}
	pthread_mutex_lock(&g_log_cs);

	//初始化日志文件
	struct tm when;
	time_t now;
	time(&now);
	when = *localtime(&now);
	char  fpath[1024], fname[128], sztime[32];
	int y = when.tm_year + 1900;
	int m = when.tm_mon + 1;

	sprintf(fname, "\\Algorithm%04d%02d.txt", y, m);
	sprintf(fpath, "%s", (LPSTR)(LPCTSTR)theApp.m_strExePath);
	strcat(fpath, fname);
	pFileLog = fopen(fpath, "a");
	if (pFileLog == NULL)
	{
		pthread_mutex_unlock(&g_log_cs);
		return;
	}

	fwrite("\r\n", sizeof(char), 2, pFileLog);
	int d = when.tm_mday;
	int h = when.tm_hour;
	int mi = when.tm_min;
	int s = when.tm_sec;

	memset(sztime, 0, 32);
	sprintf(sztime, "%04d-%02d-%02d %02d:%02d:%02d ", y, m, d, h, mi, s);
	fwrite(sztime, sizeof(char), strlen(sztime), pFileLog);
	fwrite(logMessage, sizeof(char), strlen(logMessage), pFileLog);
	if (pFileLog != NULL)
	{
		fclose(pFileLog);
	}
	pthread_mutex_unlock(&g_log_cs);
}


int CIFeatureApp::ExitInstance()
{
	CloseHandle(m_hCameraHandle);
	m_hCameraHandle = NULL;
	if (m_hCvtModule)
	{
		FreeLibrary(m_hCvtModule);
		m_hCvtModule = NULL;
	}

	ReleaseMap();
	if (pFileLog != NULL)
	{
		fclose(pFileLog);
	}
	return CWinApp::ExitInstance();
}

inline void GetRectContour(CvSeq *contour, CvRect &ROI_rect)
{
	CvRect rc;
	rc.x = 0;
	rc.y = 0;
	rc.width = 0;
	rc.height = 0;
	CvSeq *c = 0;
	for (c = contour; c != NULL; c = c->h_next)
	{
		CvRect rect = cvBoundingRect(c);
		if (rect.height + rect.y > rc.height + rc.y)
		{
			rc = rect;
		}
	}
	ROI_rect = rc;
	return;
}


/************************************************************************/
/* CameraName 查找指定相机
sName: 相机名称或者有PID_VID组成
*/
/************************************************************************/
int CameraName(char* sName)
{
	int iRet = -1;
	int count = 0;
	CoInitialize(NULL);

	// enumerate all video capture devices
	CComPtr<ICreateDevEnum> pCreateDevEnum;
	HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (void**)&pCreateDevEnum);

	CComPtr<IEnumMoniker> pEm;
	hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEm, 0);
	if (hr != NOERROR)
	{
		iRet = -2;
		CoUninitialize();
		return iRet;
	}
	pEm->Reset();
	ULONG cFetched;
	IMoniker *pM;
	while (hr = pEm->Next(1, &pM, &cFetched), hr == S_OK)
	{
		LPOLESTR pszDisplayName;
		if (S_OK == pM->GetDisplayName(0, 0, &pszDisplayName))
		{
			USES_CONVERSION;
			std::string str1;
			str1 = W2A(pszDisplayName);
			CoTaskMemFree(pszDisplayName);
			std::string str2 = sName;
			int f1 = ci_find_substr(str1, str2);
			if (f1 >= 0)
			{
				pM->Release();
				pCreateDevEnum = NULL;
				pEm = NULL;
				CoUninitialize();
				return count;
			}
		}

		IPropertyBag *pBag = 0;
		hr = pM->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pBag);
		if (SUCCEEDED(hr))
		{
			VARIANT var;
			var.vt = VT_BSTR;
			hr = pBag->Read(L"FriendlyName", &var, NULL); //还有其他属性,像描述信息等等...
			if (hr == NOERROR)
			{
				//获取设备名称			
				//WideCharToMultiByte(CP_ACP,0,var.bstrVal,-1,sName, nBufferSize ,"",NULL);
				USES_CONVERSION;
				char *p = W2A(var.bstrVal);
				SysFreeString(var.bstrVal);
				if (_stricmp(p, sName) == 0)
				{
					pBag->Release();
					pM->Release();
					pCreateDevEnum = NULL;
					pEm = NULL;
					CoUninitialize();
					return count;
				}
			}
			pBag->Release();
		}
		pM->Release();
		count++;
	}
	pCreateDevEnum = NULL;
	pEm = NULL;
	CoUninitialize();
	return iRet;
}


/************************************************************************/
/* FuncCameraThread 相机处理线程函数
要求相机支持500万像素
*/
/************************************************************************/
DWORD WINAPI FuncCameraThread(void *lpParam)
{
	theApp.SetCheckState(CHECK_UNKNOW);
	int iRet = -1;
	int iIndex = 0;
	if ((iIndex = CameraName((LPSTR)(LPCTSTR)theApp.m_szCameraName)) < 0)
	{
		iRet = -2;
		CloseHandle(theApp.m_hCameraThread);
		theApp.m_hCameraThread = NULL;
		return iRet;
	}

	theApp.m_iCameraIndex = iIndex;
	if (!theApp.m_pcap->OpenCamera(iIndex, false, theApp.m_width_camera, theApp.m_height_camera, theApp.m_dExposure, theApp.m_dFocus))
	{
		iRet = -3;
		CloseHandle(theApp.m_hCameraThread);
		theApp.m_hCameraThread = NULL;
		return iRet;
	}
	IplImage *pFrame=NULL, *pF1=NULL;
	while (!theApp.m_bQuit)
	{
		if(theApp.m_CameraType == 1)
		{
			pF1 = theApp.m_pcap->QueryFrame();
		}
		else
		{
			pFrame = theApp.m_pcap->QueryFrame();
		}

		if (WaitForSingleObject(theApp.m_hCameraHandle, theApp.m_WaitTimeout) == WAIT_OBJECT_0)
		{
			if(theApp.m_CameraType==1 && !pF1) {
				continue;
			}
			if(theApp.m_CameraType==1)
			{	
				CvMat dst = cvMat(theApp.m_width_camera, theApp.m_height_camera, CV_8UC3, pF1->imageData);
				pFrame = cvDecodeImage(&dst, 1);
			}	
			if(!pFrame)
			{
				continue;
			}

			ResetEvent(theApp.m_hCameraHandle);
			theApp.SetCheckState(CHECK_BEGIN);
			IplImage* TImg = NULL;
			//不需要做180度旋转处理
			if (theApp.m_iRotate == 0)
			{
				//裁剪图像
				CvRect I_rect;
				I_rect.x = theApp.m_iLeft;
				I_rect.y = theApp.m_iTop;
				I_rect.width = pFrame->width - theApp.m_iLeft - theApp.m_iRight;
				I_rect.height = pFrame->height - theApp.m_iTop - theApp.m_iBottom;
				cvSetImageROI(pFrame, I_rect); //设置ROI区域 
				TImg = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
				cvCopy(pFrame, TImg);
				cvResetImageROI(pFrame);
			}
			else
			{
				double de = 180.0;
				IplImage* pDstImg = FitRotate(pFrame, de, 3);

				//裁剪图像
				CvRect I_rect;
				I_rect.x = theApp.m_iLeft;
				I_rect.y = theApp.m_iTop;
				I_rect.width = pDstImg->width - theApp.m_iLeft - theApp.m_iRight;
				I_rect.height = pDstImg->height - theApp.m_iTop - theApp.m_iBottom;

				cvSetImageROI(pDstImg, I_rect); //设置ROI区域 
				TImg = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
				cvCopy(pDstImg, TImg);
				cvResetImageROI(pDstImg);
				cvReleaseImage(&pDstImg);
			}

			cv::Mat temp2(TImg, 0);
			vector<uchar> buff;
			cv::imencode(".jpg", temp2, buff);
			cvReleaseImage(&TImg);
			if ((theApp.m_psize) &&
				(*theApp.m_psize >= buff.size()) &&
				(theApp.m_pData))
			{
				memcpy(theApp.m_pData, &buff[0], buff.size());
				*theApp.m_psize = buff.size();
			}
			theApp.SetCheckState(CHECK_END);
			if(theApp.m_CameraType == 1)
				cvReleaseImage(&pFrame);
		}
	}
	theApp.m_pcap->CloseCamera();
	CloseHandle(theApp.m_hCameraThread);
	theApp.m_hCameraThread = NULL;
	return 0;
}


/************************************************************************/
/* RCN_Capture 获取图片函数
pszData:    图片数组地址
pSize:      图片大小
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_Capture(char *pszData, int *pSize)
{
	//DWORD dwTemp, dwStart = GetTickCount();
	if (!theApp.m_pcap->isOpened())
		return -1;

	if (!pszData)
		return -2;

	theApp.m_pData = (unsigned char *)pszData;
	theApp.m_psize = pSize;
	theApp.SetCheckState(CHECK_BEGIN);
	SetEvent(theApp.m_hCameraHandle);
	while (theApp.GetCheckState() != CHECK_END)
		Sleep(10);

	theApp.m_pData = NULL;
	theApp.m_psize = NULL;

	/*char bufTime[256];
	memset(bufTime, 0, 256);
	dwTemp = GetTickCount();
	sprintf(bufTime, "capture image Time = %d(%d) \n", dwTemp, dwTemp - dwStart);
	OutputDebugString(bufTime);*/

	return 0;
}

/************************************************************************/
/* RCN_SetXScaleplate 设置测试卡宽度函数
iValue:    图片宽度
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_SetXScaleplate(int iValue)
{
	theApp.m_iXScaleplate = iValue;
	return 0;
}

/************************************************************************/
/* RCN_SetYScaleplate 设置测试卡高度函数
iValue:    图片高度
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_SetYScaleplate(int iValue)
{
	theApp.m_iYScaleplate = iValue;
	return 0;
}

/************************************************************************/
/* RCN_OpenCamera 打开相机函数
pszCameraName:    相机名称或者PID_VID组合
iLeft:            裁剪左边
iTop:             裁剪顶部
iRight:           裁剪右边
iBottom:          裁剪底部
dFocus:           相机焦距
dExposure:        相机曝光值
iRotate:          图片是否要倒立
iSFlag:           非0创建文件夹
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_OpenCamera(char *pszCameraName, int iLeft, int iTop, int iRight, int iBottom, double dFocus, double dExposure, int iRotate, int iSFlag)
{
	int iRet = 0;
	if (iSFlag != 0)
	{
		SHCreateDirectoryEx(NULL, theApp.m_strPath, NULL);
	}
	ResetEvent(theApp.m_hCameraHandle);
	if (theApp.m_hCameraThread == NULL)
	{
		theApp.m_iLeft = iLeft;
		theApp.m_iTop = iTop;
		theApp.m_iRight = iRight;
		theApp.m_iBottom = iBottom;
		theApp.m_dFocus = dFocus;
		theApp.m_dExposure = dExposure;
		theApp.m_iRotate = iRotate;
		theApp.m_szCameraName = pszCameraName;
		theApp.m_hCameraThread = CreateThread(NULL, 0, FuncCameraThread, NULL, CREATE_SUSPENDED, &theApp.m_CameraThreadID);
		if (!theApp.m_hCameraThread)
		{
			iRet = -4;
			return iRet;
		}
		if (ResumeThread(theApp.m_hCameraThread) == false)
		{
			iRet = -5;
			return iRet;
		}
	}
	return iRet;
}


//线程入口函数
void * process_map(void * pData)
{
	VDATA *pVd = (VDATA *)pData;
	int iFlag = pVd->iFlag;
	CIFeatureApp *pApp = (CIFeatureApp *)pVd->iThis;
	CString ss, sText;
	for (;;)
	{
		try
		{
			if (g_iQuit == 1)
				break;

			//sText.Format("识别线程(%d)运行中...", iFlag+1);
			//OutputDebugString((LPSTR)(LPCTSTR)sText);
			PMFEATUREX q = pApp->GetMapData();
			if (q)
			{
				//sText.Format("识别线程(%d)准备处理中...", iFlag+1);
				//OutputDebugString((LPSTR)(LPCTSTR)sText);
				pApp->m_iArrFlag[iFlag] = 2;
				PMFEATURE p = q->pFeature;


				if ((strlen(pApp->m_pRcnData->szInputNo) > 0)
					&& (strcmp(pApp->m_pRcnData->szInputNo, p->strModNo) != 0))
				{
					continue;
				}

				if ((strlen(pApp->m_pRcnData->szInputName) > 0)
					&& (strcmp(pApp->m_pRcnData->szInputName, p->strVoucherName) != 0))
				{
					continue;
				}




				if ((q->pFeature->nXScaleplate == 0) || (q->pFeature->nYScaleplate == 0))
				{
					continue;
				}
				for (int i = 0; i < p->nKeyAreaCount; i++)
				{
					if (!q->pFImag[i])
					{
						continue;
					}
				}
				/*
				if(strlen(pApp->m_pRcnData->szInputName) > 0)
				{
				sText.Format("识别线程(%d)正在处理中(%s, %s)", iFlag+1, pApp->m_pRcnData->szInputName, p->strVoucherName);
				OutputDebugString((LPSTR)(LPCTSTR)sText);
				}
				*/
				char bufTime[256];
				DWORD dwTemp, dwStart = GetTickCount();


				float fxscale = 1.0;
				if (pApp->m_iXScaleplate != 0)
					fxscale = (float)q->pFeature->nXScaleplate / pApp->m_iXScaleplate;

				float fyscale = 1.0;
				if (pApp->m_iYScaleplate != 0)
					fyscale = (float)q->pFeature->nYScaleplate / pApp->m_iYScaleplate;

				/*
				CvSize dst_cvsize;
				IplImage *RImg, *pImg;
				pImg = (IplImage *)pApp->m_pdst;
				dst_cvsize.width = pApp->m_pdst->width*fxscale;   //目标图像宽度
				dst_cvsize.height = pApp->m_pdst->height*fyscale; //目标图像高度
				RImg = cvCreateImage(dst_cvsize, pImg->depth, pImg->nChannels); //创建一个目标图像
				cvResize(pImg, RImg, CV_INTER_LINEAR); //缩放
				*/

				IplImage *pImg;
				pImg = (IplImage *)pApp->m_pdst[iFlag];

				//memset(bufTime, 0, 256);
				//dwTemp = GetTickCount();
				//sprintf(bufTime, "Time34 = %d(%04d), %s, %s \n", dwTemp, dwTemp - dwStart, p->strModNo, p->strVoucherName);
				//OutputDebugString(bufTime);
				double FeatureSim = 0;

				int iRet = CmpFeatureKeyArea(pImg, pApp->m_pRcnData, q, pApp->m_iReserve, iFlag, &FeatureSim, fxscale, fyscale);
				//memset(bufTime, 0, 256);
				//dwTemp = GetTickCount();
				//sprintf(bufTime, "Time35 = %d(%04d), %s, %s \n", dwTemp, dwTemp - dwStart, p->strModNo, p->strVoucherName);
				//OutputDebugString(bufTime);
				if (iRet != 0)
				{
					continue;
				}

				if (theApp.m_maxSim < FeatureSim)
				{
					pthread_mutex_lock(&g_sim_cs);
					theApp.m_maxSim = FeatureSim;
					pthread_mutex_unlock(&g_sim_cs);
				}
				else
				{
					continue;
				}
				pthread_mutex_lock(&g_queue_cs);
				g_curIter = pApp->m_map.end();
				pthread_mutex_unlock(&g_queue_cs);

				pApp->m_bFeaRight = true;
				//存在凭证编号
				if (p->bDigitFlag != 0)
				{
					DWORD t1_zk = GetTickCount();
					CvPoint pt;
					int iw, ih;
					pt.x = p->nDigitX;
					pt.y = p->nDigitY;
					iw = p->nDigitW;
					ih = p->nDigitH;

					SYSTEMTIME st;
					CString sFileName;

					CvRect I_rect;
					I_rect.x = pt.x / fxscale;
					I_rect.y = pt.y / fyscale;
					I_rect.width = iw / fxscale;
					I_rect.height = ih / fyscale;

					cvSetImageROI(pImg, I_rect); //设置ROI区域 
					IplImage* TImg2 = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
					cvCopy(pImg, TImg2);
					cvResetImageROI(pImg);

					if (pApp->m_iReserve & 4)
					{
						GetSystemTime(&st);
						sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_编号彩图.jpg", (LPSTR)(LPCTSTR)pApp->m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
						cvSaveImage((LPSTR)(LPCTSTR)sFileName, TImg2);
					}

					uchar vMin = 255;
					uchar vMax = 0;
					vector<uchar> buff;
					buff.clear();

					if(theApp.m_idebug & 1)
					{
						std::stringstream ss;
						ss << "数字方向: " << p->nDigitDirection ;
						theApp.putLog((char*)ss.str().c_str());
					}

					cv::Mat image_grey;
					cv::Mat show_image;
					//需要做旋转
					if (p->nDigitDirection != 0)
					{
						double de = 90.0;

						if(theApp.m_idebug & 1)
						{
							theApp.putLog("重控号：开始旋转");
						}

						//IplImage* pDstImg = FitRotate(TImg2, de, 3);						
						//替换逆时针旋转90度;
						cv::Mat in_image(TImg2);
						cv::Mat temp2;
						transpose(in_image, temp2);
						flip(temp2, temp2, 0);
						if (pApp->m_iReserve & 4)
						{
							GetSystemTime(&st);
							sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_编号旋正彩图.jpg", (LPSTR)(LPCTSTR)pApp->m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
							cv::imwrite((LPSTR)(LPCTSTR)sFileName, temp2);
						}

						if(theApp.m_idebug & 1)
						{
							theApp.putLog("重控号：结束旋转");
						}

						//做滤波处理，适当处理掉杂质的影响;
						cv::medianBlur(temp2, temp2, 3);
						//cv::Mat temp2(pDstImg, 0);
						//提取红色区间;
						if(p->nDigitMethod == 2)
						{
							//红色
							processImageByRed(temp2, image_grey, pApp->m_iReserve);

						}
						else if(p->nDigitMethod == 1)
						{
							//黑色
							processImageByBlack(temp2, image_grey, pApp->m_iReserve);
						}
						else
						{
							//其它颜色;
							processImageByDark(temp2, image_grey, pApp->m_iReserve);
						}
						cv::cvtColor(image_grey, show_image, CV_GRAY2BGR);
						//cvReleaseImage(&pDstImg);
					}
					else
					{
						IplImage* graySrc = cvCreateImage(cvSize(TImg2->width, TImg2->height), 8, 1);
						cvCvtColor(TImg2, graySrc, CV_BGR2GRAY);
						//IplImage* grayDst = cvCreateImage(cvSize(TImg2->width, TImg2->height), 8, 1);
						//jySmoothImage(graySrc, grayDst);
						//cv::Mat temp2(grayDst, 0);
						cv::Mat temp2(TImg2, 0);

						//做滤波处理，适当处理掉杂质的影响;
						cv::medianBlur(temp2, temp2, 3);

						if(p->nDigitMethod == 2)
						{
							//红色
							processImageByRed(temp2, image_grey, pApp->m_iReserve);

						}
						else if(p->nDigitMethod == 1)
						{
							//黑色
							processImageByBlack(temp2, image_grey, pApp->m_iReserve);
						}
						else
						{
							//其它颜色;
							processImageByDark(temp2, image_grey, pApp->m_iReserve);
						}

						cv::cvtColor(image_grey, show_image, CV_GRAY2BGR);

						//cv::imencode(".bmp", temp2, buff);
						cvReleaseImage(&graySrc);
						//cvReleaseImage(&grayDst);
					}

					char buftxt[256];
					memset(buftxt, '\0', 256);
#if 0
					if (pApp->cvtImagex)
					{

						int iArr[7];
						iArr[0] = 0;
						iArr[1] = 0;
						iArr[2] = 0;//ROI_rect.x;
						iArr[3] = 0;//ROI_rect.y;
						iArr[4] = 0;//ROI_rect.width;
						iArr[5] = 0;//ROI_rect.height;
						iArr[6] = 0;
						if(theApp.m_idebug & 1)
						{
							theApp.putLog(p->strDigitName);
						}
						pApp->cvtImagex(&buff[0], buff.size(), 1, (LPSTR)(LPCTSTR)sFileName, iArr, buftxt, 1, p->strDigitName, (LPSTR)(LPCTSTR)theApp.m_pathModelFile);
					}
#elif 1

					int ret = -1;
					std::string result_importCode;
					std::vector<cv::Rect> rects;

					if (pApp->m_iReserve & 4)
					{
						GetSystemTime(&st);
						sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_重控号识别图.jpg", (LPSTR)(LPCTSTR)pApp->m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
						cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_grey);
					}

					ret = recognizeImportantCodes(image_grey, p->strDigitName, result_importCode, rects);
					if (theApp.m_idebug & 1)
					{
						theApp.putLog((char*)result_importCode.c_str());
					}

					//画出矩形框
					if (pApp->m_iReserve & 4)
					{
						for(int i=0; i<rects.size(); i++)
						{
							cv::rectangle(show_image, rects[i], cv::Scalar(255));
						}
						GetSystemTime(&st);
						sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_编号矩形框图.bmp", (LPSTR)(LPCTSTR)pApp->m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
						cv::imwrite((LPSTR)(LPCTSTR)sFileName, show_image);
					}
#endif
					cvReleaseImage(&TImg2);
					CString s1, s2, ss;
					//s1 = buftxt;
					s1 = result_importCode.c_str();

					int digitNumber = p->nDigitNumber;

					//拷贝出固定的码值
					char buf_fixedNumber[16];
					memset(buf_fixedNumber, 0, 16);
					strcpy(buf_fixedNumber, p->fixedNumbers);

					if (s1.GetLength() >= digitNumber)
					{
						s2 = s1.Right(digitNumber);
						char buf[20];
						memset(buf, '\0', 20);
						strncpy(buf, (LPCTSTR)(LPCTSTR)s2, digitNumber);
						ss.Format("凭证编号：%s", buf);
						//处理下“换行符”
						std::string clear_buf = buf;
						std::string::iterator it;
						for(it=clear_buf.begin(); it!=clear_buf.end(); ++it)
						{
							if(*it == '\n')
							{
								clear_buf.erase(it);
							}
						}
						memset(buf, '\0', 20);
						//strcpy(buf, clear_buf.c_str());
						strcpy(buf, p->fixedNumbers);
						strcat(buf, clear_buf.c_str());
						strcpy(pApp->m_pRcnData->szCheckNO, buf);						
						if(theApp.m_idebug & 1)
						{
							theApp.putLog(buf);
						}
					}
					if (theApp.m_idebug & 1)
					{
						DWORD t2_zk_end = GetTickCount() - t1_zk;
						char buf_tmp[1024];
						memset(buf_tmp, 0, 1024);
						sprintf(buf_tmp, "重控编号耗时：%d ms", t2_zk_end);
						theApp.putLog(buf_tmp);
					}
				}

				//增加联数识别;
				if ( FALSE && (p->bUniteNum) == TRUE)
				{
					//DWORD dd1 = GetTickCount();
					cv::Mat tmpImage = cv::Mat(pImg);
					cv::Rect imageBigRect;
					imageBigRect.x = 0;
					imageBigRect.y = 0;
					imageBigRect.width = tmpImage.cols;
					imageBigRect.height = tmpImage.rows;

					cv::Rect imageRect;
					imageRect.x = (p->nUniteX) / fxscale;
					imageRect.y = (p->nUniteY) / fyscale;
					imageRect.width = (p->nUniteW) / fxscale;
					imageRect.height = (p->nUniteH) / fyscale;

					//修正不合理区间;
					if (imageRect.x < 0)
					{
						imageRect.x = 0;
					}
					if ((imageRect.x + imageRect.width) > tmpImage.cols)
					{
						imageRect.width = tmpImage.cols - imageRect.x - 3;
					}
					if (imageRect.y < 0)
					{
						imageRect.y = 0;
					}
					if ((imageRect.y + imageRect.height) > tmpImage.rows)
					{
						imageRect.height = tmpImage.rows - imageRect.y - 3;
					}
					bool flag_success = true;
					if (imageRect.x < 0 || imageRect.y < 0 ||
						((imageRect.x + imageRect.width) > tmpImage.size().width) ||
						((imageRect.y + imageRect.height) > tmpImage.size().height) ||
						((imageRect.x + imageRect.width) < 0) ||
						((imageRect.y + imageRect.height) < 0)
						)
					{
						flag_success = false;
					}

					if (flag_success && imageBigRect.contains(cv::Point(imageRect.x, imageRect.y)) && imageBigRect.contains(cv::Point(imageRect.x + imageRect.width, imageRect.y + imageRect.height)))
					{
						cv::Mat imageUnite = tmpImage(imageRect);
						int uniteNum = -1;
						int ret_tmp = RCN_CalacUniteNum(imageUnite, 0, 0, uniteNum, 0);
						if (ret_tmp == 0)
						{
							pApp->m_pRcnData->iUnitNum = uniteNum;
						}
					}
					else
					{
						theApp.putLog("联数区间位置有问题");
					}

					/*DWORD dd2 = GetTickCount() - dd1;
					CString sszz;
					sszz.Format("Time100 %d", dd2);
					OutputDebugString(sszz);*/
				}

				//保存最相似的結果
				//if(pApp->m_pRcnData->featureSimlary < FeatureSim)
				{
					//pApp->m_pRcnData->featureSimlary = FeatureSim;					
					strcpy(pApp->m_pRcnData->szCheckTypeName, p->strVoucherName);
					strcpy(pApp->m_pRcnData->szCheckTypeID, p->strModNo);

				}

				theApp.VerifyCodeRotate = p->verifyCodeRotate;
				theApp.m_nYzmX = p->nYzmX;
				theApp.m_nYzmY = p->nYzmY;
				theApp.m_bVerifyCodeBinPW = p->bVerifyCodeBin;

				theApp.m_VerifyCodeBinValue = p->nVerifyCodeBinValue;

				theApp.m_iDirectionTitle = (q->pFeature->bDirectionTitle);

				theApp.m_QRX = p->nQRX;
				theApp.m_QRY = p->nQRY;
				theApp.m_QRWidth = p->nQRWidth;
				//如果是大票据，得逆时针旋转90;
				//theApp.m_image_clipper = cv::Mat(RImg, true);
				if ((pApp->m_pRcnData->iCheckX == 0) || (pApp->m_pRcnData->iCheckY == 0))
				{
					pApp->m_pRcnData->iX = p->nSealCenterX;
					pApp->m_pRcnData->iY = p->nSealCenterY;
				}
				else
				{
					pApp->m_pRcnData->iX = pApp->m_pRcnData->iCheckX;
					pApp->m_pRcnData->iY = pApp->m_pRcnData->iCheckY;
				}


				//根据原图中的盖章坐标点

				float xscale = 1.0;
				if(pApp->m_iXScaleplate != 0)
					xscale = (float)pApp->m_iXScaleplate / q->pFeature->nXScaleplate;

				float yscale = 1.0;
				if(pApp->m_iYScaleplate != 0)
					yscale = (float)pApp->m_iYScaleplate / q->pFeature->nYScaleplate;

				CvPoint rz, rr;
				rz.x = pApp->m_pRcnData->iX * xscale;
				rz.y = pApp->m_pRcnData->iY * yscale;

				double de = -(pApp->m_degree);
				IplImage* pDImg = FitRotate_3(pImg, de, rz, rr);

				pApp->m_pRcnData->iX = rr.x;
				pApp->m_pRcnData->iY = rr.y;
				pApp->m_pRcnData->iYzmX = (q->pFeature->nYzmX);
				pApp->m_pRcnData->iYzmY = (q->pFeature->nYzmY);
				pApp->m_pRcnData->bDirectionTitle = (q->pFeature->bDirectionTitle);

				cvReleaseImage(&pDImg);
				pApp->m_iArrFlag[iFlag] = 1;
			}
			else //map元素取空了。
			{
				pApp->m_iArrFlag[iFlag] = 1;
				Sleep(30);
			}
		}
		catch (...)
		{

		}
	}
	pApp->m_iArrFlag[iFlag] = 1;  //线程崩溃，置标志。
	return NULL;
}


//原始版本; 裁剪图片Mat 版，程序内部使用.
//裁剪图片Mat 版，程序内部使用.
//针对dpi计算的
//
//int WINAPI RCN_GetCutPictureToMemOne(DWORD dwImg, cv::Mat& result)
//{
//	int iRet = -1;
//	try
//	{
//		DWORD dw = GetTickCount();		
//		IplImage *src, *pImg;    
//		pImg = (IplImage *)dwImg;
//		if(pImg == NULL)
//		{
//			return -1;
//		}
//		IplImage * pImg_clipper = cvCreateImage(cvSize(pImg->width, pImg->height), pImg->depth, pImg->nChannels);	
//		cvCopy(pImg, pImg_clipper);
//
//#if 1
//		BYTE prev_avg_gray = get_avg_gray(pImg_clipper);
//		double scale = 0;
//		if(prev_avg_gray > 70)
//		{
//			scale = ((double)theApp.Bright)/prev_avg_gray;
//		}
//		else
//		{
//			scale = ((double)70)/prev_avg_gray;
//		}
//		cvConvertScale(pImg_clipper, pImg_clipper, scale);
//#endif
//
//		src = cvCreateImage(cvGetSize(pImg), pImg->depth, 1);
//		cvCvtColor(pImg_clipper, src, CV_RGB2GRAY);
//		IplImage* grey_auto_threshold = cvCreateImage(cvSize(src->width, src->height), 8, 1);	
//		cvCopy(src, grey_auto_threshold);
//		cvReleaseImage(&pImg_clipper);
//		CvMemStorage* storage = cvCreateMemStorage(0);    
//		CvSeq* contour = 0;    
//
//		//细化
//		if(prev_avg_gray < theApp.Bright)
//		{
//			cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY); //二值化 
//			cvDilate(src, src, 0, 1);
//		}
//		else
//		{
//			cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
//		}
//
//		//提取轮廓      
//		cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );   //CV_RETR_CCOMP
//		CvSeq *_contour1;
//		CvSeq *_contour2;
//		GetAreaMaxContour(contour, &_contour1, &_contour2);      	
//		if(_contour1 == 0)
//		{
//			cvReleaseMemStorage(&storage);
//			cvReleaseImage(&src);
//			cvReleaseImage(&grey_auto_threshold);
//		}
//
//		CvRect ROI_rect, ROI_rect2;
//		ROI_rect =  cvBoundingRect(_contour1);
//		CvBox2D rect = cvMinAreaRect2(_contour1);
//		if(ROI_rect.width < ((src->width)-200))
//		{
//			cv::Mat threshold_image;
//			cv::threshold(cv::Mat(grey_auto_threshold), threshold_image, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//			std::vector<std::vector<cv::Point> > contours;
//			std::vector<cv::Vec4i> hierarchy;
//			cv::findContours(threshold_image, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//			int max_contours_index = 0;
//			int max_width = 0;
//			int idx = 0;
//			for (; idx < contours.size(); idx++)
//			{
//				cv::Rect rectangle_char = cv::boundingRect(contours[idx]);
//				int area = rectangle_char.area();
//				if(max_width < area)
//				{
//					max_width = area;
//					max_contours_index = idx;
//				}
//			}
//			cv::Rect roi  = cv::boundingRect(contours[max_contours_index]);
//			ROI_rect = roi;
//			cv::RotatedRect rotat_roi = cv::minAreaRect(contours[max_contours_index]);
//			rect = rotat_roi;
//		}
//
//		CvPoint2D32f center;
//		CvPoint i_pt[4];
//		double angle = 0.0;
//		IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
//		if(RoiImg)
//		{
//			DWORD dw = GetTickCount();
//			iRet = 0;
//			CvPoint r_pt[4];
//			IplImage* pImgDst = rotateImagexCore(RoiImg, angle, center, i_pt, r_pt);
//			SYSTEMTIME st;
//			CString sFileName;
//			CxRectangle xr(r_pt);
//			CvRect I_rect;
//			CvPoint pt1, pt2;
//			pt1 = xr.GetTopLeftPoint();
//			pt2 = xr.GetButtomRightPoint();
//			I_rect.x = abs(pt1.x);
//			I_rect.y = abs(pt1.y);
//			I_rect.width = pt2.x - abs(pt1.x);
//			I_rect.height = pt2.y - abs(pt1.y);
//
//			cvSetImageROI(pImgDst, I_rect); //设置ROI区域 
//			theApp.m_pdst[0] = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3); 
//			cvCopy(pImgDst, theApp.m_pdst[0]); 
//			cvResetImageROI(pImgDst);
//			cvReleaseImage(&pImgDst);
//			result = cv::Mat(theApp.m_pdst[0], true);
//			cvResetImageROI(RoiImg);
//			cvReleaseImage(&RoiImg);
//			cvReleaseImage(&theApp.m_pdst[0]);
//		}
//
//		cvReleaseMemStorage(&storage);
//		cvReleaseImage(&src);
//		cvReleaseImage(&grey_auto_threshold);
//	}
//	catch (...)
//	{
//		iRet = -3;
//	}
//	return iRet;
//}

int WINAPI RCN_GetCutPictureToMemOne(DWORD dwImg, cv::Mat& result)
{
	int iRet = -1;
	int iReserve = theApp.m_idebug;
	try
	{
		DWORD dw = GetTickCount();
		IplImage *src, *pImg;
		pImg = (IplImage *)dwImg;
		if (pImg == NULL)
		{
			return -1;
		}
		RotatedRect rectPoint;
		cv::Rect roi;
		iRet = RCN_GetCutPictureParagramInner(pImg, rectPoint, roi, iReserve);
		if (iRet != 0)
		{
			theApp.putLog("获取内部参数失败");
			return iRet;
		}
		CvBox2D rect;
		CvRect ROI_rect;
		CvPoint2D32f center;
		CvPoint i_pt[4];
		double angle = 0.0;
		rect = rectPoint;
		ROI_rect = roi;
		IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
		if (RoiImg)
		{
			DWORD dw = GetTickCount();
			iRet = 0;
			CvPoint r_pt[4];
			IplImage* pImgDst = rotateImagexCore(RoiImg, angle, center, i_pt, r_pt);
			SYSTEMTIME st;
			CString sFileName;
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
			IplImage* tmpImgDst;
			tmpImgDst = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
			cvCopy(pImgDst, tmpImgDst);
			cvResetImageROI(pImgDst);
			cvReleaseImage(&pImgDst);
			result = cv::Mat(tmpImgDst, true);
			cvResetImageROI(RoiImg);
			cvReleaseImage(&RoiImg);
			cvReleaseImage(&tmpImgDst);
		}
		//cvReleaseImage(&src);		
	}
	catch (...)
	{
		iRet = -3;
	}
	return iRet;
}



//裁剪图片Mat 版，程序内部使用. 旧版本的裁剪
//
//int WINAPI RCN_GetCutPictureToMem(DWORD dwImg, RCNDATA *pRcnData, cv::Mat& result)
//{
//	int iRet = -1;
//	try
//	{
//		DWORD dw = GetTickCount();		
//		IplImage *src, *pImg;    
//		pImg = (IplImage *)dwImg;
//		if(pImg == NULL)
//		{
//			return -1;
//		}
//		IplImage * pImg_clipper = cvCreateImage(cvSize(pImg->width, pImg->height), pImg->depth, pImg->nChannels);	
//		cvCopy(pImg, pImg_clipper);
//
//#if 1
//		BYTE prev_avg_gray = get_avg_gray(pImg_clipper);
//		double scale = 0;
//		if(prev_avg_gray > 70)
//		{
//			scale = ((double)theApp.Bright)/prev_avg_gray;
//		}
//		else
//		{
//			scale = ((double)70)/prev_avg_gray;
//		}
//		cvConvertScale(pImg_clipper, pImg_clipper, scale);
//#endif
//
//		src = cvCreateImage(cvGetSize(pImg), pImg->depth, 1);
//		cvCvtColor(pImg_clipper, src, CV_RGB2GRAY);
//		IplImage* grey_auto_threshold = cvCreateImage(cvSize(src->width, src->height), 8, 1);	
//		cvCopy(src, grey_auto_threshold);
//		cvReleaseImage(&pImg_clipper);
//		CvMemStorage* storage = cvCreateMemStorage(0);    
//		CvSeq* contour = 0;    
//
//		//细化
//		if(prev_avg_gray < theApp.Bright)
//		{
//			cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY); //二值化 
//			cvDilate(src, src, 0, 1);
//		}
//		else
//		{
//			cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
//		}
//
//		//提取轮廓      
//		cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );   //CV_RETR_CCOMP
//		CvSeq *_contour1;
//		CvSeq *_contour2;
//		GetAreaMaxContour(contour, &_contour1, &_contour2);      	
//		if(_contour1 == 0)
//		{
//			cvReleaseMemStorage(&storage);
//			cvReleaseImage(&src);
//			cvReleaseImage(&grey_auto_threshold);
//		}
//
//		CvRect ROI_rect, ROI_rect2;
//		ROI_rect =  cvBoundingRect(_contour1);
//		CvBox2D rect = cvMinAreaRect2(_contour1);
//		if(ROI_rect.width < ((src->width)-200))
//		{
//			cv::Mat threshold_image;
//			cv::threshold(cv::Mat(grey_auto_threshold), threshold_image, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//			std::vector<std::vector<cv::Point> > contours;
//			std::vector<cv::Vec4i> hierarchy;
//			cv::findContours(threshold_image, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//			int max_contours_index = 0;
//			int max_width = 0;
//			int idx = 0;
//			for (; idx < contours.size(); idx++)
//			{
//				cv::Rect rectangle_char = cv::boundingRect(contours[idx]);
//				int area = rectangle_char.area();
//				if(max_width < area)
//				{
//					max_width = area;
//					max_contours_index = idx;
//				}
//			}
//			cv::Rect roi  = cv::boundingRect(contours[max_contours_index]);
//			ROI_rect = roi;
//			cv::RotatedRect rotat_roi = cv::minAreaRect(contours[max_contours_index]);
//			rect = rotat_roi;
//		}
//
//		CvPoint2D32f center;
//		CvPoint i_pt[4];
//		double angle = 0.0;
//		IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
//		if(RoiImg)
//		{	
//			iRet = 0;
//			CvPoint r_pt[4];
//			IplImage* pImgDst = rotateImagexCore(RoiImg, angle, center, i_pt, r_pt);
//			SYSTEMTIME st;
//			CString sFileName;
//			CxRectangle xr(r_pt);
//			CvRect I_rect;
//			CvPoint pt1, pt2;
//			pt1 = xr.GetTopLeftPoint();
//			pt2 = xr.GetButtomRightPoint();
//			I_rect.x = abs(pt1.x);
//			I_rect.y = abs(pt1.y);
//			I_rect.width = pt2.x - abs(pt1.x);
//			I_rect.height = pt2.y - abs(pt1.y);
//
//			cvSetImageROI(pImgDst, I_rect); //设置ROI区域 
//			theApp.m_pdst = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3); 
//			cvCopy(pImgDst, theApp.m_pdst); 
//			cvResetImageROI(pImgDst);
//			cvReleaseImage(&pImgDst);
//
//			if(pRcnData)
//			{
//				pRcnData->dAngle = angle;
//				pRcnData->fCenterX = center.x;
//				pRcnData->fCenterY = center.y;
//				pRcnData->iCutX = ROI_rect.x;
//				pRcnData->iCutY = ROI_rect.y;
//				pRcnData->iCutWidth = theApp.m_pdst->width;
//				pRcnData->iCutHeight = theApp.m_pdst->height;
//				RCN_GetCutPictureSealPoint(pRcnData);
//			}
//
//			result = cv::Mat(theApp.m_pdst, true);
//			cvResetImageROI(RoiImg);
//			cvReleaseImage(&RoiImg);
//			cvReleaseImage(&theApp.m_pdst);
//		}
//
//		cvReleaseMemStorage(&storage);
//		cvReleaseImage(&src);
//		cvReleaseImage(&grey_auto_threshold);
//	}
//	catch (...)
//	{
//		iRet = -3;
//	}
//	return iRet;
//}

int WINAPI RCN_GetCutPictureToMem(DWORD dwImg, RCNDATA *pRcnData, cv::Mat& result)
{
	int iReserve = theApp.m_idebug;
	int iRet = -1;
	try
	{
		DWORD dw = GetTickCount();
		IplImage *pImg;
		pImg = (IplImage *)dwImg;
		if (pImg == NULL)
		{
			return -1;
		}
		RotatedRect rectPoint;
		cv::Rect roi;
		iRet = RCN_GetCutPictureParagramInner(pImg, rectPoint, roi, iReserve);
		if (iRet != 0)
		{
			theApp.putLog("获取内部参数失败");
			return iRet;
		}
		CvBox2D rect;
		CvRect ROI_rect;
		CvPoint2D32f center;
		CvPoint i_pt[4];
		double angle = 0.0;
		rect = rectPoint;
		ROI_rect = roi;

		IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
		if (RoiImg)
		{
			iRet = 0;
			CvPoint r_pt[4];
			IplImage* pImgDst = rotateImagexCore(RoiImg, angle, center, i_pt, r_pt);
			SYSTEMTIME st;
			CString sFileName;
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
			IplImage* tmpImgDst;
			tmpImgDst = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
			cvCopy(pImgDst, tmpImgDst);
			cvResetImageROI(pImgDst);
			cvReleaseImage(&pImgDst);
			if (pRcnData)
			{
				pRcnData->dAngle = angle;
				pRcnData->fCenterX = center.x;
				pRcnData->fCenterY = center.y;
				pRcnData->iCutX = ROI_rect.x;
				pRcnData->iCutY = ROI_rect.y;
				pRcnData->iCutWidth = tmpImgDst->width;
				pRcnData->iCutHeight = tmpImgDst->height;
				RCN_GetCutPictureSealPoint(pRcnData);
			}
			result = cv::Mat(tmpImgDst, true);
			cvResetImageROI(RoiImg);
			cvReleaseImage(&RoiImg);
			cvReleaseImage(&tmpImgDst);
		}
	}
	catch (...)
	{
		iRet = -3;
	}
	return iRet;
}



//暴露给用户的接口.用于裁剪图片

/*
用途： 裁剪出票据
参数介绍:
dwImg: 图片的地址
bufImg: 裁剪图片的缓冲区，需要用户申请20M的空间.
pLength: 裁剪好图片的字节数。
注意：保存裁剪图片的时候，保存成jpg格式的。
*/
int WINAPI RCN_GetCutPictureToMemory(DWORD dwImg, RCNDATA *pRcnData, char* bufImage, int* pLength)
{
	int iRet = -1;
	cv::Mat image;
	iRet = RCN_GetCutPictureToMem(dwImg, pRcnData, image);
	if (iRet == 0)
	{
		vector<uchar> buff;
		cv::imencode(".jpg", image, buff);
		memcpy(bufImage, &buff[0], buff.size());
		*pLength = buff.size();
	}
	return iRet;
}



/*
用途： 裁剪出票据，增强的版本;
参数介绍:
dwImg: 图片的地址
dpi: 200/180
bufImg: 裁剪图片的缓冲区，需要用户申请20M的空间.
pLength: 裁剪好图片的字节数。
注意：保存裁剪图片的时候，保存成jpg格式的。
*/
int WINAPI RCN_GetCutPictureToMemory2(DWORD dwImg, int dpi, RCNDATA *pRcnData, char* bufImage, int* pLength)
{
	if(dpi == 0)
	{
		dpi = 200;
	}
	float scaleSize = (dpi/25.4)/theApp.m_pixNumPerMm;
	int iRet = -1;
	cv::Mat image;
	iRet = RCN_GetCutPictureToMem(dwImg, pRcnData, image);
	if (iRet == 0)
	{
		BYTE* bufImageTemp = new BYTE[20480000];
		cv::resize(image, image, cv::Size(0, 0), scaleSize, scaleSize);
		vector<uchar> buff;
		cv::imencode(".jpg", image, buff);	
		memcpy(bufImageTemp, &buff[0], buff.size());
		*pLength = buff.size();
		//设置DPI
		iRet = theApp.cvtSaveImageToMem(bufImageTemp, *pLength, 0, bufImage, 0, 50, dpi, dpi);
		delete []bufImageTemp;
	}
	return iRet;
}


int WINAPI RCN_GetCutPictureSealPoint(RCNDATA *pRcnData)
{
	int iRet = -1;
	if ((pRcnData->iCheckX <= 0) || (pRcnData->iCheckY <= 0))
	{
		return iRet;
	}
	else
	{	
		pRcnData->iX = pRcnData->iCheckX;
		pRcnData->iY = pRcnData->iCheckY;
	}

	//根据原图中的盖章坐标点
	CvPoint rz, rr;
	rz.x = pRcnData->iX;
	rz.y = pRcnData->iY;

	double de = -(pRcnData->dAngle);
	FitRotate_cut(pRcnData, de, rz, rr);

	pRcnData->iX = pRcnData->iCutX + rr.x;
	pRcnData->iY = pRcnData->iCutY + rr.y;
	return 0;
}

//针对葛发松版, 识别票据版的;
//用户输入的是mm，坐标；
int WINAPI RCN_GetCutPictureSealPointByRecognize(RCNDATA *pRcnData)
{
	double x_mm_pixNum = ((double)theApp.m_iXScaleplate)/40;
	double y_mm_pixNum = ((double)theApp.m_iYScaleplate)/40;

	int iRet = -1;
	if ((pRcnData->iCheckX <= 0) || (pRcnData->iCheckY <= 0))
	{
		return iRet;
	}
	else
	{	
		//只需要把坐标点，转换到模板图上的位置;	
		if(pRcnData->bDirectionTitle == 0)
		{
			//上；
			pRcnData->iCheckX *= x_mm_pixNum;
			pRcnData->iCheckY *= y_mm_pixNum;
		}
		else if(pRcnData->bDirectionTitle == 1)
		{
			//下
			pRcnData->iCheckX = pRcnData->iCutWidth - (pRcnData->iCheckX)*x_mm_pixNum;
			pRcnData->iCheckY = pRcnData->iCutHeight - (pRcnData->iCheckY)*y_mm_pixNum;
		}
		else if(pRcnData->bDirectionTitle == 2)
		{
			//左
			int pos_x_mm = pRcnData->iCheckX;
			int pos_y_mm = pRcnData->iCheckY;
			pRcnData->iCheckX = pos_y_mm * y_mm_pixNum;
			pRcnData->iCheckY = pRcnData->iCutHeight -  pos_x_mm * x_mm_pixNum;
		}
		else if(pRcnData->bDirectionTitle == 3)
		{
			//右
			int pos_x_mm = pRcnData->iCheckX;
			int pos_y_mm = pRcnData->iCheckY;
			pRcnData->iCheckX = pRcnData->iCutWidth - (pos_y_mm * y_mm_pixNum);
			pRcnData->iCheckY = pos_x_mm * x_mm_pixNum;
		}
		pRcnData->iX = pRcnData->iCheckX;
		pRcnData->iY = pRcnData->iCheckY;
	}

	//根据原图中的盖章坐标点
	CvPoint rz, rr;
	rz.x = pRcnData->iX;
	rz.y = pRcnData->iY;
	double de = -(pRcnData->dAngle);
	FitRotate_cut(pRcnData, de, rz, rr);
	pRcnData->iX = pRcnData->iCutX + rr.x;
	pRcnData->iY = pRcnData->iCutY + rr.y;

	return 0;
}

//针对李良，大白纸自定义盖章。
_declspec(dllexport)  int WINAPI RCN_GetInforImage(DWORD dwImg, RCNDATA *pRcnData, int iReserve)
{
	int iRet = -1;
	if (!dwImg || !pRcnData)
	{
		iRet = -2;
		return iRet;
	}
	try
	{
		pRcnData->bDirectionTitle == 0;
		iReserve = theApp.m_idebug;
		theApp.m_isSmall = false;
		theApp.m_pRcnData = pRcnData;
		IplImage *pImg = NULL, *pSrcImg = NULL;
		pSrcImg = (IplImage *)dwImg;
		Mat pic;
		if(pSrcImg->nChannels == 4)
		{
			pic = Mat(pSrcImg, TRUE);
			cv::cvtColor(pic, pic, CV_BGRA2BGR); 
			pImg = &IplImage(pic);
		}
		else
			pImg = pSrcImg;

		if (iReserve & 2)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
		}
		RotatedRect rectPoint;
		cv::Rect roi;
		iRet = RCN_GetCutPictureParagramInner(pImg, rectPoint, roi, iReserve);
		if (iRet != 0)
		{
			if (theApp.m_idebug == 1)
				theApp.putLog("获取内部参数失败");
			return iRet;
		}
		CvBox2D rect;
		CvRect ROI_rect;
		CvPoint2D32f center;
		CvPoint i_pt[4];
		double angle = 0.0;
		rect = rectPoint;
		ROI_rect = roi;
		if(ROI_rect.height > pImg->height)
			ROI_rect.height = pImg->height - 1;
		if(ROI_rect.width > pImg->width)
			ROI_rect.width = pImg->width - 1;
		IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
		pRcnData->fCenterX = center.x;
		pRcnData->fCenterY = center.y;
		pRcnData->iCutX = ROI_rect.x;
		pRcnData->iCutY = ROI_rect.y;
		pRcnData->dAngle = angle;
		if (RoiImg)
		{
			cvResetImageROI(RoiImg);
			cvReleaseImage(&RoiImg);
		}			
	}
	catch (...)
	{
		iRet = -3;
	}
	return iRet;
}



//裁剪图片，老方式；
/*
__declspec(dllexport)  int WINAPI RCN_GetCutPicture(DWORD dwImg, char *pID, char *pPath, int iImgReversal, BYTE bQulity, long xDpi, long yDpi)
{


int iRet = -1;
if(!pID || !pPath)
{
iRet = -2;
return iRet;
}
try
{
DWORD dw = GetTickCount();
IplImage *src, *pImg;
pImg = (IplImage *)dwImg;
if(pImg == NULL)
{
return -1;
}

IplImage * pImg_clipper = cvCreateImage(cvSize(pImg->width, pImg->height), pImg->depth, pImg->nChannels);
cvCopy(pImg, pImg_clipper);

#if 1
BYTE prev_avg_gray = get_avg_gray(pImg_clipper);
double scale = 0;
if(prev_avg_gray > 70)
{
scale = ((double)theApp.Bright)/prev_avg_gray;
}
else
{
scale = ((double)70)/prev_avg_gray;
}
cvConvertScale(pImg_clipper, pImg_clipper, scale);
#endif

src = cvCreateImage(cvGetSize(pImg), pImg->depth, 1);
cvCvtColor(pImg_clipper, src, CV_RGB2GRAY);
cvReleaseImage(&pImg_clipper);

IplImage* grey_auto_threshold = cvCreateImage(cvSize(src->width, src->height), 8, 1);
cvCopy(src, grey_auto_threshold);
CvMemStorage* storage = cvCreateMemStorage(0);
CvSeq* contour = 0;

//细化
if(prev_avg_gray < theApp.Bright)
{
cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY); //二值化
cvDilate(src, src, 0, 1);
}
else
{
cvThreshold(src, src, theApp.min_threshold, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
}

//提取轮廓
cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );   //CV_RETR_CCOMP
CvSeq *_contour1;
CvSeq *_contour2;
GetAreaMaxContour(contour, &_contour1, &_contour2);
if(_contour1 == 0)
{
cvReleaseMemStorage(&storage);
cvReleaseImage(&src);
cvReleaseImage(&grey_auto_threshold);
}

CvRect ROI_rect, ROI_rect2;
ROI_rect =  cvBoundingRect(_contour1);
CvBox2D rect = cvMinAreaRect2(_contour1);
if(ROI_rect.width < ((src->width)-200))
{
cv::Mat threshold_image;
cv::threshold(cv::Mat(grey_auto_threshold), threshold_image, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::findContours(threshold_image, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
int max_contours_index = 0;
int max_width = 0;
int idx = 0;
for (; idx < contours.size(); idx++)
{
cv::Rect rectangle_char = cv::boundingRect(contours[idx]);
int area = rectangle_char.area();
if(max_width < area)
{
max_width = area;
max_contours_index = idx;
}
}

cv::Rect roi  = cv::boundingRect(contours[max_contours_index]);
ROI_rect = roi;
cv::RotatedRect rotat_roi = cv::minAreaRect(contours[max_contours_index]);
rect = rotat_roi;
}

CvPoint2D32f center;
CvPoint i_pt[4];
double angle = 0.0;
IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
if(RoiImg)
{
DWORD dw = GetTickCount();
iRet = 0;
CvPoint r_pt[4];
IplImage* pImgDst = rotateImagexCore(RoiImg, angle, center, i_pt, r_pt);

SYSTEMTIME st;
CString sFileName;
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
theApp.m_pdst = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
cvCopy(pImgDst, theApp.m_pdst);
cvResetImageROI(pImgDst);
cvReleaseImage(&pImgDst);
sFileName.Format("%s/cut_%s.jpg", pPath, pID);

cv::Mat temp2(theApp.m_pdst, 0);
vector<uchar> buff;
cv::imencode(".bmp", temp2, buff);
theApp.cvtSaveImage(&buff[0], buff.size(), 1, (LPSTR)(LPCTSTR)sFileName, 0, bQulity, xDpi, yDpi);

cvResetImageROI(RoiImg);
cvReleaseImage(&RoiImg);
cvReleaseImage(&theApp.m_pdst);
}
cvReleaseMemStorage(&storage);
cvReleaseImage(&src);
cvReleaseImage(&grey_auto_threshold);
}
catch (...)
{
iRet = -3;
}
return iRet;
}
*/

//增加新的裁剪方式;
__declspec(dllexport)  int WINAPI RCN_GetCutPicture(DWORD dwImg, char *pID, char *pPath, int iImgReversal, BYTE bQulity, long xDpi, long yDpi)
{
	int iReserve = 0;
	iReserve = theApp.m_idebug;

	//theApp.putLog("新的裁剪方式");

	int iRet = -1;
	if (!pID || !pPath)
	{
		theApp.putLog("PID || PATH empty!!");
		iRet = -2;
		return iRet;
	}
	try
	{
		DWORD dw = GetTickCount();
		IplImage  *pImg;
		pImg = (IplImage *)dwImg;
		if (pImg == NULL)
		{
			theApp.putLog("pImg empty!!");
			return -1;
		}
#if 0
		if (iReserve & 1)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
			//cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
		}
		IplImage * pImg_grey = cvCreateImage(cvSize(pImg->width, pImg->height), IPL_DEPTH_8U, 1);
		cvCvtColor(pImg, pImg_grey, CV_BGR2GRAY);
		if (iReserve & 1)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_灰度图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg_grey);
			//cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
		}

		int blockSize = 25;
		int constValue = 70;
		cv::Mat bin_image;
		cv::Mat grey_Mat;
		grey_Mat = cv::Mat(pImg_grey, true);
		cv::adaptiveThreshold(grey_Mat, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);

		IplImage* image = &IplImage(bin_image);
		IplImage* smoothImg = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
		cvCopy(image, smoothImg);
		cvDilate(smoothImg, smoothImg, 0, 2);
		cvErode(smoothImg, smoothImg, 0, 2);

		//cvNamedWindow("cvDilate", CV_WINDOW_AUTOSIZE); 
		//cvShowImage("cvDilate", smoothImg);  
		//waitKey(0);

		if (iReserve & 1)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_预处理图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, smoothImg);
			//cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
		}

		vector<Point> points;
		//获取轮廓   
		std::vector<std::vector<Point>> contours;
		Mat imagex(smoothImg, 0);
		//获取轮廓：   
		findContours(imagex, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		std::vector<std::vector<Point>>::const_iterator itc = contours.begin();
		std::vector<Point>::const_iterator itpp;
		while (itc != contours.end())
		{
			for (itpp = itc->begin(); itpp != itc->end(); itpp++)
			{
				points.push_back(*itpp);
			}
			itc++;
		}


		RotatedRect rectPoint = minAreaRect(points);
		cv::Rect roi = cv::boundingRect(points);

		if (iReserve & 1)
		{
			Mat imagex2(pImg, 1);
			Point2f fourPoint2f[4];
			rectPoint.points(fourPoint2f);
			RNG &rng = theRNG();
			//根据得到的四个点的坐标  绘制矩形  
			for (int i = 0; i < 3; i++)
			{
				line(imagex2, fourPoint2f[i], fourPoint2f[i + 1], Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3);
			}
			line(imagex2, fourPoint2f[0], fourPoint2f[3], Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3);

			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪示意图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			//cvSaveImage((LPSTR)(LPCTSTR)sFileName, smoothImg);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, imagex2);
		}

#elif 1
		RotatedRect rectPoint;
		cv::Rect roi;
		iRet = RCN_GetCutPictureParagramInner(pImg, rectPoint, roi, iReserve);
		if (iRet != 0)
		{
			theApp.putLog("获取内部参数失败");
			return iRet;
		}
#endif
		CvBox2D rect;
		CvRect ROI_rect;
		CvPoint2D32f center;
		CvPoint i_pt[4];
		double angle = 0.0;
		rect = rectPoint;
		ROI_rect = roi;

		IplImage *RoiImg = capImageAndCalcAngleCore(pImg, rect, ROI_rect, i_pt, center, angle);
		if (RoiImg)
		{
			DWORD dw = GetTickCount();
			iRet = 0;
			CvPoint r_pt[4];
			IplImage* pImgDst = rotateImagexCore(RoiImg, angle, center, i_pt, r_pt);

			SYSTEMTIME st;
			CString sFileName;
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
			IplImage* tmpImgDst;
			tmpImgDst = cvCreateImage(cvSize(I_rect.width, I_rect.height), 8, 3);
			cvCopy(pImgDst, tmpImgDst);
			cvResetImageROI(pImgDst);
			cvReleaseImage(&pImgDst);
			sFileName.Format("%s/cut_%s.jpg", pPath, pID);

			cv::Mat temp2;//(theApp.m_pdst, 0);
			temp2 = cv::Mat(tmpImgDst, true);
			vector<uchar> buff;
			cv::imencode(".jpg", temp2, buff);
			theApp.cvtSaveImage(&buff[0], buff.size(), 1, (LPSTR)(LPCTSTR)sFileName, 0, bQulity, xDpi, yDpi);

			cvResetImageROI(RoiImg);
			cvReleaseImage(&RoiImg);
			cvReleaseImage(&tmpImgDst);
		}
	}
	catch (...)
	{
		iRet = -3;
	}
	return iRet;
}





__declspec(dllexport)  int WINAPI RCN_GetScalePicture(DWORD dwImg, char *pID, char *pPath, float fscale, BYTE bQulity, long xDpi, long yDpi)
{
	int iRet = -1;
	if (!pID || !pPath)
	{
		iRet = -2;
		return iRet;
	}
	try
	{
		CvSize dst_cvsize;
		IplImage *pdst, *pImg;
		pImg = (IplImage *)dwImg;

		dst_cvsize.width = pImg->width*fscale;   //目标图像宽度   
		dst_cvsize.height = pImg->height*fscale; //目标图像高度     
		pdst = cvCreateImage(dst_cvsize, pImg->depth, pImg->nChannels); //创建一个目标图像     
		cvResize(pImg, pdst, CV_INTER_LINEAR); //缩放

		CString sFileName;
		sFileName.Format("%s/cut_%s.jpg", pPath, pID);

		cv::Mat temp2(pdst, 0);
		vector<uchar> buff;
		cv::imencode(".jpg", temp2, buff);
		theApp.cvtSaveImage(&buff[0], buff.size(), 1, (LPSTR)(LPCTSTR)sFileName, 0, bQulity, xDpi, yDpi);

		cvReleaseImage(&pdst);
		iRet = 0;
	}
	catch (...)
	{
		iRet = -3;
	}
	return iRet;
}

__declspec(dllexport)  int WINAPI RCN_GetOriginPicture(DWORD dwImg, char *pID, char *pPath, BYTE bQulity, long xDpi, long yDpi)
{
	int iRet = -1;
	if (!pID || !pPath)
	{
		iRet = -2;
		return iRet;
	}
	try
	{
		IplImage *src, *pImg;
		pImg = (IplImage *)dwImg;

		CString sFileName;
		sFileName.Format("%s\\origin_%s.jpg", pPath, pID);

		cv::Mat temp2(pImg, 0);
		vector<uchar> buff;
		cv::imencode(".bmp", temp2, buff);
		theApp.cvtSaveImage(&buff[0], buff.size(), 1, (LPSTR)(LPCTSTR)sFileName, 0, bQulity, xDpi, yDpi);

		iRet = 0;
	}
	catch (...)
	{
		iRet = -3;
	}
	return iRet;
}


//CV_IMAGE_ELEM( img, T, y, x )    
void GetMaxMin(IplImage*img, int *max, int *min)
{
	uchar tmp;
	int i = 0, j = 0;
	for (i = 0; i < img->width; i++)
		for (j = 0; j < img->height; j++)
		{
			tmp = CV_IMAGE_ELEM(img, uchar, j, i);
			if (tmp > *max) *max = tmp;
			if (tmp < *min) *min = tmp;
		}
}

void PaintColor(IplImage *img, int r = 255, int g = 255, int b = 255)
{
	int i = 0, j = 0;
	for (i = 0; i < img->width; i++)
		for (j = 0; j < img->height; j++) {
			CV_IMAGE_ELEM(img, uchar, j, i * 3) = b;
			CV_IMAGE_ELEM(img, uchar, j, i * 3 + 1) = g;
			CV_IMAGE_ELEM(img, uchar, j, i * 3 + 2) = r;
		}
}

bool HSVAddS(IplImage *img, double dH, double dS, double dV)
{
	bool  bResult = false;
	int   x, y;
	int   nValue;
	float fValue;
	int   nH, nS, nV;
	float fH, fS, fV;
	unsigned char *pLine = NULL;
	unsigned char *pPixel = NULL;
	float *pFloatPixel = NULL;
	CvRect rect = cvGetImageROI(img);
	switch (img->depth)
	{
	case IPL_DEPTH_8U:
		nH = (int)dH;
		nS = (int)dS;
		nV = (int)dV;
		pLine = (unsigned char*)(img->imageData + rect.y * img->widthStep + rect.x * img->nChannels);
		for (y = 0; y < rect.height; y++, pLine += img->widthStep)
		{
			for (pPixel = pLine, x = 0; x < rect.width; x++, pPixel += img->nChannels)
			{
				// 色调                                
				nValue = (int)pPixel + nH;
				if (nValue > 180)
				{
					nValue -= 180;
				}
				else if (nValue < 0)
				{
					nValue += 180;
				}
				*pPixel = (unsigned char)nValue;
				// 饱和度                                
				nValue = (int)pPixel + nS;
				if (nValue > 255)
				{
					nValue = 255;
				}
				else if (nValue < 0)
				{
					nValue = 0;
				}
				*pPixel = (unsigned char)nValue;
				// 纯度                                
				nValue = (int)pPixel + nV;
				if (nValue > 255)
				{
					nValue = 255;
				}
				else if (nValue < 0)
				{
					nValue = 0;
				}
				*pPixel = (unsigned char)nValue;
			}
		}
		bResult = true;
		break;
	case IPL_DEPTH_32F:
		fH = (float)dH;
		fS = (float)dS;
		fV = (float)dV;
		pLine = (unsigned char*)(img->imageData + rect.y * img->widthStep + rect.x * img->nChannels);
		for (y = 0; y < rect.height; y++, pLine += img->widthStep)
		{
			pFloatPixel = (float*)pLine;
			for (pPixel = pLine, x = 0; x < rect.width; x++, *pFloatPixel += img->nChannels)
			{
				// 色调                                
				fValue = *pFloatPixel + fH;
				if (fValue > 360)
				{
					fValue -= 360;
				}
				else if (fValue < 0)
				{
					fValue += 360;
				}
				*pFloatPixel = fValue;
				// 饱和度                                
				fValue = *pFloatPixel + fS;
				if (fValue > 1)
				{
					fValue = 1;
				}
				else if (fValue < 0)
				{
					fValue = 0;
				}
				*pFloatPixel = fValue;
				// 纯度                                
				fValue = *pFloatPixel + fV;
				if (fValue > 1)
				{
					fValue = 1;
				}
				else if (fValue < 0)
				{
					fValue = 0;
				}
				*pFloatPixel = fValue;
			}
		}
		bResult = true;
		break;
	}
	return bResult;
}

// 图像色调调节
bool ImgAdjustHSV(IplImage *src, IplImage *dst, double dH, double dS, double dV)
{
	bool  bResult = false;
	if (src && dst)
	{
		if (src->depth == dst->depth &&
			src->nChannels == dst->nChannels &&
			src->width == dst->width &&
			src->height == dst->height)
		{
			int    nCOI = cvGetImageCOI(src);
			CvRect rect = cvGetImageROI(src);
			cvSetImageCOI(src, 0);
			cvSetImageCOI(dst, 0);
			cvSetImageROI(dst, rect);
			// 计算 HSV                        
			cvCvtColor(src, dst, CV_BGR2HSV);
			// 色调调节                        
			HSVAddS(dst, dH, dS, dV);
			// 还原图像                        
			cvCvtColor(dst, dst, CV_HSV2BGR);
			// 恢复 COI                        
			cvSetImageCOI(src, nCOI);
			cvSetImageCOI(dst, nCOI);
			bResult = true;
		}
	}
	return bResult;
}

int cutVerifyCode(cv::Mat in_image, cv::Mat& out_image, int angle, cv::Rect in_sealCodeZone, int iReserve)
{
	cv::Mat image_rotate;
	if (angle == 1)
	{
		//逆时针进行旋转90
		transpose(in_image, image_rotate);
		flip(image_rotate, image_rotate, 0);

	}
	else if (angle == -1)
	{
		//顺时针进行旋转90
		transpose(in_image, image_rotate);
		flip(image_rotate, image_rotate, 1);
	}
	else
	{
		//不旋转
		image_rotate = in_image;
	}

	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图_旋转图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_rotate);
	}

	//判断验证码区间的合理性
	//cv::Rect rect_image;
	//rect_image.x = 0;
	//rect_image.y = 0;
	//rect_image.width = image_rotate.size().width;
	//rect_image.height = image_rotate.size().height;

	//char buf_message[1024];
	//memset(buf_message, 0, 1024);
	//if (!rect_image.contains(cv::Point(in_sealCodeZone.x, in_sealCodeZone.y)) ||
	//	!rect_image.contains(cv::Point(in_sealCodeZone.x + in_sealCodeZone.width, in_sealCodeZone.y + in_sealCodeZone.height)))
	//{
	//	sprintf(buf_message, "%s(%d %d %d %d)", "验证码区间", in_sealCodeZone.x, in_sealCodeZone.y, in_sealCodeZone.width, in_sealCodeZone.height);
	//	theApp.putLog(buf_message);

	//	memset(buf_message, 0, 1024);
	//	sprintf(buf_message, "%s(%d, %d)", "图片的尺寸", image_rotate.size().width, image_rotate.size().height);
	//	theApp.putLog(buf_message);
	//	theApp.putLog("输入的坐标不合理");
	//	return -1;
	//}

	cv::Point pt_center;
	pt_center.x = in_sealCodeZone.x + in_sealCodeZone.width / 2;
	pt_center.y = in_sealCodeZone.y + in_sealCodeZone.height / 2;

	cv::Rect rect_image;
	rect_image.x = 0;
	rect_image.y = 0;
	rect_image.width = image_rotate.size().width;
	rect_image.height = image_rotate.size().height;

	if (!rect_image.contains(pt_center))
	{
		theApp.putLog("输入的坐标不合理");
		return -1;
	}
	if (in_sealCodeZone.x < 0)
	{
		in_sealCodeZone.x = 0;
	}
	if ((in_sealCodeZone.x + in_sealCodeZone.width) > image_rotate.cols)
	{
		in_sealCodeZone.width = image_rotate.cols - in_sealCodeZone.x - 3;
	}
	if (in_sealCodeZone.y < 0)
	{
		in_sealCodeZone.y = 0;
	}
	if ((in_sealCodeZone.y + in_sealCodeZone.height) > image_rotate.rows)
	{
		in_sealCodeZone.height = image_rotate.rows - in_sealCodeZone.y - 3;
	}
	if (in_sealCodeZone.x < 0 || in_sealCodeZone.y < 0 ||
		((in_sealCodeZone.x + in_sealCodeZone.width) > image_rotate.size().width) ||
		((in_sealCodeZone.y + in_sealCodeZone.height) > image_rotate.size().height) ||
		((in_sealCodeZone.x + in_sealCodeZone.width) < 0) ||
		((in_sealCodeZone.y + in_sealCodeZone.height) < 0)
		)
	{
		return -1;
	}

	//裁剪验证码区间
	cv::Mat imgSealCode = image_rotate(in_sealCodeZone);
	if (iReserve & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_验证码_裁剪图_彩图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imgSealCode);
	}
	out_image = imgSealCode.clone();
	return 0;
}

/*
in_image: 输入的原始图片
angle   :  (0) 不旋转 （1） 逆时针旋转 （-1） 顺时针旋转
in_std_image: 标准的验证码图
out_result: 输出的识别结果
*/
int VerifySealCodeBarInner(cv::Mat in_image, cv::Rect in_sealCodeZone, int angle, cv::Mat in_std_image, char* out_result, int *pMinOper, int iReserve)
{
	//首先对图片进行旋转;
	cv::Mat image_rotate;
	if (angle == 1)
	{
		//逆时针进行旋转90
		transpose(in_image, image_rotate);
		flip(image_rotate, image_rotate, 0);
	}
	else if (angle == -1)
	{
		//顺时针进行旋转90
		transpose(in_image, image_rotate);
		flip(image_rotate, image_rotate, 1);
	}
	else
	{
		//不旋转
		image_rotate = in_image;
	}

	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图_旋转图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_rotate);
	}

	//判断验证码区间的合理性
	/*
	cv::Rect rect_image;
	rect_image.x = 0;
	rect_image.y = 0;
	rect_image.width = image_rotate.size().width;
	rect_image.height = image_rotate.size().height;
	char buf_message[1024];
	memset(buf_message, 0, 1024);
	if(!rect_image.contains(cv::Point(in_sealCodeZone.x, in_sealCodeZone.y)) ||
	!rect_image.contains(cv::Point(in_sealCodeZone.x + in_sealCodeZone.width, in_sealCodeZone.y + in_sealCodeZone.height)))
	{
	sprintf(buf_message, "%s(%d %d %d %d)", "验证码区间", in_sealCodeZone.x, in_sealCodeZone.y, in_sealCodeZone.width, in_sealCodeZone.height);
	theApp.putLog(buf_message);

	memset(buf_message, 0, 1024);
	sprintf(buf_message, "%s(%d, %d)", "图片的尺寸", image_rotate.size().width, image_rotate.size().height);
	theApp.putLog(buf_message);
	theApp.putLog("输入的坐标不合理");
	return -1;
	}
	*/
	cv::Point pt_center;
	pt_center.x = in_sealCodeZone.x + in_sealCodeZone.width / 2;
	pt_center.y = in_sealCodeZone.y + in_sealCodeZone.height / 2;

	cv::Rect rect_image;
	rect_image.x = 0;
	rect_image.y = 0;
	rect_image.width = image_rotate.size().width;
	rect_image.height = image_rotate.size().height;

	if (!rect_image.contains(pt_center))
	{
		theApp.putLog("输入的坐标不合理");
		return -1;
	}



	if (in_sealCodeZone.x < 0)
	{
		in_sealCodeZone.x = 0;
	}
	if ((in_sealCodeZone.x + in_sealCodeZone.width) > image_rotate.cols)
	{
		in_sealCodeZone.width = image_rotate.cols - in_sealCodeZone.x - 3;
	}
	if (in_sealCodeZone.y < 0)
	{
		in_sealCodeZone.y = 0;
	}
	if ((in_sealCodeZone.y + in_sealCodeZone.height) > image_rotate.rows)
	{
		in_sealCodeZone.height = image_rotate.rows - in_sealCodeZone.y - 3;
	}
	if (in_sealCodeZone.x < 0 || in_sealCodeZone.y < 0 ||
		((in_sealCodeZone.x + in_sealCodeZone.width) > image_rotate.size().width) ||
		((in_sealCodeZone.y + in_sealCodeZone.height) > image_rotate.size().height) ||
		((in_sealCodeZone.x + in_sealCodeZone.width) < 0) ||
		((in_sealCodeZone.y + in_sealCodeZone.height) < 0)
		)
	{
		return -1;
	}

	//裁剪验证码区间
	cv::Mat imgSealCode = image_rotate(in_sealCodeZone);
	if (iReserve & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_验证码_裁剪图_彩图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imgSealCode);
	}

	cv::Mat grey_clipperSealCode;
	if (imgSealCode.channels() == 3)
	{
		cv::cvtColor(imgSealCode, grey_clipperSealCode, CV_BGR2GRAY);
	}
	else
	{
		grey_clipperSealCode = imgSealCode;
	}

	if (iReserve & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_验证码_裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, grey_clipperSealCode);
	}
	//增加验证码是否需要二值处理;
	if (theApp.m_bVerifyCodeBinPW == TRUE)
	{
		int blockSize = 25;
		int constValue = theApp.m_VerifyCodeBinValue;
		cv::adaptiveThreshold(grey_clipperSealCode, grey_clipperSealCode, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
		if (iReserve & 4)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_二值处理图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, grey_clipperSealCode);
		}	
	}



	std::string result_sealCode = "";
	int iRet = compareSealCode(in_std_image, grey_clipperSealCode, result_sealCode, pMinOper);
	memcpy(out_result, result_sealCode.c_str(), 14);
	return iRet;
}

//计算字符串相似度算法
int ldistance(const string source, const string target)
{
	//step 1
	int n = source.length();
	int m = target.length();
	if (m == 0) return n;
	if (n == 0) return m;
	//Construct a matrix
	typedef vector< vector<int> >  Tmatrix;
	Tmatrix matrix(n + 1);
	for (int i = 0; i <= n; i++)  matrix[i].resize(m + 1);

	//step 2 Initialize
	for (int i = 1; i <= n; i++) matrix[i][0] = i;
	for (int i = 1; i <= m; i++) matrix[0][i] = i;

	//step 3
	for (int i = 1; i <= n; i++)
	{
		const char si = source[i - 1];
		//step 4
		for (int j = 1; j <= m; j++)
		{
			const char dj = target[j - 1];
			//step 5
			int cost;
			if (si == dj) {
				cost = 0;
			}
			else {
				cost = 1;
			}
			//step 6
			const int above = matrix[i - 1][j] + 1;
			const int left = matrix[i][j - 1] + 1;
			const int diag = matrix[i - 1][j - 1] + cost;
			matrix[i][j] = min(above, min(left, diag));
		}
	}//step7
	return matrix[n][m];
}

void GetCurDir(char* szDir)
{
	TCHAR szPath[1024];
#ifdef TESTDEMO
	GetModuleFileName(GetModuleHandle("IFeature.dll"), szPath, 1024);
#else

	//GetModuleFileName(GetModuleHandle("ABC.Dfjy.Device.STAMP.IABCFeature.dll"), szPath, 1024);
	HMODULE hModule;
	hModule = DumpModuleFeature();
	GetModuleFileName(hModule, szPath, 1024);
#endif
	GetLongPathName(szPath, szDir, MAX_PATH);
	PathRemoveFileSpec(szDir);
	return;
}

/*
in_image: 输入的二值图
points: 返回的匹配点
pathTemplate: 模板图片的路径
*/
int calcMatchedFeaturePoints(cv::Mat in_image, std::vector<cv::Point>& points, char* pathTemplate)
{
	//char bufPath[1024];
	//GetCurDir(bufPath);
	//strcat(bufPath,"\\VerifyCodeImg\\templateVerifyCode.png");

	Mat input1 = imread(pathTemplate, 0);
	Mat input2 = in_image;

	//缩小图片，提高检测速度
	cv::resize(input1, input1, cv::Size(0, 0), theApp.m_scaleVerifyCode, theApp.m_scaleVerifyCode);
	cv::resize(input2, input2, cv::Size(0, 0), theApp.m_scaleVerifyCode, theApp.m_scaleVerifyCode);
	if (input2.channels() == 3)
	{
		cv::cvtColor(input2, input2, CV_BGR2GRAY);
	}

	if (input1.empty() || input2.empty())
	{
		return -1;
	}

	/************************************************************************/
	/*下面进行提取特征点*/
	/************************************************************************/
	SiftFeatureDetector feature;
	vector<KeyPoint> kerpoints1;
	feature.detect(input1, kerpoints1);
	if (kerpoints1.size() < 1)
	{
		return -1;
	}

	Mat output1;
	vector<KeyPoint> kerpoints2;
	feature.detect(input2, kerpoints2);
	if (kerpoints2.size() < 1)
	{
		return -1;
	}

	Mat output2;
	/************************************************************************/
	/* 下面进行特征向量提取 */
	/************************************************************************/
	SiftDescriptorExtractor descript;
	Mat description1;
	descript.compute(input1, kerpoints1, description1);
	Mat description2;
	descript.compute(input2, kerpoints2, description2);
	/************************************************************************/
	/* 下面进行特征向量临近匹配 */
	/************************************************************************/
	vector<DMatch> matches;
	BFMatcher matcher;
	Mat image_match;
	matcher.match(description1, description2, matches);

	/************************************************************************/
	/* 下面计算向量距离的最大值与最小值 */
	/************************************************************************/
	double max_dist = 0, min_dist = 100;
	for (int i = 0; i < description1.rows; i++)
	{
		if (matches.at(i).distance > max_dist)
		{
			max_dist = matches[i].distance;
		}
		if (matches[i].distance < min_dist)
		{
			min_dist = matches[i].distance;
		}
	}
	//cout << "最小距离为" << min_dist << endl;
	//cout << "最大距离为" << max_dist << endl;
	/************************************************************************/
	/* 得到距离小于而V诶最小距离的匹配 */
	/************************************************************************/
	vector<DMatch> good_matches;
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance < 5 * min_dist)
		{
			good_matches.push_back(matches[i]);
			//cout <<"第一个图中的"<< matches[i].queryIdx<<"匹配了第二个图中的"<<matches[i].trainIdx<<endl;
			int index = matches[i].trainIdx;
			cv::Point pt = kerpoints2[index].pt;
			pt.x = pt.x / theApp.m_scaleVerifyCode;
			pt.y = pt.y / theApp.m_scaleVerifyCode;
			points.push_back(pt);
		}
	}
	return 0;
}


//移除竖直方向的孤立点，以及不匹配竖直验证码形状的点
void removeErrorPointsVertical(std::vector<cv::Point>& in_points,
							   std::vector<cv::Point>& out_points,
							   std::vector<CResultLabel>& labels,
							   std::vector<cv::Rect>& out_boxes)
{
	//根据距离先过滤，过滤掉孤立的点
	//保留密集的点
	int max_distence = 50;
	int min_neighbour = 1;
	int num_maxNear = 0;
	int index_num_maxNear = 0;
	for (int i = 0; i < in_points.size(); i++)
	{
		int num_neighbour = 0;
		cv::Point pt1 = in_points[i];
		CResultLabel label;
		for (int j = 0; j < in_points.size(); j++)
		{
			if (i == j)
			{
				continue;
			}
			cv::Point pt2 = in_points[j];
			int distence = sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
			if (distence < max_distence)
			{
				num_neighbour++;
			}
		}
		if (num_neighbour >= min_neighbour)
		{
			label.isValid = true;
			label.num_neighbour = num_neighbour;
			labels.push_back(label);
			if (num_maxNear < num_neighbour)
			{
				num_maxNear = num_neighbour;
				index_num_maxNear = i;
			}
		}
		else
		{
			labels.push_back(label);
		}
	}

	labels[index_num_maxNear].isMaxConcentrated = true;

	//针对竖直的
	//对密集点，根据形状的特性进行过滤
	//根据水平间距，竖直间距,再过滤
#if 1
	int deltaXmax = theApp.DeltaXMax;
	int deltaYmax = theApp.DeltaYMax;
	int deltaYmin = theApp.DeltaYMin;
	for (int i = 0; i < in_points.size(); i++)
	{
		if (labels[i].isValid == false)
		{
			continue;
		}
		cv::Point pt1 = in_points[i];
		bool MatchShapeFlag = false;
		for (int j = 0; j < in_points.size(); j++)
		{
			if (labels[i].isValid == false || i == j)
			{
				continue;
			}

			cv::Point pt2 = in_points[j];
			if (abs(pt1.x - pt2.x) < deltaXmax && abs(pt1.y - pt2.y) > deltaYmin && abs(pt1.y - pt2.y) < deltaYmax)
			{
				MatchShapeFlag = true;
				//这是一个近似的验证码区间;				
				//判断当前的点，再矩形框中是否有相近点				
				//out_boxes.push_back(cv::Rect(pt1, pt2));
				//lastSuccessPoint = pt1;
			}
		}
		if (MatchShapeFlag == false)
		{
			labels[i].isValid = false;
		}
		else
		{
			out_points.push_back(pt1);
		}
	}

#endif
}

double calcTwoPointDistence(cv::Point pt1, cv::Point pt2)
{
	double distence = sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
	return distence;
}

bool calcPointsCenter(std::vector<cv::Point> points, cv::Point& center)
{
	int x_sum = 0;
	int y_sum = 0;
	for (int i = 0; i < points.size(); i++)
	{
		x_sum += points[i].x;
		y_sum += points[i].y;
	}
	center.x = x_sum / points.size();
	center.y = y_sum / points.size();
	return true;
}

void removeOverLapPoint(cv::Mat& image, std::vector<cv::Point> in_points, std::vector<cv::Point>& out_points, int iReserve)
{
	int numClass = 0;
	std::vector<std::vector<cv::Point>> points_kind; //不同点集的集合
	std::vector<CResultLabel> labes(in_points.size());
	for (int i = 0; i < in_points.size(); i++)
	{
		if (labes[i].isProcessedClassify == true)
		{
			continue;
		}
		std::vector<cv::Point> points; //一类的点
		cv::Point pt1 = in_points[i];
		labes[i].isProcessedClassify = true;
		for (int j = 0; j < in_points.size(); j++)
		{
			if (i == j || labes[j].isProcessedClassify == true)
			{
				continue;
			}
			cv::Point pt2 = in_points[j];
			double distence = calcTwoPointDistence(pt1, pt2);
			if (distence < 50)
			{
				points.push_back(pt2);
				labes[j].isProcessedClassify = true;
			}
		}
		points_kind.push_back(points);
	}
	if (iReserve & 3)
	{
		cv::Mat image_show;
		cv::cvtColor(image, image_show, CV_GRAY2BGR);
		std::vector<cv::Scalar> color_map;
		color_map.push_back(cv::Scalar(0, 0, 255));
		color_map.push_back(cv::Scalar(0, 255, 0));
		color_map.push_back(cv::Scalar(255, 0, 0));
		color_map.push_back(cv::Scalar(0, 255, 255));
		color_map.push_back(cv::Scalar(255, 255, 0));
		cv::RNG rng;
		for (int i = 0; i < points_kind.size(); i++)
		{
			cv::Scalar color = color_map[i % 5];
			for (int j = 0; j < points_kind[i].size(); j++)
			{
				cv::circle(image_show, points_kind[i][j], 3, color, CV_FILLED);
			}
		}

		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_点分类图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_show);
	}

	//对同一类的点，求中心点
	for (int i = 0; i < points_kind.size(); i++)
	{
		cv::Point center;
		std::vector<cv::Point> points;
		points = points_kind[i];
		if (points.size() == 0)
		{
			continue;
		}
		calcPointsCenter(points, center);
		out_points.push_back(center);
	}
}


//竖直方向的验证码匹配
void shapeMatchVertical(std::vector<cv::Point> in_points, std::vector<cv::Rect>& out_clear_boxes)
{
	int deltaXMin = theApp.DeltaXMin;
	int deltaXmax = theApp.DeltaXMax;
	int deltaYmax = theApp.DeltaYMax;
	int deltaYmin = theApp.DeltaYMin;
	std::vector<CResultLabel> labes(in_points.size());
	std::vector<cv::Rect> out_boxes;
	for (int i = 0; i < in_points.size(); i++)
	{
		if (labes[i].isProcessedClassify == true)
		{
			continue;
		}
		labes[i].isProcessedClassify = true;
		cv::Point pt1 = in_points[i];
		bool MatchShapeFlag = false;
		for (int j = 0; j < in_points.size(); j++)
		{
			if (i == j || labes[j].isProcessedClassify == true)
			{
				continue;
			}

			cv::Point pt2 = in_points[j];
			if (abs(pt1.x - pt2.x) > deltaXMin && abs(pt1.x - pt2.x) < deltaXmax && abs(pt1.y - pt2.y) > deltaYmin && abs(pt1.y - pt2.y) < deltaYmax)
			{
				labes[j].isProcessedClassify = true;
				//MatchShapeFlag = true;
				//这是一个近似的验证码区间;				
				//判断当前的点，再矩形框中是否有相近点	
				//调整矩形的尺寸，可完全包括验证码区间
				cv::Rect  rect = cv::Rect(pt1, pt2);
				rect.x -= 45;
				rect.width += 90;
				rect.y -= 40;
				rect.height += 80;
				out_boxes.push_back(rect);
			}
		}
	}

	//移除重叠的被包含的验证码区域
	for (int i = 0; i < out_boxes.size(); i++)
	{
		bool is_contained = false;
		for (int j = 0; j < out_boxes.size(); j++)
		{
			if (i == j)
			{
				continue;
			}

			cv::Rect  tmp = out_boxes[j];
			tmp.x -= 20;
			tmp.y -= 20;
			tmp.width += 40;
			tmp.height += 40;

			cv::Point leftTop = cv::Point(out_boxes[i].x, out_boxes[i].y);
			cv::Point rightDown = cv::Point(out_boxes[i].x + out_boxes[i].width, out_boxes[i].y + out_boxes[i].height);
			if (tmp.contains(leftTop) && tmp.contains(rightDown))
			{
				is_contained = true;
			}
		}
		if (is_contained == false)
		{
			out_clear_boxes.push_back(out_boxes[i]);
		}
	}
}

void removeErrorPointsHorizontal(std::vector<cv::Point>& in_points, std::vector<cv::Point>& out_points, std::vector<CResultLabel>& labels,
								 std::vector<cv::Rect>& out_boxes)
{

	//根据距离先过滤，过滤掉孤立的点
	//保留密集的点
	int max_distence = 50;
	int min_neighbour = 3;
	int num_maxNear = 0;
	int index_num_maxNear = 0;
	for (int i = 0; i < in_points.size(); i++)
	{
		int num_neighbour = 0;
		cv::Point pt1 = in_points[i];
		CResultLabel label;
		for (int j = 0; j < in_points.size(); j++)
		{
			if (i == j)
			{
				continue;
			}
			cv::Point pt2 = in_points[j];
			int distence = sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
			if (distence < max_distence)
			{
				num_neighbour++;
			}
		}
		if (num_neighbour >= min_neighbour)
		{
			label.isValid = true;
			label.num_neighbour = num_neighbour;
			labels.push_back(label);

			if (num_maxNear < num_neighbour)
			{
				num_maxNear = num_neighbour;
				index_num_maxNear = i;
			}
		}
		else
		{
			labels.push_back(label);
		}
	}

	labels[index_num_maxNear].isMaxConcentrated = true;

	//针对水平的
	//对密集点，根据形状的特性进行过滤
	//根据水平间距，竖直间距,再过滤
#if 1
	int deltaXmin = theApp.DeltaYMin;
	int deltaXmax = theApp.DeltaYMax;
	int deltaYmax = theApp.DeltaXMax;
	for (int i = 0; i < in_points.size(); i++)
	{
		if (labels[i].isValid == false)
		{
			continue;
		}
		cv::Point pt1 = in_points[i];
		bool MatchShapeFlag = false;
		for (int j = 0; j < in_points.size(); j++)
		{
			if (labels[i].isValid == false || i == j)
			{
				continue;
			}
			cv::Point pt2 = in_points[j];
			if (abs(pt1.x - pt2.x) < deltaXmax && abs(pt1.x - pt2.x) > deltaXmin  && abs(pt1.y - pt2.y) < deltaYmax)
			{
				MatchShapeFlag = true;
				//这是一个近似的验证码区间;				
				//判断当前的点，再矩形框中是否有相近点				
				//out_boxes.push_back(cv::Rect(pt1, pt2));
				//lastSuccessPoint = pt1;
			}
		}
		if (MatchShapeFlag == false)
		{
			labels[i].isValid = false;
		}
		else
		{
			out_points.push_back(pt1);
		}
	}
#endif
}


//水平方向的验证码匹配
void shapeMatchHorizontal(std::vector<cv::Point> in_points, std::vector<cv::Rect>& out_clear_boxes)
{
	std::vector<cv::Rect> out_boxes;
	int deltaXMin = theApp.DeltaYMin;
	int deltaXmax = theApp.DeltaYMax;
	int deltaYmin = theApp.DeltaXMin;
	int deltaYmax = theApp.DeltaXMax;
	std::vector<CResultLabel> labes(in_points.size());
	for (int i = 0; i < in_points.size(); i++)
	{
		if (labes[i].isProcessedClassify == true)
		{
			continue;
		}
		labes[i].isProcessedClassify = true;
		cv::Point pt1 = in_points[i];
		bool MatchShapeFlag = false;
		for (int j = 0; j < in_points.size(); j++)
		{
			if (i == j || labes[j].isProcessedClassify == true)
			{
				continue;
			}

			cv::Point pt2 = in_points[j];
			if (abs(pt1.x - pt2.x) > deltaXMin && abs(pt1.x - pt2.x) < deltaXmax && abs(pt1.y - pt2.y) > deltaYmin && abs(pt1.y - pt2.y) < deltaYmax)
			{
				labes[j].isProcessedClassify = true;
				//MatchShapeFlag = true;
				//这是一个近似的验证码区间;				
				//判断当前的点，再矩形框中是否有相近点	
				//调整矩形的尺寸，可完全包括验证码区间
				cv::Rect  rect = cv::Rect(pt1, pt2);
				rect.x -= 40;
				rect.width += 80;
				rect.y -= 40;
				rect.height += 80;
				if (rect.x > 0 && rect.y > 0)
				{
					out_boxes.push_back(rect);
				}
			}
		}
	}

	//移除重叠的被包含的验证码区域
	for (int i = 0; i < out_boxes.size(); i++)
	{
		bool is_contained = false;
		for (int j = 0; j < out_boxes.size(); j++)
		{
			if (i == j)
			{
				continue;
			}

			cv::Rect  tmp = out_boxes[j];
			tmp.x -= 20;
			tmp.y -= 20;
			tmp.width += 40;
			tmp.height += 40;
			cv::Point leftTop = cv::Point(out_boxes[i].x, out_boxes[i].y);
			cv::Point rightDown = cv::Point(out_boxes[i].x + out_boxes[i].width, out_boxes[i].y + out_boxes[i].height);
			if (tmp.contains(leftTop) && tmp.contains(rightDown))
			{
				is_contained = true;
			}
		}
		if (is_contained == false)
		{
			out_clear_boxes.push_back(out_boxes[i]);
		}
	}
}

/*
in_image: 二值图
out_candiate_boxes: 输出的候选验证码区域
isVertical: true .验证码的方向： true,竖直， false 水平
*/
void findCandiateVerifyCodeBox(cv::Mat in_image, std::vector<cv::Rect>& out_candiate_boxes, int iReserve,
							   std::vector<cv::Point> matchPoints, bool is_vertical = true)
{
	int ret = -1;
	std::vector<cv::Point> out_points;
	std::vector<CResultLabel> out_labes;
	std::vector<cv::Rect> out_boxes;
	std::vector<cv::Point> result_fit_center;
	out_boxes.clear();
	if (is_vertical)
	{
		//竖直
		removeErrorPointsVertical(matchPoints, out_points, out_labes, out_boxes);
		removeOverLapPoint(in_image, out_points, result_fit_center, iReserve);
		shapeMatchVertical(result_fit_center, out_boxes);
	}
	else
	{
		//水平
		removeErrorPointsHorizontal(matchPoints, out_points, out_labes, out_boxes);
		removeOverLapPoint(in_image, out_points, result_fit_center, iReserve);
		shapeMatchHorizontal(result_fit_center, out_boxes);
	}
	out_candiate_boxes = out_boxes;
	if (iReserve & 7)
	{
		cv::Mat result_show_3;
		cv::cvtColor(in_image, result_show_3, CV_GRAY2BGR);
		for (int i = 0; i < out_boxes.size(); i++)
		{
			cv::rectangle(result_show_3, out_boxes[i], cv::Scalar(255, 0, 0), 3);
		}

		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪验证码区间.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, result_show_3);
	}
}


//识别验证码，没有票据识别的版本
int recognizeVerifyCodeNoType(cv::Mat in_image, cv::Mat in_image_std, std::vector<cv::Rect> in_boxes, std::string& out_verfiyCode, int iReserve)
{
	int iRet = -1;
	int minErrorOper = 1000000;
	std::string mostSimlaryResult = "";
	for (int i = 0; i < in_boxes.size(); i++)
	{
		cv::Rect tmpROI = in_boxes[i];
		//越界保护
		if (tmpROI.x < 0)
		{
			tmpROI.x = 0;
		}
		if (tmpROI.y < 0)
		{
			tmpROI.y = 0;
		}

		if ((tmpROI.x + tmpROI.width) > in_image.cols)
		{
			tmpROI.width = in_image.cols - tmpROI.x - 3;
		}

		if ((tmpROI.y + tmpROI.height) > in_image.rows)
		{
			tmpROI.height = in_image.rows - tmpROI.y - 3;
		}

		if (tmpROI.width <= 0 || tmpROI.height <= 0)
		{
			continue;
		}

		cv::Mat imageROI = in_image(tmpROI);
		if (iReserve & 3)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
		}

		cv::Mat image_rotate;
		bool flag_Vertical = false;
		//如果是竖直的，需要旋转90		
		if (imageROI.size().height > imageROI.size().width)
		{
			flag_Vertical = true;
			//逆时针进行旋转90
			transpose(imageROI, image_rotate);
			flip(image_rotate, image_rotate, 0);
			if (iReserve & 3)
			{
				SYSTEMTIME st;
				GetSystemTime(&st);
				CString sFileName;
				sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图_90.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
				cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_rotate);
			}
		}
		else
		{
			image_rotate = imageROI;
		}

		//识别		
		int minOper = 100000;
		std::string resultVerifyCode;
		iRet = compareSealCodeNoType(in_image_std, image_rotate, resultVerifyCode, &minOper);

		/*	char buf_message[1024];
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "iRet = %d, min operate: %d", iRet, minOper);
		theApp.putLog(buf_message);*/

		if (minOper < minErrorOper)
		{
			minErrorOper = minOper;
			mostSimlaryResult = resultVerifyCode;
		}

		if (iRet == 0 && minOper == 0)
		{
			out_verfiyCode = resultVerifyCode;
			return 0;
		}

		//如果跟标准差别比较大，需要再旋转180;
		transpose(imageROI, image_rotate);
		flip(image_rotate, image_rotate, 2);
		if (!flag_Vertical)
		{
			transpose(image_rotate, image_rotate);
			flip(image_rotate, image_rotate, 2);
		}

		if (iReserve & 3)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图_180.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_rotate);
		}

		//识别
		minOper = 100000;
		resultVerifyCode = "";
		iRet = compareSealCodeNoType(in_image_std, image_rotate, resultVerifyCode, &minOper);
		if (minOper < minErrorOper)
		{
			minErrorOper = minOper;
			mostSimlaryResult = resultVerifyCode;
		}

		//memset(buf_message, 0, sizeof buf_message);
		//sprintf(buf_message, "iRet = %d, min operate: %d", iRet, minOper);
		//theApp.putLog(buf_message);

		if (iRet == 0 && minOper == 0)
		{
			out_verfiyCode = resultVerifyCode;
			return 0;
		}
	}
	//如果输入的有错误，比如误输入以为，则返回最相近的结果
	out_verfiyCode = mostSimlaryResult;
	return iRet;
}


/*
此函数用于支持 输入的验证码的位数 不是12位的
*/
//识别验证码，没有票据识别的版本
int recognizeVerifyCodeNoTypeNoStdImage(cv::Mat in_image, std::vector<cv::Rect> in_boxes, std::string& out_verfiyCode, int iReserve)
{
	int iRet = -1;
	char buf_message[1024];
	std::string mostSimlaryResult = "";
	std::vector<std::string> results_candiate;
	std::vector<float> results_confidences;
	for (int i = 0; i < in_boxes.size(); i++)
	{
		cv::Rect tmpROI = in_boxes[i];

		//越界保护
		if (tmpROI.x < 0)
		{
			tmpROI.x = 0;
		}
		if (tmpROI.y < 0)
		{
			tmpROI.y = 0;
		}

		if ((tmpROI.x + tmpROI.width) > in_image.cols)
		{
			tmpROI.width = in_image.cols - tmpROI.x - 3;
		}

		if ((tmpROI.y + tmpROI.height) > in_image.rows)
		{
			tmpROI.height = in_image.rows - tmpROI.y - 3;
		}

		if (tmpROI.width <= 0 || tmpROI.height <= 0)
		{
			continue;
		}

		cv::Mat imageROI = in_image(tmpROI);
		if (iReserve & 7)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
		}
		cv::Mat image_rotate;
		bool flag_Vertical = false;

		//如果是竖直的，需要旋转90		
		if (imageROI.size().height > imageROI.size().width)
		{
			flag_Vertical = true;
			//逆时针进行旋转90
			transpose(imageROI, image_rotate);
			flip(image_rotate, image_rotate, 0);
			if (iReserve & 3)
			{
				SYSTEMTIME st;
				GetSystemTime(&st);
				CString sFileName;
				sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图_90.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
				cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_rotate);
			}
		}
		else
		{
			image_rotate = imageROI;
		}

		//识别
		std::string result;
		std::string result_1;
		std::vector<cv::Rect> resultRects;
		float confidence = 0;
		recognizeSealCodeNoTypeNoStdImg(image_rotate, result, result_1, resultRects, &confidence, 1);
		/*
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "recognize result: %s", result_1.c_str());
		theApp.putLog(buf_message);*/

		std::string clearStr;
		removeSpaceEnter(result, clearStr);
		if (clearStr.size() == 14 && clearStr.at(0) == '@' && clearStr.at(13) == '@')
		{
			results_candiate.push_back(clearStr.substr(1, 12));
			results_confidences.push_back(confidence);
			memset(buf_message, 0, sizeof buf_message);
			sprintf(buf_message, "recognize result shape matched: %s, score: %f", result.c_str(), confidence);
			theApp.putLog(buf_message);
		}

		//再旋转180;
		transpose(imageROI, image_rotate);
		flip(image_rotate, image_rotate, 2);
		if (!flag_Vertical)
		{
			transpose(image_rotate, image_rotate);
			flip(image_rotate, image_rotate, 2);
		}

		if (iReserve & 3)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图_180.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_rotate);
		}

		//识别
		result = "";
		resultRects.clear();
		result_1 = "";
		confidence = 0;
		recognizeSealCodeNoTypeNoStdImg(image_rotate, result, result_1, resultRects, &confidence, 1);

		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "recognize result: %s", result_1.c_str());
		//theApp.putLog(buf_message);

		clearStr = "";
		removeSpaceEnter(result, clearStr);
		if (clearStr.size() == 14 && clearStr.at(0) == '@' && clearStr.at(13) == '@')
		{
			results_candiate.push_back(clearStr.substr(1, 12));
			results_confidences.push_back(confidence);
			memset(buf_message, 0, sizeof buf_message);
			sprintf(buf_message, "recognize result shape matched: %s, score: %f", result.c_str(), confidence);
			theApp.putLog(buf_message);
		}
	}

	int MaxConfiIndex = 0;
	float maxConf = 0;
	for (int i = 0; i < results_confidences.size(); i++)
	{
		if (maxConf < results_confidences[i])
		{
			maxConf = results_confidences[i];
			MaxConfiIndex = i;
		}
	}

	if (results_candiate.size() > 0)
	{
		std::string tmp = results_candiate[MaxConfiIndex];
		out_verfiyCode = results_candiate[MaxConfiIndex];
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "recognize result : %s, index: %d", tmp.c_str(), MaxConfiIndex);
		theApp.putLog(buf_message);
	}
	else
	{
		return -1;
	}
	return 0;
}


//不识别票据类型版本, 验证码识别！
/*
dwImg:
*/
__declspec(dllexport)  int WINAPI RCN_VerifySealCodeBarNoRecognizeType(DWORD dwImg, char *szInVerifyCode, int iReserve, char* sealCodeOut)
{
	int iRet = -1;
	DWORD ts = GetTickCount();
	//获取原始图片
	IplImage *pImg;
	pImg = (IplImage *)dwImg;
	cv::Mat image = cv::Mat(pImg, true);
	if (iReserve & 3)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image);
	}


	//获取到裁剪的图片
	cv::Mat resultCutImg;
	RCN_GetCutPictureToMem(dwImg, NULL, resultCutImg);
	if (iReserve & 3)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, resultCutImg);
	}

	//根据不同的光照，进行二值化处理
	IplImage pImg_tmp = resultCutImg;
	int averageLight = get_avg_gray(&pImg_tmp);

	char buf_message[1024];
	sprintf(buf_message, "平均亮度: %d", averageLight);
	theApp.putLog(buf_message);

	cv::Mat grey_image;
	cv::cvtColor(resultCutImg, grey_image, CV_BGR2GRAY);
	cv::Mat threshold_image;
	if (averageLight > 180)
	{
		cv::threshold(grey_image, threshold_image, 130, 255, CV_THRESH_BINARY);
	}
	else if (averageLight > 80)
	{
		cv::threshold(grey_image, threshold_image, 40, 255, CV_THRESH_BINARY);
	}
	else
	{
		cv::threshold(grey_image, threshold_image, 20, 255, CV_THRESH_BINARY);
	}

	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, threshold_image);
	}

	DWORD t1 = GetTickCount();
	std::vector<cv::Point> matchPoints;

	char bufPathTemplatePath[1024];
	GetCurDir(bufPathTemplatePath);
	strcat(bufPathTemplatePath, "\\VerifyCodeImg\\templateVerifyCode.png");

	iRet = calcMatchedFeaturePoints(threshold_image, matchPoints, bufPathTemplatePath);

	DWORD t2 = GetTickCount() - t1;
	if (iRet != 0)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "匹配上失败");
		theApp.putLog(buf_message);
		return -1;
	}

	memset(buf_message, 0, sizeof buf_message);
	sprintf(buf_message, "匹配上的点数: %d, 时间： %d ms ", matchPoints.size(), t2);
	theApp.putLog(buf_message);

	//获取裁剪的验证码区域

	//水平
	std::vector<cv::Rect> out_candiate_boxes;
	findCandiateVerifyCodeBox(threshold_image, out_candiate_boxes, iReserve, matchPoints, false);

	//竖直
	std::vector<cv::Rect> out_candiate_boxes_1;
	findCandiateVerifyCodeBox(threshold_image, out_candiate_boxes_1, iReserve, matchPoints, true);

	std::vector<cv::Rect> candiate_boxes;
	//收集所有的矩形
	candiate_boxes = out_candiate_boxes_1;
	for (int i = 0; i < out_candiate_boxes.size(); i++)
	{
		cv::Rect tmpRect = out_candiate_boxes[i];
		candiate_boxes.push_back(tmpRect);
	}

#if 1

	cv::Rect rectSealCodeZone;
	int len = strlen(szInVerifyCode);
	if (len == 12)
	{
		//用于形成标准图
		BYTE *pcImg = new BYTE[204800];
		memset(pcImg, 0, 204800);
		DWORD dw = 204800;
		if (theApp.cvtGenerateSealCodeBarImageToMem(szInVerifyCode, pcImg, dw) != 0)
		{
			delete[]pcImg;
			return -5;
		}

		BMP bt;
		BITMAPFILEHEADER *bf = (BITMAPFILEHEADER *)pcImg;
		memcpy(&bt.bfHeader, bf, sizeof(BITMAPFILEHEADER));
		BITMAPINFOHEADER *bi = (BITMAPINFOHEADER *)(pcImg + sizeof(BITMAPFILEHEADER));
		memcpy(&bt.biHeader, bi, sizeof(BITMAPINFOHEADER));
		bt.bmpData = (unsigned char *)(pcImg + sizeof(BITMAPINFOHEADER) + sizeof(BITMAPFILEHEADER));

		IplImage *ZImg = bt.BMP2Ipl();
		IplImage *BImg = cvCreateImage(cvGetSize(ZImg), ZImg->depth, 1);
		cvCvtColor(ZImg, BImg, CV_RGB2GRAY);

		//针对自动定位的操作，添加验字的操作
		cv::Mat image_mat = cv::Mat(ZImg, true);
		char bufPath[1024];
		GetCurDir(bufPath);
		strcat(bufPath, "\\VerifyCodeImg\\addVerifyCode.png");

		cv::Mat roiImg = cv::imread(bufPath);
		cv::Mat result;
		result = image_mat(cv::Rect(0, 0, 43, 44));
		roiImg.copyTo(result);

		cv::Mat result_2;
		result_2 = image_mat(cv::Rect(238 + 10, 30 + 10, 43, 44));
		roiImg.copyTo(result_2);
		if (iReserve & 7)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_标准图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_mat);
		}

		cv::Mat std_image_code = image_mat;
		char result_buf[32];
		memset(result_buf, 0, 32);
		int minOper_1 = 1000;
		int minOper_2 = 1000;
		std::string verifyCodeResult;
		iRet = recognizeVerifyCodeNoType(resultCutImg, std_image_code, candiate_boxes, verifyCodeResult, iReserve);
		if (iRet == 0)
		{
			memcpy(sealCodeOut, verifyCodeResult.c_str(), 12);
		}
		else
		{
			if (verifyCodeResult.size() >= 12)
			{
				memcpy(sealCodeOut, verifyCodeResult.c_str(), 12);
			}
			else
			{
				memcpy(sealCodeOut, verifyCodeResult.c_str(), verifyCodeResult.size());
			}
		}
		cvReleaseImage(&BImg);
		cvReleaseImage(&ZImg);
		bt.bmpData = NULL;
		if (pcImg != NULL)
		{
			delete[]pcImg;
			pcImg = NULL;
		}
	}
	else
	{
		//根据裁剪的区间，都去识别
		//返回最合理的
		std::string verifyCodeResult;
		iRet = recognizeVerifyCodeNoTypeNoStdImage(resultCutImg, candiate_boxes, verifyCodeResult, iReserve);
		if (iRet == 0)
		{
			memcpy(sealCodeOut, verifyCodeResult.c_str(), 12);
		}
	}

#endif
	DWORD t3 = GetTickCount() - ts;
	memset(buf_message, 0, sizeof buf_message);
	sprintf(buf_message, "识别总时间： %d ms ", t3);
	theApp.putLog(buf_message);
	return iRet;
}

//识别票据类型版本， 验证码识别!
__declspec(dllexport)  int WINAPI RCN_VerifySealCodeBarNew(char *szInVerifyCode, POINT point, int iImgReversal, int iReserve, char* sealCodeOut)
{
	if(point.x  == 0 || point.y == 0)
	{
		return 0;
	}

	int iRet = -1;

	//增加识别库目录是否存在;
	iReserve = theApp.m_idebug;
	CString ocrDirectoryFile;
	ocrDirectoryFile = theApp.m_pathModelFile;
	//ocrDirectoryFile + "\\Featurex\\OCRDict\\orient.ocrdata";
	int ret_fileExist = PathFileExists(ocrDirectoryFile + "\\DFJY\\Template\\Dfjyocr.dat");
	if (!ret_fileExist)
	{
		return -13;
	}
	if(iReserve & 1)
	{
		char buf_message_1[1024];
		memset(buf_message_1, 0 , 1024);
		sprintf(buf_message_1, "验证码位置：%d, %d mm", point.x, point.y);
		theApp.putLog(buf_message_1);
	}
	iReserve = theApp.m_idebug;
	float x_dpi = ((float)theApp.m_iXScaleplate) / 40;
	float y_dpi = ((float)theApp.m_iYScaleplate) / 40;

	//point.x = point.x * theApp.m_pixNumPerMm;
	//point.y = point.y * theApp.m_pixNumPerMm;
	//无需旋转情况
	if (theApp.VerifyCodeRotate == 2)
	{
		point.x = point.x * x_dpi;
		point.y = point.y * y_dpi;
	}
	else
	{
		point.x = point.x * y_dpi;
		point.y = point.y * x_dpi;
	}

	if (!szInVerifyCode)
	{
		iRet = -2;
		return iRet;
	}

	if (theApp.m_image_clipper.empty())
	{
		return iRet;
	}

	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始的裁剪图-验证码.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, theApp.m_image_clipper);
	}
	cv::Rect rectSealCodeZone;
	char buf_message[1024] = { 0 };
	memset(buf_message, 0, 1024);
	//rectSealCodeZone.x = point.x - 170 + theApp.m_OffSet_x;
	//rectSealCodeZone.y = point.y - 55 + theApp.m_OffSet_y;	
	//rectSealCodeZone.width = 335 + theApp.m_OffSet_w;
	//rectSealCodeZone.height = 120 + theApp.m_OffSet_h;

	double scaleVerifyCode = theApp.m_pixNumPerMm / 6.9;
#ifdef SMALLVERIFYCODEZONE
	//rectSealCodeZone.x = point.x - 117*scaleVerifyCode;
	rectSealCodeZone.x = point.x - 102 * scaleVerifyCode;
	rectSealCodeZone.y = point.y - 50 * scaleVerifyCode;
	//rectSealCodeZone.width = 235*scaleVerifyCode;
	rectSealCodeZone.width = 205 * scaleVerifyCode;
	rectSealCodeZone.height = 85 * scaleVerifyCode;
#else
	rectSealCodeZone.x = point.x - 112 * scaleVerifyCode;
	rectSealCodeZone.y = point.y - 65 * scaleVerifyCode;
	rectSealCodeZone.width = 225 * scaleVerifyCode;
	rectSealCodeZone.height = 115 * scaleVerifyCode;
#endif

	int len = strlen(szInVerifyCode);

	//sprintf(buf_message, "x: %d, y:%d, len: %d, scale:%.3f", rectSealCodeZone.x, rectSealCodeZone.y, len, scaleVerifyCode);
	//theApp.putLog(buf_message);
	cv::Mat image = theApp.m_image_clipper;
	if (len == 12)
	{
		//用于形成标准图
		BYTE *pcImg = new BYTE[204800];
		memset(pcImg, 0, 204800);
		DWORD dw = 204800;
		if (theApp.cvtGenerateSealCodeBarImageToMem(szInVerifyCode, pcImg, dw) != 0)
		{
			delete[]pcImg;
			return -5;
		}

		BMP bt;
		BITMAPFILEHEADER *bf = (BITMAPFILEHEADER *)pcImg;
		memcpy(&bt.bfHeader, bf, sizeof(BITMAPFILEHEADER));
		BITMAPINFOHEADER *bi = (BITMAPINFOHEADER *)(pcImg + sizeof(BITMAPFILEHEADER));
		memcpy(&bt.biHeader, bi, sizeof(BITMAPINFOHEADER));
		bt.bmpData = (unsigned char *)(pcImg + sizeof(BITMAPINFOHEADER) + sizeof(BITMAPFILEHEADER));	
		IplImage *ZImg = bt.BMP2Ipl();
		IplImage *BImg = cvCreateImage(cvGetSize(ZImg), ZImg->depth, 1);
		cvCvtColor(ZImg, BImg, CV_RGB2GRAY);
		if (iReserve & 4)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_标准图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cvSaveImage((LPSTR)(LPCTSTR)sFileName, ZImg);
		}
		cv::Mat std_image_code = cv::Mat(BImg, true);
		char result_buf[32];
		memset(result_buf, 0, 32);
		int minOper_1 = 1000;
		int minOper_2 = 1000;
		if (theApp.VerifyCodeRotate == 0)
		{
			iRet = VerifySealCodeBarInner(image, rectSealCodeZone, 1, std_image_code, result_buf, &minOper_1, iReserve);
		}
		else if (theApp.VerifyCodeRotate == 1)
		{
			iRet = VerifySealCodeBarInner(image, rectSealCodeZone, -1, std_image_code, result_buf, &minOper_2, iReserve);
		}
		else if (theApp.VerifyCodeRotate == 2)
		{
			iRet = VerifySealCodeBarInner(image, rectSealCodeZone, 0, std_image_code, result_buf, &minOper_1, iReserve);
		}

		memcpy(sealCodeOut, result_buf, 14);
		cvReleaseImage(&BImg);
		cvReleaseImage(&ZImg);
		bt.bmpData = NULL;
		if (pcImg != NULL)
		{
			delete[]pcImg;
			pcImg = NULL;
		}
	}
	else if (len == 0)
	{
		cv::Mat ROIVerifyCode;
		if (theApp.VerifyCodeRotate == 0)
		{
			iRet = cutVerifyCode(image, ROIVerifyCode, 1, rectSealCodeZone, iReserve);
		}
		else if (theApp.VerifyCodeRotate == 1)
		{
			iRet = cutVerifyCode(image, ROIVerifyCode, -1, rectSealCodeZone, iReserve);
		}
		else if (theApp.VerifyCodeRotate == 2)
		{
			iRet = cutVerifyCode(image, ROIVerifyCode, 0, rectSealCodeZone, iReserve);
		}

		if (iRet != 0)
		{
			return iRet;
		}

		if(ROIVerifyCode.channels()==3)
		{
			cv::cvtColor(ROIVerifyCode, ROIVerifyCode, CV_BGR2GRAY);
		}
		if (theApp.m_bVerifyCodeBinPW == TRUE)
		{
			int blockSize = 25;
			int constValue = theApp.m_VerifyCodeBinValue;
			cv::adaptiveThreshold(ROIVerifyCode, ROIVerifyCode, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
			if (iReserve & 4)
			{
				SYSTEMTIME st;
				GetSystemTime(&st);
				CString sFileName;
				sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_二值处理图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
				cv::imwrite((LPSTR)(LPCTSTR)sFileName, ROIVerifyCode);
			}	
		}


		std::string result;
		std::vector<cv::Rect> char_rects_std;
		iRet = recognizeSealCode(ROIVerifyCode, result, char_rects_std, 1);
		if (iRet != 0)
		{
			return iRet;
		}

		memcpy(sealCodeOut, result.c_str(), result.size());	
	}
	return iRet;
}


/************************************************************************/
/* RCN_VerifySealCodeBar 验证码检测函数
dwImg:                句柄地址
szInVerifyCode:       验证码图片路径
point:                验证码中心点
iImgReversal:         正上面还是倒立
iReserve:             控制中间图片输出
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_VerifySealCodeBar(DWORD dwImg, char *szInVerifyCode, POINT point, int iImgReversal, int iReserve)
{
	int iRet = -1;
	char buf_result[24];
	RCN_VerifySealCodeBarNoRecognizeType(dwImg, szInVerifyCode, iReserve, buf_result);
	return iRet;
}


/************************************************************************/
/* RCN_AdjustCameraCutArea 设置相机图片裁剪区域函数
iLeft:      裁剪左边
iTop:       裁剪顶部
iRight:     裁剪右边
iBottom:    裁剪底部
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_AdjustCameraCutArea(int iLeft, int iTop, int iRight, int iBottom)
{
	if (!theApp.m_pcap->isOpened())
	{
		return -1;
	}
	theApp.m_iLeft = iLeft;
	theApp.m_iTop = iTop;
	theApp.m_iRight = iRight;
	theApp.m_iBottom = iBottom;
	return 0;
}


/************************************************************************/
/* RCN_AdjustCameraFocusExposure 设置相机属性函数
dFocus:     焦距
dExposure:  曝光值
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_AdjustCameraFocusExposure(double dFocus, double dExposure)
{
	if (!theApp.m_pcap->isOpened())
	{
		return -1;
	}
	return 0;
}


/************************************************************************/
/* RCN_OpenPropertyDlg 打开相机属性函数
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_OpenPropertyDlg()
{
	int iError = -1;
	if (theApp.m_pcap->isOpened())
	{
		CString sName;
		sName = "相机设置";
		char buf[256];
		memset(buf, 0, 256);
		sprintf(buf, "%s 属性", (LPSTR)(LPCTSTR)sName);
		HWND h = FindWindow(NULL, buf);
		if (h)
		{
			SetForegroundWindow(h);
			iError = 0;
			return iError;
		}
		theApp.m_pcap->OpenCameraProperty(sName);
		iError = 0;
	}
	return iError;
}


/************************************************************************/
/* RCN_AdjustCameraParameter 调整相机参数函数
dwParent:      父窗口句柄
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_AdjustCameraParameter(DWORD dwParent)
{
	AFX_MANAGE_STATE(AfxGetStaticModuleState());
	int iError = -1;
	if (theApp.m_pcap->isOpened())
	{
		iError = -2;
		theApp.m_iBackupLeft = theApp.m_iLeft;
		theApp.m_iBackupTop = theApp.m_iTop;
		theApp.m_iBackupRight = theApp.m_iRight;
		theApp.m_iBackupBottom = theApp.m_iBottom;
		theApp.m_dBackupFocus = theApp.m_dFocus;
		theApp.m_dBackupExposure = theApp.m_dExposure;
		CCameraDlg *cDlg = NULL;
		if (dwParent == 0)
		{
			cDlg = new CCameraDlg(NULL);
		}
		else
		{
			CWnd* pParent = (CWnd*)dwParent;
			cDlg = new CCameraDlg(pParent);
		}
		cDlg->SetCutArea(theApp.m_iLeft, theApp.m_iTop, theApp.m_iRight, theApp.m_iBottom);
		if (cDlg->DoModal() == IDOK)
		{
			RCN_AdjustCameraFocusExposure(theApp.m_dFocus, theApp.m_dExposure);
			iError = 0;
		}
		else
		{
			theApp.m_iLeft = theApp.m_iBackupLeft;
			theApp.m_iTop = theApp.m_iBackupTop;
			theApp.m_iRight = theApp.m_iBackupRight;
			theApp.m_iBottom = theApp.m_iBackupBottom;
			theApp.m_dFocus = theApp.m_dBackupFocus;
			theApp.m_dExposure = theApp.m_dBackupExposure;
			RCN_AdjustCameraFocusExposure(theApp.m_dFocus, theApp.m_dExposure);
		}
		delete cDlg;
	}
	return iError;
}


/************************************************************************/
/* RCN_GetCameraParameter 获取相机参数函数
iLeft:      裁剪左边
iTop:       裁剪顶部
iRight:     裁剪右边
iBottom:    裁剪底部
dFocus:     相机焦距
dExposure:  相机曝光值
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_GetCameraParameter(int &iLeft, int &iTop, int &iRight, int &iBottom, double &dFocus, double &dExposure)
{
	iLeft = theApp.m_iLeft;
	iTop = theApp.m_iTop;
	iRight = theApp.m_iRight;
	iBottom = theApp.m_iBottom;
	dFocus = theApp.m_dFocus;
	dExposure = theApp.m_dExposure;
	return 0;
}


//iType=0标准的验证码， iType=1拍摄的验证码
__declspec(dllexport)  int WINAPI RCN_TestRecognizeSealCode(DWORD dwImg, char *pResult, int iType)
{
	IplImage *pImg;
	pImg = (IplImage *)dwImg;
	cv::Mat ROIVerifyCode(pImg, 0);
	//cv::resize(ROIVerifyCode, ROIVerifyCode, cv::Size(0, 0), 1.5, 1.5);
	//cv::medianBlur(ROIVerifyCode, ROIVerifyCode, 3);

	int ret = -1;
	cv::Mat grey_pic;
	if (ROIVerifyCode.channels() == 3)
	{
		cv::cvtColor(ROIVerifyCode, grey_pic, CV_BGR2GRAY);
	}
	else
	{
		grey_pic = ROIVerifyCode;
	}

	std::string result;
	std::vector<cv::Rect> char_rects_pic;
	int iRet = recognizeSealCode(grey_pic, result, char_rects_pic, iType);
	if (iRet != 0)
	{
		return iRet;
	}
	strcpy(pResult, result.c_str());
	//DrawRectAndSave(grey_pic, char_rects_pic);
	return 0;
}

//这个是针对有票据识别的
int WINAPI recognizeSealCode(cv::Mat img, std::string &result, std::vector<cv::Rect>& rects, int ocr_type = 0)
{
	if (img.empty())
	{
		result = "";
		return -1;
	}

	cv::Mat grey, img_Threshold;
	if (img.channels() == 3)
	{
		cv::cvtColor(img, grey, CV_BGR2GRAY);
	}
	else
	{
		grey = img;
	}


	BYTE *pData, bByte;
	int istep;
	/*
	cv::Scalar scalar = cv::mean(grey);
	double dScalar =  scalar[0];
	IplImage* pscImg = &IplImage(grey);
	BYTE* pData = (BYTE *)pscImg->imageData;
	BYTE  bByte;
	int istep = pscImg->widthStep/sizeof(BYTE);
	for(int i=0; i<pscImg->height; i++)
	{
	for(int j=0; j<pscImg->width; j++)
	{
	bByte = pData[i*istep+j];
	if(bByte <= (BYTE)dScalar + 20)
	{
	int iic = bByte - 20;
	if(iic <= 0)
	pData[i*istep+j] = 0;
	else
	pData[i*istep+j] = (BYTE)iic;
	}
	}
	}*/
	//CString ssxx;
	//ssxx = "E:\\work\\OCRSealx\\bin\\22\\igrey.jpg"; 
	//cvSaveImage((LPSTR)(LPCTSTR)ssxx, pscImg);

	//int blockSize = 25;
	//int constValue = 10;
	//cv::adaptiveThreshold(grey, img_Threshold, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blockSize, constValue);


	//myThreshold(grey, img_Threshold);
	/*
	IplImage* pscImg = &IplImage(img_Threshold);
	IplImage* smoothImg = cvCreateImage(img_Threshold.size(), IPL_DEPTH_8U, 1);  //cvGetSize(
	cvErode(pscImg, smoothImg, 0, 1);
	cvDilate(smoothImg, smoothImg, 0, 1);
	grey = smoothImg;
	imshow("1key", grey);
	waitKey();
	*/

	tesseract::TessBaseAPI api;
	char szDir[1024];
	memset(szDir, '\0', 1024);
	memcpy(szDir, (LPSTR)(LPCTSTR)theApp.m_pathModelFile, theApp.m_pathModelFile.GetLength());
	//strcat(szDir, "\\Featurex");
	strcat(szDir, "/DFJY/Template");

	if (ocr_type == 0)
	{
		api.Init(szDir, "orient", tesseract::OEM_TESSERACT_ONLY);
	}
	else
	{
		api.Init(szDir, "orient_1", tesseract::OEM_TESSERACT_ONLY);
	}
	//解决字符序列倾斜时，无法识别的问题！
	api.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);//设置自动进行版面分析//PSM_AUTO_OSD PSM_AUTO PSM_SINGLE_BLOCK PSM_SINGLE_BLOCK
	api.SetVariable("tessedit_char_whitelist", "0123456789ABCDEF[]");
	api.SetImage((uchar*)grey.data, grey.size().width, grey.size().height, grey.channels(), grey.step1());


	char *pr = api.GetUTF8Text();
	IplImage* pImg = &IplImage(grey);
#if 1
	tesseract::ResultIterator* ri = api.GetIterator();
	tesseract::PageIteratorLevel level = tesseract::RIL_SYMBOL;
	int h = img.size().height;
	int w = img.size().width;
	std::stringstream ss;
	ss.str("");
	if (ri != 0)
	{
		int icc = 0;
		//theApp.putLog("获取矩形框开始");
		do {
			const char* word = ri->GetUTF8Text(level);
			if (word == NULL)
			{
				continue;
			}
			float conf = ri->Confidence(level);
			int x1, y1, x2, y2;
			ri->BoundingBox(level, &x1, &y1, &x2, &y2);
			cv::Rect rect(cv::Point(x1, y1), cv::Point(x2, y2));
			if (strcmp(word, " ") != 0 && (abs(y2 - y1) >= h / 8) && (abs(y2 - y1) <= h / 2) && (abs(x2 - x1) <= w / 7) && (abs(x2 - x1) > 5))
			{	
				ss << word;
				rects.push_back(rect);
				if (rects.size() == 14)
				{
					break;
				}
			}
		} while (ri->Next(level));
		//theApp.putLog("获取矩形框结束");
		delete ri;
	}
	else
	{
		result = "";
		return -3;
	}
	result = ss.str();
#endif
	return 0;
}


//针对识别，重控号;
int recognizeImportantCodes(cv::Mat grey, char* fontName, std::string &result, std::vector<cv::Rect>& rects)
{
	if (grey.empty())
	{
		result = "";
		return -1;
	}



	tesseract::TessBaseAPI api;
	char szDir[1024];
	memset(szDir, '\0', 1024);
	memcpy(szDir, (LPSTR)(LPCTSTR)theApp.m_pathModelFile, theApp.m_pathModelFile.GetLength());	
	strcat(szDir, "/DFJY/Template");
	api.Init(szDir, fontName, tesseract::OEM_TESSERACT_ONLY);

	//解决字符序列倾斜时，无法识别的问题！
	api.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);//设置自动进行版面分析//PSM_AUTO_OSD PSM_AUTO PSM_SINGLE_BLOCK PSM_SINGLE_BLOCK
	api.SetVariable("tessedit_char_whitelist", "0123456789");
	api.SetImage((uchar*)grey.data, grey.size().width, grey.size().height, grey.channels(), grey.step1());
	char *pr = api.GetUTF8Text();
	tesseract::ResultIterator* ri = api.GetIterator();
	tesseract::PageIteratorLevel level = tesseract::RIL_SYMBOL;
	int h = grey.size().height;
	int w = grey.size().width;
	std::stringstream ss;
	ss.str("");
	if (ri != 0)
	{		
		do {
			const char* word = ri->GetUTF8Text(level);
			if (word == NULL)
			{
				continue;
			}
			float conf = ri->Confidence(level);
			int x1, y1, x2, y2;
			ri->BoundingBox(level, &x1, &y1, &x2, &y2);
			cv::Rect rect(cv::Point(x1, y1), cv::Point(x2, y2));
			if (strcmp(word, " ") != 0 && (abs(y2 - y1) >= h / 5) && (abs(x1 -x2) > 3))
			{	
				ss << word;
				rects.push_back(rect);
			}
		} while (ri->Next(level));	
		delete ri;
	}
	else
	{
		result = "";
		return -3;
	}
	result = ss.str();
	return 0;
}


//针对没有票据识别的
int WINAPI recognizeSealCodeNoType(cv::Mat img, std::string &result, std::vector<cv::Rect>& rects, int ocr_type = 0)
{
	if (img.empty())
	{
		result = "";
		return -1;
	}
	cv::Mat grey;
	if (img.channels() == 3)
	{
		cv::cvtColor(img, grey, CV_BGR2GRAY);
	}
	else
	{
		grey = img;
	}

	tesseract::TessBaseAPI api;
	char szDir[1024];
	memset(szDir, '\0', 1024);
	/*TCHAR szPath[MAX_PATH];
	if( !GetModuleFileName( NULL, szPath, MAX_PATH ) )
	{
	return NULL;
	}
	GetLongPathName(szPath, szDir, MAX_PATH);
	PathRemoveFileSpec(szDir);
	*/
	memcpy(szDir, (LPSTR)(LPCTSTR)theApp.m_pathModelFile, theApp.m_pathModelFile.GetLength());
	strcat(szDir, "\\Featurex");
	if (ocr_type == 0)
	{
		api.Init(szDir, "orient_std", tesseract::OEM_TESSERACT_ONLY);
	}
	else
	{
		api.Init(szDir, "orient_real", tesseract::OEM_TESSERACT_ONLY);
	}

	//解决字符序列倾斜时，无法识别的问题！
	api.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);//设置自动进行版面分析//PSM_AUTO_OSD PSM_AUTO PSM_SINGLE_BLOCK PSM_SINGLE_BLOCK
	api.SetVariable("tessedit_char_whitelist", "0123456789ABCDEF@");
	api.SetImage((uchar*)grey.data, grey.size().width, grey.size().height, grey.channels(), grey.step1());
	char *pr = api.GetUTF8Text();

#if 1

	tesseract::ResultIterator* ri = api.GetIterator();
	tesseract::PageIteratorLevel level = tesseract::RIL_SYMBOL;
	int h = img.size().height;
	int w = img.size().width;
	std::stringstream ss;
	ss.str("");
	if (ri != 0)
	{
		//theApp.putLog("获取矩形框开始");
		do {
			const char* word = ri->GetUTF8Text(level);
			if (word == NULL)
			{
				continue;
			}
			float conf = ri->Confidence(level);
			int x1, y1, x2, y2;
			ri->BoundingBox(level, &x1, &y1, &x2, &y2);
			cv::Rect rect(cv::Point(x1, y1), cv::Point(x2, y2));
			if (strcmp(word, " ") != 0 && strcmp(word, "@") != 0 && (abs(y2 - y1) >= h / 6) && (abs(y2 - y1) <= h / 2) && (abs(x2 - x1) <= w / 7))
			{
				ss << word;
				rects.push_back(rect);
				if (rects.size() == 14)
				{
					break;
				}
			}
		} while (ri->Next(level));
		//theApp.putLog("获取矩形框结束");
		delete ri;
	}
	result = ss.str();
#endif
	return 0;
}


//针对没有票据识别的 而且输入字符不是12位的
int WINAPI recognizeSealCodeNoTypeNoStdImg(cv::Mat img, std::string &result, std::string& result_1, std::vector<cv::Rect>& rects, float* pConfidence, int ocr_type = 0)
{
	if (img.empty())
	{
		result = "";
		return -1;
	}
	cv::Mat grey;
	if (img.channels() == 3)
	{
		cv::cvtColor(img, grey, CV_BGR2GRAY);
	}
	else
	{
		grey = img;
	}
	tesseract::TessBaseAPI api;
	char szDir[1024];
	memset(szDir, '\0', 1024);
	/*TCHAR szPath[MAX_PATH];
	if( !GetModuleFileName( NULL, szPath, MAX_PATH ) )
	{
	return NULL;
	}
	GetLongPathName(szPath, szDir, MAX_PATH);
	PathRemoveFileSpec(szDir);*/
	memcpy(szDir, (LPSTR)(LPCTSTR)theApp.m_pathModelFile, theApp.m_pathModelFile.GetLength());
	strcat(szDir, "\\Featurex");
	if (ocr_type == 0)
	{
		api.Init(szDir, "orient_std", tesseract::OEM_TESSERACT_ONLY);
	}
	else
	{
		api.Init(szDir, "orient_real", tesseract::OEM_TESSERACT_ONLY);
	}

	//解决字符序列倾斜时，无法识别的问题！
	api.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);//设置自动进行版面分析//PSM_AUTO_OSD PSM_AUTO PSM_SINGLE_BLOCK PSM_SINGLE_BLOCK
	api.SetVariable("tessedit_char_whitelist", "0123456789ABCDEF@");
	api.SetImage((uchar*)grey.data, grey.size().width, grey.size().height, grey.channels(), grey.step1());
	char *pr = api.GetUTF8Text();
	result_1 = pr;

#if 1
	tesseract::ResultIterator* ri = api.GetIterator();
	tesseract::PageIteratorLevel level = tesseract::RIL_SYMBOL;
	int h = img.size().height;
	int w = img.size().width;
	std::stringstream ss;
	ss.str("");
	float confi_sum = 0;
	if (ri != 0)
	{
		//theApp.putLog("获取矩形框开始");
		do {
			const char* word = ri->GetUTF8Text(level);
			if (word == NULL)
			{
				continue;
			}
			float conf = ri->Confidence(level);
			int x1, y1, x2, y2;
			ri->BoundingBox(level, &x1, &y1, &x2, &y2);
			cv::Rect rect(cv::Point(x1, y1), cv::Point(x2, y2));
			if ((abs(y2 - y1) >= h / 6) && (abs(y2 - y1) <= h / 2) && (abs(x2 - x1) <= w / 7))
			{
				ss << word;
				rects.push_back(rect);
				confi_sum += conf;
			}
		} while (ri->Next(level));
		//theApp.putLog("获取矩形框结束");
		delete ri;
	}
	*pConfidence = confi_sum;
	result = ss.str();
#endif
	return 0;
}


int removeSpaceEnter(std::string i_str, std::string& o_str)
{
	std::stringstream ss;
	ss.str("");
	for (int i = 0; i < i_str.size(); i++)
	{
		if (i_str[i] == ' ' ||  i_str[i] == '\n')
		{
			continue;
		}
		ss << i_str[i];
	}
	o_str = ss.str();
	return 0;
}


bool findMaxMinX(std::vector<cv::Rect> rects, int *pMin, int* pMax)
{
	int min_x = 1000000;
	int max_x = 0;
	for (int i = 0; i < rects.size(); i++)
	{
		if ((rects[i].x + rects[i].width) > max_x)
		{
			max_x = rects[i].x + rects[i].width;
		}

		if (rects[i].x < min_x)
		{
			min_x = rects[i].x;
		}
	}
	*pMin = min_x;
	*pMax = max_x;
	return true;
}




//针对 票据识别版的间距匹配
bool MatchDistence(std::vector<cv::Rect>& rects_std, std::vector<cv::Rect>& rects_real)
{
	if (rects_std.size() != rects_real.size() || rects_std.size() != 14)
	{
		return false;
	}
	std::vector<int> distence_std;
	int min_distence_std = 100000;
	std::vector<int> distence_real;
	int min_distence_real = 100000;
	int x_min_std = 10000;
	int x_max_std = 0;
	findMaxMinX(rects_std, &x_min_std, &x_max_std);
	int deltaStd = x_max_std - x_min_std;
	int x_min_real = 10000;
	int x_max_real = 0;
	findMaxMinX(rects_real, &x_min_real, &x_max_real);
	int deltaReal = x_max_real - x_min_real;
	for (int i = 0; i < (rects_std.size() - 1); i++)
	{
		int j = i + 1;
		if (i == 6)
		{
			continue;
		}
		int distence = rects_std[j].x - (rects_std[i].x + rects_std[i].width);
		if (distence < min_distence_std)
		{
			min_distence_std = distence;
		}
		distence_std.push_back(distence);
		int distence_real_elem = rects_real[j].x - (rects_real[i].x + rects_real[i].width);
		if (distence_real_elem < min_distence_real)
		{
			min_distence_real = distence_real_elem;
		}
		distence_real.push_back(distence_real_elem);
	}
	std::stringstream ss1;
	std::stringstream ss2;
	std::stringstream ss3;
	ss1.str("");
	ss2.str("");
	ss3.str("");
	double x1_real = min_distence_real;
	double x2_std = min_distence_std;
	double scale_num = (double)deltaReal / deltaStd;
	bool match_distence = true;
	ss1 << "real:      <<" << deltaReal << ">>";
	ss2 << "std-after: <<" << deltaStd << ">>";
	ss3 << "std-prev:  <<" << deltaStd << ">>";
	for (int i = 0; i < distence_std.size(); i++)
	{
		ss1 << distence_real[i] << " ";
		ss3 << distence_std[i] << " ";
		distence_std[i] = distence_std[i] * scale_num;
		ss2 << distence_std[i] << " ";
		if (abs(distence_std[i] - distence_real[i]) > theApp.m_minDistence)
		{
			match_distence = false;
			break;
		}
	}
	if(theApp.m_idebug&7)
	{
		theApp.putLog((char*)ss1.str().c_str());
		theApp.putLog((char*)ss3.str().c_str());
		theApp.putLog((char*)ss2.str().c_str());
	}
	return match_distence;
}


//针对非票据识别的版本
bool MatchDistenceNoType(std::vector<cv::Rect>& rects_std, std::vector<cv::Rect>& rects_real)
{
	if (rects_std.size() != rects_real.size() || rects_std.size() != 12)
	{
		return false;
	}
	std::vector<int> distence_std;
	int min_distence_std = 100000;
	std::vector<int> distence_real;
	int min_distence_real = 100000;
	int x_min_std = 10000;
	int x_max_std = 0;
	findMaxMinX(rects_std, &x_min_std, &x_max_std);
	int deltaStd = x_max_std - x_min_std;
	int x_min_real = 10000;
	int x_max_real = 0;
	findMaxMinX(rects_real, &x_min_real, &x_max_real);
	int deltaReal = x_max_real - x_min_real;
	for (int i = 0; i < (rects_std.size() - 1); i++)
	{
		int j = i + 1;
		if (i == 5)
		{
			continue;
		}

		int distence = rects_std[j].x - (rects_std[i].x + rects_std[i].width);
		if (distence < min_distence_std)
		{
			min_distence_std = distence;
		}
		distence_std.push_back(distence);
		int distence_real_elem = rects_real[j].x - (rects_real[i].x + rects_real[i].width);
		if (distence_real_elem < min_distence_real)
		{
			min_distence_real = distence_real_elem;
		}
		distence_real.push_back(distence_real_elem);
	}

	std::stringstream ss1;
	std::stringstream ss2;
	std::stringstream ss3;
	ss1.str("");
	ss2.str("");
	ss3.str("");
	double x1_real = min_distence_real;
	double x2_std = min_distence_std;
	double scale_num = deltaReal / deltaStd;
	bool match_distence = true;
	ss1 << "real:      <<" << deltaReal << ">>";
	ss2 << "std-after: <<" << deltaStd << ">>";
	ss3 << "std-prev:  <<" << deltaStd << ">>";
	for (int i = 0; i < distence_std.size(); i++)
	{
		ss1 << distence_real[i] << " ";
		ss3 << distence_std[i] << " ";
		distence_std[i] = distence_std[i] * scale_num;
		ss2 << distence_std[i] << " ";
		if (abs(distence_std[i] - distence_real[i]) > theApp.m_minDistence)
		{
			match_distence = false;
			break;
		}
	}
	theApp.putLog((char*)ss1.str().c_str());
	theApp.putLog((char*)ss3.str().c_str());
	theApp.putLog((char*)ss2.str().c_str());
	return match_distence;
}

//针对票据识别的版本
int WINAPI compareSealCode(cv::Mat i_stand_image, cv::Mat i_real_image, std::string& result_sealCode, int *pMinOper)
{
	int ret = -1;
	cv::Mat grey;
	if (i_stand_image.channels() == 3)
	{
		cv::cvtColor(i_stand_image, grey, CV_BGR2GRAY);
	}
	else
	{
		grey = i_stand_image;
	}

	cv::Mat grey_real;
	if (i_real_image.channels() == 3)
	{
		cv::cvtColor(i_real_image, grey_real, CV_BGR2GRAY);
	}
	else
	{
		grey_real = i_real_image;
	}

	std::string result;
	std::vector<cv::Rect> char_rects_std;

	//兼容windows7 下字符C被切断；
	//////
	//cv::resize(grey, grey, cv::Size(0, 0), 0.6, 0.6);
	/////
	ret = recognizeSealCode(grey, result, char_rects_std);
	if (result.size() < 14 || char_rects_std.size() < 14)
	{
		return -1;
	}

	//去掉回车，空格
	std::string result_clear;
	removeSpaceEnter(result, result_clear);
	std::string out_log = "标准验证码识别的结果：";
	out_log += result;

	if(theApp.m_idebug&7)
	{	
		theApp.putLog((char*)out_log.c_str());
		theApp.putLog((char*)result_clear.c_str());
	}
	//*****************DEBUG**************************
	if(theApp.m_idebug & 4)
		DrawRectAndSave(grey, char_rects_std);
	//*****************END***************************

	std::string result_real;
	std::string result_real_clear;
	std::vector<cv::Rect> char_rects_real;
	ret = recognizeSealCode(grey_real, result_real, char_rects_real, 1);
	if (result_real.size() < 14 || char_rects_real.size() < 14)
	{
		return -1;
	}

	out_log = "裁剪的验证码识别结果为：";
	out_log += result_real;
	removeSpaceEnter(result_real, result_real_clear);
	//获取【】所包含的字符

	size_t found = result_real_clear.find(result_clear);
	if (found != std::string::npos)
	{
		result_sealCode = result_clear;
		result_real_clear = result_clear;
	}
	else
		result_sealCode = result_real_clear;
	if(theApp.m_idebug&1)
	{	
		theApp.putLog((char*)out_log.c_str());
		theApp.putLog((char*)result_real_clear.c_str());
	}
	if (result_clear != result_real_clear)
	{
		return -1;
	}

	//theApp.putLog("验证码字符串与模板一样");
	if(theApp.m_idebug & 4)
		DrawRectAndSave(grey_real, char_rects_real);

	//再检验，识别图字符的间隔的比例，与模板的字符间隔比!
	bool match_bool = MatchDistence(char_rects_std, char_rects_real);
	if (match_bool == false)
	{
		return -1;
	}

	//theApp.putLog("验证码的字符间隔匹配成功");
#if 0

	tesseract::TessBaseAPI *api = new tesseract::TessBaseAPI();
	api->Init(NULL, "eng", tesseract::OEM_TESSERACT_ONLY);
	api->SetPageSegMode(tesseract::PSM_AUTO);//设置自动进行版面分析//PSM_AUTO_OSD PSM_AUTO PSM_SINGLE_BLOCK
	api->SetVariable("tessedit_char_whitelist", "0123456789ABCDEFGHIJKLRVN");
	api->SetImage((uchar*)grey.data, grey.size().width, grey.size().width, grey.channels(), grey.step1());
	api->Recognize(0);
	char * result = api->GetUTF8Text();
	std::cout << result << std::endl;
	tesseract::ResultIterator* ri = api->GetIterator();
	tesseract::PageIteratorLevel level = tesseract::RIL_SYMBOL;
	printf("-----------------------begin-------------------------------\n");
	if (ri != 0) {
		do {
			const char* word = ri->GetUTF8Text(level);
			if (strcmp(word, " ") == 0)
			{
				continue;
			}
			float conf = ri->Confidence(level);
			int x1, y1, x2, y2;
			ri->BoundingBox(level, &x1, &y1, &x2, &y2);
			printf("word: '%s';  \tconf: %.2f; BoundingBox: %d,%d,%d,%d;\n",
				word, conf, x1, y1, x2, y2);
			//cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0));
		} while (ri->Next(level));
	}
#endif
	return 0;
}


//针对没有票据识别的版本
int WINAPI compareSealCodeNoType(cv::Mat i_stand_image, cv::Mat i_real_image, std::string& result_sealCode, int *pMinOper)
{
	int ret = -1;
	cv::Mat grey;
	if (i_stand_image.channels() == 3)
	{
		cv::cvtColor(i_stand_image, grey, CV_BGR2GRAY);
	}
	else
	{
		grey = i_stand_image;
	}

	cv::Mat grey_real;
	if (i_real_image.channels() == 3)
	{
		cv::cvtColor(i_real_image, grey_real, CV_BGR2GRAY);
	}
	else
	{
		grey_real = i_real_image;
	}

	std::string result;
	std::vector<cv::Rect> char_rects_std;
	ret = recognizeSealCodeNoType(grey, result, char_rects_std, 0);
	if (result.size() < 12 || char_rects_std.size() < 12)
	{
		return -1;
	}

	//去掉回车，空格
	std::string result_clear;
	removeSpaceEnter(result, result_clear);

	std::string out_log = "标准验证码识别的结果：";
	out_log += result;

	theApp.putLog((char*)out_log.c_str());
	theApp.putLog((char*)result_clear.c_str());

	//*****************DEBUG**************************
	//DrawRectAndSave(grey, char_rects_std);
	//*****************END***************************

	std::string result_real;
	std::string result_real_clear;
	std::vector<cv::Rect> char_rects_real;
	ret = recognizeSealCodeNoType(grey_real, result_real, char_rects_real, 1);
	if (result_real.size() < 12 || char_rects_real.size() < 12)
	{
		return -1;
	}

	//DrawRectAndSave(grey_real, char_rects_real);

	out_log = "裁剪的验证码识别结果为：";
	out_log += result_real;
	removeSpaceEnter(result_real, result_real_clear);
	//获取【】所包含的字符

	size_t found = result_real_clear.find(result_clear);
	if (found != std::string::npos)
	{
		result_sealCode = result_clear;
		result_real_clear = result_clear;
	}
	else
		result_sealCode = result_real_clear;

	//计算字符串的相似度
	*pMinOper = ldistance(result_clear, result_real_clear);
	theApp.putLog((char*)out_log.c_str());
	theApp.putLog((char*)result_real_clear.c_str());
	if (result_clear != result_real_clear)
	{
		return -1;
	}
	theApp.putLog("验证码字符串与模板一样");

	//再检验，识别图字符的间隔的比例，与模板的字符间隔比!
	bool match_bool = MatchDistenceNoType(char_rects_std, char_rects_real);
	if (match_bool == false)
	{
		return -1;
	}
	theApp.putLog("验证码的字符间隔匹配成功");
	return 0;
}


void WINAPI DrawRectAndSave(cv::Mat img, std::vector<cv::Rect> rects)
{
	//theApp.putLog("开始画矩形框");
	cv::Mat show_rects;
	cv::cvtColor(img, show_rects, CV_GRAY2BGR);
	IplImage* pImg = &IplImage(img);
	CString ss;
	for (int i = 0; i < rects.size(); i++)
	{
		cv::Rect rc = rects[i];
		cv::rectangle(show_rects, rc, cv::Scalar(0, 255, 0));
		/*
		IplImage* imgNo = cvCreateImage(cvSize(rc.width , rc.height), IPL_DEPTH_8U, 1);
		cvSetImageROI(pImg, rc);
		cvCopy(pImg, imgNo);
		cvResetImageROI(pImg);

		ss.Format("E:\\work\\OCRSealx\\bin\\22\\it%02d.jpg", i+1);
		cvSaveImage((LPSTR)(LPCTSTR)ss, imgNo);
		cvResetImageROI(imgNo);
		cvReleaseImage(&imgNo);
		*/
	}
	SYSTEMTIME st;
	GetSystemTime(&st);
	CString sFileName;
	sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_验证码-字符框.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
	cv::imwrite((LPSTR)(LPCTSTR)sFileName, show_rects);
	//theApp.putLog("画矩形框结束");
}




//依据坐标进行裁剪
//dwImg： 输入的原图
//iReserve: 调试参数
int WINAPI RCN_CutImageByCoordinate(DWORD dwImg, char* out_bufImage, int *pLength, int iReserve)
{
	if (dwImg == NULL || out_bufImage == NULL)
	{
		return -1;
	}
	cv::Point pt1, pt2, pt3, pt4;
	pt1 = cv::Point(957, 60);
	pt2 = cv::Point(2037, 72);
	pt3 = cv::Point(2021, 1593);
	pt4 = cv::Point(947, 1584);

	std::vector<cv::Point> roiPoints;
	roiPoints.push_back(pt1);
	roiPoints.push_back(pt2);
	roiPoints.push_back(pt3);
	roiPoints.push_back(pt4);

	std::string path_image = "D:\\workSpace\\image\\test_cutImage\\";
	cv::Mat image = cv::imread(path_image + "1.jpg");

	cv::RotatedRect tmpRoRect = cv::minAreaRect(roiPoints);
	cv::Rect roiRect = cv::boundingRect(roiPoints);
	IplImage* pImg = &IplImage(image);

	CvPoint2D32f center;
	CvPoint i_pt[4];
	double angle = 0.0;
	IplImage* roiImage = capImageAndCalcAngleCore(pImg, tmpRoRect, roiRect, i_pt, center, angle);

	cv::Mat image_result;
	image_result = cv::Mat(roiImage, true);
	cv::imwrite(path_image + "result.jpg", image_result);
	vector<uchar> buff;
	cv::imencode(".bmp", image_result, buff);
	memcpy(out_bufImage, &buff[0], buff.size());
	*pLength = buff.size();

	return 0;
}


/*
作用： 从文件读取图片
参数介绍：
pathImage:  图片的路径，
bufImage:   图片的存储地址
pLength:  图片的空间大小.
*/
__declspec(dllexport)  int WINAPI RCN_CaptureFromFile(char* pathImage, char * bufImage, int * pLength)
{
	if (pathImage == NULL || bufImage == NULL)
	{
		return -1;
	}
	cv::Mat image;
	image = cv::imread(pathImage);
	vector<uchar> buff;
	cv::imencode(".jpg", image, buff);
	memcpy(bufImage, &buff[0], buff.size());
	*pLength = buff.size();
	return 0;
}


//形成二维码的接口;
//in_inforQR ： 二维码的内容.
//in_nameQR ： 二维码文件的名字.
//返回值 0 成功， -1 失败。

__declspec(dllexport)  int WINAPI RCN_GenQRCode(char* in_inforQR, char* in_nameQR)
{
	int iRet = -1;
	theApp.putLog("call QR_GenerateQRCode start");
	char buf_message[1024];
	memset(buf_message, 0, 1024);
	sprintf(buf_message, "QRMess: %s, fileName: %s", in_inforQR, in_nameQR);
	theApp.putLog(buf_message);
	iRet = theApp.QR_GenerateQRCode(in_inforQR, in_nameQR);
	theApp.putLog("call QR_GenerateQRCode end");
	return iRet;
}


//二维码的识别;
//pDirection: 1, 朝右； -1, 朝左;
__declspec(dllexport)  int WINAPI RCN_RecoQRCode(DWORD dwImg, char* out_QRInfor)
{
	theApp.putLog("start QRCode recognize!");

	int iRet = -1;
	IplImage * pImg = (IplImage *)dwImg;
	cv::Mat image = cv::Mat(pImg, true);
	std::string out_recognize;
	std::vector<cv::Point> pointQR;

	if (image.channels() == 3)
	{
		cv::cvtColor(image, image, CV_BGR2GRAY);
	}

	iRet = theApp.QR_Recognize(image, out_recognize, pointQR);

	//目前只支持一个验证码的;
	if (iRet == 1)
	{
		memcpy(out_QRInfor, out_recognize.c_str(), out_recognize.size());
		theApp.putLog(out_QRInfor);
		std::stringstream ss;
		for (int i = 0; i < pointQR.size(); i++)
		{
			ss.str("");
			ss << "x: " << pointQR[i].x << " y:" << pointQR[i].y << std::endl;
			theApp.putLog((char*)ss.str().c_str());
		}

		iRet = 0;
	}
	else
	{
		iRet = -1;
	}
	theApp.putLog("end QRCode recognize!");
	return iRet;
}

//普通二维码的识别;

/*
参数介绍：
dwImg:图片指针;
out_QRInfor: 二维码信息

返回值： 0 成功 -1 失败
*/
__declspec(dllexport)  int WINAPI RCN_RecoQRCodeNormal(DWORD dwImg, char* out_QRInfor)
{
	theApp.putLog("start QRCode recognize!");

	int iRet = -1;
	IplImage * pImg = (IplImage *)dwImg;
	cv::Mat image = cv::Mat(pImg, true);
	std::string out_recognize;
	std::vector<cv::Point> pointQR;

	if (image.channels() == 3)
	{
		cv::cvtColor(image, image, CV_BGR2GRAY);
	}

	iRet = theApp.QR_Recognizex(image, out_recognize, pointQR);

	//目前只支持一个验证码的;
	if (iRet == 0)
	{
		memcpy(out_QRInfor, out_recognize.c_str(), out_recognize.size());
		theApp.putLog(out_QRInfor);
		std::stringstream ss;
		for (int i = 0; i < pointQR.size(); i++)
		{
			ss.str("");
			ss << "x: " << pointQR[i].x << " y:" << pointQR[i].y << std::endl;
			theApp.putLog((char*)ss.str().c_str());
		}

		iRet = 0;
	}
	else
	{
		iRet = -1;
	}
	theApp.putLog("end QRCode recognize!");
	return iRet;
}


__declspec(dllexport)  int WINAPI RCN_RecoQRCodebyPoint(int x, int y, char* out_QRInfor)
{
	int iRet = -1;
	float x_dpi = ((float)theApp.m_iXScaleplate) / 40;
	float y_dpi = ((float)theApp.m_iYScaleplate) / 40;

	//无需旋转情况
	//if (theApp.VerifyCodeRotate == 3)
	{
		if((x == 0) && (y == 0))
		{
			//x = theApp.m_nYzmX;
			//y = theApp.m_nYzmY;
			x = theApp.m_QRX;
			y = theApp.m_QRY;
		}
		x = x * x_dpi;
		y = y * y_dpi;
	}

	if (!out_QRInfor)
	{
		iRet = -2;
		return iRet;
	}
	cv::Rect rectSealCodeZone;
	rectSealCodeZone.x = x - 50;
	rectSealCodeZone.y = y - 50;
	//rectSealCodeZone.width = 180;
	//rectSealCodeZone.height = 180;
	rectSealCodeZone.width = 100 + theApp.m_QRWidth * x_dpi;
	rectSealCodeZone.height = 100 + theApp.m_QRWidth * y_dpi;
	cv::Mat image = theApp.m_image_clipper;
	//裁剪验证码区间
	cv::Mat imgSealCode = image(rectSealCodeZone);
	if (theApp.m_idebug & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_二维码_裁剪图_彩图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imgSealCode);
	}
	BOOL bExist = FALSE;
	float fdt1 = 0.1, fdt2 = 0.1;
	for(int j=1; j<28;j++)
	{
		fdt2 = 0.1;
		for(int i=1; i<25; i++)
		{
			cv::Mat gaussiarImage;
			cv::GaussianBlur(imgSealCode, gaussiarImage, cv::Size(0, 0), 3);
			cv::addWeighted(imgSealCode, fdt1, gaussiarImage, -fdt2, 0, gaussiarImage);
			cv::Mat imgCode = gaussiarImage;

			cv::cvtColor(imgCode, imgCode, CV_BGR2GRAY); 
			std::string out_recognize;
			std::vector<cv::Point> pointQR;
			iRet = theApp.QR_Recognizex(imgCode, out_recognize, pointQR);
			if(iRet == 0)
			{
				strcpy(out_QRInfor, out_recognize.c_str());
				bExist = TRUE;
				break;
			}
			else
			{
				iRet = -3;
			}
			fdt2 += 0.1;
		}
		fdt1 += 0.1;
		if(bExist)
			break;
	}

	/*
	cv::Mat gaussiarImage;
	cv::GaussianBlur(imgSealCode, gaussiarImage, cv::Size(0, 0), 3);
	cv::addWeighted(imgSealCode, 1.5, gaussiarImage, -0.3, 0, gaussiarImage);
	imgSealCode = gaussiarImage;

	cv::cvtColor(imgSealCode, imgSealCode, CV_BGR2GRAY); 
	std::string out_recognize;
	std::vector<cv::Point> pointQR;
	iRet = theApp.QR_Recognizex(imgSealCode, out_recognize, pointQR);
	if(iRet == 0)
	{
	strcpy(out_QRInfor, out_recognize.c_str());
	}
	else
	{
	iRet = -3;
	}*/
	return iRet;
}

//依据二维码，计算角度
#define PI 3.1415926
int calcAngleByQR(std::vector<cv::Point>& pointQR, double* pAngle)
{
	double angle = 0;
	int iRet = -1;
	if (pointQR.size() == 4)
	{
		cv::Point pt1 = pointQR[0];
		cv::Point pt2 = pointQR[1];
		int deltaY = pt1.y - pt2.y;
		int deltaX = pt1.x - pt2.x;
		double tanSeta = (double)deltaY / deltaX;
		//顺时针，正的.
		angle = std::atan(tanSeta);
		iRet = 0;
		//弧度转换为角度.
		*pAngle = angle * 180 / PI;
	}
	else
	{
		*pAngle = 0;
	}
	return iRet;
}




//依据二维码的坐标，进行A4纸的裁剪.
//dwImg: 传入的图片
//pathCutImage: 图片保存的路径
__declspec(dllexport)  int WINAPI RCN_CutImageByQRCode(DWORD dwImg, char* pathCutImage)
{
	IplImage * pImg = (IplImage *)dwImg;
	cv::Mat image = cv::Mat(pImg, true);
	if (image.channels() == 3)
	{
		cv::cvtColor(image, image, CV_BGR2GRAY);
	}

	int iRet = -1;
	std::string out_recognize;
	std::vector<cv::Point> pointQR;
	iRet = theApp.QR_Recognize(image, out_recognize, pointQR);

#if 0
	int width_A4 = theApp.m_A4Width;
	int height_A4 = theApp.m_A4Height;
#elif 1
	int width_A4 = 297 * theApp.m_pixNumPerMm;
	int height_A4 = 210 * theApp.m_pixNumPerMm;
#endif

	char buf_message[1024];
	if (iRet == 1)
	{
		//判断出二维码的正方向, 朝左/右
		// 1: 朝右
		//-1： 朝左
		int directionA4 = 0;

		if (pointQR.size() == 4)
		{
			int ret_angle = 0;
			double angle = 0.0;
			ret_angle = calcAngleByQR(pointQR, &angle);

			//对整张图片旋转angle 角度
			cv::Point center(pImg->width / 2, pImg->height / 2);
			CvPoint i_pt[4];
			CvPoint r_pt[4];
			i_pt[0] = cv::Point(0, 0);
			i_pt[1] = cv::Point(0, pImg->height);
			i_pt[2] = cv::Point(pImg->width, 0);
			i_pt[3] = cv::Point(pImg->width, pImg->height);
			IplImage* pImgDst = rotateImagexCore(pImg, angle, center, i_pt, r_pt);
			cv::Mat image_rotate = cv::Mat(pImgDst, true);
			if (image_rotate.channels() == 3)
			{
				cv::cvtColor(image_rotate, image_rotate, CV_BGR2GRAY);
			}
			out_recognize = "";
			pointQR.clear();
			iRet = theApp.QR_Recognize(image_rotate, out_recognize, pointQR);
			if (iRet != 1 || pointQR.size() != 4)
			{
				return -1;
			}
			if (pointQR[0].x > pointQR[1].x)
			{
				//朝右
				int rightEdge = theApp.m_QRRight*theApp.m_pixNumPerMm;
				int downEdge = theApp.m_QRDown*theApp.m_pixNumPerMm;
				cv::Rect A4_ROI;
				A4_ROI.x = pointQR[0].x + rightEdge - width_A4;
				A4_ROI.y = pointQR[0].y + downEdge - height_A4;
				A4_ROI.width = width_A4;
				A4_ROI.height = height_A4;
				cv::Mat A4_imageROI = image_rotate(A4_ROI);
				cv::imwrite(pathCutImage, A4_imageROI);
			}
			else
			{
				//朝左
				int leftEdge = theApp.m_QRRight*theApp.m_pixNumPerMm;
				int upEdge = theApp.m_QRDown*theApp.m_pixNumPerMm;
				cv::Rect A4_ROI;
				A4_ROI.x = pointQR[0].x - leftEdge;
				A4_ROI.y = pointQR[0].y - upEdge;
				A4_ROI.width = width_A4;
				A4_ROI.height = height_A4;
				cv::Mat A4_imageROI = image_rotate(A4_ROI);
				cv::imwrite(pathCutImage, A4_imageROI);
			}
			iRet = 0;
			cvReleaseImage(&pImgDst);
		}
		else
		{
			iRet = -1;
		}
	}
	return iRet;
}


#define RANSREPROJTHRESHOLD 5
void crossCheckMatching(Ptr<DescriptorMatcher>& descriptorMatcher,
						const Mat& descriptors1, const Mat& descriptors2,
						vector<DMatch>& filteredMatches12, int knn = 1)
{
	filteredMatches12.clear();
	vector<vector<DMatch> > matches12, matches21;
	descriptorMatcher->knnMatch(descriptors1, descriptors2, matches12, knn);
	descriptorMatcher->knnMatch(descriptors2, descriptors1, matches21, knn);
	for (size_t m = 0; m < matches12.size(); m++)
	{
		bool findCrossCheck = false;
		for (size_t fk = 0; fk < matches12[m].size(); fk++)
		{
			DMatch forward = matches12[m][fk];

			for (size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++)
			{
				DMatch backward = matches21[forward.trainIdx][bk];
				if (backward.trainIdx == forward.queryIdx)
				{
					filteredMatches12.push_back(forward);
					findCrossCheck = true;
					break;
				}
			}
			if (findCrossCheck) break;
		}
	}
}


int doIteration(const Mat& img1, Mat& img2, bool isWarpPerspective,
				vector<KeyPoint>& keypoints1, const Mat& descriptors1, vector<KeyPoint>& keypoints2,
				Ptr<FeatureDetector>& detector, Ptr<DescriptorExtractor>& descriptorExtractor,
				Ptr<DescriptorMatcher>& descriptorMatcher, double ransacReprojThreshold, vector<cv::Point>& out_pt1,
				vector<cv::Point>& out_pt2)
{
	Mat H12;
	Mat descriptors2;
	descriptorExtractor->compute(img2, keypoints2, descriptors2);

	vector<DMatch> filteredMatches;
	crossCheckMatching(descriptorMatcher, descriptors1, descriptors2, filteredMatches, 1);

	if (filteredMatches.size() < 6) return 0;

	vector<int> queryIdxs(filteredMatches.size()), trainIdxs(filteredMatches.size());
	for (size_t i = 0; i < filteredMatches.size(); i++)
	{
		queryIdxs[i] = filteredMatches[i].queryIdx;
		trainIdxs[i] = filteredMatches[i].trainIdx;
	}

	if (!isWarpPerspective && ransacReprojThreshold >= 0)
	{
		vector<Point2f> points1; KeyPoint::convert(keypoints1, points1, queryIdxs);
		vector<Point2f> points2; KeyPoint::convert(keypoints2, points2, trainIdxs);
		H12 = findHomography(Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold);
	}

	if (!H12.empty()) // filter outliers
	{
		vector<char> matchesMask(filteredMatches.size(), 0);
		vector<Point2f> points1; KeyPoint::convert(keypoints1, points1, queryIdxs);
		vector<Point2f> points2; KeyPoint::convert(keypoints2, points2, trainIdxs);
		Mat points1t; perspectiveTransform(Mat(points1), points1t, H12);

		double maxInlierDist = ransacReprojThreshold < 0 ? RANSREPROJTHRESHOLD : ransacReprojThreshold;
		//double maxInlierDist = 20;
		for (size_t i1 = 0; i1 < points1.size(); i1++)
		{
			if (norm(points2[i1] - points1t.at<Point2f>((int)i1, 0)) <= maxInlierDist) // inlier
			{
				out_pt1.push_back(points1t.at<Point2f>((int)i1, 0));
				out_pt2.push_back(points2[i1]);

				//转换特征点坐标到原图上
				//cv::Point2f pt_map = cv::Point2f(points2[i1].x + pKA->nKeySearchX, points2[i1].y + pKA->nKeySearchY);
				//if(featureZone.contains(pt_map))
				{
					//pointCountInFeatureBox++;
					matchesMask[i1] = 1;
				}

			}

		}
		return countNonZero(matchesMask);
	}
	else
	{
		return 0;
	}
}


int WINAPI compareFeatureValueMatchTextBlock(IplImage *img1, IplImage *img2, double dvalue[3], vector<cv::Point>& out_pt1,
											 vector<cv::Point>& out_pt2)
{
	dvalue[0] = 0.0;
	dvalue[1] = 0.0;
	if (!img1 || !img2)
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

		Ptr<FeatureDetector> detector = FeatureDetector::create(s);
		Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create(s);//"SURF");
		char buf2[7];
		memset(buf2, '\0', 7);
		buf2[0] = 70;
		buf2[1] = 108;
		buf2[2] = 97;
		buf2[3] = 110;
		buf2[4] = 110;
		buf2[5] = 66;
		s = buf2;
		s += "ased";
		Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create(s);//"FlannBased");//"BruteForce");

		Mat imgm1(img2);
		Mat imgm2(img1);

		//计算二个裁剪区间的特征点;
		vector<KeyPoint> keypoints1;
		detector->detect(imgm1, keypoints1);
		dvalue[0] = keypoints1.size();

		vector<KeyPoint> keypoints2;
		detector->detect(imgm2, keypoints2);
		dvalue[1] = keypoints2.size();

		if (keypoints1.size() < 1) return -3;
		Mat descriptors1;
		descriptorExtractor->compute(imgm1, keypoints1, descriptors1);
		if (keypoints2.size() < 1) return -4;

		dvalue[2] = doIteration(imgm1, imgm2, isWarpPerspective, keypoints1, descriptors1,
			keypoints2, detector, descriptorExtractor, descriptorMatcher, ransacReprojThreshold, out_pt1, out_pt2);
	}
	catch (...)
	{
		return -2;
	}
	return 0;
}


bool compareByY(cv::Point pt1, cv::Point pt2)
{
	return	pt1.y < pt2.y;
}

bool compareByX(cv::Point pt1, cv::Point pt2)
{
	return	pt1.x < pt2.x;
}


int isLongInterval(std::vector<cv::Point> in_points, std::vector<cv::Point>& out_result, int max_interval = 30)
{
	int iRet = -1;
	std::vector<cv::Point> sortByYPoints = in_points;
	std::sort(sortByYPoints.begin(), sortByYPoints.end(), compareByY);
	for (int i = 0; i < sortByYPoints.size() - 1; i++)
	{
		cv::Point pt1 = sortByYPoints[i];
		int j = i + 1;
		cv::Point pt2 = sortByYPoints[j];
		if (abs(pt2.y - pt1.y) > max_interval)
		{
			std::vector<cv::Point> sortByXPoints = in_points;
			std::sort(sortByXPoints.begin(), sortByXPoints.end(), compareByX);

			pt1.x = sortByXPoints[0].x;
			pt2.x = sortByXPoints[sortByXPoints.size() - 1].x;

			out_result.push_back(pt1);
			out_result.push_back(pt2);
			iRet = 0;
			break;
		}
	}
	return iRet;
}


int isLongIntervalX(std::vector<cv::Point> in_points, std::vector<cv::Point>& out_result, int max_interval = 30)
{
	int iRet = -1;
	std::vector<cv::Point> sortByYPoints = in_points;
	std::sort(sortByYPoints.begin(), sortByYPoints.end(), compareByX);
	for (int i = 0; i < sortByYPoints.size() - 1; i++)
	{
		cv::Point pt1 = sortByYPoints[i];
		int j = i + 1;
		cv::Point pt2 = sortByYPoints[j];
		if (abs(pt2.x - pt1.x) > max_interval)
		{
			std::vector<cv::Point> sortByXPoints = in_points;
			std::sort(sortByXPoints.begin(), sortByXPoints.end(), compareByY);

			pt1.y = sortByXPoints[0].y;
			pt2.y = sortByXPoints[sortByXPoints.size() - 1].y;

			out_result.push_back(pt1);
			out_result.push_back(pt2);
			iRet = 0;
			break;
		}
	}
	return iRet;
}


//基本的字符串处理操作
int find_space_num(const std::string& in_qrCode_infor)
{
	if (in_qrCode_infor == "")
	{
		return 0;
	}
	int num_space = 0;
	for (int i = 0; i < in_qrCode_infor.size(); i++)
	{
		if (in_qrCode_infor[i] == ' ')
		{
			num_space++;
		}
	}
	return num_space;
}


//获取有效的信息
//返回值是 有效信息的个数
int getInforPosTextAndVerifyCode(const std::string& in_qrCode_infor, cv::Point& pt_text, cv::Point& pt_verifyCode)
{
	int validINforNum = 0;
	validINforNum = find_space_num(in_qrCode_infor);
	if (validINforNum == 0)
	{
		return 0;
	}

	std::stringstream ss;
	ss << in_qrCode_infor;
	int x_text = 0, y_text = 0;
	int x_verifyCode = 0, y_verifyCode = 0;
	if (validINforNum == 1)
	{
		ss >> x_text >> y_text;
		pt_text.x = x_text;
		pt_text.y = y_text;
		return 1;
	}
	else if (validINforNum == 3)
	{
		ss >> x_text >> y_text >> x_verifyCode >> y_verifyCode;
		pt_text.x = x_text;
		pt_text.y = y_text;
		pt_verifyCode.x = x_verifyCode;
		pt_verifyCode.y = y_verifyCode;

		return 2;
	}
	return 0;
}


//识别文本
//此接口，实现模板与裁剪图片的比较, 以及验证码的验证.
//原始的图片 dwImg
//模板的图片路径 pathModel
//12位的验证码 in_verifyCode
//匹配的结果，成功/失败
__declspec(dllexport)  int WINAPI RCN_MatchTextBlockByModel(DWORD dwImg, char* pathModel, char* in_verifyCode, int iReserve)
{
	int iRet = -1;
	int BlockSize = 20;
	double m_scale = theApp.m_scaleText;
	char buf_message[1024];
	if (iReserve & 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "缩放比例：%.3f", m_scale);
		theApp.putLog(buf_message);
	}

	DWORD ts = GetTickCount();
	//获取原始图片
	IplImage *pImg;
	pImg = (IplImage *)dwImg;
	cv::Mat image = cv::Mat(pImg, true);
	cv::resize(image, image, cv::Size(0, 0), m_scale, m_scale);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image);
	}

	if (image.channels() == 3)
	{
		cv::cvtColor(image, image, CV_BGR2GRAY);
	}

	//进行二维码识别,获取文本的相对位置坐标;
	std::string out_recognize;
	std::vector<cv::Point> pointQR;
	iRet = theApp.QR_Recognize(image, out_recognize, pointQR);
	theApp.putLog((char*)out_recognize.c_str());
	if (iRet != 1 || out_recognize.size() == 0 || pointQR.size() != 4)
	{
		return -1;
	}


	///////////////////////
	int ret_angle = 0;
	double angle = 0.0;
	ret_angle = calcAngleByQR(pointQR, &angle);
	if (iReserve == 1)
	{
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "angle: %f", angle);
		theApp.putLog(buf_message);
	}

	//对整张图片旋转angle 角度
	cv::Point center(pImg->width / 2, pImg->height / 2);
	CvPoint i_pt[4];
	CvPoint r_pt[4];
	i_pt[0] = cv::Point(0, 0);
	i_pt[1] = cv::Point(0, pImg->height);
	i_pt[2] = cv::Point(pImg->width, 0);
	i_pt[3] = cv::Point(pImg->width, pImg->height);
	IplImage* pImgDst = rotateImagexCore(pImg, angle, center, i_pt, r_pt);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图的旋转图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImgDst);
	}

	cv::Mat image_rotate = cv::Mat(pImgDst, true);
	cvReleaseImage(&pImgDst);

	//////////////////////////
	out_recognize = "";
	pointQR.clear();
	if (image_rotate.channels() == 3)
	{
		cv::cvtColor(image_rotate, image_rotate, CV_BGR2GRAY);
	}

	iRet = theApp.QR_Recognize(image_rotate, out_recognize, pointQR);
	if (iRet != 1 || out_recognize.size() == 0 || pointQR.size() != 4)
	{
		return -1;
	}

	cv::Point pt_text, pt_verifyCode;
	int num_infor = getInforPosTextAndVerifyCode(out_recognize, pt_text, pt_verifyCode);
	if (num_infor == 0)
	{
		return -1;
	}

	cv::Mat templateImg;
	templateImg = cv::imread(pathModel, 0);
	IplImage* templateImg_1 = NULL;
	templateImg_1 = &IplImage(templateImg);
	int averageLight_template = get_avg_gray(templateImg_1);
	cv::Mat templateImgBin;
	if (iReserve == 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "模板的亮度:%d", averageLight_template);
		theApp.putLog(buf_message);
	}

	myThreshold(templateImg, templateImgBin, BlockSize);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_模板二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, templateImgBin);
	}

	//先处理文本	， 文本的相对于二维码的坐标位置。
	cv::Point point;
	//区分方向
	if (pointQR[0].x > pointQR[1].x)
	{
		//朝右
		point.x = pointQR[1].x - pt_text.x * theApp.m_pixNumPerMm * m_scale;
		point.y = pointQR[1].y - pt_text.y * theApp.m_pixNumPerMm * m_scale;
	}
	else
	{
		//朝左
		point.x = pointQR[1].x + pt_text.x * theApp.m_pixNumPerMm * m_scale;
		point.y = pointQR[1].y + pt_text.y * theApp.m_pixNumPerMm * m_scale;
	}
	cv::Rect rectROI;
	if (templateImgBin.rows > templateImgBin.cols)
	{
		rectROI.x = point.x - templateImgBin.size().height;
		rectROI.y = point.y - templateImgBin.size().height;
		rectROI.width = 2 * templateImgBin.size().height;
		rectROI.height = 2 * templateImgBin.size().height;
	}
	else
	{
		rectROI.x = point.x - templateImgBin.size().width;
		rectROI.y = point.y - templateImgBin.size().width;
		rectROI.width = 2 * templateImgBin.size().width;
		rectROI.height = 2 * templateImgBin.size().width;
	}

	if (rectROI.x < 0)
	{
		rectROI.x = 0;
	}
	if ((rectROI.x + rectROI.width) > image.cols)
	{
		rectROI.x = image.cols - rectROI.width - 3;
	}

	if (rectROI.y < 0)
	{
		rectROI.y = 0;
	}
	if ((rectROI.y + rectROI.height) > image.rows)
	{
		rectROI.y = image.rows - rectROI.height - 3;
	}

	if (rectROI.x < 0 || rectROI.y < 0 ||
		((rectROI.x + rectROI.width) > image.size().width) ||
		((rectROI.y + rectROI.height) > image.size().height) ||
		((rectROI.x + rectROI.width) < 0) ||
		((rectROI.y + rectROI.height) < 0)
		)
	{
		return -1;
	}

	cv::Mat imageROI = image_rotate(rectROI);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
	}

	cv::Mat greyROI;
	if (imageROI.channels() == 3)
	{
		cv::cvtColor(imageROI, greyROI, CV_BGR2GRAY);
	}
	else
	{
		greyROI = imageROI;
	}

	IplImage* pImg_tmp = &IplImage(greyROI);
	int averageLight = get_avg_gray(pImg_tmp);
	if (iReserve == 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "裁剪区间调整前的亮度:%d", averageLight);
		theApp.putLog(buf_message);
	}

	double scale_light = 0;
	scale_light = ((double)averageLight_template) / averageLight;
	cvConvertScale(pImg_tmp, pImg_tmp, scale_light);
	int averageLightAdjust = get_avg_gray(pImg_tmp);
	if (iReserve == 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "裁剪区间调整后的亮度:%d", averageLightAdjust);
		theApp.putLog(buf_message);
	}
	greyROI = cv::Mat(pImg_tmp, true);
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间光照调整.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, greyROI);
	}

	cv::Mat threshold_image;
	//对裁剪区域，进行亮度调整
	myThreshold(greyROI, threshold_image, BlockSize);
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, threshold_image);
	}

	//二值测试

	//膨胀测试
	cv::Mat elem = cv::getStructuringElement(CV_8UC1, cv::Size(2, 2));
	cv::erode(templateImgBin, templateImgBin, elem);
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间_放大.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, threshold_image);
	}

	cv::erode(threshold_image, threshold_image, elem);
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_模板膨胀图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, templateImgBin);
	}

	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间膨胀图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, threshold_image);
	}

	IplImage *tempImage_1 = &IplImage(templateImgBin);
	IplImage *testImage = &IplImage(threshold_image);

	double value[3];
	value[0] = 0;
	value[1] = 0;
	value[2] = 0;
	vector<cv::Point> out_pt1; //模板上的点
	vector<cv::Point> out_pt2; //原图上的点
	compareFeatureValueMatchTextBlock(testImage, tempImage_1, value, out_pt1, out_pt2);
	double score = value[2] / value[0];
	int numFeature = value[0];
	int numMatch = value[2];
	if (iReserve == 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "模板的点数：%d , 匹配的点数:%d . 占比：%.f", numFeature, numMatch, score * 100);
		theApp.putLog(buf_message);
	}

	if (iReserve & 7)
	{
		cv::Mat image2_color;
		cv::cvtColor(threshold_image, image2_color, CV_GRAY2BGR);
		for (int i = 0; i < out_pt2.size(); i++)
		{
			cv::circle(image2_color, out_pt2[i], 3, cv::Scalar(0, 0, 255), 2, CV_FILLED);
		}
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_匹配结果图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image2_color);
	}

	//判断是否有断层
	//近邻的点， y间隔比较大
	//先按照y排序

	std::vector<cv::Point> result_longDistence;
	int iRet_tmp = isLongInterval(out_pt2, result_longDistence, 20);
	if (iRet_tmp == 0)
	{
		//有断层;
		if (iReserve & 1)
		{
			cv::Mat image3_color;
			cv::cvtColor(threshold_image, image3_color, CV_GRAY2BGR);
			cv::Point pt1 = result_longDistence[0];
			cv::Point pt2 = result_longDistence[1];
			cv::rectangle(image3_color, pt1, pt2, cv::Scalar(0, 0, 255), 3);
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_有断层图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, image3_color);

			//转换坐标点到原图;
			cv::Point pt3 = pt1;
			cv::Point pt4 = pt2;
			pt3.x = pt3.x / 2 + rectROI.x;
			pt3.y = pt3.y / 2 + rectROI.y;
			pt4.x = pt4.x / 2 + rectROI.x;
			pt4.y = pt4.y / 2 + rectROI.y;

			cv::Mat cutColorImageShow;
			if (image.channels() == 1)
			{
				cv::cvtColor(image, cutColorImageShow, CV_GRAY2BGR);
			}
			else
			{
				cutColorImageShow = image;
			}

			cv::rectangle(cutColorImageShow, pt3, pt4, cv::Scalar(0, 0, 255), 3);
			GetSystemTime(&st);
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_有断层原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, cutColorImageShow);
		}
	}

	//伪装占比
	int like = theApp.Percent;
	int per = 20;
	if (like > 100) like = 100;
	if (like < 0) like = 80;
	if (like > 80)
	{
		per += (like - 80) * 2;
	}
	else
	{
		per -= (80 - like) / 4;
	}

	if ((score * 100) > per)
	{
		if (iRet_tmp != 0)
		{
			iRet = 0;
		}
		else
		{
			iRet = 1;
		}
	}
	else
	{
		iRet = -1;
	}

	if (iReserve == 1)
	{
		DWORD t3 = GetTickCount() - ts;
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "验证文本块总时间： %d ms ", t3);
		theApp.putLog(buf_message);
	}
	return iRet;
}


//辨别文本块是否合理
int isValidTextBlock(cv::Mat img, int score = 5)
{
	cv::Mat tmp = img.clone();
	cv::Mat grey_tmp;
	if (tmp.channels() == 3)
	{
		cv::cvtColor(tmp, grey_tmp, CV_BGR2GRAY);
	}
	else
	{
		grey_tmp = tmp;
	}
	cv::threshold(grey_tmp, grey_tmp, 240, 255, CV_THRESH_BINARY);
	/// 设定bin数目
	int histSize = 255;
	/// 设定取值范围 ( R,G,B) )
	float range[] = { 0, 256 };
	const float* histRange = { range };
	bool uniform = true; bool accumulate = false;
	MatND hist;
	/// 计算直方图:
	calcHist(&grey_tmp, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
	int sum_pix = img.size().width* img.size().height;
	int sum_dark = hist.at<float>(0);
	double percentRate = (double(sum_dark)) / sum_pix;
	if (percentRate * 100 > score)
	{
		return 0;
	}
	return -1;
}


//识别文本
//此接口，实现模板与裁剪图片的比较, 以及验证码的验证.
//原始的图片 dwImg
//模板的图片路径 pathModel
//字号 in_fontNumber
//匹配的结果，成功/失败
__declspec(dllexport)  int WINAPI RCN_MatchTextBlockByModelByHandsInputs(DWORD dwImg, char* pathModel, POINT in_point, char* in_fontNumber, int iReserve)
{

	int iRet = -1;
	int BlockSize = 20;
	//float m_scale = 0.573;
	double m_scale = theApp.m_scaleText;

	char buf_message[1024];
	if (iReserve == 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "缩放比例：%.3f", m_scale);
		theApp.putLog(buf_message);
	}


	DWORD ts = GetTickCount();
	//获取原始图片
	IplImage *pImg;
	pImg = (IplImage *)dwImg;

	cv::Mat image = cv::Mat(pImg, true);
	cv::resize(image, image, cv::Size(0, 0), m_scale, m_scale);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image);
	}

	if (image.channels() == 3)
	{
		cv::cvtColor(image, image, CV_BGR2GRAY);
	}



	//进行二维码识别,获取文本的相对位置坐标;
	std::string out_recognize;
	std::vector<cv::Point> pointQR;
	iRet = theApp.QR_Recognize(image, out_recognize, pointQR);

	if (iReserve == 1)
	{
		theApp.putLog("二维码信息:");
		theApp.putLog((char*)out_recognize.c_str());
	}

	if (iRet != 1 || out_recognize.size() == 0 || pointQR.size() != 4)
	{
		return -1;
	}

	if (iReserve == 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "输入坐标 x: %d, y: %d", in_point.x, in_point.y);
		theApp.putLog(buf_message);
	}





	///////////////////////

	int ret_angle = 0;
	double angle = 0.0;
	ret_angle = calcAngleByQR(pointQR, &angle);
	if (iReserve == 1)
	{
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "angle: %f", angle);
		theApp.putLog(buf_message);
	}

	//对整张图片旋转angle 角度

	cv::Point center(pImg->width / 2, pImg->height / 2);
	CvPoint i_pt[4];
	CvPoint r_pt[4];
	i_pt[0] = cv::Point(0, 0);
	i_pt[1] = cv::Point(0, pImg->height);
	i_pt[2] = cv::Point(pImg->width, 0);
	i_pt[3] = cv::Point(pImg->width, pImg->height);
	IplImage* pImgDst = rotateImagexCore(pImg, angle, center, i_pt, r_pt);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图的旋转图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImgDst);
	}
	cv::Mat image_rotate = cv::Mat(pImgDst, true);
	cvReleaseImage(&pImgDst);

	//对旋转进行裁剪




	//再旋转



	//再依据左上角裁剪文本区


	//////////////////////////
	out_recognize = "";
	pointQR.clear();

	if (image_rotate.channels() == 3)
	{
		cv::cvtColor(image_rotate, image_rotate, CV_BGR2GRAY);
	}

	iRet = theApp.QR_Recognize(image_rotate, out_recognize, pointQR);
	if (iRet != 1 || out_recognize.size() == 0 || pointQR.size() != 4)
	{
		return -1;
	}


	//memset(buf_message, 0, sizeof buf_message);
	//sprintf(buf_message, "识别结果:%s", m_scale);


	//std::string message_pt_text = message_QR;
	cv::Point pt_text, pt_verifyCode;

	//int num_infor = getInforPosTextAndVerifyCode(out_recognize, pt_text, pt_verifyCode);
	//if(num_infor == 0)
	//{
	//	return -1;
	//}
	pt_text.x = in_point.x;
	pt_text.y = in_point.y;

	cv::Mat templateImg;
	templateImg = cv::imread(pathModel, 0);
	if (iReserve & 3)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_模板原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, templateImg);
	}



	//cv::resize(templateImg, templateImg, cv::Size(0, 0), 1.5, 1.5); //CV_INTER_CUBIC

	//if(iReserve & 1)
	//{
	//	SYSTEMTIME st;
	//	GetSystemTime(&st);
	//	CString sFileName;
	//	sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_模板放大图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour+8, st.wMinute, st.wSecond, st.wMilliseconds);
	//	//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
	//	cv::imwrite((LPSTR)(LPCTSTR)sFileName, templateImg);
	//}


#if 0

	IplImage* templateImg_1 = NULL;
	templateImg_1 = &IplImage(templateImg);
	int averageLight_template = get_avg_gray(templateImg_1);
	cv::Mat templateImgBin;
	if (iReserve & 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "模板的亮度:%d", averageLight_template);
		theApp.putLog(buf_message);
	}
	myThreshold(templateImg, templateImgBin, BlockSize);


	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_模板二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, templateImgBin);
	}
#endif

	//return 0;


	int width_A4 = theApp.m_A4Width;
	int height_A4 = theApp.m_A4Height;


	cv::Mat A4_imageROI_Rotate;
	//先处理文本	， 文本的相对于二维码的坐标位置。
	cv::Point point;
	point.x = in_point.x*theApp.m_pixNumPerMm * m_scale;
	point.y = in_point.y* theApp.m_pixNumPerMm * m_scale;
	//区分方向
	if (pointQR[0].x > pointQR[1].x)
	{
		//朝右
		//point.x =pointQR[1].x -  pt_text.x * theApp.m_pixNumPerMm * m_scale; 
		//point.y =pointQR[1].y - pt_text.y * theApp.m_pixNumPerMm * m_scale;

		int rightEdge = theApp.m_QRRight;
		int downEdge = theApp.m_QRDown;
		cv::Rect A4_ROI;
		A4_ROI.x = pointQR[0].x + rightEdge - width_A4;
		A4_ROI.y = pointQR[0].y + downEdge - height_A4;
		A4_ROI.width = width_A4;
		A4_ROI.height = height_A4;
		cv::Mat A4_imageROI = image_rotate(A4_ROI);



		//逆时针转90
		transpose(A4_imageROI, A4_imageROI_Rotate);
		flip(A4_imageROI_Rotate, A4_imageROI_Rotate, 0);


	}
	else
	{
		//朝左
		//point.x =pointQR[1].x + pt_text.x * theApp.m_pixNumPerMm * m_scale; 
		//point.y =pointQR[1].y + pt_text.y * theApp.m_pixNumPerMm * m_scale;

		int leftEdge = theApp.m_QRRight;
		int upEdge = theApp.m_QRDown;
		cv::Rect A4_ROI;
		A4_ROI.x = pointQR[0].x - leftEdge;
		A4_ROI.y = pointQR[0].y - upEdge;
		A4_ROI.width = width_A4;
		A4_ROI.height = height_A4;
		cv::Mat A4_imageROI = image_rotate(A4_ROI);


		transpose(A4_imageROI, A4_imageROI_Rotate);
		flip(A4_imageROI_Rotate, A4_imageROI_Rotate, 1);

	}

	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间的旋正图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, A4_imageROI_Rotate);
	}







	cv::Rect rectROI;
	rectROI.x = point.x - templateImg.size().width * 3 / 2;
	rectROI.y = point.y - templateImg.size().height * 3 / 2;
	rectROI.width = 3 * templateImg.size().width;
	rectROI.height = 3 * templateImg.size().height;






	if (rectROI.x < 0)
	{
		rectROI.x = 0;
	}
	if ((rectROI.x + rectROI.width) > A4_imageROI_Rotate.cols)
	{

		rectROI.width = A4_imageROI_Rotate.cols - rectROI.x - 3;
	}


	if (rectROI.y < 0)
	{
		rectROI.y = 0;
	}
	if ((rectROI.y + rectROI.height) > A4_imageROI_Rotate.rows)
	{

		rectROI.height = A4_imageROI_Rotate.rows - rectROI.y - 3;
	}


	if (rectROI.x < 0 || rectROI.y < 0 ||
		((rectROI.x + rectROI.width) > A4_imageROI_Rotate.size().width) ||
		((rectROI.y + rectROI.height) > A4_imageROI_Rotate.size().height) ||
		((rectROI.x + rectROI.width) < 0) ||
		((rectROI.y + rectROI.height) < 0)
		)
	{
		return -1;
	}



	if (iReserve == 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "裁剪区间的中心坐标 x:%d, y:%d.", point.x, point.y);
		theApp.putLog(buf_message);

		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "裁剪区间的坐标 x:%d, y:%d, width: %d, height:%d", rectROI.x, rectROI.y, rectROI.width, rectROI.height);
		theApp.putLog(buf_message);
	}



	//cv::Mat imageROI = A4_imageROI_Rotate(rectROI);

	std::string path = "D:\\workSpace\\image\\MatchTextBlock\\D\\";
	cv::Mat imageROI = cv::imread(path + "c-02.jpg");

	if (iReserve & 7)
	{
		//cv::Mat tmpImage = resultCutImg.clone();

		//cv::rectangle(tmpImage, rectROI, cv::Scalar(0, 0, 255));


		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
	}
	//return 0;




	cv::Mat greyROI;
	if (imageROI.channels() == 3)
	{
		cv::cvtColor(imageROI, greyROI, CV_BGR2GRAY);
	}
	else
	{
		greyROI = imageROI;
	}


	IplImage *tempImage_1 = NULL;
	IplImage *testImage = NULL;


	cv::Mat MidImgTest;
	cv::Mat MidImgTemplate;
	if (strcmp(in_fontNumber, "四号") == 0)
	{
		theApp.putLog("四号");

		MidImgTest = greyROI;
		MidImgTemplate = templateImg;
	}
	else if (strcmp(in_fontNumber, "小四") == 0)
	{
		theApp.putLog("小四");


		cv::Mat test_threshold, template_threshold;
		myThreshold(greyROI, test_threshold, BlockSize);
		myThreshold(templateImg, template_threshold, BlockSize);

		cv::Mat elem = cv::getStructuringElement(CV_8UC1, cv::Size(2, 2));
		cv::Mat test_erode, template_erode;
		cv::erode(template_threshold, template_erode, elem);

		cv::erode(test_threshold, test_erode, elem);

		MidImgTest = test_erode;
		MidImgTemplate = template_erode;
	}
	else if (strcmp(in_fontNumber, "五号") == 0)
	{
		theApp.putLog("五号");
		return -1;
	}
	cv::Mat expandImgTest, expandImgTemplate;
	cv::resize(MidImgTemplate, expandImgTemplate, cv::Size(0, 0), 1.5, 1.5);
	cv::resize(MidImgTest, expandImgTest, cv::Size(0, 0), 2, 2);

	tempImage_1 = &IplImage(expandImgTemplate);
	testImage = &IplImage(expandImgTest);

	double value[3];
	value[0] = 0;
	value[1] = 0;
	value[2] = 0;
	vector<cv::Point> out_pt1; //模板上的点
	vector<cv::Point> out_pt2; //原图上的点
	compareFeatureValueMatchTextBlock(testImage, tempImage_1, value, out_pt1, out_pt2);

	double score = value[2] / value[0];


	int numFeature = value[0];
	int numMatch = value[2];

	if (iReserve == 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "模板的点数：%d , 匹配的点数:%d . 占比：%.f", numFeature, numMatch, score * 100);
		theApp.putLog(buf_message);
	}


	if (iReserve & 7)
	{
		cv::Mat image2_color;
		cv::cvtColor(expandImgTest, image2_color, CV_GRAY2BGR);
		for (int i = 0; i < out_pt2.size(); i++)
		{
			cv::circle(image2_color, out_pt2[i], 3, cv::Scalar(0, 0, 255), 2, CV_FILLED);
		}
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_匹配结果图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image2_color);
	}
	std::vector<cv::Point> result_longDistence;
#if 0
	int iRet_tmp = isLongIntervalX(out_pt2, result_longDistence, theApp.distenceChars);
	if (iRet_tmp == 0)
	{
		//有断层;
		if (iReserve & 1)
		{
			cv::Mat image3_color;
			cv::cvtColor(threshold_image, image3_color, CV_GRAY2BGR);

			cv::Point pt1 = result_longDistence[0];
			cv::Point pt2 = result_longDistence[1];



			//cv::line(image3_color, cv::Point(0, pt1.y), cv::Point(image3_color.cols, pt1.y), cv::Scalar(0, 255, 0));			
			//cv::line(image3_color, cv::Point(0, pt2.y), cv::Point(image3_color.cols, pt2.y), cv::Scalar(0, 255, 0));
			cv::rectangle(image3_color, pt1, pt2, cv::Scalar(0, 0, 255), 3);
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_有断层图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, image3_color);

			//转换坐标点到原图;
			cv::Point pt3 = pt1;
			cv::Point pt4 = pt2;
			pt3.x = pt3.x / 2 + rectROI.x;
			pt3.y = pt3.y / 2 + rectROI.y;
			pt4.x = pt4.x / 2 + rectROI.x;
			pt4.y = pt4.y / 2 + rectROI.y;


			cv::Mat cutColorImageShow;

			if (image.channels() == 1)
			{
				cv::cvtColor(image, cutColorImageShow, CV_GRAY2BGR);
			}
			else
			{
				cutColorImageShow = image;
			}

			//cv::line(cutColorImageShow, cv::Point(pt3.x-30, pt3.y), cv::Point(pt3.x+30, pt3.y), cv::Scalar(0, 0, 255), 3);			
			//cv::line(cutColorImageShow, cv::Point(pt4.x-30, pt4.y), cv::Point(pt4.x+30, pt4.y), cv::Scalar(0, 0, 255), 3);
			cv::rectangle(cutColorImageShow, pt3, pt4, cv::Scalar(0, 0, 255), 3);

			//SYSTEMTIME st;
			GetSystemTime(&st);
			//CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_有断层原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, cutColorImageShow);

		}
	}

#endif

	//伪装占比
#if 0
	int like = theApp.Percent;
	int per = 19;
	if (like > 100) like = 100;
	if (like < 0) like = 80;
	if (like > 80)
	{
		per += (like - 80) * 2;
	}
	else
	{
		per -= (80 - like) / 4;
	}
#endif

	int per = theApp.Percent;

	if ((score * 100) >= per)
	{
		//if(iRet_tmp != 0)
		{
			iRet = 0;
		}
		//else
		//{
		//	iRet = 1;
		//}
	}
	else
	{
		iRet = -1;
	}

	if (iReserve == 1)
	{
		DWORD t3 = GetTickCount() - ts;
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "验证文本块总时间： %d ms ", t3);
		theApp.putLog(buf_message);
	}
	return iRet;


}


int isValidPosition(int pos)
{
	int iRet = -1;
	int arr_valid[12] = { 5, 6, 9, 10, 13, 14, 17, 18, 21, 22, 25, 26 };
	for (int i = 0; i < 12; i++)
	{
		if (pos == arr_valid[i])
		{
			iRet = 0;
			break;
		}
	}
	return iRet;
}


int bSums(Mat src, int flag_img, int iDebug)
{
	cv::Mat thresholdImg;
	myThreshold(src, thresholdImg);
	//threshold(src,thresholdImg,200,255,THRESH_BINARY);
	if (iDebug & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		if (flag_img == 0)
		{
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		}
		else
		{
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_WORD二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		}
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		//cv::imwrite((LPSTR)(LPCTSTR)sFileName, thresholdImg);
	}
	int counter = 0;
	//迭代器访问像素点  
	Mat_<uchar>::iterator it = thresholdImg.begin<uchar>();
	Mat_<uchar>::iterator itend = thresholdImg.end<uchar>();
	for (; it != itend; ++it)
	{
		if ((*it) == 0) counter += 1;//二值化后，像素点是0或者255  
	}
	return counter;
}



int isSameDist(cv::Mat testImg, cv::Mat temImg, int iDebug)
{
	cv::Mat testImg_tmp = testImg.clone();
	cv::Mat temImg_tmp = temImg.clone();

	int sum_test = 0;
	int sum_temp = 0;
	sum_test = bSums(testImg_tmp, 0, iDebug);
	sum_temp = bSums(temImg_tmp, 1, iDebug);

	if (iDebug&1)
	{
		char buf_message[1024];
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "黑色数， 测试图:%d, 模板图:%d， 阈值:%d", sum_test, sum_temp, theApp.errorNumDarkPixel);
		theApp.putLog(buf_message);
	}
	if (abs(sum_test - sum_temp) < theApp.errorNumDarkPixel)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}



//用途： 依据给定的文本块，辨别文本块的对错
// dwImg: 用印机拍照的图片.
// pathModel: 合同的照片路径.
// in_row: 行数
// in_col: 列数
// fontType: 字体的类型.宋体 楷体
// fontNumber: 字号, 小四, 四号
// iReserve: 调试参数.
__declspec(dllexport)  int WINAPI RCN_MatchTextBlockByModelNumberNew(DWORD dwImg, char* pathModel, int in_row, int in_col, char* fontType, char* in_fontNumber, int iReserve)
{
	if (!dwImg)
	{
		return -2;
	}
	int iRet = -1;
	int BlockSize = 20;

	//iReserve = 2;

	//iRet = isValidPosition(in_blockNumber);
	//if(iRet != 0)
	//{
	//	return 1;
	//}
	char buf_message[1024];
	DWORD ts = GetTickCount();
	//获取原始图片
	IplImage *pImg;
	pImg = (IplImage *)dwImg;

	cv::Mat image = cv::Mat(pImg, true);
	//cv::resize(image, image, cv::Size(0, 0), m_scale, m_scale);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image);
	}

	if (image.channels() == 3)
	{
		cv::cvtColor(image, image, CV_BGR2GRAY);
	}

	//进行二维码识别, 做合同的裁剪;
	std::string out_recognize;
	std::vector<cv::Point> pointQR;
	iRet = theApp.QR_Recognize(image, out_recognize, pointQR);
	if (iReserve == 1)
	{
		theApp.putLog("二维码信息:");
		theApp.putLog((char*)out_recognize.c_str());
	}

	if (iRet != 1 || out_recognize.size() == 0 || pointQR.size() != 4)
	{
		return -1;
	}
	int ret_angle = 0;
	double angle = 0.0;
	ret_angle = calcAngleByQR(pointQR, &angle);
	if (iReserve & 1)
	{
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "angle: %f", angle);
		theApp.putLog(buf_message);
	}

	//对整张图片旋转angle 角度
	cv::Point center(pImg->width / 2, pImg->height / 2);
	CvPoint i_pt[4];
	CvPoint r_pt[4];
	i_pt[0] = cv::Point(0, 0);
	i_pt[1] = cv::Point(0, pImg->height);
	i_pt[2] = cv::Point(pImg->width, 0);
	i_pt[3] = cv::Point(pImg->width, pImg->height);
	IplImage* pImgDst = rotateImagexCore(pImg, angle, center, i_pt, r_pt);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图的旋转图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImgDst);
	}
	cv::Mat image_rotate = cv::Mat(pImgDst, true);
	cvReleaseImage(&pImgDst);
	//////////////////////////
	out_recognize = "";
	pointQR.clear();

	if (image_rotate.channels() == 3)
	{
		cv::cvtColor(image_rotate, image_rotate, CV_BGR2GRAY);
	}

	iRet = theApp.QR_Recognize(image_rotate, out_recognize, pointQR);
	if (iRet != 1 || out_recognize.size() == 0 || pointQR.size() != 4)
	{
		return -1;
	}

#if 0
	int width_A4 = theApp.m_A4Width;
	int height_A4 = theApp.m_A4Height;
#elif 1
	int width_A4 = 297 * theApp.m_pixNumPerMm;
	int height_A4 = 210 * theApp.m_pixNumPerMm;
#endif
	cv::Mat A4_imageROI_Rotate;
	//先处理文本	， 文本的相对于二维码的坐标位置。
	cv::Point point;

	//区分方向
	if (pointQR[0].x > pointQR[1].x)
	{
		//朝右
		int rightEdge = theApp.m_QRRight * theApp.m_pixNumPerMm;
		int downEdge = theApp.m_QRDown * theApp.m_pixNumPerMm;
		cv::Rect A4_ROI;
		A4_ROI.x = pointQR[0].x + rightEdge - width_A4;
		A4_ROI.y = pointQR[0].y + downEdge - height_A4;
		A4_ROI.width = width_A4;
		A4_ROI.height = height_A4;
		cv::Mat A4_imageROI = image_rotate(A4_ROI);
		//逆时针转90
		transpose(A4_imageROI, A4_imageROI_Rotate);
		flip(A4_imageROI_Rotate, A4_imageROI_Rotate, 0);
	}
	else
	{
		//朝左
		int leftEdge = theApp.m_QRRight * theApp.m_pixNumPerMm;
		int upEdge = theApp.m_QRDown * theApp.m_pixNumPerMm;
		cv::Rect A4_ROI;
		A4_ROI.x = pointQR[0].x - leftEdge;
		A4_ROI.y = pointQR[0].y - upEdge;
		A4_ROI.width = width_A4;
		A4_ROI.height = height_A4;
		cv::Mat A4_imageROI = image_rotate(A4_ROI);
		transpose(A4_imageROI, A4_imageROI_Rotate);
		flip(A4_imageROI_Rotate, A4_imageROI_Rotate, 1);
	}
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间的旋正图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, A4_imageROI_Rotate);
	}
	//裁剪指定的块
	cv::Mat a4PrintImg;
	a4PrintImg = cv::imread(pathModel, 0);
	if (a4PrintImg.empty())
	{
		return -1;
	}

	cv::resize(a4PrintImg, a4PrintImg, cv::Size(793, 1122));

	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_A4打印原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, a4PrintImg);
	}

	//尺寸的统一
	int width_template = a4PrintImg.size().width;
	int width_test = A4_imageROI_Rotate.size().width;
	double scale = double(width_template) / width_test;
	cv::resize(A4_imageROI_Rotate, A4_imageROI_Rotate, cv::Size(0, 0), scale, scale);

	//拍照的图, 打印的图, 
	cv::Mat testImg_roi, templateImg_roi;
	iRet = cut_imageROIBlock(A4_imageROI_Rotate, a4PrintImg, testImg_roi, templateImg_roi, in_row, in_col, iReserve);
	if (iRet != 0)
	{
		return -1;
	}
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_A4裁剪.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, templateImg_roi);
	}
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, testImg_roi);
	}

	cv::Mat greyROI;
	if (testImg_roi.channels() == 3)
	{
		cv::cvtColor(testImg_roi, greyROI, CV_BGR2GRAY);
	}
	else
	{
		greyROI = testImg_roi;
	}

	//判断黑色像素的分布， 如果裁剪偏差大，很有可能判断像素的分布，失败!
	iRet = isSameDist(greyROI, templateImg_roi, iReserve);
	if (iRet != 0)
	{
		return iRet;
	}
	cv::Mat testImg_roi_1;
	cv::Mat templateImg_roi_1;
	iRet = cut_imageROIBlockFeature(A4_imageROI_Rotate, a4PrintImg, testImg_roi_1, templateImg_roi_1, in_row, in_col, iReserve);
	if (iRet != 0)
	{
		return -1;
	}
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间放大.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, testImg_roi_1);
	}

	if (testImg_roi_1.channels() == 3)
	{
		cv::cvtColor(testImg_roi_1, greyROI, CV_BGR2GRAY);
	}
	else
	{
		greyROI = testImg_roi_1;
	}


	IplImage *tempImage_1 = NULL;
	IplImage *testImage = NULL;

	cv::Mat MidImgTest;
	cv::Mat MidImgTemplate;

	/*if(strcmp(in_fontNumber, "四号") == 0)
	{
	theApp.putLog("四号");

	MidImgTest = greyROI;
	MidImgTemplate = templateImg;
	}
	else if(strcmp(in_fontNumber, "小四") == 0)
	{*/

	//theApp.putLog("小四");

	cv::Mat test_threshold, template_threshold;
	myThreshold(greyROI, test_threshold, BlockSize);
	myThreshold(templateImg_roi, template_threshold, BlockSize);


	cv::Mat elem = cv::getStructuringElement(CV_8UC1, cv::Size(2, 2));
	cv::Mat test_erode, template_erode;
	cv::erode(template_threshold, template_erode, elem);
	cv::erode(test_threshold, test_erode, elem);
	MidImgTest = test_erode;
	MidImgTemplate = template_erode;

	//去掉膨胀;
	/*	MidImgTest = test_threshold;
	MidImgTemplate = template_threshold;*/

	/*
	}
	else if(strcmp(in_fontNumber, "五号") == 0)
	{
	theApp.putLog("五号");
	return -1;
	}*/
	cv::Mat expandImgTest, expandImgTemplate;
	cv::resize(MidImgTemplate, expandImgTemplate, cv::Size(0, 0), 1.5, 1.5);
	cv::resize(MidImgTest, expandImgTest, cv::Size(0, 0), 2, 2);

	tempImage_1 = &IplImage(expandImgTemplate);
	testImage = &IplImage(expandImgTest);

	double value[3];
	value[0] = 0;
	value[1] = 0;
	value[2] = 0;
	vector<cv::Point> out_pt1;
	vector<cv::Point> out_pt2; //原图上的点
	iRet = compareFeatureValueMatchTextBlock(testImage, tempImage_1, value, out_pt1, out_pt2);
	int numFeature = value[0];
	int numMatch = value[2];
	int numTest = value[1];
	//模板或者测试区域没有特征点.
	if (iRet == -3 || iRet == -4)
	{
		int diffNum = abs(numTest - numFeature);
		if (iReserve & 1)
		{
			memset(buf_message, 0, sizeof buf_message);
			sprintf(buf_message, "模板或测试区没有特征,测试: %d, 模板: %d", numTest, numFeature);
			theApp.putLog(buf_message);
		}


		if (diffNum < theApp.keyPointsError)
		{
			return 0;
		}
		else
		{
			return -1;
		}
	}

	if (iRet != 0)
	{
		return -1;
	}

	double score = 0.0;
	if (numFeature > 0)
	{
		score = value[2] / value[0];
	}
	else
	{
		score = 0.0;
	}


	if (iReserve & 1)
	{
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "编号：%d %d, 模板的点数：%d , 匹配的点数:%d, 测试区间的点数：%d .占比：%.f", in_row, in_col, numFeature, numMatch, numTest, score * 100);
		theApp.putLog(buf_message);
	}

	if (iReserve & 1)
	{
		cv::Mat image2_color;
		cv::cvtColor(expandImgTest, image2_color, CV_GRAY2BGR);
		for (int i = 0; i < out_pt2.size(); i++)
		{
			cv::circle(image2_color, out_pt2[i], 3, cv::Scalar(0, 0, 255), 2, CV_FILLED);
		}
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_匹配结果图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image2_color);
	}

	//针对特征特别少的
	if (numFeature < 50 && numTest < 150 && numMatch > 0)
	{
		return 0;
	}

	int per = theApp.Percent;
	if ((score * 100) >= per)
	{
		iRet = 0;
	}
	else
	{
		iRet = -1;
	}

	//匹配完成后，如果成功的，再次比较特征点的分布情况
	//if(iRet == 0)
	//{
	//	cv::Mat blockImgTemp, blockImgTest;
	//	iRet = cut_imageROIBlock(A4_imageROI_Rotate, a4PrintImg, blockImgTest, blockImgTemp, in_row, in_col, iReserve);	
	//	if(blockImgTest.channels() == 3)
	//	{
	//		cv::cvtColor(blockImgTest, blockImgTest, CV_BGR2GRAY);
	//	}
	//	//比较每一个字的特征点数;

	//	int sum_test=0;
	//	int sum_temp=0;
	//	sum_test = bSums(blockImgTest, 0, iReserve);
	//	sum_temp = bSums(blockImgTemp, 1, iReserve);
	//}

	if (iReserve == 1)
	{
		DWORD t3 = GetTickCount() - ts;
		memset(buf_message, 0, sizeof buf_message);
		sprintf(buf_message, "验证文本块总时间： %d ms ", t3);
		theApp.putLog(buf_message);
	}
	return iRet;
}

//获取特征点的数目;
int calcFeaturePointNum(cv::Mat img)
{
	Ptr<FeatureDetector> detector = FeatureDetector::create("SIFT");
	vector<cv::KeyPoint> keypoints;
	detector->detect(img, keypoints);
	return keypoints.size();
}


//根据距离先过滤，过滤掉孤立的点
//保留密集的点
void removeErrorPoints(std::vector<cv::Point>& in_points, std::vector<CResultLabel>& labels)
{
	int max_distence = 50;
	int min_neighbour = 1;

	int num_maxNear = 0;
	int index_num_maxNear = 0;

	for (int i = 0; i < in_points.size(); i++)
	{
		int num_neighbour = 0;
		cv::Point pt1 = in_points[i];
		CResultLabel label;
		for (int j = 0; j < in_points.size(); j++)
		{
			if (i == j)
			{
				continue;
			}
			cv::Point pt2 = in_points[j];
			int distence = sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
			if (distence < max_distence)
			{
				num_neighbour++;
			}
		}
		if (num_neighbour >= min_neighbour)
		{

			label.isValid = true;
			label.num_neighbour = num_neighbour;
			labels.push_back(label);

			if (num_maxNear < num_neighbour)
			{
				num_maxNear = num_neighbour;
				index_num_maxNear = i;
			}
		}
		else
		{
			labels.push_back(label);
		}
	}

	labels[index_num_maxNear].isMaxConcentrated = true;

}


//根据旧的验证码，获取角度;
int getAngleByVerifyCode(cv::Mat image, double *out_angle, cv::Rect& out_verifyCodeZone, int iReserve)
{

	int iRet = -1;
	std::vector<cv::Point> matchPoints;
	char bufPathTemplatePath[1024];
	GetCurDir(bufPathTemplatePath);
	strcat(bufPathTemplatePath, "\\VerifyCodeImg\\templateOldVerifyCode.png");
	iRet = calcMatchedFeaturePoints(image, matchPoints, bufPathTemplatePath);

	//移除孤立的点
	std::vector<CResultLabel> out_labes;
	removeErrorPoints(matchPoints, out_labes);

	bool is_found_verifyCode = false;
	cv::Point pt_right;
	for (int i = 0; i < matchPoints.size(); i++)
	{
		if (out_labes[i].isValid == true && out_labes[i].isMaxConcentrated == true)
		{
			pt_right = matchPoints[i];
			is_found_verifyCode = true;
		}
	}

	cv::Rect roi_VerifyCode;
	if (is_found_verifyCode)
	{
		//先按照朝右放;
		roi_VerifyCode.x = pt_right.x - 100;
		roi_VerifyCode.y = pt_right.y - 50;
		roi_VerifyCode.width = 150;
		roi_VerifyCode.height = 380;

		out_verifyCodeZone = roi_VerifyCode;


		cv::Mat roiImage = image(roi_VerifyCode);
		//获取旋转角度.
		cv::Mat roi_bin;
		myThreshold(roiImage, roi_bin, 30);
		cv::threshold(roi_bin, roi_bin, 30, 255, CV_THRESH_BINARY_INV);

		if (iReserve & 1)
		{
			SYSTEMTIME st;
			GetSystemTime(&st);
			CString sFileName;
			sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪验证码区间.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
			//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
			cv::imwrite((LPSTR)(LPCTSTR)sFileName, roi_bin);
		}


		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(roi_bin, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		std::vector<cv::Point> pts;
		if (contours.size() > 0)
		{
			for (int i = 0; i < contours.size(); i++)
			{
				for (int j = 0; j < contours[i].size(); j++)
				{
					pts.push_back(contours[i][j]);
				}
			}
		}

		cv::RotatedRect rotaR = cv::minAreaRect(pts);
		cv::Point2f pt_arr[4];
		rotaR.points(pt_arr);

		cv::Mat color_show;
		cv::cvtColor(roiImage, color_show, CV_GRAY2BGR);

		cv::Scalar color_map[4];
		color_map[0] = cv::Scalar(255, 0, 0);
		color_map[1] = cv::Scalar(0, 255, 0);
		color_map[2] = cv::Scalar(0, 139, 139);
		color_map[3] = cv::Scalar(80, 10, 139);

		for (int j = 0; j < 4; j++)
		{
			line(color_show, pt_arr[j], pt_arr[(j + 1) % 4], cv::Scalar(0, 0, 255), 1, 8);
			//cv::circle(color_show, pt_arr[j], 3, color_map[j], 3);
		}

		//找到下方的二个点.	
		std::vector<cv::Point> sortByYPoints(pt_arr, pt_arr + 4);
		std::sort(sortByYPoints.begin(), sortByYPoints.end(), compareByY);
		cv::Point pt_r, pt_l;
		if (sortByYPoints[2].x > sortByYPoints[3].x)
		{
			pt_r = sortByYPoints[2];
			pt_l = sortByYPoints[3];
		}
		else
		{
			pt_l = sortByYPoints[2];
			pt_r = sortByYPoints[3];
		}

		int iRetAngle = -1;
		std::vector<cv::Point> pts_QR;
		pts_QR.push_back(pt_r);
		pts_QR.push_back(pt_l);
		pts_QR.push_back(cv::Point(0, 0));
		pts_QR.push_back(cv::Point(0, 0));

		double angle;
		iRetAngle = calcAngleByQR(pts_QR, &angle);
		*out_angle = angle;

		return 0;
	}
	return -1;
}


//这个接口，根据验证码的信息，做A4的裁剪
//先按照合同朝右放的
__declspec(dllexport)  int WINAPI RCN_CutImageByVerifyCode(DWORD dwImg, POINT in_point, char* in_verifyCode, int iReserve)
{
	//获取原始图片
	IplImage *pImg;
	pImg = (IplImage *)dwImg;
	cv::Mat image = cv::Mat(pImg, true);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image);
	}


	cv::Mat grey_image;
	cv::cvtColor(image, grey_image, CV_BGR2GRAY);

	int iRet = -1;
	double angle;
	cv::Rect  roi_verifyCode;
	//获取到角度
	iRet = getAngleByVerifyCode(grey_image, &angle, roi_verifyCode, iReserve);

	//旋转正原图片
	cv::Point center(pImg->width / 2, pImg->height / 2);
	CvPoint i_pt[4];
	CvPoint r_pt[4];
	i_pt[0] = cv::Point(0, 0);
	i_pt[1] = cv::Point(0, pImg->height);
	i_pt[2] = cv::Point(pImg->width, 0);
	i_pt[3] = cv::Point(pImg->width, pImg->height);
	IplImage* pImgDst = rotateImagexCore(pImg, angle, center, i_pt, r_pt);
	cv::Mat image_rotate = cv::Mat(pImgDst, true);
	cvReleaseImage(&pImgDst);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_A4旋转正图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_rotate);
	}
	//裁剪验证码区间
	cv::Mat roi_Img = image_rotate(roi_verifyCode);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_验证码区间图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, roi_Img);
	}
	//旋转正验证码
	cv::Mat roi_rotateImg;
	//逆时针进行旋转90
	transpose(roi_Img, roi_rotateImg);
	flip(roi_rotateImg, roi_rotateImg, 0);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_验证码旋转正图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, roi_rotateImg);
	}
	//生成标准的验证码


	return 0;
}

int GenerateVerifyCodeOld(std::string& verifyCodeMessage, cv::Mat& imageVerifyCode)
{


	int iPrinterType = 0;
	int iFontSize = 32;
	char szImageFile[1024] = { 0 };

	char szTradeCode[1024] = { 0 };
	memset(szTradeCode, 0, sizeof szTradeCode);
	memcpy(szTradeCode, verifyCodeMessage.c_str(), 12);

	char szDir[1024] = { 0 };
	GetCurDir(szDir);

	char *bufImg = new char[2048000];
	int ret = theApp.GenerateVerifyCodeToMemFile(iPrinterType, szTradeCode, iFontSize, bufImg, 2048000, 1, szDir, FALSE, 7);
	//FILE *stream;
	//if((stream = fopen("D:/a11.bmp", "wb")) != NULL)
	//{
	//	fwrite(bufImg, 1, ret, stream);
	//	fclose(stream);
	//}
	unsigned long pImgVerifyCode = RCN_CreateImage(bufImg, ret);
	imageVerifyCode = cv::Mat((IplImage *)pImgVerifyCode).clone();
	RCN_ReleaseImage(pImgVerifyCode);
	delete[]bufImg;
	return 0;
}







//识别旧的验证码
int WINAPI recognizeSealCodeOldVersion(cv::Mat img, std::string &result, std::string std_train_file, std::string real_train_file, std::vector<cv::Rect>& rects, int ocr_type = 0)
{
	if (img.empty())
	{
		result = "";
		return -1;
	}
	cv::Mat grey;
	if (img.channels() == 3)
	{
		cv::cvtColor(img, grey, CV_BGR2GRAY);
	}
	else
	{
		grey = img;
	}
	tesseract::TessBaseAPI api;
	char szDir[1024];
	memset(szDir, '\0', 1024);
	/*TCHAR szPath[MAX_PATH];
	if( !GetModuleFileName( NULL, szPath, MAX_PATH ) )
	{
	return NULL;
	}
	GetLongPathName(szPath, szDir, MAX_PATH);
	PathRemoveFileSpec(szDir);*/

	memcpy(szDir, (LPSTR)(LPCTSTR)theApp.m_pathModelFile, theApp.m_pathModelFile.GetLength());
	strcat(szDir, "\\Featurex");
	if (ocr_type == 0)
	{
		api.Init(szDir, std_train_file.c_str(), tesseract::OEM_TESSERACT_ONLY);
	}
	else
	{
		api.Init(szDir, real_train_file.c_str(), tesseract::OEM_TESSERACT_ONLY);
	}

	//解决字符序列倾斜时，无法识别的问题！
	api.SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);//设置自动进行版面分析//PSM_AUTO_OSD PSM_AUTO PSM_SINGLE_BLOCK PSM_SINGLE_BLOCK
	api.SetVariable("tessedit_char_whitelist", "0123456789ABCDEF[]");
	api.SetImage((uchar*)grey.data, grey.size().width, grey.size().height, grey.channels(), grey.step1());
	char *pr = api.GetUTF8Text();
#if 1
	tesseract::ResultIterator* ri = api.GetIterator();
	tesseract::PageIteratorLevel level = tesseract::RIL_SYMBOL;
	int h = img.size().height;
	int w = img.size().width;
	std::stringstream ss;
	ss.str("");
	if (ri != 0)
	{
		//theApp.putLog("获取矩形框开始");
		do {
			const char* word = ri->GetUTF8Text(level);
			if (word == NULL)
			{
				continue;
			}
			float conf = ri->Confidence(level);
			int x1, y1, x2, y2;
			ri->BoundingBox(level, &x1, &y1, &x2, &y2);
			cv::Rect rect(cv::Point(x1, y1), cv::Point(x2, y2));
			if (strcmp(word, " ") != 0 && (abs(y2 - y1) >= h / 6) && (abs(y2 - y1) <= h / 2) && (abs(x2 - x1) <= w / 7))
			{
				ss << word;
				rects.push_back(rect);
				if (rects.size() == 14)
				{
					break;
				}
			}
		} while (ri->Next(level));
		//theApp.putLog("获取矩形框结束");
		delete ri;
	}
	result = ss.str();
#endif


	return 0;
}


//识别旧的验证码;
int WINAPI compareSealCodeOldVersion(cv::Mat i_stand_image, cv::Mat i_real_image, int iReserve)
{
	int ret = -1;
	cv::Mat grey;
	if (i_stand_image.channels() == 3)
	{
		cv::cvtColor(i_stand_image, grey, CV_BGR2GRAY);
	}
	else
	{
		grey = i_stand_image;
	}

	cv::Mat grey_real;
	if (i_real_image.channels() == 3)
	{
		cv::cvtColor(i_real_image, grey_real, CV_BGR2GRAY);
	}
	else
	{
		grey_real = i_real_image;
	}

	std::string result;
	std::string std_train_file = "orient_old_std";
	std::string real_train_file = "orient_old_real";
	std::vector<cv::Rect> rect_std;

	ret = recognizeSealCodeOldVersion(grey, result, std_train_file, real_train_file, rect_std);
	//去掉回车，空格
	std::string result_clear;
	removeSpaceEnter(result, result_clear);
	std::string out_log = "标准验证码识别的结果：";
	out_log += result;
	if (iReserve == 1)
	{
		theApp.putLog((char*)out_log.c_str());
		theApp.putLog((char*)result_clear.c_str());
	}
	std::string result_real;
	std::string result_real_clear;
	std::vector<cv::Rect> rect_real;
	ret = recognizeSealCodeOldVersion(grey_real, result_real, std_train_file, real_train_file, rect_real, 1);
	out_log = "裁剪的验证码识别结果为：";
	out_log += result_real;
	removeSpaceEnter(result_real, result_real_clear);
	if (iReserve == 1)
	{
		theApp.putLog((char*)out_log.c_str());
		theApp.putLog((char*)result_real_clear.c_str());
	}
	//字符判定
	if (result_clear != result_real_clear)
	{
		if (iReserve == 1)
		{
			theApp.putLog("验证码字符串与模板不一样");
		}
		return -1;
	}

	if (iReserve == 1)
	{
		theApp.putLog("验证码字符串与模板一样");
	}
	//再检验，识别图字符的间隔的比例，与模板的字符间隔比!
	bool match_bool = MatchDistence(rect_std, rect_real);
	if (match_bool == false)
	{
		if (iReserve == 1)
		{
			theApp.putLog("验证码的字符间隔匹配失败");
		}
		return -1;
	}
	if (iReserve == 1)
	{
		theApp.putLog("验证码的字符间隔匹配成功");
	}
	return 0;
}




bool compareByRectX(cv::Rect rect1, cv::Rect rect2)
{
	return	rect1.x < rect2.x;
}

//针对老验证码裁剪
//切割出只有字符和【】的区间
//in_image: 灰度图。
//out_image: 输出的裁剪区间
int cutOffVerifyCodePartByOldVersion(cv::Mat in_image, cv::Mat& out_image, int iReserve)
{
	cv::Mat image = in_image.clone();
	cv::Mat img_threshold;
	cv::threshold(image, img_threshold, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
	cv::Mat elem = cv::getStructuringElement(CV_8UC1, cv::Size(3, 3));
	//cv::erode(templateImgBin, templateImgBin, elem);
	cv::dilate(img_threshold, img_threshold, elem, cv::Point(-1, -1), 2);

	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, img_threshold);
	}
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(img_threshold, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	std::vector<cv::Rect> candiateRect;
	cv::Mat showImg;
	cv::cvtColor(image, showImg, CV_GRAY2BGR);
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Rect rect;
		rect = cv::boundingRect(contours[i]);
		if (rect.height < 10)
		{
			continue;
		}
		cv::rectangle(showImg, rect, cv::Scalar(0, 255, 0), 2);
		candiateRect.push_back(rect);
	}
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区字符分布图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, showImg);
	}

	std::sort(candiateRect.begin(), candiateRect.end(), compareByRectX);
	//移除前面三个;
	std::vector<cv::Rect> avaliableRect;
	for (int i = 3; i < candiateRect.size(); i++)
	{
		avaliableRect.push_back(candiateRect[i]);
	}
	//cv::Mat showImg2;
	//cv::cvtColor(image, showImg2, CV_GRAY2BGR);
	//for(int i=0; i<avaliableRect.size(); i++)
	//{
	//	cv::rectangle(showImg2, avaliableRect[i], cv::Scalar(0, 255, 0), 2);
	//}
	//cv::imshow("showImg2", showImg2);
	//cv::waitKey();
	//切割出只有数字和【】部分;
	cv::Rect roiRect;
	roiRect.x = avaliableRect[0].x - 10;
	roiRect.y = 0;
	roiRect.width = image.cols - roiRect.x;
	roiRect.height = image.rows;
	cv::Mat image_roi;
	image_roi = image(roiRect);
	out_image = image_roi.clone();
	//cv::imshow("roiRect", image_roi);
	//cv::waitKey();
	return 0;
}


//功能： 识别旧版本的验证码.
//参数简解：dwImg, 原始图片的地址
//in_point，验证码左上角的位置.
//	in_verifyCode, 输入的12位的验证码.
//	iReserve, 调试接口, 一般为0
__declspec(dllexport)  int WINAPI RCN_VerifyCodeOldVersion(DWORD dwImg, POINT in_point, char* in_verifyCode, int iReserve)
{
	if (!dwImg || !in_verifyCode || strlen(in_verifyCode) != 12)
	{
		return -1;
	}
	int iRet = -1;
	int BlockSize = 20;

	//iReserve = 2;

	//iRet = isValidPosition(in_blockNumber);
	//if(iRet != 0)
	//{
	//	return 1;
	//}
	char buf_message[1024];
	DWORD ts = GetTickCount();
	//获取原始图片
	IplImage *pImg;
	pImg = (IplImage *)dwImg;

	cv::Mat image = cv::Mat(pImg, true);
	//cv::resize(image, image, cv::Size(0, 0), m_scale, m_scale);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原始图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image);
	}

	if (image.channels() == 3)
	{
		cv::cvtColor(image, image, CV_BGR2GRAY);
	}

	//进行二维码识别, 做合同的裁剪;
	std::string out_recognize;
	std::vector<cv::Point> pointQR;
	iRet = theApp.QR_Recognize(image, out_recognize, pointQR);
	if (iReserve == 1)
	{
		theApp.putLog("二维码信息:");
		theApp.putLog((char*)out_recognize.c_str());
	}

	if (iRet != 1 || out_recognize.size() == 0 || pointQR.size() != 4)
	{
		return -1;
	}
	int ret_angle = 0;
	double angle = 0.0;
	ret_angle = calcAngleByQR(pointQR, &angle);
	if (iReserve == 1)
	{
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "angle: %f", angle);
		theApp.putLog(buf_message);
	}

	//对整张图片旋转angle 角度
	cv::Point center(pImg->width / 2, pImg->height / 2);
	CvPoint i_pt[4];
	CvPoint r_pt[4];
	i_pt[0] = cv::Point(0, 0);
	i_pt[1] = cv::Point(0, pImg->height);
	i_pt[2] = cv::Point(pImg->width, 0);
	i_pt[3] = cv::Point(pImg->width, pImg->height);
	IplImage* pImgDst = rotateImagexCore(pImg, angle, center, i_pt, r_pt);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图的旋转图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImgDst);
	}
	cv::Mat image_rotate = cv::Mat(pImgDst, true);
	cvReleaseImage(&pImgDst);
	//////////////////////////
	out_recognize = "";
	pointQR.clear();

	if (image_rotate.channels() == 3)
	{
		cv::cvtColor(image_rotate, image_rotate, CV_BGR2GRAY);
	}

	iRet = theApp.QR_Recognize(image_rotate, out_recognize, pointQR);
	if (iRet != 1 || out_recognize.size() == 0 || pointQR.size() != 4)
	{
		return -1;
	}

#if 0
	int width_A4 = theApp.m_A4Width;
	int height_A4 = theApp.m_A4Height;
#elif 1
	int width_A4 = 297 * theApp.m_pixNumPerMm;
	int height_A4 = 210 * theApp.m_pixNumPerMm;
#endif
	cv::Mat A4_imageROI_Rotate;
	//先处理文本	， 文本的相对于二维码的坐标位置。
	cv::Point point;

	//区分方向
	if (pointQR[0].x > pointQR[1].x)
	{
		//朝右
		int rightEdge = theApp.m_QRRight*theApp.m_pixNumPerMm;
		int downEdge = theApp.m_QRDown*theApp.m_pixNumPerMm;
		cv::Rect A4_ROI;
		A4_ROI.x = pointQR[0].x + rightEdge - width_A4;
		A4_ROI.y = pointQR[0].y + downEdge - height_A4;
		A4_ROI.width = width_A4;
		A4_ROI.height = height_A4;
		cv::Mat A4_imageROI = image_rotate(A4_ROI);
		//逆时针转90
		transpose(A4_imageROI, A4_imageROI_Rotate);
		flip(A4_imageROI_Rotate, A4_imageROI_Rotate, 0);
	}
	else
	{
		//朝左
		int leftEdge = theApp.m_QRRight*theApp.m_pixNumPerMm;
		int upEdge = theApp.m_QRDown*theApp.m_pixNumPerMm;
		cv::Rect A4_ROI;
		A4_ROI.x = pointQR[0].x - leftEdge;
		A4_ROI.y = pointQR[0].y - upEdge;
		A4_ROI.width = width_A4;
		A4_ROI.height = height_A4;
		cv::Mat A4_imageROI = image_rotate(A4_ROI);
		transpose(A4_imageROI, A4_imageROI_Rotate);
		flip(A4_imageROI_Rotate, A4_imageROI_Rotate, 1);
	}
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间的旋正图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, A4_imageROI_Rotate);
	}
	//依据验证码接口，提供
	double scale_rect = theApp.m_pixNumPerMm / 7.1;
	cv::Rect rect_verifyCode;
	rect_verifyCode.x = in_point.x*theApp.m_pixNumPerMm - 10;
	rect_verifyCode.y = in_point.y*theApp.m_pixNumPerMm - 10;
	rect_verifyCode.width = 350 * scale_rect;
	rect_verifyCode.height = 91 * scale_rect;

	if (rect_verifyCode.x < 0)
	{
		rect_verifyCode.x = 0;
	}
	if ((rect_verifyCode.x + rect_verifyCode.width) > A4_imageROI_Rotate.cols)
	{

		rect_verifyCode.width = A4_imageROI_Rotate.cols - rect_verifyCode.x - 3;
	}


	if (rect_verifyCode.y < 0)
	{
		rect_verifyCode.y = 0;
	}
	if ((rect_verifyCode.y + rect_verifyCode.height) > A4_imageROI_Rotate.rows)
	{

		rect_verifyCode.height = A4_imageROI_Rotate.rows - rect_verifyCode.y - 3;
	}


	if (rect_verifyCode.x < 0 || rect_verifyCode.y < 0 ||
		((rect_verifyCode.x + rect_verifyCode.width) > A4_imageROI_Rotate.size().width) ||
		((rect_verifyCode.y + rect_verifyCode.height) > A4_imageROI_Rotate.size().height) ||
		((rect_verifyCode.x + rect_verifyCode.width) < 0) ||
		((rect_verifyCode.y + rect_verifyCode.height) < 0)
		)
	{
		return -1;
	}

	cv::Mat roi_image;
	roi_image = A4_imageROI_Rotate(rect_verifyCode);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区间图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, roi_image);
	}
	cv::Mat image_std;
	std::string verifyCodeMessage = in_verifyCode;
	GenerateVerifyCodeOld(verifyCodeMessage, image_std);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_验证码图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_std);
	}
	//去掉裁剪区间的“验证码”区间
	cv::Mat roi_image_cut;
	cutOffVerifyCodePartByOldVersion(roi_image, roi_image_cut, iReserve);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_验证码切割图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, roi_image_cut);
	}
	//识别接口	
	iRet = compareSealCodeOldVersion(image_std, roi_image_cut, iReserve);
	return iRet;
}

//针对直方图比对,的切割;
int cut_imageROIBlock(cv::Mat inTestImg, cv::Mat inTemplateImg, cv::Mat& outTestROI, cv::Mat& outTemplateROI, int inRow, int inCol, int inDebug)
{

	int iRet = -1;
	cv::Mat imgCutWhite;
	cv::Rect roiText;
	cv::Size blockSize;
	blockSize.width = inTemplateImg.size().width / 8;
	blockSize.height = inTemplateImg.size().height / 16;

	roiText.x = inTemplateImg.size().width / 4 + (inCol - 1) * blockSize.width;
	roiText.y = inTemplateImg.size().height / 8 + (inRow - 1) * blockSize.height;
	roiText.width = blockSize.width;
	roiText.height = blockSize.height;

	try
	{
		cv::Mat roiImgTextTemplate;
		roiImgTextTemplate = inTemplateImg(roiText);

		cv::Mat roiImgTextTest;
		roiImgTextTest = inTestImg(roiText);

		outTestROI = roiImgTextTest.clone();
		outTemplateROI = roiImgTextTemplate.clone();
		iRet = 0;
	}
	catch (...)
	{
		iRet = -1;
	}
	return iRet;
}

//针对特征匹配,测试图的切割要扩大点;
int cut_imageROIBlockFeature(cv::Mat inTestImg, cv::Mat inTemplateImg, cv::Mat& outTestROI, cv::Mat& outTemplateROI, int inRow, int inCol, int inDebug)
{

	int iRet = -1;
	cv::Mat imgCutWhite;
	cv::Rect roiText;
	cv::Size blockSize;
	blockSize.width = inTemplateImg.size().width / 8;
	blockSize.height = inTemplateImg.size().height / 16;

	roiText.x = inTemplateImg.size().width / 4 + (inCol - 1) * blockSize.width;
	roiText.y = inTemplateImg.size().height / 8 + (inRow - 1) * blockSize.height;
	roiText.width = blockSize.width;
	roiText.height = blockSize.height;

	try
	{
		cv::Mat roiImgTextTemplate;
		roiImgTextTemplate = inTemplateImg(roiText);

		cv::Mat roiImgTextTest;
		roiText.x -= 10;
		roiText.y -= 10;
		roiText.width += 20;
		roiText.height += 20;
		roiImgTextTest = inTestImg(roiText);
		outTestROI = roiImgTextTest.clone();
		outTemplateROI = roiImgTextTemplate.clone();
		iRet = 0;
	}
	catch (...)
	{
		iRet = -1;
	}
	return iRet;
}




/**********************************************************************/
// 用于设置识别算法参数
int RCN_SetAlgorithmParam(SALOGRITHMPARAM algoParam)
{
	theApp.min_threshold = algoParam.Threshold;
	//theApp.m_pixNumPerMm = algoParam.PixNum;
	theApp.m_pixNumPerMm = (double)(algoParam.XScaleplate + algoParam.YScaleplate)/80;
	theApp.Bright = algoParam.Bright;
	theApp.m_iXScaleplate = algoParam.XScaleplate;
	theApp.m_iYScaleplate = algoParam.YScaleplate;
	theApp.m_idebug = algoParam.Debug;
	theApp.m_pathModelFile = algoParam.szPathModel;
	theApp.m_modelFileNameHead = algoParam.szModelNameHead;
	theApp.m_modelFileNameBranch = algoParam.szModelNameBranch;

	if(theApp.m_modelFileNameHead.GetLength() == 0)
	{
		theApp.m_modelFileNameHead = "DfjyModelHead.dat";
	}
	if(theApp.m_modelFileNameBranch.GetLength() == 0)
	{
		theApp.m_modelFileNameBranch = "DfjyModelBranch.dat";
	}
	if (theApp.m_idebug & 1)
	{
		char buf_message[1024];
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "threshold： %d, pixNum: %.3f, Bright:%d, xScale:%d, yScale:%d, mDebug:%d, path:%s",
			theApp.min_threshold, theApp.m_pixNumPerMm, theApp.Bright, theApp.m_iXScaleplate,
			theApp.m_iYScaleplate, theApp.m_idebug, (LPSTR)(LPCTSTR)theApp.m_pathModelFile);
		theApp.putLog(buf_message);
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "总行的名： %s, 分行的模板名: %s", theApp.m_modelFileNameHead, theApp.m_modelFileNameBranch);
		theApp.putLog(buf_message);
	}

	return 0;
}

/**********************************************************************/
// 用于获取识别算法参数
__declspec(dllexport)  int WINAPI RCN_GetAlgorithmParam(SALOGRITHMPARAM& algoParam)
{
	algoParam.Threshold = theApp.min_threshold;
	algoParam.PixNum = theApp.m_pixNumPerMm;
	algoParam.Bright = theApp.Bright;
	algoParam.XScaleplate = theApp.m_iXScaleplate;
	algoParam.YScaleplate = theApp.m_iYScaleplate;
	algoParam.Debug = theApp.m_idebug;
	strcpy(algoParam.szPathModel, (LPSTR)(LPCTSTR)theApp.m_pathModelFile);
	return 0;
}


//初始化成功后，再加载相应的模板文件
__declspec(dllexport)  int WINAPI RCN_LoadTemplate(SALOGRITHMPARAM algoParam)
{
	RCN_SetAlgorithmParam(algoParam);
	//判断esm.ini是否存在;如果存在，则读取esm.ini的debug值;
	if (theApp.m_pathModelFile.IsEmpty())
	{
		theApp.m_pathModelFile = theApp.m_strExePath;
	}
	int ret_fileExist = PathFileExists(theApp.m_pathModelFile + "\\Featurex\\esm.ini");
	if (ret_fileExist)
	{
		char winDir[1024];
		memset(winDir, 0, sizeof(winDir));
		sprintf_s(winDir, "%s\\Featurex\\esm.ini", (LPSTR)(LPCTSTR)theApp.m_pathModelFile);	
		theApp.m_idebug = GetPrivateProfileInt("Camera", "Debug", 0, winDir);
		theApp.Bright =  GetPrivateProfileInt("Camera", "Bright", 0, winDir);
		theApp.min_threshold = GetPrivateProfileInt("Algorithm", "Threshold", 80, winDir);
		theApp.m_iXScaleplate = GetPrivateProfileInt("Camera", "XScaleplate", 337, winDir);
		theApp.m_iYScaleplate = GetPrivateProfileInt("Camera", "YScaleplate", 335, winDir);
		theApp.m_CameraType = GetPrivateProfileInt("Camera", "CameraType", 0, winDir);
	}

	if (theApp.m_idebug != 0)
	{
		SHCreateDirectoryEx(NULL, theApp.m_strPath, NULL);

		//预审批结果目录
		SHCreateDirectoryEx(NULL, theApp.m_pathPreAdj, NULL);
	}


	//先加载总行的;
	theApp.IniImgConfig( "Head/" + theApp.m_modelFileNameHead);
	//再加载分行的;
	theApp.IniImgConfig( "Branch/" + theApp.m_modelFileNameBranch);
	return 0;
}





bool compareRectX(CvRect a, CvRect b)
{
	return a.x < b.x;
}

bool compareRectX2(cv::Rect a, cv::Rect b)
{
	return a.x < b.x;
}

bool compareRectY(CvRect a, CvRect b)
{
	return a.y < b.y;
}

bool compareRectY2(cv::Rect a, cv::Rect b)
{
	return a.y < b.y;
}

bool compareRectArea(CvRect a, CvRect b)
{
	int area_a = a.width*a.height;
	int area_b = b.width*b.height;
	return area_a > area_b;
}


bool compareRectArea2(cv::Rect a, cv::Rect b)
{
	return a.area()> b.area();
}


bool compareRectHeight(cv::Rect a, cv::Rect b)
{
	return a.height < b.height;
}


//cvSmooth函数平滑滤波; cvErode腐蚀，cvDilate膨胀，这两个形态学函数总是成对出现，前者可以消除较小独点如噪音，后者可以使不连通的图像合并成块。
vector<IplImage*>  mycvCharSegment(IplImage* image, IplImage *img2, int iflag, int iReserve)
{
	vector<IplImage*> characters;
	IplImage* smoothImg = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	cvCopy(image, smoothImg);
	//cvNamedWindow("膨胀图", CV_WINDOW_AUTOSIZE); 
	//cvShowImage("膨胀图", smoothImg);  
	//waitKey(0);

	IplImage *pBak = cvCreateImage(cvGetSize(smoothImg), IPL_DEPTH_8U, 1);
	cvCopy(smoothImg, pBak);
	CvSeq* contours = NULL;
	CvMemStorage* storage = cvCreateMemStorage(0);
	int count = cvFindContours(smoothImg, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	vector<CvRect> comrcs;
	bool bb = false;
	CvHistogram* his = NULL;
	for (CvSeq* c = contours; c != NULL; c = c->h_next)
	{
		CvRect rc = cvBoundingRect(c, 0);
		//if((iflag == 1) && (rc.height < 3 || rc.width < 8))
		if (rc.height < 160 || rc.width < 160 || rc.width >(image->width / 2) || rc.height >(image->height / 2))
			continue;

		comrcs.push_back(rc);
	}
	// 1. 水平
	// 0 竖直
	if (iflag == 1)
		sort(comrcs.begin(), comrcs.end(), compareRectY);
	else
		sort(comrcs.begin(), comrcs.end(), compareRectX);

	vector<CvRect>::iterator itr, itr_next;
	int icc = 1;
	CString ss;
	int isize = comrcs.size();
	ss.Format("总数： %d", isize);
	//AfxMessageBox(ss);
	for (itr = comrcs.begin(); itr != comrcs.end(); itr++)
	{
		CvRect rc = *itr;
		cvDrawRect(img2, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(255, 255, 255));

		IplImage* imgNo = cvCreateImage(cvSize(rc.width, rc.height), IPL_DEPTH_8U, 1);
		cvSetImageROI(pBak, rc);
		cvCopy(pBak, imgNo, 0);
		cvResetImageROI(pBak);
		cvResetImageROI(imgNo);

		IplImage* dst = cvCreateImage(cvSize(rc.width, rc.height), IPL_DEPTH_8U, 1);
		cvResize(imgNo, dst);
		characters.push_back(dst);
		//ss.Format("E:\\work\\TestOpenCV\\pic\\33\\img%02d.jpg", icc++); 
		//cvSaveImage((LPSTR)(LPCTSTR)ss, dst);

	}
	//cvNamedWindow("字符分割", CV_WINDOW_AUTOSIZE);  
	//cvShowImage("字符分割", img2);  
	//waitKey();
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_DPI裁取计算DPI目标图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, img2);
	}
	cvReleaseImage(&pBak);
	cvReleaseImage(&smoothImg);
	return characters;
}

//输入的参数为灰度图;
int  CalcDPIInner(cv::Mat in_image, int& num_width_pixel, int& num_height_pixel, int iReserve)
{
	vector<IplImage*>  p;

	int BlockSize = 10;
	//std::string path = "E:\\work\\TestOpenCV\\pic\\33\\";
	//cv::Mat img= cv::imread(path + "33.jpg", 0);
	cv::Mat img = in_image;
	//IplImage* image = cvCreateImage(cvSize(img.rows, img.cols), 8, 1);  
	IplImage image;
	cv::Mat bin_image;
	//myThreshold(img, bin_image, BlockSize); 

	int blockSize = 25;
	int constValue = 10;
	cv::Mat local;
	cv::adaptiveThreshold(img, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);

	//cv::imshow("111", bin_image);
	image = IplImage(bin_image);

	//cvNamedWindow("二值图", CV_WINDOW_AUTOSIZE);  
	//cvShowImage("二值图", image); 
	//waitKey(0);
	//IplImage *img2 = cvCreateImage(cvSize(img.rows, img.cols), 8, 1);  
	IplImage img2;
	img2 = IplImage(img);
	p = mycvCharSegment(&image, &img2, 1, iReserve);
	if (p.size() <= 0)
	{
		return -2;
	}
	IplImage* imageROI_0 = p[0];
	num_width_pixel = imageROI_0->width;
	num_height_pixel = imageROI_0->height;

	//释放无用的空间;
	vector<IplImage*>::iterator ita;
	for (ita = p.begin(); ita != p.end(); ita++)
	{
		IplImage* pdst = *ita;
		cvReleaseImage(&pdst);
	}
	p.clear();
	return 0;
}


//自动计算DPI
//dwImg: 输入参数 获取图片的地址;
//dpiX: 输出参数 X方向的dpi;
//dpiY: 输出参数 Y方向的dpi;
// 返回值: -1: 裁剪错误;
//         -2: 检测矩形失败;
//         0：成功
__declspec(dllexport)  int WINAPI RCN_CalacDPI(DWORD dwImg, int& dpiX, int& dpiY, int iReserve)
{
	iReserve = theApp.m_idebug;
	cv::Mat imageCut;
	int iRet = RCN_GetCutPictureToMemOne(dwImg, imageCut);
	if (iRet != 0)
	{
		return -1;
	}

	if(imageCut.size().width < 300 || imageCut.size().height < 300)
	{
		return -1;
	}

	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_DPI裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageCut);
	}
	cv::Mat imageCutGrey;
	if (imageCut.channels() == 3)
	{
		cv::cvtColor(imageCut, imageCutGrey, CV_BGR2GRAY);
	}
	else
	{
		imageCutGrey = imageCut;
	}
	int width_pixel = 0;
	int height_pixel = 0;

	cv::Rect cutBlackLineZone;
	cutBlackLineZone.x = 20;
	cutBlackLineZone.y = 20;
	cutBlackLineZone.width = imageCutGrey.size().width - 40;
	cutBlackLineZone.height = imageCutGrey.size().height - 40;
	imageCutGrey = imageCutGrey(cutBlackLineZone);	

	iRet = CalcDPIInner(imageCutGrey, width_pixel, height_pixel, iReserve);
	if (iRet != 0)
	{
		return -2;
	}
	dpiX = width_pixel;
	dpiY = height_pixel;
	return 0;
}



//增加识别联数相关的接口函数;
//cvSmooth函数平滑滤波; cvErode腐蚀，cvDilate膨胀，这两个形态学函数总是成对出现，前者可以消除较小独点如噪音，后者可以使不连通的图像合并成块。
vector<IplImage*>  mycvCharSegmentUnite(IplImage* image, IplImage *img2, int iflag, int iReserve)
{
	//theApp.putLog("mycvCharSegmentUnite....start");

	vector<IplImage*> characters;
	IplImage* smoothImg = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	cvCopy(image, smoothImg);
	//cvNamedWindow("膨胀图", CV_WINDOW_AUTOSIZE); 
	//cvShowImage("膨胀图", smoothImg);  
	//waitKey(0);
	//cv::Mat elem = cv::getStructuringElement(MORPH_RECT, cv::Size(2, 2));
	//cv::Mat result_erode;
	//cv::dilate(cv::Mat(image), result_erode, elem);
	IplConvKernel * myModel;
	myModel = cvCreateStructuringElementEx(2, 2, 1, 1, CV_SHAPE_RECT);
	cvDilate(smoothImg, smoothImg, myModel);
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_联数膨胀图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, smoothImg);
	}

	IplImage *pBak = cvCreateImage(cvGetSize(smoothImg), IPL_DEPTH_8U, 1);
	cvCopy(smoothImg, pBak);
	CvSeq* contours = NULL;
	CvMemStorage* storage = cvCreateMemStorage(0);
	int count = cvFindContours(smoothImg, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	int width_image = image->width;
	int height_image = image->height;

	char bufMessage[1024];
	vector<CvRect> comrcs, comrcs_clear;
	bool bb = false;
	CvHistogram* his = NULL;
	for (CvSeq* c = contours; c != NULL; c = c->h_next)
	{
		//memset(bufMessage, 0, 1024);
		CvRect rc = cvBoundingRect(c, 0);
		double area_char = rc.width * rc.height;

		//sprintf(bufMessage, "area： %.3f", area_char);
		//theApp.putLog(bufMessage);

		//if((iflag == 1) && (rc.height < 3 || rc.width < 3))
		//	continue;
		//if((iflag == 0) && (rc.height < 3 || rc.width < 3))
		//	continue;
		if (((iflag == 1) && (rc.height < 3 || rc.width < 3 || rc.width > width_image / 2)) || area_char < 30)
			continue;
		if (((iflag == 0) && (rc.height < 3 || rc.width < 3 || rc.height > height_image / 2)) || area_char < 30)
			continue;
		comrcs.push_back(rc);
	}
	//过滤下，被包含的矩形；
	int num_contour = comrcs.size();
	for (int i = 0; i < num_contour; i++)
	{
		cv::Point pt;
		pt.x = comrcs[i].x + (comrcs[i].width) / 2;
		pt.y = comrcs[i].y + (comrcs[i].height) / 2;
		bool isContain = false;
		for (int j = 0; j < num_contour; j++)
		{
			if (i == j)
			{
				continue;
			}
			bool contion = pt.x > comrcs[j].x && pt.x < (comrcs[j].x + comrcs[j].width) && pt.y > comrcs[j].y && pt.y < (comrcs[j].y + comrcs[j].height);
			if (contion)
			{
				isContain = true;
				break;
			}

		}
		if (isContain == false)
		{
			comrcs_clear.push_back(comrcs[i]);
		}
	}
	// 1. 水平
	// 0 竖直
	if (iflag == 0)
		sort(comrcs_clear.begin(), comrcs_clear.end(), compareRectY);
	else
		sort(comrcs_clear.begin(), comrcs_clear.end(), compareRectX);

	vector<CvRect>::iterator itr, itr_next;
	int icc = 1;
	CString ss;
	int isize = comrcs_clear.size();
	ss.Format("总数： %d", isize);
	//AfxMessageBox(ss);

	//再过滤下，二个大矩形的外面的杂质;
	vector<CvRect> rectArea;
	sort(comrcs_clear.begin(), comrcs_clear.end(), compareRectArea);
	//排完序，面积有大到小;
	if (comrcs_clear.size() < 3)
	{
		characters.clear();
		return characters;
	}
	int y_small = 0, y_max = 0;
	int x_small = 0, x_max = 0;

	if (iflag == 0)
	{
		//竖直
		int y_start = comrcs_clear[0].y;
		int y_end = comrcs_clear[1].y;
		if (y_start < y_end)
		{
			y_small = y_start + comrcs_clear[0].height;
			y_max = y_end;
		}
		else
		{
			y_small = y_end + comrcs_clear[1].height;
			y_max = y_start;
		}
	}
	else
	{
		//水平
		int x_start = comrcs_clear[0].x;
		int x_end = comrcs_clear[1].x;
		if (x_start < x_end)
		{
			x_small = x_start + comrcs_clear[0].width;
			x_max = x_end;
		}
		else
		{
			x_small = x_end + comrcs_clear[1].width;
			x_max = x_start;
		}
	}

	//筛选出'第'与'联',之间的轮廓;
	for (itr = comrcs_clear.begin(); itr != comrcs_clear.end(); itr++)
	{
		CvRect rc = *itr;
		if (iflag == 0)
		{
			if (rc.y <= y_small || rc.y >= y_max)
				continue;
		}
		else
		{
			if (rc.x <= (x_small - 3) || rc.x >= x_max)
				continue;
		}
#ifdef _DEBUG
		cvDrawRect(img2, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(255, 255, 255));
#endif
		//cvDrawRect(img2, cvPoint(rc.x, rc.y), cvPoint(rc.x + rc.width, rc.y + rc.height), CV_RGB(255, 255, 255)); 

		IplImage* imgNo = cvCreateImage(cvSize(rc.width, rc.height), IPL_DEPTH_8U, 1);
		cvSetImageROI(pBak, rc);
		cvCopy(pBak, imgNo, 0);
		cvResetImageROI(pBak);
		cvResetImageROI(imgNo);

		IplImage* dst = cvCreateImage(cvSize(rc.width, rc.height), IPL_DEPTH_8U, 1);
		cvResize(imgNo, dst);
		characters.push_back(dst);
		cvReleaseImage(&imgNo);
		//ss.Format("E:\\work\\TestOpenCV\\pic\\33\\img%02d.jpg", icc++); 
		//cvSaveImage((LPSTR)(LPCTSTR)ss, dst);

	}
	//cvNamedWindow("字符分割", CV_WINDOW_AUTOSIZE);  
	//cvShowImage("字符分割", img2);  
	//waitKey();
	if (iReserve == 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_联数字符切割图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, img2);
	}
	cvReleaseImage(&pBak);
	cvReleaseImage(&smoothImg);

	//theApp.putLog("mycvCharSegmentUnite....end");
	return characters;
}

//输入的参数为灰度图;
int  CalcUniteNumInner(cv::Mat in_image, int& out_uniteNum, int iReserve)
{
	char buf_message[1024];
	memset(buf_message, 0, 1024);
	vector<IplImage*>  p, p_clear;
	int BlockSize = 10;
	cv::Mat img = in_image;
	//IplImage* image = cvCreateImage(cvSize(img.rows, img.cols), 8, 1);
	//对图像进行增强;

#if 1
	IplImage* image2 = cvCreateImage(cvSize(img.cols, img.rows), 8, 1);
	//*image2 = IplImage(img);
	cvCopy(&IplImage(img), image2);
	BYTE* pData = (BYTE *)image2->imageData;
	BYTE  bByte;
	cv::Scalar scalar = cv::mean(img);
	double dScalar = scalar[0];
	int istep = image2->widthStep / sizeof(BYTE);
	//theApp.putLog("图片增强开始");
	for (int i = 0; i < image2->height; i++)
	{
		for (int j = 0; j < image2->width; j++)
		{
			bByte = pData[i*istep + j];
#if 0
			//农行标准机器
			if (bByte >= (BYTE)dScalar - 5)
#elif 1
			//临时用的
			if (bByte >= (BYTE)dScalar - 2)
#endif
			{
				pData[i*istep + j] = 255;
			}
		}
	}
	//theApp.putLog("图片增强结束");
	cv::Mat image_improve;
	image_improve = cv::Mat(image2, true);
	cvReleaseImage(&image2);
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_联数增强图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, img2);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_improve);
	}
#endif

	IplImage image;
	cv::Mat bin_image;

	int blockSize = 25;
	int constValue = 10;

	cv::Mat local;
	//cv::adaptiveThreshold(img, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	cv::adaptiveThreshold(image_improve, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_联数二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, img2);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, bin_image);
	}

	image = IplImage(bin_image);
	//IplImage *img2 = cvCreateImage(cvSize(img.rows, img.cols), 8, 1);  
	IplImage img2;
	img2 = IplImage(img);

	if (in_image.cols > in_image.rows)
	{
		p = mycvCharSegmentUnite(&image, &img2, 1, iReserve);
	}
	else
	{
		p = mycvCharSegmentUnite(&image, &img2, 0, iReserve);
	}

	int num_contour = p.size();

	if (num_contour == 1)
	{
		//区分1 / 4
		IplImage* tmpSmallImage = p[0];
		float scale = (((float)(tmpSmallImage->width)) / (tmpSmallImage->height));
		sprintf(buf_message, "1/4 scale: %.2f", scale);
		theApp.putLog(buf_message);
		if (scale > 0.4 && scale < 1.5)
		{
			//胖的
			out_uniteNum = 4;
		}
		else
		{
			//瘦的
			out_uniteNum = 1;
		}
	}
	else if (num_contour == 0)
	{
		//针对间距特别小，且有印色的;
		out_uniteNum = 4;
	}
	else
	{
		out_uniteNum = num_contour;
		if (out_uniteNum > 4 || out_uniteNum < 1)
		{
			out_uniteNum = 0;
		}
	}
	//释放无用的空间;
	vector<IplImage*>::iterator ita;
	for (ita = p.begin(); ita != p.end(); ita++)
	{
		IplImage* pdst = *ita;
		cvReleaseImage(&pdst);
	}
	p.clear();
	return 0;
}


//计算联数;
//in_image: 裁剪好的 “第几联的部分”
//in_style_unite: 联的风格, 横/纵, 预留参数
//in_direct_unite: 联的方向, 上/下/左/右, 预留参数
//out_unite_num： 输出的联数
//iReserve: 调试接口 
//返回值： 错误类型， 0 正常;
int RCN_CalacUniteNum(cv::Mat in_image, int in_style_unite, int in_direct_unite, int& out_unite_num, int iReserve)
{
	//theApp.putLog("calc unite start......");

	DWORD t1_start = GetTickCount();
	iReserve = theApp.m_idebug;
	if (in_image.data == NULL)
	{
		return -8;
	}
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_联数裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, BImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, in_image);
	}
	cv::Mat grey_image;
	if (in_image.channels() == 3)
	{
		cv::cvtColor(in_image, grey_image, CV_BGR2GRAY);
	}
	else
	{
		grey_image = in_image;
	}
	int iRet = -1;
	int unite_num = 0;
	iRet = CalcUniteNumInner(grey_image, unite_num, iReserve);
	if (iRet == 0)
	{
		out_unite_num = unite_num;
	}
	else
	{
		out_unite_num = -1;
		return -1;
	}
	DWORD t2_end = GetTickCount() - t1_start;

	char szBuf[1024];
	memset(szBuf, 0, 1024);
	sprintf(szBuf, "联数： %d, 耗時: %d ms", out_unite_num, t2_end);
	theApp.putLog(szBuf);

	return 0;
}

//获取裁剪接口的内部参数,可接收配置的参数;
int GetCutPictureParagramInnerAtom(IplImage *pImg, int in_blockSize, int in_constValue, cv::RotatedRect& rectPoint, cv::Rect& roi, int iReserve)
{
	//theApp.putLog("自适应裁剪接口");
	if (pImg == NULL)
	{
		theApp.putLog("pImg empty!!");
		return -1;
	}
	//if (iReserve & 3)
	//{
	//	SYSTEMTIME st;
	//	GetSystemTime(&st);
	//	CString sFileName;
	//	sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
	//	cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
	//	//cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
	//}
	IplImage * pImg_grey = cvCreateImage(cvSize(pImg->width, pImg->height), IPL_DEPTH_8U, 1);
	cvCvtColor(pImg, pImg_grey, CV_BGR2GRAY);
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_灰度图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg_grey);
		//cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
	}


#if 0
	//农行标准机器裁剪参数
	int blockSize = 25;
	int constValue = 70;
#elif 1
	//临时测试机器参数
	//int blockSize = 25;
	//int constValue = 60;
#endif
	int blockSize = in_blockSize;
	int constValue = in_constValue;

	cv::Mat bin_image;
	cv::Mat grey_Mat;
	grey_Mat = cv::Mat(pImg_grey, true);
	//cv::adaptiveThreshold(grey_Mat, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);

	//cv::adaptiveThreshold(grey_Mat, bin_image, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	cv::Mat gaussiarImage;
#if 1
	cv::GaussianBlur(grey_Mat, gaussiarImage, cv::Size(0, 0), 3);
	cv::addWeighted(grey_Mat, 1.5, gaussiarImage, -0.5, 0, gaussiarImage);
#elif 0
	blur(grey_Mat, gaussiarImage, Size(3,3));
#endif
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_锐化图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, grey_Mat);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, gaussiarImage);
	}
	//cv::Canny(gaussiarImage, bin_image, 80, 255);
	cv::Canny(gaussiarImage, bin_image, theApp.min_threshold, 255);
	IplImage* image = &IplImage(bin_image);
	IplImage* smoothImg = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	cvCopy(image, smoothImg);
	//如果垫子反光，膨胀得关闭
	if(theApp.Bright == 0)
	{
		cvDilate(smoothImg, smoothImg, 0, 1);	
	}

	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_预处理图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cvSaveImage((LPSTR)(LPCTSTR)sFileName, smoothImg);
		//cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
	}

	vector<Point> points;
	//获取轮廓   
	std::vector<std::vector<Point>> contours;
	Mat imagex(smoothImg, 0);
	//获取轮廓：   
	//findContours(imagex, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
#if 0
	findContours(imagex, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<Point>>::const_iterator itc = contours.begin();
	std::vector<Point>::const_iterator itpp;
	while (itc != contours.end())
	{
		for (itpp = itc->begin(); itpp != itc->end(); itpp++)
		{
			points.push_back(*itpp);
		}
		itc++;
	}
#elif 1
	cv::findContours(imagex, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int areaMax = 0;
	int index_MaxArea = 0;
	if(contours.size() < 1)
	{
		return -1;
	}
	for(int i=0; i<contours.size(); i++)
	{
		cv::Rect rect = cv::boundingRect(contours[i]);
		if(rect.area() > areaMax)
		{
			areaMax = rect.area();
			index_MaxArea = i;
		}	
	}
	points = contours[index_MaxArea];
#endif

	rectPoint = minAreaRect(points);
	Point2f fourPoint2f[4];
	rectPoint.points(fourPoint2f);
	for (int i = 0; i < 4; i++)
	{
		points.push_back(fourPoint2f[i]);
	}
	roi = cv::boundingRect(points);
	if(roi.x < 0)
		roi.x = 0;
	if(roi.y < 0)
		roi.y = 0;

	if (iReserve & 2)
	{
		Mat imagex2(pImg, 1);
		RNG &rng = theRNG();
		//根据得到的四个点的坐标  绘制矩形  
		for (int i = 0; i < 3; i++)
		{
			line(imagex2, fourPoint2f[i], fourPoint2f[i + 1], Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3);
		}
		line(imagex2, fourPoint2f[0], fourPoint2f[3], Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3);

		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪示意图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, smoothImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imagex2);
	}
	cvReleaseImage(&pImg_grey);
	cvReleaseImage(&smoothImg);
	return 0;
}







//票据识别的 图片裁剪
//裁剪图片的内部用的接口;
//新的裁剪方式
//
//int RCN_GetCutPictureParagramInner(IplImage *pImg, cv::RotatedRect& rectPoint, cv::Rect& roi, int iReserve)
//{
//	theApp.putLog("自适应裁剪接口");
//	if (pImg == NULL)
//	{
//		theApp.putLog("pImg empty!!");
//		return -1;
//	}
//	if (iReserve & 1)
//	{
//		SYSTEMTIME st;
//		GetSystemTime(&st);
//		CString sFileName;
//		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
//		cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
//		//cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
//	}
//	IplImage * pImg_grey = cvCreateImage(cvSize(pImg->width, pImg->height), IPL_DEPTH_8U, 1);
//	cvCvtColor(pImg, pImg_grey, CV_BGR2GRAY);
//	if (iReserve & 1)
//	{
//		SYSTEMTIME st;
//		GetSystemTime(&st);
//		CString sFileName;
//		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_灰度图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
//		cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg_grey);
//		//cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
//	}
//
//
//#if 0
//	//农行标准机器裁剪参数
//	int blockSize = 25;
//	int constValue = 70;
//#elif 1
//	//临时测试机器参数
//	int blockSize = 25;
//	int constValue = 60;
//#endif
//	cv::Mat bin_image;
//	cv::Mat grey_Mat;
//	grey_Mat = cv::Mat(pImg_grey, true);
//	cv::adaptiveThreshold(grey_Mat, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
//	IplImage* image = &IplImage(bin_image);
//	IplImage* smoothImg = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
//	cvCopy(image, smoothImg);
//	cvDilate(smoothImg, smoothImg, 0, 2);
//	cvErode(smoothImg, smoothImg, 0, 2);
//
//	//cvNamedWindow("cvDilate", CV_WINDOW_AUTOSIZE); 
//	//cvShowImage("cvDilate", smoothImg);  
//	//waitKey(0);
//
//	if (iReserve & 1)
//	{
//		SYSTEMTIME st;
//		GetSystemTime(&st);
//		CString sFileName;
//		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_预处理图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
//		cvSaveImage((LPSTR)(LPCTSTR)sFileName, smoothImg);
//		//cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageROI);
//	}
//
//	vector<Point> points;
//	//获取轮廓   
//	std::vector<std::vector<Point>> contours;
//	Mat imagex(smoothImg, 0);
//	//获取轮廓：   
//	findContours(imagex, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//	std::vector<std::vector<Point>>::const_iterator itc = contours.begin();
//	std::vector<Point>::const_iterator itpp;
//	while (itc != contours.end())
//	{
//		for (itpp = itc->begin(); itpp != itc->end(); itpp++)
//		{
//			points.push_back(*itpp);
//		}
//		itc++;
//	}
//	rectPoint = minAreaRect(points);
//	Point2f fourPoint2f[4];
//	rectPoint.points(fourPoint2f);
//	for (int i = 0; i < 4; i++)
//	{
//		points.push_back(fourPoint2f[i]);
//	}
//	roi = cv::boundingRect(points);
//	if (iReserve & 1)
//	{
//		Mat imagex2(pImg, 1);
//		RNG &rng = theRNG();
//		//根据得到的四个点的坐标  绘制矩形  
//		for (int i = 0; i < 3; i++)
//		{
//			line(imagex2, fourPoint2f[i], fourPoint2f[i + 1], Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3);
//		}
//		line(imagex2, fourPoint2f[0], fourPoint2f[3], Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3);
//
//		SYSTEMTIME st;
//		GetSystemTime(&st);
//		CString sFileName;
//		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪示意图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
//		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, smoothImg);
//		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imagex2);
//	}
//	cvReleaseImage(&pImg_grey);
//	cvReleaseImage(&smoothImg);
//	return 0;
//}




int RCN_GetCutPictureParagramInner(IplImage *pImg, cv::RotatedRect& rectPoint, cv::Rect& roi, int iReserve)
{

	int iRet1 = -1;
	//iRet =  GetCutPictureParagramInnerAtom(IplImage *pImg, int in_blockSize, int in_constValue, cv::RotatedRect& rectPoint, cv::Rect& roi, int iReserve)
	/*
	int blockSize1 = 25;
	int constValue1 = 50;
	cv::RotatedRect rectPoint1;
	cv::Rect roi1;
	iRet1 = GetCutPictureParagramInnerAtom(pImg, blockSize1, constValue1, rectPoint1, roi1, iReserve);

	int iRet2 = -1;
	int blockSize2 = 25;
	int constValue2 = 60;
	cv::RotatedRect rectPoint2;
	cv::Rect roi2;
	iRet2 = GetCutPictureParagramInnerAtom(pImg, blockSize2, constValue2, rectPoint2, roi2, iReserve);

	int iRet3 = -1;
	int blockSize3 = 25;
	int constValue3 = 70;
	cv::RotatedRect rectPoint3;
	cv::Rect roi3;
	iRet3 = GetCutPictureParagramInnerAtom(pImg, blockSize3, constValue3, rectPoint3, roi3, iReserve);

	if (iRet1 !=0  && iRet2 != 0 && iRet3 != 0)
	{
	return -1;
	}
	//返回最大的面积
	if(roi1.area() > roi2.area() && roi1.area() > roi3.area())
	{ 
	rectPoint = rectPoint1;
	roi = roi1;
	}
	else if (roi2.area() > roi1.area() && roi2.area() > roi3.area())
	{
	rectPoint = rectPoint2;
	roi = roi2;
	}
	else
	{
	rectPoint = rectPoint3;
	roi = roi3;
	}
	*/

	int blockSize1 = 25;
	int constValue1 = 30;
	cv::RotatedRect rectPoint1;
	cv::Rect roi1;
	iRet1 = GetCutPictureParagramInnerAtom(pImg, blockSize1, constValue1, rectPoint1, roi1, iReserve);

	if (iRet1 != 0)
	{
		return iRet1;
	}

	if(roi1.width < 100 || roi1.height < 100)
	{
		return -10;
	}


	rectPoint = rectPoint1;
	roi = roi1;

	return 0;
}


//检测空白;
/*
dwImg: 预留， NULL代表需要识别的版本;如果非空，代表需要裁剪， 需要用户指定方向;
centerPt: 检测区域的中心;
areaX: 相对于中心X方向的偏移
areaY: 相对于中心Y方向的偏移
direction: 预留， 放票据抬头朝向方向,上下左右
iReserve: 预留。
isWhite: 是否是空白; true 是 、false 不是
*/
__declspec(dllexport)  int WINAPI RCN_DetectWhiteSpace(DWORD dwImg, POINT centerPt, int areaX, int areaY,  int direction, bool &isWhite, int iReserve)
{

	iReserve = theApp.m_idebug;
	cv::Mat image_cutImage;
	if (dwImg != NULL)
	{
		//获取裁剪图
		return -1;
	}
	image_cutImage = theApp.m_image_clipper;
	cv::Mat imageRotate;
	if (theApp.m_iDirectionTitle == 3)
	{
		//裁剪图朝右
		//逆时针进行旋转90
		transpose(image_cutImage, imageRotate);
		flip(imageRotate, imageRotate, 0);

	}
	else if (theApp.m_iDirectionTitle == 0)
	{
		//裁剪图朝上
		imageRotate = image_cutImage.clone();
	}
	else
	{
		//未知类型

		return -1;
	}

	float x_dpi = ((float)theApp.m_iXScaleplate) / 40;
	float y_dpi = ((float)theApp.m_iYScaleplate) / 40;

	int width_pix = 0;
	int height_pix = 0;
	//无需旋转情况
	if (theApp.VerifyCodeRotate == 2)
	{
		centerPt.x = centerPt.x * x_dpi;
		centerPt.y = centerPt.y * y_dpi;
		width_pix = areaX * x_dpi*2;
		height_pix = areaY * y_dpi*2;
	}
	else
	{
		centerPt.x = centerPt.x * y_dpi;
		centerPt.y = centerPt.y * x_dpi;
		width_pix = areaX * y_dpi*2;
		height_pix = areaY * x_dpi*2;
	}
	cv::Rect whiteZone;
	whiteZone.x = centerPt.x - width_pix/2;
	whiteZone.y = centerPt.y - height_pix/2;
	whiteZone.width = width_pix;
	whiteZone.height = height_pix;


	if (whiteZone.x < 0)
	{
		whiteZone.x = 0;
	}
	if ((whiteZone.x + whiteZone.width) > imageRotate.cols)
	{
		whiteZone.width = imageRotate.cols - whiteZone.x - 3;
	}
	if (whiteZone.y < 0)
	{
		whiteZone.y = 0;
	}
	if ((whiteZone.y + whiteZone.height) > imageRotate.rows)
	{
		whiteZone.height = imageRotate.rows - whiteZone.y - 3;
	}
	if (whiteZone.x < 0 || whiteZone.y < 0 ||
		((whiteZone.x + whiteZone.width) > imageRotate.size().width) ||
		((whiteZone.y + whiteZone.height) > imageRotate.size().height) ||
		((whiteZone.x + whiteZone.width) < 0) ||
		((whiteZone.y + whiteZone.height) < 0)
		)
	{
		return -1;
	}

	//裁剪需要检测的区域
	cv::Mat imgDetectedBlank = imageRotate(whiteZone);
	if (iReserve & 1)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_空白检测_裁剪图_彩图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imgDetectedBlank);
	}

	cv::Mat imgDetectedBlankGrey;
	if (imgDetectedBlank.channels() == 3)
	{
		cv::cvtColor(imgDetectedBlank, imgDetectedBlankGrey, CV_BGR2GRAY);
	}
	else
	{
		imgDetectedBlankGrey = imgDetectedBlank;
	}

	//计算轮廓数;
	cv::Mat imgDetectedBlankGreyBin;
	int blockSize = 25;
	int constValue = 10;
	cv::Mat local;
	//cv::adaptiveThreshold(img, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	cv::adaptiveThreshold(imgDetectedBlankGrey, imgDetectedBlankGreyBin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_空白检测二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, img2);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imgDetectedBlankGreyBin);
	}

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(imgDetectedBlankGreyBin, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	std::vector<cv::Rect> candiateRect;
	cv::Mat showImg;
	cv::cvtColor(imgDetectedBlankGrey, showImg, CV_GRAY2BGR);
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Rect rect;
		rect = cv::boundingRect(contours[i]);
		if (rect.height < 10)
		{
			continue;
		}
		cv::rectangle(showImg, rect, cv::Scalar(0, 255, 0), 2);
		candiateRect.push_back(rect);
	}

	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_空白检测结果图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, showImg);
	}
	if (candiateRect.size() == 0)
	{
		isWhite = true;
	}
	else
	{
		isWhite = false;
	}
	return 0;
}


int calcContoursReasonableInner(cv::Mat image, std::vector<cv::Rect>& rectsOfImage, int height_contours, int width_contours, PKeyArea pkey_Area, int directionText, int image_type, int searchZoneThresholdValue)
{
	if(image.data == NULL)
	{
		return -1;
	}
	//当前特征的切割间隔;
	int distence_gap  = pkey_Area->distence_gap;
	rectsOfImage.clear();
	int blockSize = 25;
	//默认用10 ,含有底纹的用30;
	if(searchZoneThresholdValue < 5 || searchZoneThresholdValue > 60)
	{
		searchZoneThresholdValue = 10;
	}

	int constValue = searchZoneThresholdValue;

	//if(image_type == 1)
	//{
	//	constValue = 20;
	//	//如果有底纹的采用30;
	//}
	//else
	//{
	//	constValue = 10;
	//}

	cv::Mat binImage;
	cv::adaptiveThreshold(image, binImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	if (theApp.m_idebug & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_特征区间二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, binImage);
	}

	//cv::Mat eleMat = cv::getStructuringElement(0, cv::Size(3,3));
	//cv::dilate(binImage, binImage, eleMat, cv::Point(-1, -1), 1);
	//cv::threshold(image, binImage, 150, 255, CV_THRESH_BINARY_INV);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(binImage, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	std::vector<cv::Rect> candiateRect;

	cv::Mat showImg;
	cv::cvtColor(image, showImg, CV_GRAY2BGR);
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Rect rect;
		rect = cv::boundingRect(contours[i]);
		//过滤掉完全不合理的部分;
		if (rect.height < height_contours && rect.width < width_contours)
		{
			continue;
		}
		//if(image_type == 1)
		//{
		//	//对于搜索区间的特大的过滤掉
		//	if(rect.width > (image.cols - 3) || rect.height > (image.rows -3))
		//	{
		//		continue;
		//	}
		//}
		candiateRect.push_back(rect);
	}

	//过滤掉嵌套的轮廓;
	//过滤下，被包含的矩形；
	std::vector<cv::Rect> candiateRect2 = candiateRect;
#if 0
	int num_contour = candiateRect.size();
	for (int i = 0; i < num_contour; i++)
	{
		cv::Point pt;
		pt.x = candiateRect[i].x + (candiateRect[i].width) / 2;
		pt.y = candiateRect[i].y + (candiateRect[i].height) / 2;
		bool isContain = false;
		for (int j = 0; j < num_contour; j++)
		{
			if (i == j)
			{
				continue;
			}
			bool contion = pt.x > candiateRect[j].x && pt.x < (candiateRect[j].x + candiateRect[j].width) && pt.y > candiateRect[j].y && pt.y < (candiateRect[j].y + candiateRect[j].height);
			if (contion)
			{
				isContain = true;
				break;
			}

		}
		if (isContain == false)
		{			
			candiateRect2.push_back(candiateRect[i]);
		}
	}
#endif

	//由于同一个字，可能被拆分成多个区域，
	//模板，和搜索区间的图，得到的轮廓，可能不一致
	//故需 合并矩形区域;
	//标题的方向, 上、下、左、右： 0 1 2 3
	//if(p->bDirectionTitle == 0 || p->bDirectionTitle == 1)
	int iccc = 0;
	vector<cv::Rect>::iterator itr, itr2, itr_next; 
	if(directionText == 0)
	{
		//水平
		sort(candiateRect2.begin(), candiateRect2.end(), compareRectX2);
	}
	else
	{
		//竖直
		sort(candiateRect2.begin(), candiateRect2.end(), compareRectY2);
	}

	//	vector<cv::Rect>::iterator itr,itr_next;  
	//int icc = 1;
	vector<cv::Rect> comrcsMargeX; 
	cv::Rect rtmp = cv::Rect(0, 0, 0, 0);
	//if(p->bDirectionTitle == 0 || p->bDirectionTitle == 1)
	if(directionText == 0)
	{
		//水平
		uchar *bY = new uchar[image.size().height + 1];
		memset(bY, 0, image.size().height + 1);
		for (int y = 0; y < image.size().height; y++)
		{
			for (int x = 0; x < image.size().width; x++)
			{
				//uchar s = CV_IMAGE_ELEM(smoothImg, uchar, y, x);
				uchar s = binImage.at<uchar>(y, x);
				if (s != 0)
				{
					bY[y] = 1;
					break;
				}
			}
		}

		int iMaxLen = 0, ipos = 0, itmp = 0;
		for(int i=0; i<image.size().height; i++)
		{	
			if(bY[i] != 0)
				iccc++;
			else
			{
				if(iccc > iMaxLen)
				{
					iMaxLen = iccc;		
					ipos = itmp;
				}
				iccc = 0;
				itmp = i;
			}
		}

		for(itr = candiateRect2.begin(); itr != candiateRect2.end(); )   
		{
			itr2 = itr;
			cv::Rect rc = *itr;
			if(((rc.y + rc.height) <= ipos) || (rc.y > (ipos + iMaxLen)))
			{
				itr = candiateRect2.erase(itr2);
			}
			else
				itr++;
		}
		delete[] bY;

		for ( itr = candiateRect2.begin(); itr != candiateRect2.end(); itr++ )   
		{
			cv::Rect rc = *itr; 
			if(rtmp.width == 0)
			{
				rtmp = rc;
			}
			else
			{
				if(rc.x - (rtmp.x + rtmp.width) >= distence_gap)
				{
					comrcsMargeX.push_back(rtmp);
					rtmp = rc;
				}
				else
				{
					if((rc.x + rc.width) >= (rtmp.x + rtmp.width))
						rtmp.width = rc.x + rc.width - rtmp.x;

					if(rc.y <= rtmp.y) 
					{
						if((rc.y + rc.height) >= (rtmp.y + rtmp.height))
						{
							rtmp.y = rc.y;
							rtmp.height = rc.height;
						}
						else
						{
							if(rtmp.y - (rc.y + rc.height) < 12)
							{
								rtmp.height = rtmp.height + (rtmp.y - rc.y);
								rtmp.y = rc.y;
							}		
						}
					}
					else 
					{
						if((rc.y + rc.height) > (rtmp.y + rtmp.height))
						{
							rtmp.height = rc.height + (rc.y - rtmp.y);
						}
					}
				}
			}
		}
		if(rtmp.width != 0)
			comrcsMargeX.push_back(rtmp);
	}
	else
	{
		//竖直
		uchar *bX = new uchar[image.size().width + 1];
		memset(bX, 0, image.size().width + 1);
		for (int x = 0; x < image.size().width; x++)
		{
			for (int y = 0; y < image.size().height; y++)
			{
				//uchar s = CV_IMAGE_ELEM(smoothImg, uchar, y, x);
				uchar s = binImage.at<uchar>(y, x);
				if (s != 0)
				{
					bX[x] = 1;
					break;
				}
			}
		}
		int iMaxLen = 0, ipos = 0, itmp = 0;
		for(int i=0; i<image.size().width; i++)
		{	
			if(bX[i] != 0)
				iccc++;
			else
			{
				if(iccc > iMaxLen)
				{
					iMaxLen = iccc;		
					ipos = itmp;
				}
				iccc = 0;
				itmp = i;
			}
		}

		for(itr = candiateRect2.begin(); itr != candiateRect2.end(); )   
		{
			itr2 = itr;
			cv::Rect rc = *itr;
			if(((rc.x + rc.width) <= ipos) || (rc.x > (ipos + iMaxLen)))
			{
				itr = candiateRect2.erase(itr2);
			}
			else
				itr++;
		}
		delete[] bX;

		for ( itr = candiateRect2.begin(); itr != candiateRect2.end(); itr++ )   
		{
			cv::Rect rc = *itr; 
			if(rtmp.height == 0)
			{
				rtmp = rc;
			}
			else
			{
				if(rc.y - (rtmp.y + rtmp.height) >= distence_gap)
				{
					comrcsMargeX.push_back(rtmp);
					rtmp = rc;
				}
				else
				{
					if((rc.y + rc.height) >= (rtmp.y + rtmp.height))
						rtmp.height = rc.y + rc.height - rtmp.y;

					if(rc.x <= rtmp.x)  
					{
						if((rc.x + rc.width) >= (rtmp.x + rtmp.width)) 
						{
							rtmp.x = rc.x;
							rtmp.width = rc.width;
						}
						else
						{
							if(rtmp.x - (rc.x + rc.width) < 12)
							{
								rtmp.width = rtmp.width + (rtmp.x - rc.x);
								rtmp.x = rc.x;
							}
						}
					}
					else 
					{
						if((rc.x + rc.width) > (rtmp.x + rtmp.width)) 
						{
							rtmp.width = rc.width + (rc.x - rtmp.x);
						}
					}
				}
			}
		}

		if(rtmp.height != 0)
			comrcsMargeX.push_back(rtmp);
	}
	rectsOfImage = comrcsMargeX;
	if (theApp.m_idebug & 4)
	{
		for(int i=0; i<rectsOfImage.size(); i++)
		{
			cv::rectangle(showImg, rectsOfImage[i], cv::Scalar(0, 255, 0), 2);
		}
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_轮廓检测结果图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, showImg);
	}
	return 0;
}

void drawResonableRectInner(cv::Mat image, std::vector<cv::Rect> rects)
{
	cv::Mat showImage;
	cv::cvtColor(image, showImage, CV_GRAY2BGR);	
	for(int i=0; i < rects.size(); i++)
	{
		cv::rectangle(showImage, rects[i], cv::Scalar(0, 255, 0));
	}
	if (theApp.m_idebug & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_有效轮廓图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, showImage);
	}
}

//画出过滤后最终的矩形
void drawResonableRect(cv::Mat image_std, cv::Mat image_real, std::vector<cv::Rect> rectsOfStd, std::vector<cv::Rect> rectsOfReal)
{
	drawResonableRectInner(image_std, rectsOfStd);
	drawResonableRectInner(image_real, rectsOfReal);
}


//计算合理的轮廓区间,优化识别速度
int calcContoursReasonable(cv::Mat image_std, cv::Mat image_real, PKeyArea pkey_Area, int searchThresholdValue, std::vector<cv::Rect>& rectsOfStd, std::vector<cv::Rect>& rectsOfReal)
{
	int iRetStd = -1;
	std::vector<cv::Rect> rectsOfStdTmp;
	//依据标准图，确定文字的方向，水平 0 ，纵向 1;
	int directionText = 0;
	if(image_std.cols > image_std.rows)
	{
		directionText = 0;
	}
	else
	{
		directionText = 1;
	}

	iRetStd = calcContoursReasonableInner(image_std, rectsOfStdTmp, 5, 5, pkey_Area, directionText, 0, searchThresholdValue);
	if(iRetStd != 0 || rectsOfStdTmp.size() == 0)
	{
		return -1;
	}

	////找到面积最大的二个
	int numFeatureStd = rectsOfStdTmp.size();
	//std::sort(rectsOfStdTmp.begin(), rectsOfStdTmp.end(), compareRectArea2);
	//std::vector<cv::Rect> rectsOfStdTmpArea;
	//if(numFeatureStd >= 2)
	//{
	//	rectsOfStdTmpArea.push_back(rectsOfStdTmp[0]);
	//	rectsOfStdTmpArea.push_back(rectsOfStdTmp[1]);
	//}
	//else
	//{
	//	rectsOfStdTmpArea = rectsOfStdTmp;
	//}

	////////找到高度较小的那个	
	int iRetReal = -1;
	//std::sort(rectsOfStdTmpArea.begin(), rectsOfStdTmpArea.end(), compareRectHeight);
	//int reference_std_height = 10;
	//reference_std_height = (rectsOfStdTmpArea[0].height - 5) > 10 ? (rectsOfStdTmpArea[0].height - 5): 10;
	//int reference_std_width = 10;


	std::vector<cv::Rect> rectsOfRealTmp;
	iRetReal = calcContoursReasonableInner(image_real, rectsOfRealTmp, 5, 5, pkey_Area, directionText, 1, searchThresholdValue);
	if(iRetReal != 0 || rectsOfRealTmp.size() == 0)
	{
		return -1;
	}
	int numFeatureReal = rectsOfRealTmp.size();
	if(numFeatureStd >= 2 && numFeatureReal >= 2)
	{
		//返回最大的二个轮廓
		//标准的	
		std::sort(rectsOfStdTmp.begin(), rectsOfStdTmp.end(), compareRectArea2);
		rectsOfStd.push_back(rectsOfStdTmp[0]);
		rectsOfStd.push_back(rectsOfStdTmp[1]);

		//裁剪的
		//由于合并，可能会有错误的合并;
		std::sort(rectsOfRealTmp.begin(), rectsOfRealTmp.end(), compareRectArea2);
		rectsOfReal.push_back(rectsOfRealTmp[0]);
		if(numFeatureReal == 2)
		{	
			rectsOfReal.push_back(rectsOfRealTmp[1]);
		}
		else
		{
			//杂质的面积，可能会大于有效面积‘撤销’.比较前三个面积，与标准的比较;
#if 0
			if(directionText == 0)
			{
				//水平
				int diff_x_1 = abs(rectsOfRealTmp[1].x - rectsOfRealTmp[0].x);
				int diff_x_2 = abs(rectsOfRealTmp[2].x - rectsOfRealTmp[0].x);

				if(diff_x_1 < diff_x_2 )
				{
					rectsOfReal.push_back(rectsOfRealTmp[1]);
				}
				else
				{
					rectsOfReal.push_back(rectsOfRealTmp[2]);
				}
			}
			else
			{
				//竖直
				int diff_y_1 = abs(rectsOfRealTmp[1].y - rectsOfRealTmp[0].y);
				int diff_y_2 = abs(rectsOfRealTmp[2].y - rectsOfRealTmp[0].y);
				if(diff_y_1 < diff_y_2)
				{
					rectsOfReal.push_back(rectsOfRealTmp[1]);
				}
				else
				{
					rectsOfReal.push_back(rectsOfRealTmp[2]);
				}
			}
#elif 1
			rectsOfReal.push_back(rectsOfRealTmp[1]);
			rectsOfReal.push_back(rectsOfRealTmp[2]);
#endif
		}	
	}
	else if(numFeatureStd == 1 && numFeatureReal >= 1)
	{
		//单个特征的，需要特殊处理
		rectsOfStd = rectsOfStdTmp;	
		std::sort(rectsOfRealTmp.begin(), rectsOfRealTmp.end(), compareRectArea2);
		rectsOfReal.push_back(rectsOfRealTmp[0]);
	}
	else
	{
		return -1;
	}
	return 0;
}

//判断轮廓的方向，是否一致
//一致的情况下，再判断是横向还是纵向
//**********参数介绍**************
// std_vects : 标准的轮廓;
// real_vects : 裁剪的轮廓；
// direction : 方向，0 横向/ 1 纵向
// 返回值： 0 方向一致， 其它，不一致
int contoursFeatureDirection(std::vector<cv::Rect> std_vects, std::vector<cv::Rect> real_vects, int& direction)
{
	if(std_vects.size() != 2 || real_vects.size() < 2)
	{
		return -1;
	}
	int direction_std = -1;
	if(abs(std_vects[0].x - std_vects[1].x)  < 10)
	{
		direction_std = 1;
	}
	else
	{
		direction_std = 0;
	}

	int direction_real = -1;
	if(abs(real_vects[0].x - real_vects[1].x)  < 10)
	{
		direction_real = 1;
	}
	else
	{
		direction_real = 0;
	}

	if(direction_std != direction_real)
	{
		return -13;
	}
	direction = direction_std;
	return 0;
}



//判定轮廓的间隔，是否一致
int compareIntervalFeature(std::vector<cv::Rect> std_vects, std::vector<cv::Rect> real_vects, int direction)
{
	if(std_vects.size() != 2 || real_vects.size() != 2)
	{
		return -1;
	}
	int distence_std = 0;
	int distence_real = 0;
	if(direction == 1)
	{
		//纵向
		std::sort(std_vects.begin(), std_vects.end(), compareRectY2);
		distence_std = std_vects[1].y - std_vects[0].y - std_vects[0].height;
		std::sort(real_vects.begin(), real_vects.end(), compareRectY2);
		distence_real = real_vects[1].y - real_vects[0].y - real_vects[0].height;
	}
	else
	{
		//横向
		std::sort(std_vects.begin(), std_vects.end(), compareRectX2);
		distence_std = std_vects[1].x - std_vects[0].x - std_vects[0].width;
		std::sort(real_vects.begin(), real_vects.end(), compareRectX2);
		distence_real = real_vects[1].x - real_vects[0].x - real_vects[0].width;
	}
	if(abs(distence_real - distence_std) < 10)
	{
		return 0;
	}
	return -1;
}




//依据轮廓的宽高，间隔判断特征是否一致
//目的用于识别提速
//************输入参数介绍***********
//image_std: 标准特征区的图片
//image_real: 裁剪特征区的图片
//返回值： 0 一致， 其它不一样
//********************************
int filterContoursFeatures(cv::Mat image_std, cv::Mat image_real, PKeyArea pkey_Area, int searchImagebinValue, int debug)
{
	if(debug == 0)
	{
		int index = 0;
		index++;
		index++;
	}

	int iRet = -1;
	int iReserve;
	iReserve = theApp.m_idebug;
	if (iReserve & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_特征标准图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_std);
	}
	if (iReserve & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_特征裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_real);
	}
	cv::Mat greyImageStd;
	cv::Mat greyImageReal;
	if(image_std.channels() == 3)
	{
		cv::cvtColor(image_std, greyImageStd, CV_BGR2GRAY);
	}
	else
	{
		greyImageStd = image_std;
	}

	if(image_real.channels() == 3)
	{
		cv::cvtColor(image_real, greyImageReal, CV_BGR2GRAY);
	}
	else
	{
		greyImageReal = image_real;
	}

	//单个字、二个字的
	int numFeatureStd = 0;
	std::vector<cv::Rect> rectsOfStd;  //最多二个
	std::vector<cv::Rect> rectsOfReal; //最多三个
	iRet = calcContoursReasonable(greyImageStd, greyImageReal, pkey_Area, searchImagebinValue, rectsOfStd, rectsOfReal);
	if(iRet != 0)
	{
		return iRet;
	}
	//画出过滤好的矩形；
	drawResonableRect(greyImageStd, greyImageReal, rectsOfStd, rectsOfReal);
	if(rectsOfStd.size() >=2 && rectsOfReal.size() >= 2)
	{
		//特征字，是二个的
		int absDiffWidth1 = 0;
		int absDiffHeight1 = 0;
		absDiffWidth1 = abs(rectsOfStd[0].width - rectsOfReal[0].width);
		absDiffHeight1 = abs(rectsOfStd[0].height - rectsOfReal[0].height);
		int absDiffWidth2 = 0;
		int absDiffHeight2 = 0;
		absDiffWidth2 = abs(rectsOfStd[1].width - rectsOfReal[1].width);
		absDiffHeight2 = abs(rectsOfStd[1].height - rectsOfReal[1].height);
		int absDiffDistence = 0;
		if( (absDiffWidth1 < 13 && absDiffHeight1 < 13) || (absDiffWidth2 < 13 && absDiffHeight2 < 13))
		{
			//轮廓宽高一致；
			//判断字体方向，横/纵,
			//裁剪的和标准的方向一致后，再判断间隔是否一致;
#if 1
			int direction_feature = -1;
			iRet = contoursFeatureDirection(rectsOfStd, rectsOfReal, direction_feature);	
			if(iRet != 0)
			{
				return iRet;
			}
			std::vector<cv::Rect> rectsOfRealElem2;
			rectsOfRealElem2.push_back(rectsOfReal[0]);
			rectsOfRealElem2.push_back(rectsOfReal[1]);
			iRet = compareIntervalFeature(rectsOfStd, rectsOfRealElem2, direction_feature);			
			if(iRet != 0 && rectsOfReal.size() > 2)
			{	
				rectsOfRealElem2.clear();
				rectsOfRealElem2.push_back(rectsOfReal[0]);
				rectsOfRealElem2.push_back(rectsOfReal[2]);
				iRet = compareIntervalFeature(rectsOfStd, rectsOfRealElem2, direction_feature);
			}
#endif
		}
		else
		{
			iRet = -103;
		}
	}
	else
	{
		//特征字，是一个的
		int absDiffWidth1 = 0;
		int absDiffHeight1 = 0;
		absDiffWidth1 = abs(rectsOfStd[0].width - rectsOfReal[0].width);
		absDiffHeight1 = abs(rectsOfStd[0].height - rectsOfReal[0].height);
		if(absDiffWidth1 < 13 && absDiffHeight1 < 13)
		{
			iRet = 0;
		}
		else
		{
			iRet = -103;
		}
	}
	return iRet;
}


__declspec(dllexport)  int WINAPI RCN_BgrToGray(char* pImgPath)
{
	int iRet = -1;
	if(!pImgPath)
		return iRet;

	cv::Mat imgSealCode = cv::imread(pImgPath, 3);
	//cv::adaptiveThreshold(image_std, image_std, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 25, 10);

	cv::Mat gaussiarImage;
	cv::GaussianBlur(imgSealCode, gaussiarImage, cv::Size(0, 0), 3);
	cv::addWeighted(imgSealCode, 1.5, gaussiarImage, -0.7, 0, gaussiarImage);
	imgSealCode = gaussiarImage;

	cv::cvtColor(imgSealCode, imgSealCode, CV_BGR2GRAY); 
	std::string out_recognize;
	std::vector<cv::Point> pointQR;
	iRet = theApp.QR_Recognizex(imgSealCode, out_recognize, pointQR);
	if(iRet == 0)
	{
		AfxMessageBox(out_recognize.c_str());
		pointQR.clear();
		iRet = 0;
	}
	else
	{
		iRet = -3;
	}
	return iRet;
}


int SealPicBoundingRect(IplImage* pImg, CvRect &rcRect, BOOL &bCenterWhite)  
{  
	bCenterWhite = TRUE;
	int iRet = -1;
	Mat pic;
	if(pImg->nChannels == 4)
	{
		pic = Mat(pImg, TRUE);
		cv::cvtColor(pic, pic, CV_BGRA2BGR); 
		pImg = &IplImage(pic);
	}
	else
		pImg = pImg;

	cv::Mat imageMat = cv::Mat(pImg, true);
	if (imageMat.channels() == 3)
	{
		cv::cvtColor(imageMat, imageMat, CV_BGR2GRAY);
	}

	cv::Mat bin_image;
	int blockSize = 25;
	int constValue = 10;
	cv::adaptiveThreshold(imageMat, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);

	IplImage *image = &IplImage(bin_image);	
	//imshow("AAA", bin_image); 


	vector<IplImage*> characters;
	IplImage* smoothImg = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	cvCopy(image, smoothImg);
	//cvDilate(smoothImg, smoothImg, 0, 2);
	//cvErode(smoothImg, smoothImg, 0, 2);

	CvSeq* contours = NULL;    
	CvMemStorage* storage = cvCreateMemStorage(0);    
	int count = cvFindContours(smoothImg, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); 

	vector<CvRect> comrcs;  
	bool bb = false;
	CvHistogram* his = NULL;
	for ( CvSeq* c = contours; c != NULL; c = c->h_next )   
	{  
		CvRect rc = cvBoundingRect(c, 0); 
		if(rc.height < 50 || rc.width < 50)
			continue;

		comrcs.push_back(rc); 
	}

	sort(comrcs.begin(), comrcs.end(), compareRectX);
	vector<CvRect>::iterator itr, itr2, itr_next;  
	int iccc = 0;
	vector<CvRect> comrcsMargeX; 
	CvRect rtmp = cvRect(0, 0, 0, 0);
	uchar *bX = new uchar[smoothImg->width + 1];
	memset(bX, 0, smoothImg->width + 1);
	for (int x = 0; x < smoothImg->width; x++)
	{
		for (int y = 0; y < smoothImg->height; y++)
		{
			uchar s = CV_IMAGE_ELEM(smoothImg, uchar, y, x);
			if (s != 0)
			{
				bX[x] = 1;
				break;
			}
		}
	}

	int iMaxLen = 0, ipos = 0, itmp = 0;
	for(int i=0; i<smoothImg->width; i++)
	{	
		if(bX[i] != 0)
			iccc++;
		else
		{
			if(iccc > iMaxLen)
			{
				iMaxLen = iccc;		
				ipos = itmp;
			}
			iccc = 0;
			itmp = i;
		}
	}

	for(itr = comrcs.begin(); itr != comrcs.end(); )   
	{
		itr2 = itr;
		CvRect rc = *itr;
		if(((rc.x + rc.width) <= ipos) || (rc.x > (ipos + iMaxLen)))
		{
			itr = comrcs.erase(itr2);
		}
		else
			itr++;
	}
	delete[] bX;

	for ( itr = comrcs.begin(); itr != comrcs.end(); itr++ )   
	{
		CvRect rc = *itr; 
		if(rtmp.height == 0)
		{
			rtmp = rc;
		}
		else
		{
			if(rc.y - (rtmp.y + rtmp.height) >= 3)
			{
				comrcsMargeX.push_back(rtmp);
				rtmp = rc;
			}
			else
			{
				if((rc.y + rc.height) >= (rtmp.y + rtmp.height))
					rtmp.height = rc.y + rc.height - rtmp.y;

				if(rc.x <= rtmp.x)  
				{
					if((rc.x + rc.width) >= (rtmp.x + rtmp.width)) 
					{
						rtmp.x = rc.x;
						rtmp.width = rc.width;
					}
					else
					{
						if(rtmp.x - (rc.x + rc.width) < 12)
						{
							rtmp.width = rtmp.width + (rtmp.x - rc.x);
							rtmp.x = rc.x;
						}
					}
				}
				else 
				{
					if((rc.x + rc.width) > (rtmp.x + rtmp.width)) 
					{
						rtmp.width = rc.width + (rc.x - rtmp.x);
					}
				}
			}
		}
	}

	if(rtmp.height != 0)
		comrcsMargeX.push_back(rtmp);

	//cv::Mat image_color;
	//cv::cvtColor(cv::Mat(img2), image_color, CV_GRAY2BGR);
	//IplImage *imgXX2 = &IplImage(pImg);
	CString sssx;
	CvRect rc, rcMax = cvRect(0, 0, 0, 0);
	for ( itr = comrcsMargeX.begin(); itr != comrcsMargeX.end(); itr++ )   
	{  
		rc = *itr;  
		if((rc.width * rc.height) > (rcMax.width * rcMax.height))
		{
			rcMax = rc;
			iRet = 0;
		}
	}
	rcRect = rcMax;
	if(iRet == 0)
	{
		cv::Rect rectSealCenter;
		rectSealCenter.x = rcMax.x + (rc.width / 2) - 6;
		rectSealCenter.y = rcMax.y + (rc.height / 2) - 6;
		rectSealCenter.width = 10;
		rectSealCenter.height = 10;
		cv::Mat imgSealCenter = bin_image(rectSealCenter);
		IplImage *pImgCenter = &IplImage(imgSealCenter);	
		for (int x = 0; x < pImgCenter->width; x++)
		{
			for (int y = 0; y < pImgCenter->height; y++)
			{
				uchar s = CV_IMAGE_ELEM(pImgCenter, uchar, y, x);
				if (s != 0)
				{
					bCenterWhite = FALSE;
					break;
				}
			}
		}
	}
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&smoothImg);
	smoothImg = NULL;
	return iRet;
}


__declspec(dllexport)  int WINAPI RCN_CompareSealPic(DWORD dwSealPic, DWORD dwSealModel)
{
	int iRet = -1;
	if ((!dwSealPic || !dwSealModel) || (dwSealPic == dwSealModel))
	{
		iRet = -2;
		return iRet;
	}

	IplImage *pImg = NULL, *pModelImg = NULL;
	pImg = (IplImage *)dwSealPic;
	pModelImg = (IplImage *)dwSealModel;

	Mat pic;
	if(pImg->nChannels == 4)
	{
		pic = Mat(pImg, TRUE);
		cv::cvtColor(pic, pic, CV_BGRA2BGR); 
		pImg = &IplImage(pic);
	}
	else
		pImg = pImg;

	cv::Mat imageMat = cv::Mat(pImg, true);
	if (imageMat.channels() == 3)
	{
		cv::cvtColor(imageMat, imageMat, CV_BGR2GRAY);
	}

	Mat picModel;
	if(pModelImg->nChannels == 4)
	{
		picModel = Mat(pModelImg, TRUE);
		cv::cvtColor(picModel, picModel, CV_BGRA2BGR); 
		pModelImg = &IplImage(picModel);
	}
	else
		pModelImg = pModelImg;

	cv::Mat imageModelMat = cv::Mat(pModelImg, true);
	if (imageModelMat.channels() == 3)
	{
		cv::cvtColor(imageModelMat, imageModelMat, CV_BGR2GRAY);
	}

	cv::Mat img_model;
	cv::Mat img_scene;
	int blockSize = 25;
	int constValue = 10;
	cv::adaptiveThreshold(imageModelMat, img_model, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blockSize, constValue);
	cv::adaptiveThreshold(imageMat, img_scene, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blockSize, constValue);

	SiftFeatureDetector detector;
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	detector.detect(img_model, keypoints_1);
	detector.detect(img_scene, keypoints_2);

#if 0
	Mat img_keypoints_1, img_keypoints_2;
	drawKeypoints(img_model, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawKeypoints(img_scene, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	imshow("1key", img_keypoints_1);
	imshow("2key", img_keypoints_2);
#endif

	SiftDescriptorExtractor extractor;
	Mat descriptors_1, descriptors_2;
	extractor.compute(img_model, keypoints_1, descriptors_1);
	extractor.compute(img_scene, keypoints_2, descriptors_2);

	std::vector<std::vector<cv::DMatch>> knnMatches;
	vector< DMatch > matches;  
	cv::BFMatcher matcher;
	matcher.knnMatch(descriptors_1, descriptors_2, knnMatches, 2);
	//vector<cv::DMatch> good_matches;
	const float minRatio=1.f / 1.2f;  
	for (int i=0; i<knnMatches.size(); i++)  
	{  
		const DMatch& bestMatch = knnMatches[i][0];  
		const DMatch& betterMatch = knnMatches[i][1];  
		float distanceRatio = bestMatch.distance/betterMatch.distance;  
		if (distanceRatio <= minRatio)//0.75)  //minRatio 
		{  
			matches.push_back(bestMatch);  
		}  
	}  

	if(!matches.size() || matches.size()<4)  
	{  
		return iRet;  
	}   
	else //单应性矩阵的计算最少得使用4个点  
	{  
		if(matches.size() >= 200)
		{
			iRet = 0;
		}
		//for( int i = 0; i < matches.size(); i++ )  
		//{   
		//good_matches.push_back(matches[i]);  
		//}  
	}
	/*
	IplImage *pImg = NULL, *pImgModel = NULL;
	pImg = (IplImage *)dwSealPic;
	CvRect rc;
	BOOL bCenterWhite = FALSE;
	SealPicBoundingRect(pImg, rc, bCenterWhite);

	CvRect rc2;
	BOOL bCenterWhite2 = FALSE;
	pImgModel = (IplImage *)dwSealModel;
	SealPicBoundingRect(pImgModel, rc2, bCenterWhite2);
	if(rc.width > 10 && rc2.width > 10)
	{
	if(abs((rc.width*rc.height * 1.021) - (rc2.width*rc2.height)) < 1200)
	{
	if(bCenterWhite == bCenterWhite2)
	iRet = 0;
	}
	}
	*/
	return iRet;
}


void filterValidSealShape(std::vector<std::vector<cv::Point> > in_contours,  int index_maxContours, std::vector<int>& result)
{
	result.clear();
	std::vector<cv::Rect> allRectsOfCircle;
	for(int i=0; i<in_contours.size(); i++)
	{
		cv::Rect tmpRect = cv::boundingRect(in_contours[i]);		
		if(tmpRect.width < 100 || tmpRect.height < 100  || tmpRect.width > 500 || tmpRect.height > 500 || tmpRect.area() < 20000 )
		{
			continue;
		}
		result.push_back(i);
		cv::RotatedRect rotRect = fitEllipse(in_contours[i]);
		cv::Rect rectCircle = rotRect.boundingRect();
		allRectsOfCircle.push_back(rectCircle);
	}	
	//过滤掉嵌套的圆,引起的原因，可能是一些小半圆弧，引起的
	std::vector<int>::iterator it = result.begin();
	for(int i=0; i<allRectsOfCircle.size(); i++)
	{
		cv::Rect tmpRectO = allRectsOfCircle[i];
		bool isContained = false;
		for(int j=0; j<allRectsOfCircle.size(); j++)
		{
			if(i == j)
			{
				continue;
			}
			cv::Rect tmpRectI = allRectsOfCircle[j];			
			if(tmpRectI.contains(tmpRectO.tl()) && tmpRectI.contains(tmpRectO.br()))
			{
				isContained = true;
				break;
			}
		}
		if(isContained == true)
		{
			it = result.erase(it);
		}
		else
		{
			it++;
		}	
	}
}

int dealResultReturn(std::vector<cv::Point2f> pts_center, POINT OutArraySealCenter[30], int iReserve)
{
	if(pts_center.size() == 0)
	{
		return -3;
	}
	//先分行；
	std::sort(pts_center.begin(), pts_center.end(), compareByY);	
	//记录分割的行；
	std::vector<std::vector<cv::Point2f>> rows_cut;
	int min_distence_y = 200;	
	std::vector<cv::Point2f> elem_row;
	int min_y = pts_center[0].y;
	for(int i=0; i< pts_center.size(); i++)
	{
		cv::Point2f pt_first = pts_center[i];				
		if(abs(min_y - pt_first.y) < min_distence_y)
		{
			elem_row.push_back(pt_first);
		}
		else
		{
			rows_cut.push_back(elem_row);
			elem_row.clear();
			elem_row.push_back(pt_first);
			min_y = pt_first.y;
		}
	}
	rows_cut.push_back(elem_row);
	elem_row.clear();
	//对每一行排序
	for(int i=0; i<rows_cut.size(); i++)
	{
		std::vector<cv::Point2f>::iterator it_beg = rows_cut[i].begin();
		std::vector<cv::Point2f>::iterator it_end = rows_cut[i].end();
		std::sort(it_beg, it_end, compareByX);
	}

	//返回结果
	int k = 0;
	for(int i=0; i<rows_cut.size(); i++)
	{
		for(int j=0; j<rows_cut[i].size(); j++)
		{
			cv::Point2f pt = rows_cut[i][j];
			OutArraySealCenter[k].x = pt.x;
			OutArraySealCenter[k].y = pt.y;			
			if(iReserve&1)
			{
				std::stringstream ss;
				ss.str("");
				ss << "印章ID:  " << k << " 印章中心: " << OutArraySealCenter[k].x  << " " << OutArraySealCenter[k].y;
				theApp.putLog((char*)ss.str().c_str());
			}
			k++;
			if(k > 30)
			{
				return -3;
			}
		}
	}
	return 0;
}


/************************************************************************/
/* RCN_CorrectionDeviceCoordinate2 印章校准函数, 新版本的
dwImg:                 图片句柄
OutArraySealCenter[30]:印章的个数不是固定的
sealType:              0 圆形 1 三角 2 矩形, 目前只支持圆形
iReserve:              控制图片输出，通常为0
*/
/************************************************************************/
__declspec(dllexport)  int WINAPI RCN_CorrectionDeviceCoordinate2(DWORD dwImg, POINT OutArraySealCenter[30], int sealType, int iReserve)
{
	if(sealType != 0)
	{
		return -4;
	}

	if(!dwImg)
	{
		return -1;
	}

	//将数组清零
	for(int i=0; i<30; i++)
	{
		OutArraySealCenter[i].x = 0;
		OutArraySealCenter[i].y = 0;
	}

	IplImage* img = (IplImage *)dwImg;
	cv::Mat imageRaw = cv::Mat(img, true);
	if(imageRaw.data == NULL)
	{
		return -1;
	}
	if(imageRaw.rows < 1000 || imageRaw.cols < 1000)
	{	
		return -2;
	}
	iReserve = theApp.m_idebug;

	//对原始图，进行裁剪
	//获取裁剪图;

	//RCNDATA rcnData;
	//cv::Mat imageCutIn;
	//int iRet_1 = RCN_GetCutPictureToMem(dwImg, &rcnData, imageCutIn);	
	//if(iRet_1 != 0)
	//	return iRet_1;


	//上下左右，各切100;
	int cutEdgeWidth = 150;
	//int cutEdgeWidth = 25;
	cv::Mat imageCut;

	imageCut = imageRaw(cv::Rect(cutEdgeWidth, cutEdgeWidth, (imageRaw.cols - 2*cutEdgeWidth), (imageRaw.rows - 2*cutEdgeWidth)));
	//imageCut = imageCutIn(cv::Rect(cutEdgeWidth, cutEdgeWidth, (imageCutIn.cols - 2*cutEdgeWidth), (imageCutIn.rows - 2*cutEdgeWidth)));

	//去白纸的边缘部分；
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_边缘裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageCut);
	}
	cv::Mat image;
	cv::cvtColor(imageCut, image, CV_BGR2GRAY);
	cv::Mat image_bin;
	int blockSize = 25;
	int constValue = 30;
	cv::adaptiveThreshold(image, image_bin, 255, CV_ADAPTIVE_THRESH_MEAN_C,  CV_THRESH_BINARY_INV, blockSize, constValue);
	cv::Mat elemStr = cv::getStructuringElement(MORPH_RECT, cv::Size(3, 3));
	cv::dilate(image_bin, image_bin, elemStr, cv::Point(-1, -1), 2);

	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_bin);
	}


	//查找最大的外围轮廓;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	//CV_RETR_LIST CV_RETR_CCOMP  CV_RETR_EXTERNAL
	cv::findContours(image_bin, contours, hierarchy, CV_RETR_EXTERNAL  , CV_CHAIN_APPROX_SIMPLE); //CV_CHAIN_APPROX_NONE  CHAIN_APPROX_SIMPLE
	if(contours.size() == 0)
	{
		return -3;
	}

	if (iReserve & 2)
	{
		cv::Mat imageColor = imageCut.clone();
		int idx = 0;
		for( ; idx >= 0; idx = hierarchy[idx][0] )
		{
			cv::Scalar color( rand()&255, rand()&255, rand()&255 );
			drawContours( imageColor, contours, idx, color, CV_FILLED, 8, hierarchy );
		}

		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_轮廓检测图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageColor);
	}

	std::vector<int> result_shape_indexs;
	filterValidSealShape(contours, 0, result_shape_indexs);	
	if(result_shape_indexs.size() < 3)
	{
		return -3;
	}

	std::vector<cv::Point2f> pts_center;
	cv::Mat imageFinally = imageRaw.clone();
	cv::Scalar color(0, 255, 0);
	for(int indexCircle = 0; indexCircle<result_shape_indexs.size(); indexCircle++)
	{
		//cv::Scalar color( rand()&255, rand()&255, rand()&255 );		
		cv::RotatedRect rotRect = fitEllipse(contours[result_shape_indexs[indexCircle]]);
		//cv::Rect rectTmp = cv::boundingRect(contours[result_shape_indexs[indexCircle]]);
		//cv::Rect rectTmp = rotRect.boundingRect();
		//rectTmp.x += cutEdgeWidth;
		//rectTmp.y += cutEdgeWidth;
		//cv::rectangle(imageFinally, rectTmp, color, 3);
		cv::Point2f center;
		center = rotRect.center;

		//center.x = center.x + cutEdgeWidth + rcnData.iCutX;
		//center.y = center.y + cutEdgeWidth + rcnData.iCutY;

		center.x = center.x + cutEdgeWidth;
		center.y = center.y + cutEdgeWidth;


		//int ramCircle = (rectTmp.size().height + rectTmp.size().width)/4;
		int ramCircle = (rotRect.size.width + rotRect.size.height)/4;
		cv::circle(imageFinally, center, ramCircle, color, 6);
		cv::circle(imageFinally, center, 3, color, 3);
		pts_center.push_back(center);
	}
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_有效圆示意图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageFinally);
	}
	int iRet = -1;
	iRet = dealResultReturn(pts_center, OutArraySealCenter, iReserve);
	return iRet;
}


//帧差法，提取运动物体;
void extractMotionObject(cv::Mat img_src_1, cv::Mat img_src_2, cv::Mat img_src_3, cv::Mat& result_diffImage)
{
	int first_diff_threshold = 5;
	int second_diff_threshold = 5;

	cv::Mat img_gray_1, img_gray_2, img_gray_3;
	cv::Mat gray_diff_1, gray_diff_2;
	cv::Mat diff_result;

	int gaussBlurBox = 3;
	cv::cvtColor(img_src_1, img_gray_1, CV_BGR2GRAY);
	cv::GaussianBlur(img_gray_1, img_gray_1, cv::Size(gaussBlurBox, gaussBlurBox), 0);

	cv::cvtColor(img_src_2, img_gray_2, CV_BGR2GRAY);
	cv::GaussianBlur(img_gray_2, img_gray_2, cv::Size(gaussBlurBox, gaussBlurBox), 0);

	cv::cvtColor(img_src_3, img_gray_3, CV_BGR2GRAY);
	cv::GaussianBlur(img_gray_3, img_gray_3, cv::Size(gaussBlurBox, gaussBlurBox), 0);

	cv::subtract(img_gray_2, img_gray_1, gray_diff_1);
	cv::threshold(gray_diff_1, gray_diff_1, first_diff_threshold, 255, CV_THRESH_BINARY);
	cv::subtract(img_gray_3, img_gray_2, gray_diff_2);
	cv::threshold(gray_diff_2, gray_diff_2, second_diff_threshold, 255, CV_THRESH_BINARY);
	cv::bitwise_and(gray_diff_1, gray_diff_2, diff_result);

	result_diffImage = diff_result.clone();
}


//获取轮廓的数目;
//image_bin: 二值图;
//image_src: 用于显示的图片;
//返回值： 返回轮廓的个数;
int GetContoursNum(cv::Mat threshold_image, cv::Mat image_src, int iReserve)
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(threshold_image, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	int idx = 0;
	int contours_valid = 0;
	for (; idx < contours.size(); idx++)
	{
		cv::Rect rectangle_char = cv::boundingRect(contours[idx]);
		int area = rectangle_char.area();
		if(area < 400)
		{
			continue;
		}
		if(iReserve & 2)
			cv::rectangle(image_src, rectangle_char, cv::Scalar(0, 0, 255));
		contours_valid++;
	}
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_遮挡检测图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_src);
	}
	return contours_valid;
}





//功能：对相机遮挡进行报警;
//dwImg1 dwImg2 dwImg3 , 连续的三张图片.
//isCover: 返回值，true 遮挡， false 正常；
// iReserve: 0, 正常遮挡报警检测， 1， 半遮挡报警检测；
__declspec(dllexport)  int WINAPI RCN_OcclusionAlarm(DWORD dwImg1, DWORD dwImg2, DWORD dwImg3, bool& isCover, double sensitivity, int iReserve)
{
	bool isOcclusionHalfDetect = false;
	if(iReserve == 1)
	{
		isOcclusionHalfDetect = true;
	}

	iReserve = theApp.m_idebug;
	if(iReserve&1)
	{
		theApp.putLog("开始遮挡检测");
	}
#if 0
	iReserve = theApp.m_idebug;
	if(iReserve&1)
	{
		theApp.putLog("开始遮挡检测");
	}
	//检查数据是否有问题;
	if(!dwImg1 || !dwImg2 || !dwImg3)
	{
		return -8;
	}
	IplImage* img1 = (IplImage *)dwImg1;
	cv::Mat imageMat_1 = cv::Mat(img1, true);
	if(imageMat_1.data == NULL)
	{
		return -8;
	}

	IplImage* img2 = (IplImage *)dwImg2;
	cv::Mat imageMat_2 = cv::Mat(img2, true);
	if(imageMat_2.data == NULL)
	{
		return -8;
	}

	IplImage* img3 = (IplImage *)dwImg3;
	cv::Mat imageMat_3 = cv::Mat(img3, true);
	if(imageMat_3.data == NULL)
	{
		return -8;
	}

	//提取运动目标
	cv::Mat result_motion;
	extractMotionObject(imageMat_1, imageMat_2, imageMat_3, result_motion);
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_遮挡报警差分图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, result_motion);
	}
	//统计白色的像素值;
	int numWhitePoint = cv::countNonZero(result_motion);
	int totalSum = imageMat_1.rows*imageMat_1.cols;
	double sensity_current = ((double)numWhitePoint)/(double)totalSum;
	if(iReserve&1)
	{
		std::stringstream ss;
		ss.str("");
		ss << "白色的占比: " << sensity_current << " 设定的阈值: " << sensitivity;
		theApp.putLog((char*)ss.str().c_str());
	}
	if(sensity_current > sensitivity)
	{
		isCover=true;
	}
	else
	{
		isCover = false;
	}

#elif 1
	if(!dwImg1)
	{
		return -8;
	}
	IplImage* img1 = (IplImage *)dwImg1;
	cv::Mat imageMat_1 = cv::Mat(img1, true);
	if(imageMat_1.data == NULL)
	{
		return -8;
	}
	cv::Mat image_grey;
	cv::cvtColor(imageMat_1, image_grey,  CV_BGR2GRAY);
	int gaussBlurBox = 5;
	cv::GaussianBlur(image_grey, image_grey, cv::Size(gaussBlurBox, gaussBlurBox), 0);

	cv::Mat threshold_image;
	int blockSize = 25;
	int constValue = 10;
	cv::adaptiveThreshold(image_grey, threshold_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);

	//std::vector<std::vector<cv::Point> > contours;
	//std::vector<cv::Vec4i> hierarchy;
	//cv::findContours(threshold_image, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	//int idx = 0;
	//int contours_valid = 0;
	//for (; idx < contours.size(); idx++)
	//{
	//	cv::Rect rectangle_char = cv::boundingRect(contours[idx]);
	//	int area = rectangle_char.area();
	//	if(area < 400)
	//	{
	//		continue;
	//	}
	//	cv::rectangle(imageMat_1, rectangle_char, cv::Scalar(0, 0, 255));
	//	contours_valid++;
	//}
	//if (iReserve & 2)
	//{
	//	SYSTEMTIME st;
	//	GetSystemTime(&st);
	//	CString sFileName;
	//	sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_遮挡检测图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
	//	//cvSaveImage((LPSTR)(LPCTSTR)sFileName, pImg);
	//	cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageMat_1);
	//}
	int contours_valid = 0;
	if(isOcclusionHalfDetect == false)
	{
		contours_valid = GetContoursNum(threshold_image, imageMat_1, iReserve);
		if(iReserve&1)
		{
			std::stringstream ss;
			ss.str("");
			ss << "轮廓数: " << contours_valid << " 设定的阈值: " << sensitivity;
			theApp.putLog((char*)ss.str().c_str());
		}
		if(contours_valid < sensitivity)
		{
			isCover=true;
		}
		else
		{
			isCover = false;
		}
	}
	else
	{
		//遮挡一半，检测；
		cv::Mat image_left, image_right, image_up, image_down;
		cv::Rect rect_left, rect_right, rect_up, rect_down;
		//左面检测;
		rect_left = cv::Rect(0, 0, threshold_image.cols/2, threshold_image.rows);
		image_left = threshold_image(rect_left);	
		contours_valid = GetContoursNum(image_left, imageMat_1(rect_left), iReserve);
		if(contours_valid < sensitivity)
		{
			isCover=true;
			return 0;
		}

		//右面检测
		rect_right = cv::Rect(threshold_image.cols/2, 0, threshold_image.cols/2, threshold_image.rows);
		image_right = threshold_image(rect_right);
		contours_valid = GetContoursNum(image_right, imageMat_1(rect_right), iReserve);
		if(contours_valid < sensitivity)
		{
			isCover=true;
			return 0;
		}

		//上面检测
		rect_up = cv::Rect(0, 0, threshold_image.cols, threshold_image.rows/2);
		image_up = threshold_image(rect_up);
		contours_valid = GetContoursNum(image_up, imageMat_1(rect_up), iReserve);
		if(contours_valid < sensitivity)
		{
			isCover=true;
			return 0;
		}

		//下面检测
		rect_down = cv::Rect(0, threshold_image.rows/2, threshold_image.cols, threshold_image.rows/2);
		image_down = threshold_image(rect_down);
		contours_valid = GetContoursNum(image_down, imageMat_1(rect_down), iReserve);
		if(contours_valid < sensitivity)
		{
			isCover=true;
			return 0;
		}
		isCover = false;
	}

#endif
	return 0;
}


void filteredRed(const Mat &inputImage, Mat &resultGray, Mat &resultColor)
{
	Mat hsvImage;
	cvtColor(inputImage, hsvImage, CV_BGR2HSV);
	resultGray = Mat(hsvImage.rows, hsvImage.cols,CV_8U,cv::Scalar(255));  
	resultColor = Mat(hsvImage.rows, hsvImage.cols,CV_8UC3,cv::Scalar(255, 255, 255));
	double H=0.0,S=0.0,V=0.0;   
	for(int i=0;i<hsvImage.rows;i++)
	{
		for(int j=0;j<hsvImage.cols;j++)
		{
			H=hsvImage.at<Vec3b>(i,j)[0];
			S=hsvImage.at<Vec3b>(i,j)[1];
			V=hsvImage.at<Vec3b>(i,j)[2];

			//if((S >= 70 && S<155) || (S >= 35 && S<65))
			if(S >= 20)
			{       
				if(((  H>=0 && H < 25) || H >= 140) )
				{
					resultGray.at<uchar>(i,j)=0;
					resultColor.at<Vec3b>(i, j)[0] = inputImage.at<Vec3b>(i, j)[0];
					resultColor.at<Vec3b>(i, j)[1] = inputImage.at<Vec3b>(i, j)[1];
					resultColor.at<Vec3b>(i, j)[2] = inputImage.at<Vec3b>(i, j)[2];
				}
			}
		}
	}
}



void filteredBlack(const Mat &inputImage, Mat &resultGray, Mat &resultColor)
{
	Mat hsvImage;
	cvtColor(inputImage, hsvImage, CV_BGR2HSV);
	resultGray = Mat(hsvImage.rows, hsvImage.cols,CV_8U,cv::Scalar(255));  
	resultColor = Mat(hsvImage.rows, hsvImage.cols,CV_8UC3,cv::Scalar(255, 255, 255));
	double H=0.0,S=0.0,V=0.0;   
	for(int i=0;i<hsvImage.rows;i++)
	{
		for(int j=0;j<hsvImage.cols;j++)
		{
			H=hsvImage.at<Vec3b>(i,j)[0];
			S=hsvImage.at<Vec3b>(i,j)[1];
			V=hsvImage.at<Vec3b>(i,j)[2];

			//if((S >= 70 && S<155) || (S >= 35 && S<65))
			if(V <= 120)
			{       
				resultGray.at<uchar>(i,j)=0;
				resultColor.at<Vec3b>(i, j)[0] = inputImage.at<Vec3b>(i, j)[0];
				resultColor.at<Vec3b>(i, j)[1] = inputImage.at<Vec3b>(i, j)[1];
				resultColor.at<Vec3b>(i, j)[2] = inputImage.at<Vec3b>(i, j)[2];
			}
		}
	}
}



//重控号, 按照红色的处理.
//输出的是，二值图片
void processImageByRed(cv::Mat image_color, cv::Mat &image_bin, int iReserve)
{
	SYSTEMTIME st;
	CString sFileName;

	cv::Mat image_red, image_red_color;
	filteredRed(image_color, image_red, image_red_color);
	if (iReserve & 4)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_红色区域图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_red_color);
	}

	cv::Mat image_red_color_grey;
	cv::cvtColor(image_red_color, image_red_color_grey, CV_BGR2GRAY);
	if (iReserve & 4)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_红色区域灰度图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_red_color_grey);
	}


	//cv::Mat image_red_color_bin;
	//cv::threshold(image_red_color_grey, image_red_color_bin, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
	//if (iReserve & 4)
	//{
	//	GetSystemTime(&st);
	//	sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_红色区域二值图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
	//	cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_red_color_bin);
	//}


	cv::Mat image_gray_bin;	
	int blockSize = 25;
	int constValue = 30;	
	cv::adaptiveThreshold(image_red_color_grey, image_gray_bin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	if (iReserve & 4)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_红色区域二值图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_gray_bin);
	}

	cv::Mat image_tmp = image_gray_bin.clone();

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(image_gray_bin, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	//提取字符有效的轮廓;
	std::vector<cv::Rect> validRect;
	std::vector<cv::Point> validPoints;
	for(int i=0; i<contours.size(); i++)
	{
		cv::Rect tmpRect = cv::boundingRect(contours[i]);
		if(tmpRect.height < image_gray_bin.rows/6 || tmpRect.width < 3)
		{
			continue;
		}
		validRect.push_back(tmpRect);
		validPoints.push_back(tmpRect.br());
		validPoints.push_back(tmpRect.tl());
	}
	//算出有效字符区间，去掉周围无用的杂质!
	cv::Rect rectValid = cv::boundingRect(validPoints);
	cv::Mat image_grey_roi = image_tmp(rectValid);

	if (iReserve & 4)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_红色区域二值裁剪图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_grey_roi);
	}
	image_bin = image_grey_roi.clone();
}


void processImageByBlack(cv::Mat image_color, cv::Mat &image_bin, int iReserve)
{
	SYSTEMTIME st;
	CString sFileName;

	cv::Mat image_red, image_red_color;
	filteredBlack(image_color, image_red, image_red_color);
	if (iReserve & 4)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_黑色区域图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_red_color);
	}

	cv::Mat image_red_color_grey;
	cv::cvtColor(image_red_color, image_red_color_grey, CV_BGR2GRAY);
	if (iReserve & 4)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_黑色区域灰度图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_red_color_grey);
	}

	//cv::Mat image_red_color_bin;
	//cv::threshold(image_red_color_grey, image_red_color_bin, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
	//if (iReserve & 4)
	//{
	//	GetSystemTime(&st);
	//	sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_黑色区域二值图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
	//	cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_red_color_bin);
	//}
	//image_bin = image_red_color_bin.clone();

	cv::Mat image_gray_bin;	
	int blockSize = 25;
	int constValue = 30;	
	cv::adaptiveThreshold(image_red_color_grey, image_gray_bin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	if (iReserve & 4)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_黑色区域二值图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_gray_bin);
	}

	cv::Mat imagetmp = image_gray_bin.clone();
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(image_gray_bin, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	//提取字符有效的轮廓;
	std::vector<cv::Rect> validRect;
	std::vector<cv::Point> validPoints;
	for(int i=0; i<contours.size(); i++)
	{
		cv::Rect tmpRect = cv::boundingRect(contours[i]);
		if(tmpRect.height < image_gray_bin.rows/6 || tmpRect.width < 3)
		{
			continue;
		}
		validRect.push_back(tmpRect);
		validPoints.push_back(tmpRect.br());
		validPoints.push_back(tmpRect.tl());
	}
	//算出有效字符区间，去掉周围无用的杂质!
	cv::Rect rectValid = cv::boundingRect(validPoints);
	cv::Mat image_grey_roi = imagetmp(rectValid);

	image_bin = image_grey_roi.clone();
}


void processImageByOther(cv::Mat image_color, cv::Mat &image_grey, int iReserve)
{
	SYSTEMTIME st;
	CString sFileName;

	cv::Mat image_grey_tmp;
	cv::cvtColor(image_color, image_grey_tmp, CV_BGR2GRAY);
	if (iReserve & 4)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_其它色区域图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_grey_tmp);
	}
	image_grey = image_grey_tmp.clone();
}


//重控号， 按照黑色处理.
// 输出的是裁剪好的 重控号，不含其它杂质
void processImageByDark(cv::Mat image_color, cv::Mat &image_grey, int iReserve)
{
	SYSTEMTIME st;
	CString sFileName;

	//分离蓝色通道;
	//std::vector<cv::Mat> imageBGR;
	//cv::split(image_color, imageBGR);
	//cv::Mat goodMat = imageBGR[2];
	cv::Mat goodMat;
	cv::cvtColor(image_color, goodMat, CV_BGR2GRAY);
	if (iReserve & 4)
	{

		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_灰度图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, goodMat);
	}

	cv::Mat image_grey_tmp = goodMat.clone();
	//处理下底纹比较深的，与字符联起来了;
	cv::Mat image_gray_bin;	
	int blockSize = 25;
	int constValue = 30;	
	cv::adaptiveThreshold(image_grey_tmp, image_gray_bin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	if (iReserve & 4)
	{
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_编号预处理图.bmp", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_gray_bin);
	}

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(image_gray_bin, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	//提取字符有效的轮廓;
	std::vector<cv::Rect> validRect;
	std::vector<cv::Point> validPoints;
	for(int i=0; i<contours.size(); i++)
	{
		cv::Rect tmpRect = cv::boundingRect(contours[i]);
		if(tmpRect.height < image_gray_bin.rows/6)
		{
			continue;
		}
		validRect.push_back(tmpRect);
		validPoints.push_back(tmpRect.br());
		validPoints.push_back(tmpRect.tl());
	}
	//算出有效字符区间，去掉周围无用的杂质!
	cv::Rect rectValid = cv::boundingRect(validPoints);
	cv::Mat image_grey_roi = image_grey_tmp(rectValid);
	image_grey = image_grey_roi.clone();
}


//空白检测，全篇识别;

//检测空白;
/*
dwImg: 裁剪图;
iReserve: 预留。
isWhite: 是否是空白; true 是 、false 不是
*/
__declspec(dllexport)  int WINAPI RCN_DetectWhiteSpaceA4(DWORD dwImg,  bool &isWhite, int iReserve)
{
	iReserve = theApp.m_idebug;	
	if(!dwImg)
	{
		return -8;
	}
	IplImage* img = (IplImage *)dwImg;
	cv::Mat imageMat = cv::Mat(img, true);
	if(imageMat.data == NULL)
	{
		return -8;
	}
	if(imageMat.rows < 100 || imageMat.cols < 100)
	{
		return -8;
	}
	//裁剪边缘的黑边;
	cv::Rect whiteZone;
	whiteZone.x = 20;
	whiteZone.y = 20;
	whiteZone.width = imageMat.cols - 40;
	whiteZone.height = imageMat.rows - 40;
	//裁剪需要检测的区域
	cv::Mat imgDetectedBlank = imageMat(whiteZone);
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_空白检测_裁剪图_彩图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imgDetectedBlank);
	}

	cv::Mat imgDetectedBlankGrey;
	if (imgDetectedBlank.channels() == 3)
	{
		cv::cvtColor(imgDetectedBlank, imgDetectedBlankGrey, CV_BGR2GRAY);
	}
	else
	{
		imgDetectedBlankGrey = imgDetectedBlank;
	}

	//计算轮廓数;
	cv::Mat imgDetectedBlankGreyBin;
	int blockSize = 25;
	int constValue = 10;
	cv::Mat local;
	//cv::adaptiveThreshold(img, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	cv::adaptiveThreshold(imgDetectedBlankGrey, imgDetectedBlankGreyBin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_空白检测二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, img2);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imgDetectedBlankGreyBin);
	}

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(imgDetectedBlankGreyBin, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	std::vector<cv::Rect> candiateRect;
	cv::Mat showImg = imgDetectedBlank.clone();
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Rect rect;
		rect = cv::boundingRect(contours[i]);
		if (rect.height < 10 || rect.width < 10)
		{
			continue;
		}
		cv::rectangle(showImg, rect, cv::Scalar(0, 255, 0), 2);
		candiateRect.push_back(rect);
	}

	if (iReserve & 7)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_空白检测结果图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, showImg);
	}
	if (candiateRect.size() == 0)
	{
		isWhite = true;
	}
	else
	{
		isWhite = false;
	}
	return 0;
}



//统计白色像素的个数；
int bSumsWhite(Mat src)  
{  

	int counter = 0;  
	//迭代器访问像素点  
	Mat_<uchar>::iterator it = src.begin<uchar>();  
	Mat_<uchar>::iterator itend = src.end<uchar>();    
	for (; it!=itend; ++it)  
	{  
		if((*it)>0) counter+=1;//二值化后，像素点是0或者255  
	}             
	return counter;  
}



//检测印泥盒，防止被套章;
/*
输入的参数：
dwImg : 输入的原图， 包含印章部分;
ptLeftUp: 印章区域的左上点
ptRightDown: 印章区域的右下点
isFound: 印章是否被发现;
threshold_color: 100
iReserve: 预留字段；
返回值： 0 正常， -1 异常；
*/
__declspec(dllexport)  int WINAPI RCN_DetectInkpadBox(DWORD dwImg, POINT ptLeftUp, POINT ptRightDown, int threshold_color, bool &isFound, int iReserve)
{
	iReserve = theApp.m_idebug;	
	//判断输入数据的合理性
	if(!dwImg)
	{
		return -8;
	}

	IplImage* img = (IplImage *)dwImg;
	cv::Mat imageMat = cv::Mat(img, true);
	if(imageMat.data == NULL)
	{
		return -8;
	}
	if(imageMat.rows < 100 || imageMat.cols < 100)
	{
		return -8;
	}

	int width_image = imageMat.cols;
	int height_image = imageMat.rows;

	if(ptLeftUp.x < 0 || ptLeftUp.y < 0 || ptLeftUp.x > width_image || ptLeftUp.y > height_image)
	{
		return -9;
	}
	if(ptRightDown.x < 0 || ptRightDown.y < 0 || ptRightDown.x >width_image || ptRightDown.y > height_image)
	{
		return -9;
	}
	if(ptRightDown.x <= ptLeftUp.x || ptRightDown.y <= ptLeftUp.y)
	{
		return -9;
	}

	//裁剪印泥盒区域;
	cv::Rect rectInpad;
	rectInpad.x = ptLeftUp.x;
	rectInpad.y = ptLeftUp.y;
	rectInpad.width = ptRightDown.x - ptLeftUp.x;
	rectInpad.height = ptRightDown.y - ptLeftUp.y;
	cv::Mat imageInpad = imageMat(rectInpad);	
	if (iReserve & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_印章区域裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageInpad);
	}


	//印泥盒，处理部分
	cv::Mat imageInpadGrey;
	cv::cvtColor(imageInpad, imageInpadGrey, CV_BGR2GRAY);
	int blockSize = 25;
	int constValue = threshold_color;
	cv::Mat imageInpadGreyBin;
	//cv::adaptiveThreshold(imageInpadGrey, imageInpadGreyBin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	cv::threshold(imageInpadGrey, imageInpadGreyBin, threshold_color, 255, CV_THRESH_BINARY_INV);
	if (iReserve & 4)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_印章区域二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, img2);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageInpadGreyBin);
	}

	//统计白色像素数的占比；
	int sum_white = bSumsWhite(imageInpadGreyBin);
	int total_sum = imageInpad.rows * imageInpad.cols;
	double whitePercent = (double)sum_white/(double)total_sum;	

	if(iReserve & 1)
	{
		char buf_message[1024];
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "Inpad box percent: %.2f", whitePercent);
		theApp.putLog(buf_message);
	}

	if(whitePercent > 0.3)
	{
		isFound = true;
	}
	else
	{
		isFound = false;
	}
	return 0;
}



///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

//自动选位置盖章
/*
输入： 
1 一系列的模板图片，以及该模板盖章的位置；
2 输入的原始图片;
输出：
对放进去的材料，盖合适的位置；

详细步骤：
数据的来源：1 相机拍摄的原图；扫描仪/高拍仪 扫描的模板图。
2 模板图上的位置；位置是固定的，相当于mm位置。

处理步骤： 1 对输入的拍摄图，进行裁剪，裁剪出票据部分；
2 裁剪图 与 模板图 依次进行比对，筛选出对应的模板图
3 换算盖章位置

处理的细节：
1 模板图片如何传入接口/还是特定的目录下放图片？（牵涉到坐标，推荐依次调用接口，传入模板信息）
2 提Y方向投影特征接口
*/


class PerspectiveTransform
{
public:
	PerspectiveTransform();

	PerspectiveTransform(float inA11, float inA21, 
		float inA31, float inA12, 
		float inA22, float inA32, 
		float inA13, float inA23, 
		float inA33);

	static PerspectiveTransform quadrilateralToQuadrilateral(float x0, float y0, float x1, float y1,
		float x2, float y2, float x3, float y3, float x0p, float y0p, float x1p, float y1p, float x2p, float y2p,
		float x3p, float y3p);

	static PerspectiveTransform squareToQuadrilateral(float x0, float y0, float x1, float y1, float x2,
		float y2, float x3, float y3);


	static PerspectiveTransform quadrilateralToSquare(float x0, float y0, float x1, float y1, float x2,
		float y2, float x3, float y3);


	PerspectiveTransform buildAdjoint();

	PerspectiveTransform times(PerspectiveTransform other);

	void transformPoints(vector<float> &points);

private:
	float a11;
	float a21;
	float a31;
	float a12; 
	float a22;
	float a32; 
	float a13;
	float a23; 
	float a33;
};


PerspectiveTransform::PerspectiveTransform(float inA11, float inA21, 
										   float inA31, float inA12, 
										   float inA22, float inA32, 
										   float inA13, float inA23, 
										   float inA33) : a11(inA11), a12(inA12), a13(inA13), a21(inA21), a22(inA22), a23(inA23),
										   a31(inA31), a32(inA32), a33(inA33) 
{

}

PerspectiveTransform PerspectiveTransform::quadrilateralToQuadrilateral(float x0, float y0, float x1, float y1,
																		float x2, float y2, float x3, float y3, float x0p, float y0p, float x1p, float y1p, float x2p, float y2p,
																		float x3p, float y3p) 
{
	PerspectiveTransform qToS = PerspectiveTransform::quadrilateralToSquare(x0, y0, x1, y1, x2, y2, x3, y3);
	PerspectiveTransform sToQ = PerspectiveTransform::squareToQuadrilateral(x0p, y0p, x1p, y1p, x2p, y2p, x3p, y3p);
	return sToQ.times(qToS);
}

PerspectiveTransform PerspectiveTransform::squareToQuadrilateral(float x0, float y0, float x1, float y1, float x2,
																 float y2, float x3, float y3) 
{
	float dx3 = x0 - x1 + x2 - x3;
	float dy3 = y0 - y1 + y2 - y3;
	if (dx3 == 0.0f && dy3 == 0.0f) 
	{
		PerspectiveTransform result(PerspectiveTransform(x1 - x0, x2 - x1, x0, y1 - y0, y2 - y1, y0, 0.0f,0.0f, 1.0f));
		return result;
	} 
	else 
	{
		float dx1 = x1 - x2;
		float dx2 = x3 - x2;
		float dy1 = y1 - y2;
		float dy2 = y3 - y2;
		float denominator = dx1 * dy2 - dx2 * dy1;
		float a13 = (dx3 * dy2 - dx2 * dy3) / denominator;
		float a23 = (dx1 * dy3 - dx3 * dy1) / denominator;
		PerspectiveTransform result(PerspectiveTransform(x1 - x0 + a13 * x1, x3 - x0 + a23 * x3, x0, y1 - y0
			+ a13 * y1, y3 - y0 + a23 * y3, y0, a13, a23, 1.0f));
		return result;
	}
}

PerspectiveTransform PerspectiveTransform::quadrilateralToSquare(float x0, float y0, float x1, float y1, float x2,
																 float y2, float x3, float y3) 
{
	// Here, the adjoint serves as the inverse:
	return squareToQuadrilateral(x0, y0, x1, y1, x2, y2, x3, y3).buildAdjoint();
}

PerspectiveTransform PerspectiveTransform::buildAdjoint() 
{
	// Adjoint is the transpose of the cofactor matrix:
	PerspectiveTransform result(PerspectiveTransform(a22 * a33 - a23 * a32, a23 * a31 - a21 * a33, a21 * a32
		- a22 * a31, a13 * a32 - a12 * a33, a11 * a33 - a13 * a31, a12 * a31 - a11 * a32, a12 * a23 - a13 * a22,
		a13 * a21 - a11 * a23, a11 * a22 - a12 * a21));

	return result;
}

PerspectiveTransform PerspectiveTransform::times(PerspectiveTransform other) 
{
	PerspectiveTransform result(PerspectiveTransform(a11 * other.a11 + a21 * other.a12 + a31 * other.a13,
		a11 * other.a21 + a21 * other.a22 + a31 * other.a23, a11 * other.a31 + a21 * other.a32 + a31
		* other.a33, a12 * other.a11 + a22 * other.a12 + a32 * other.a13, a12 * other.a21 + a22
		* other.a22 + a32 * other.a23, a12 * other.a31 + a22 * other.a32 + a32 * other.a33, a13
		* other.a11 + a23 * other.a12 + a33 * other.a13, a13 * other.a21 + a23 * other.a22 + a33
		* other.a23, a13 * other.a31 + a23 * other.a32 + a33 * other.a33));

	return result;
}

void PerspectiveTransform::transformPoints(vector<float> &points) 
{
	int max = points.size();
	for (int i = 0; i < max; i += 2) 
	{
		float x = points[i];
		float y = points[i + 1];
		float denominator = a13 * x + a23 * y + a33;
		points[i] = (a11 * x + a21 * y + a31) / denominator;
		points[i + 1] = (a12 * x + a22 * y + a32) / denominator;
	}
}




bool compareYpts(cv::Point pt1, cv::Point pt2)
{
	return (pt1.y < pt2.y);
}
//////////////////////////////////////////////////////////////////////////
//模板图部分，进行内容裁剪;
//////////////////////////////////////////////////////////////////////////
//参数简介： 二值图, 灰度图, 彩色图;
void mycvCharSegmentz(IplImage* image, IplImage *img2, cv::Mat image_color, int iflag, cv::Mat& resultImage, int iResverse) 
{  

	IplImage* smoothImg = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	cvCopy(image, smoothImg);


	IplImage *pBak = cvCreateImage(cvGetSize(smoothImg), IPL_DEPTH_8U, 1); 
	cvCopy(smoothImg, pBak);

	CvSeq* contours = NULL;    
	CvMemStorage* storage = cvCreateMemStorage(0);    
	int count = cvFindContours(smoothImg, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); 

	vector<CvRect> comrcs;  
	bool bb = false;
	CvHistogram* his = NULL;
	vector<Point> points; 
	for ( CvSeq* c = contours; c != NULL; c = c->h_next )   
	{  

		CvRect rc = cvBoundingRect(c, 0); 
		if(rc.height < 4 && rc.width < 4)
			continue;

		if(rc.x < 50 || rc.x > img2->width-50 || rc.y < 50 || rc.y > img2->height - 50)
			continue;

		comrcs.push_back(rc); 
		CvPoint* mytemppoint = new CvPoint[c->total]; //分配轮廓点
		for (int i = 0; i < c->total; i++) 
		{
			mytemppoint[i]=*CV_GET_SEQ_ELEM(CvPoint,c,i);//一个一个的读取数据
			points.push_back(mytemppoint[i]);
		}	
	}
	RotatedRect rectPoint = minAreaRect(points);
	Point2f fourPoint2f[4];   
	rectPoint.points(fourPoint2f); 

	CvPoint r_pt[4];
	for(int i=0; i<4; i++)
	{
		r_pt[i].x = fourPoint2f[i].x;
		r_pt[i].y = fourPoint2f[i].y;
	}

	//CxRectangle xxr(r_pt);
	///Debug///
	//CvPoint pt_topLeft = xxr.GetTopLeftPoint();
	//CvPoint pt_topRight = xxr.GetTopRightPoint();
	//CvPoint pt_bottomLeft = xxr.GetButtomLeftPoint();
	//CvPoint pt_bottomRight = xxr.GetButtomRightPoint();
	////
	//std::vector<cv::Point2f> pts;
	//for(int i=0; i<4; i++)
	//{
	//	cv::Point2f pt;
	//	pt = fourPoint2f[i];
	//	pts.push_back(pt);
	//}
	//std::sort(pts.begin(), pts.end(), compareYpts);
	std::vector<cv::Point2f> pts_a4_rotate;

	/*if(pts[0].x < pts[1].x)
	{
	pts_a4_rotate.push_back(pts[0]);
	pts_a4_rotate.push_back(pts[1]);
	}
	else
	{
	pts_a4_rotate.push_back(pts[1]);
	pts_a4_rotate.push_back(pts[0]);
	}

	if(pts[2].x < pts[3].x)
	{
	pts_a4_rotate.push_back(pts[2]);
	pts_a4_rotate.push_back(pts[3]);
	}
	else
	{
	pts_a4_rotate.push_back(pts[3]);
	pts_a4_rotate.push_back(pts[2]);
	}*/

	pts_a4_rotate.clear();
	cv::Rect rect_tmp = cv::boundingRect(points);
	pts_a4_rotate.push_back(rect_tmp.tl());
	pts_a4_rotate.push_back(cv::Point(rect_tmp.x + rect_tmp.width, rect_tmp.y));
	pts_a4_rotate.push_back(cv::Point(rect_tmp.x, rect_tmp.y + rect_tmp.height));
	pts_a4_rotate.push_back(rect_tmp.br());

	cv::Mat img = image_color;
	int img_height = img.rows;
	int img_width = img.cols;
	Mat img_trans = Mat::zeros(img_height, img_width, CV_8UC3);
	//PerspectiveTransform tansform = PerspectiveTransform::quadrilateralToQuadrilateral(
	//	0,0,
	//	img_width-1,0,
	//	0,img_height-1,
	//	img_width-1,img_height-1,
	//	xxr.GetTopLeftPoint().x, xxr.GetTopLeftPoint().y,
	//	xxr.GetTopRightPoint().x, xxr.GetTopRightPoint().y,
	//	xxr.GetButtomLeftPoint().x, xxr.GetButtomLeftPoint().y,
	//	xxr.GetButtomRightPoint().x, xxr.GetButtomRightPoint().y);

	//Cxrectangle 判断的点位置对应有时出错，

	PerspectiveTransform tansform = PerspectiveTransform::quadrilateralToQuadrilateral(
		0,0,
		img_width-1,0,
		0,img_height-1,
		img_width-1,img_height-1,
		pts_a4_rotate[0].x, pts_a4_rotate[0].y,
		pts_a4_rotate[1].x, pts_a4_rotate[1].y,
		pts_a4_rotate[2].x, pts_a4_rotate[2].y,
		pts_a4_rotate[3].x, pts_a4_rotate[3].y);

	vector<float> ponits;
	for(int i=0;i<img_height;i++)
	{
		for(int j=0;j<img_width;j++)
		{
			ponits.push_back(j);
			ponits.push_back(i);
		}
	}

	tansform.transformPoints(ponits);
	for(int i=0;i<img_height;i++)
	{
		uchar*  t= img_trans.ptr<uchar>(i);
		for (int j=0;j<img_width;j++)
		{
			int tmp = i*img_width+j;
			int x = ponits[tmp*2];
			int y = ponits[tmp*2+1];
			if(x<0||x>(img_width-1)||y<0||y>(img_height-1))
				continue;

			uchar* p = img.ptr<uchar>(y);
			t[j*3] = p[x*3];
			t[j*3+1] = p[x*3+1];
			t[j*3+2] = p[x*3+2];
		}
	}	
	resultImage = img_trans.clone();
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&pBak);
	cvReleaseImage(&smoothImg);	
}  


//针对裁剪图，进行内容裁剪，并纠正图片的畸变, 标准图;
// 参数介绍： 输入的彩色图、 输出的彩色的只含有内容图、 预留参数; 
int cutImageByContentsStd(cv::Mat image_src, cv::Mat& image_result, int iReserve)
{
	cv::Mat image_grey;
	if(image_src.channels() == 3)
	{
		cv::cvtColor(image_src, image_grey, CV_BGR2GRAY);
	}
	else
	{
		image_grey = image_src;
	}
	cv::Mat img= image_grey;
	IplImage* image = cvCreateImage(cvSize(img.rows, img.cols), 8, 1);  
	cv::Mat bin_image;
	int blockSize = 25;
	int constValue = 10;
	cv::Mat local;
	cv::adaptiveThreshold(img, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_标准二值图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		//cvSaveImage((LPSTR)(LPCTSTR)sFileName, img2);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, bin_image);
	}
	*image = IplImage(bin_image);	
	IplImage *img2 = cvCreateImage(cvSize(img.rows, img.cols), 8, 1);  
	*img2 = IplImage(img);
	cv::Mat image_content;
	mycvCharSegmentz(image, img2, image_src, 1, image_content, iReserve);
	image_result = image_content.clone();	
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪区内容图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_result);
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
// 实时图部分， 进行内容裁剪;
//////////////////////////////////////////////////////////////////////////
void  mycvCharSegmentx(IplImage* image, IplImage *img2, cv::Mat image_color, int iflag,  cv::Mat& resultImage, int iResverse)  
{  
	//vector<IplImage*> characters;
	IplImage* smoothImg = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	cvCopy(image, smoothImg);
	IplImage *pBak = cvCreateImage(cvGetSize(smoothImg), IPL_DEPTH_8U, 1); 
	cvCopy(smoothImg, pBak);
	CvSeq* contours = NULL;    
	CvMemStorage* storage = cvCreateMemStorage(0);    
	int count = cvFindContours(smoothImg, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); 
	vector<CvRect> comrcs;  
	bool bb = false;
	CvHistogram* his = NULL;
	vector<Point> points; 
	for ( CvSeq* c = contours; c != NULL; c = c->h_next )   
	{  

		CvRect rc = cvBoundingRect(c, 0); 
		if((rc.height < 10 && rc.width < 10) || rc.width*rc.height < 50)
			continue;

		//if(rc.x < 50 || rc.x > img2->width-50 || rc.y < 50 || rc.y > img2->height - 50)
		//	continue;

		if(rc.x < 5 || rc.x > img2->width-5 || rc.y < 5 || rc.y > img2->height - 5)
			continue;

		comrcs.push_back(rc); 

		CvPoint* mytemppoint = new CvPoint[c->total]; //分配轮廓点
		for (int i = 0; i < c->total; i++) 
		{
			mytemppoint[i]=*CV_GET_SEQ_ELEM(CvPoint,c,i);//一个一个的读取数据
			points.push_back(mytemppoint[i]);
		}
	}
	/*
	RotatedRect rectPoint = minAreaRect(points);
	Point2f fourPoint2f[4];   
	rectPoint.points(fourPoint2f); 

	CvPoint r_pt[4];
	for(int i=0; i<4; i++)
	{
	r_pt[i].x = fourPoint2f[i].x;
	r_pt[i].y = fourPoint2f[i].y;
	}
	*/

	std::vector<cv::Point2f> pts_a4_rotate;
	pts_a4_rotate.clear();
	cv::Rect rect_tmp = cv::boundingRect(points);
	pts_a4_rotate.push_back(rect_tmp.tl());
	pts_a4_rotate.push_back(cv::Point(rect_tmp.x + rect_tmp.width, rect_tmp.y));
	pts_a4_rotate.push_back(cv::Point(rect_tmp.x, rect_tmp.y + rect_tmp.height));
	pts_a4_rotate.push_back(rect_tmp.br());


	//CxRectangle xxr(r_pt);
	cv::Mat img = image_color;
	int img_height = img.rows;
	int img_width = img.cols;
	Mat img_trans = Mat::zeros(img_height, img_width, CV_8UC3);

	/*
	PerspectiveTransform tansform = PerspectiveTransform::quadrilateralToQuadrilateral(
	0,0,
	img_width-1,0,
	0,img_height-1,
	img_width-1,img_height-1,
	xxr.GetButtomLeftPoint().x, xxr.GetButtomLeftPoint().y,
	xxr.GetTopLeftPoint().x, xxr.GetTopLeftPoint().y,
	xxr.GetButtomRightPoint().x, xxr.GetButtomRightPoint().y,
	xxr.GetTopRightPoint().x, xxr.GetTopRightPoint().y
	);


	//选择180度
	PerspectiveTransform tansform = PerspectiveTransform::quadrilateralToQuadrilateral(
	0,0,
	img_width-1,0,
	0,img_height-1,
	img_width-1,img_height-1,
	xxr.GetTopRightPoint().x, xxr.GetTopRightPoint().y,
	xxr.GetButtomRightPoint().x, xxr.GetButtomRightPoint().y,
	xxr.GetTopLeftPoint().x, xxr.GetTopLeftPoint().y,
	xxr.GetButtomLeftPoint().x, xxr.GetButtomLeftPoint().y
	);

	//选择90度
	PerspectiveTransform tansform = PerspectiveTransform::quadrilateralToQuadrilateral(
	0,0,
	img_width-1,0,
	0,img_height-1,
	img_width-1,img_height-1,
	xxr.GetTopLeftPoint().x, xxr.GetTopLeftPoint().y,
	xxr.GetTopRightPoint().x, xxr.GetTopRightPoint().y,
	xxr.GetButtomLeftPoint().x, xxr.GetButtomLeftPoint().y,
	xxr.GetButtomRightPoint().x, xxr.GetButtomRightPoint().y);

	*/
	PerspectiveTransform tansform = PerspectiveTransform::quadrilateralToQuadrilateral(
		0,0,
		img_width-1,0,
		0,img_height-1,
		img_width-1,img_height-1,
		pts_a4_rotate[0].x, pts_a4_rotate[0].y,
		pts_a4_rotate[1].x, pts_a4_rotate[1].y,
		pts_a4_rotate[2].x, pts_a4_rotate[2].y,
		pts_a4_rotate[3].x, pts_a4_rotate[3].y);

	vector<float> ponits;
	for(int i=0;i<img_height;i++)
	{
		for(int j=0;j<img_width;j++)
		{
			ponits.push_back(j);
			ponits.push_back(i);
		}
	}

	tansform.transformPoints(ponits);
	for(int i=0;i<img_height;i++)
	{
		uchar*  t= img_trans.ptr<uchar>(i);
		for (int j=0;j<img_width;j++)
		{
			int tmp = i*img_width+j;
			int x = ponits[tmp*2];
			int y = ponits[tmp*2+1];
			if(x<0||x>(img_width-1)||y<0||y>(img_height-1))
				continue;

			uchar* p = img.ptr<uchar>(y);
			t[j*3] = p[x*3];
			t[j*3+1] = p[x*3+1];
			t[j*3+2] = p[x*3+2];
		}
	}	
	resultImage = img_trans.clone();
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&pBak);
	cvReleaseImage(&smoothImg);
}  


//针对裁剪图， 进行内容裁剪，并纠正图片的畸变, 拍照图;
// 参数介绍： 输入的彩色图、 输出的彩色的只含有内容图、 预留参数; 
int cutImageByContentsReal(cv::Mat image_src, cv::Mat& image_result, int iReserve)
{
	cv::Mat image_grey;
	if(image_src.channels() == 3)
	{
		cv::cvtColor(image_src, image_grey, CV_BGR2GRAY);
	}
	else
	{
		image_grey = image_src;
	}
	cv::Mat img = image_grey;
	IplImage* image = cvCreateImage(cvSize(img.rows, img.cols), 8, 1);
	cv::Mat bin_image;
	int blockSize = 25;
	//int constValue = 10;
	int constValue = 30;
	cv::Mat local;
	cv::adaptiveThreshold(img, bin_image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
	*image = IplImage(bin_image);	
	IplImage *img2 = cvCreateImage(cvSize(img.rows, img.cols), 8, 1);  
	*img2 = IplImage(img);
	cv::Mat image_content;
	mycvCharSegmentx(image, img2, image_src, 1, image_content, iReserve);
	image_result = image_content.clone();
	return 0;
}


//设置初始化环境, 依次调用该接口，设置模板信息。
//dwImg: 模板图片
//ptSeal: 盖章的位置
//type_image: 0 电子照片， 1 所拍的原始图片
//directionTitle: 标题方向 0 1 2 3 上下左右
//iReserve: 调试参数
//返回值类型： 0 成功 -1
__declspec(dllexport)  int WINAPI RCN_SetTemplatesInfors(DWORD dwImg, POINT ptSeal, char *pStrID, int type_image, int directionTitle, int iReserve)
{
	iReserve = theApp.m_idebug;	
	//判断输入数据的合理性
	if(!dwImg)
	{
		return -8;
	}

	IplImage* img = (IplImage *)dwImg;
	cv::Mat imageMat = cv::Mat(img, true);
	if(imageMat.data == NULL)
	{
		return -8;
	}
	if(imageMat.rows < 100 || imageMat.cols < 100)
	{
		return -8;
	}

	int iRet = -1;
	cv::Mat image_cut;
	if(type_image == 1)
	{
		//需要做裁剪
		RCNDATA rcnData;
		cv::Mat imageCutIn;
		iRet = RCN_GetCutPictureToMem(dwImg, &rcnData, imageCutIn);	
		if(iRet != 0)
			return iRet;
		image_cut = imageCutIn.clone();
	}
	else
	{
		image_cut = imageMat;
	}

	//切割出内容部分；
	cv::Mat image_content;
	cutImageByContentsStd(image_cut, image_content, iReserve);

	//计算特征，按长的边进行投影;
	CTemplateInfor templateInfor;
	templateInfor.m_strID = pStrID;
	templateInfor.SetImage(image_content);
	templateInfor.SetSealPoint(ptSeal);
	templateInfor.CalacFeatureProj(0, theApp.m_strPath, iReserve);
	templateInfor.SetRawImage(imageMat);
	templateInfor.calacError();

	theApp.m_templateInforLists.push_back(templateInfor);

	return 0;
}

//设置印泥盒区域
__declspec(dllexport)  int WINAPI RCN_SetZoneInpadBox(POINT pt_leftUp, POINT pt_rightDown)
{
	int x = pt_leftUp.x;
	int y = pt_leftUp.y;
	int width = pt_rightDown.x - pt_leftUp.x;
	int height = pt_rightDown.y - pt_leftUp.y;

	theApp.m_zoneInpad.x = x;
	theApp.m_zoneInpad.y = y;
	theApp.m_zoneInpad.width = width;
	theApp.m_zoneInpad.height = height;

	return 0;
}


//设置合同、票据的有效区域;
__declspec(dllexport)  int WINAPI RCN_SetImageZone(POINT pt_leftUp, POINT pt_rightDown)
{
	int x = pt_leftUp.x;
	int y = pt_leftUp.y;
	int width = pt_rightDown.x - pt_leftUp.x;
	int height = pt_rightDown.y - pt_leftUp.y;

	theApp.m_zonea4.x = x;
	theApp.m_zonea4.y = y;
	theApp.m_zonea4.width = width;
	theApp.m_zonea4.height = height;

	return 0;
}


//设置模板印章位置,临时的测试接口
__declspec(dllexport)  int WINAPI RCN_SetTemplateSealPos(POINT pt_sealPtIn)
{
	theApp.m_ptSealTemplate.x = pt_sealPtIn.x;
	theApp.m_ptSealTemplate.y = pt_sealPtIn.y;
	return 0;
}



//比较投影是否一致;
//参数简介：
// 模板的投影特征， 实时拍照的投影特征， 匹配到时的方向
// 应该按照统计去做，不符合超过太多，就认为匹配不一致;
// 
bool compareProject(std::vector<int> in_templateProj, std::vector<int> in_realProj, int& direction_match)
{
	bool isSame = true;	
	direction_match = 0;
	int size_template = in_templateProj.size();
	if(in_realProj.size() != size_template || size_template == 0)
	{
		return -1;
	}

	char buf_message[1024];

	//int num_error_line = 0;
	//二个方向的误差;
	int num_error_line_z = 0;
	int num_error_line_r = 0;

	//正序比较
	for(int i=0; i<size_template; i++)
	{
		int real_proj_s = in_realProj[i];
		int std_proj_s = in_templateProj[i];
		int diff_project = abs(real_proj_s - std_proj_s);	
		if(theApp.m_idebug & 1)
		{
			memset(buf_message, 0, 1024);
			sprintf(buf_message, "index: %d, diff: %d", i, diff_project);
			//theApp.putLog(buf_message);
		}

		if(diff_project > theApp.m_errorMaxProj)
		{		
			//num_error_line++;
			//break;
			num_error_line_z++;
		}
	}

	if(theApp.m_idebug & 1)
	{
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "正向时异常的行数: %d ", num_error_line_z);
		theApp.putLog(buf_message);
	}

	//if(num_error_line > theApp.m_errorMaxLine)
	//{
	//	isSame = false;
	//}
	//isSame = true;
	//num_error_line = 0;
	//direction_match = 1;

	//反序比较
	for(int i=0; i<size_template; i++)
	{
		int real_proj_e = in_realProj[size_template - i -1];
		int std_proj_s = in_templateProj[i];
		if(abs(real_proj_e - std_proj_s) > theApp.m_errorMaxProj)
		{	
			//num_error_line++;	
			num_error_line_r++;
		}
	}
	if(theApp.m_idebug & 1)
	{
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "反向时异常的行数: %d ", num_error_line_r);
		theApp.putLog(buf_message);
	}

	//if(num_error_line > theApp.m_errorMaxLine)
	//{
	//	isSame = false;
	//}

	if(num_error_line_z < theApp.m_errorMaxLine || num_error_line_r < theApp.m_errorMaxLine)
	{
		isSame = true;
		if(abs(num_error_line_z - num_error_line_r) < 50)
		{
			direction_match = 2;
		}
		else if(num_error_line_z <= num_error_line_r)
		{
			direction_match = 0;
		}
		else
		{
			direction_match = 1;
		}
	}
	else
	{
		isSame = false;
	}
	return isSame;
}


//模板比对;
//用途: 对输入的图片，进行相似度计算
//image_std: 输入的原图
//image_real: 输入的原图
//sim_text : 相似度
// iReserve: 调试参数.
int RCN_MatchTextBlockForPreAdjustment(cv::Mat image_std, cv::Mat image_real, int* sim_text, int iReserve)
{
	iReserve = theApp.m_idebug;
	int iRet = -1;
	int BlockSize = 20;
	char buf_message[1024];
	DWORD ts = GetTickCount();

	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_特征标准图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_std);
	}

	if(iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_特征实时图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_real);
	}

	cv::Mat image_grey_std;
	cv::Mat image_grey_real;

	if(image_std.channels() == 3)
	{
		cv::cvtColor(image_std, image_grey_std, CV_BGR2GRAY);
	}
	else
	{
		image_grey_std = image_std;
	}

	if(image_real.channels() == 3)
	{
		cv::cvtColor(image_real, image_grey_real, CV_BGR2GRAY);
	}
	else
	{
		image_grey_real = image_real;
	}

	//判断黑色像素的分布， 如果裁剪偏差大，很有可能判断像素的分布，失败!
	iRet = isSameDist(image_grey_real, image_grey_std, iReserve);
	if(iRet != 0)
	{
		//memcpy(fontType, "0", 1);
		*sim_text = 0;
		return 0;
	}

	IplImage *tempImage_1 = NULL;
	IplImage *testImage = NULL;

	cv::Mat MidImgTest;
	cv::Mat MidImgTemplate;

	cv::Mat test_threshold, template_threshold;
	myThreshold(image_grey_real, test_threshold, BlockSize);
	myThreshold(image_grey_std, template_threshold, BlockSize);


	cv::Mat elem = cv::getStructuringElement(CV_8UC1,cv::Size(2, 2));
	cv::Mat test_erode, template_erode;
	cv::erode(template_threshold, template_erode, elem);
	cv::erode(test_threshold, test_erode, elem);
	MidImgTest = test_erode;
	MidImgTemplate = template_erode;		

	cv::Mat expandImgTest, expandImgTemplate;
	cv::resize(MidImgTemplate, expandImgTemplate, cv::Size(0, 0), 1.5, 1.5);
	cv::resize(MidImgTest, expandImgTest, cv::Size(0, 0), 2, 2);

	tempImage_1 = &IplImage(expandImgTemplate);
	testImage = &IplImage(expandImgTest);

	double value[3];
	value[0] = 0;
	value[1] = 0;
	value[2] = 0;
	vector<cv::Point> out_pt1; 
	vector<cv::Point> out_pt2; //原图上的点
	iRet = compareFeatureValueMatchTextBlock(testImage, tempImage_1, value, out_pt1, out_pt2);
	int numFeature = value[0];
	int numMatch = value[2];
	int numTest = value[1];
	//模板或者测试区域没有特征点.
	if(iRet == -3 || iRet == -4)
	{
		int diffNum = abs(numTest - numFeature);
		if(iReserve & 1)
		{
			memset(buf_message, 0, sizeof buf_message);
			sprintf(buf_message, "模板或测试区没有特征,测试: %d, 模板: %d", numTest, numFeature);
			theApp.putLog(buf_message);
		}
		if(diffNum < theApp.keyPointsError)
		{
			*sim_text = 80;
			return 0;
		}
		else
		{
			*sim_text = 0;
			return 0;
		}
	}

	if(iRet != 0)
	{
		//memcpy(fontType, "0", 1);
		*sim_text = 0;
		return -1;
	}
	else
	{
		int diffNum = abs(numTest - numFeature);
		if(diffNum >= 220)
		{
			*sim_text = 0;
			return -1;
		}
		else
		{
			if(numFeature < 30 && numTest < 30) //处理几乎都是白色背景块，采用了两块区域比较可以认为这个合理。
				*sim_text = 80;
			else
				*sim_text = numMatch;
		}
	}
	/*
	double score =0.0;
	if(numFeature>0)
	{
	score =value[2]/value[0];
	}
	else
	{
	score = 0.0;
	}


	if(iReserve & 1)
	{
	memset(buf_message, 0, sizeof buf_message);
	sprintf(buf_message, " 模板的点数：%d , 匹配的点数:%d, 测试区间的点数：%d .占比：%.f",  numFeature, numMatch, numTest, score*100);
	theApp.putLog(buf_message);
	}

	int realScore = score*100;
	int transferScore = 80;
	if(realScore>10)
	{
	transferScore += (realScore - 10)/2;
	}
	else
	{
	transferScore -= (10 - realScore)*4;
	}

	if(transferScore>100)
	{
	transferScore = 100;
	}
	if(transferScore < 0)
	{
	transferScore = 0;
	}

	*sim_text = transferScore;

	iRet = 0;
	if(iReserve & 1)
	{
	DWORD t3 = GetTickCount() - ts;
	memset(buf_message, 0, sizeof buf_message);
	sprintf(buf_message, "验证文本块总时间： %d ms ", t3);
	theApp.putLog(buf_message);
	}*/
	return iRet;
}



//出入图片， 返回模板信息（盖章位置）
//dwImg: 图片
//out_sealPoint: 输出原图片的印章位置;
//返回值： 0 成功，否则 失败
__declspec(dllexport)  int WINAPI RCN_CompareTemplate(DWORD dwImg, POINT& out_sealPoint, int iReserve)
{
	DWORD t1 = GetTickCount();

	int iRet = -1;
	//裁剪图片
	iReserve = theApp.m_idebug;	
	//判断输入数据的合理性
	if(!dwImg)
	{
		return -8;
	}

	IplImage* img = (IplImage *)dwImg;
	cv::Mat imageMat = cv::Mat(img, true);
	if(imageMat.data == NULL)
	{
		return -8;
	}
	if(imageMat.rows < 100 || imageMat.cols < 100)
	{
		return -8;
	}

	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_原图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageMat);
	}



	int width_image = imageMat.cols;
	int height_image = imageMat.rows;	
	//区域是否合理
	if(theApp.m_zonea4.x < 0 || theApp.m_zonea4.y < 0 || 
		(theApp.m_zonea4.x + theApp.m_zonea4.width) > width_image ||
		(theApp.m_zonea4.y + theApp.m_zonea4.height) > height_image)
	{
		return -9;
	}

	//裁剪出票据的大区域;
	cv::Mat image_big = imageMat(theApp.m_zonea4);
	//模板图 与 裁剪图， ;
	//提取Y 方向的投影特征；
	IplImage iplImg = IplImage(image_big);
	//获取裁剪图;
	RCNDATA rcnData;
	cv::Mat imageCutIn;
	iRet = RCN_GetCutPictureToMem((DWORD)&iplImg, &rcnData, imageCutIn);	
	if(iRet != 0)
		return iRet;


	//黑边影响内容裁剪; 周围各去掉 15
	cv::Rect rect_valid;
	rect_valid.x = 15;
	rect_valid.y = 15;
	rect_valid.width = imageCutIn.cols - 50;
	rect_valid.height = imageCutIn.rows - 50;

	imageCutIn = imageCutIn(rect_valid);	
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageCutIn);
	}



	//获取内容裁剪图
	cv::Mat image_content;
	cutImageByContentsReal(imageCutIn, image_content, iReserve);			
	if (iReserve & 2)
	{
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_内容裁剪图.jpg", (LPSTR)(LPCTSTR)theApp.m_strPath, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_content);
	}



	char buf_message[1024];


	//尺寸与模板统一
	bool isSameProj=false;
	int match_direction = 0;   //0: 与模板方向一致  1： 正好相反;
	int match_id = 0;    //匹配到的模板的id;
	int num_template = theApp.m_templateInforLists.size();	

	int i=0;
	for(; i<num_template; i++)
	{
		CTemplateInfor tempInforStd = theApp.m_templateInforLists[i];	
		int width_std = tempInforStd.m_image.cols;
		int height_std = tempInforStd.m_image.rows;
		int isTransverse_std = width_std/height_std;  //是否横向的

		int width_real = image_content.cols;
		int height_real = image_content.rows;
		int isTransverse_real = width_real/height_real;  //是否横向的		

		//依据模板图片 动态改变投影误差
		theApp.m_errorMaxProj=tempInforStd.m_row_pixel_error;
		theApp.m_errorMaxLine=tempInforStd.m_rows_num_error;

		if(isTransverse_real == isTransverse_std)
		{
			//方向一致
			cv::resize(image_content, image_content, cv::Size(width_std, height_std));
		}
		else
		{
			//不一致
			cv::resize(image_content, image_content, cv::Size(height_std, width_std));
		}

		//获取投影值
		CTemplateInfor realInfor;
		realInfor.SetImage(image_content);
		realInfor.CalacFeatureProj(1, theApp.m_strPath, iReserve);

		if(iReserve&1)
		{
			memset(buf_message, 0, 1024);
			sprintf(buf_message, "模板图序号: %d, %s ", i, tempInforStd.m_strID.c_str());
			theApp.putLog(buf_message);
		}

		if(iReserve&1)
		{
			memset(buf_message, 0, 1024);
			sprintf(buf_message, "行方向的误差: %d , 整片行的误差: %d .", theApp.m_errorMaxProj, theApp.m_errorMaxLine);
			theApp.putLog(buf_message);
		}

		//判断 投影是否一致
		isSameProj = compareProject(tempInforStd.m_feature_proj, realInfor.m_feature_proj, match_direction);
		if(isSameProj)
		{
			if(iReserve&1)
			{
				memset(buf_message, 0, 1024);
				sprintf(buf_message, "投影匹配一致 %s", tempInforStd.m_strID.c_str());
				theApp.putLog(buf_message);
			}

			//匹配到一致的跳出
			match_id = i;

			//采取密集区域，做特征匹配! 
			//CTemplateInfor matchTempInforStd = theApp.m_templateInforLists[match_id];
			std::vector<cv::Rect> result_rects;
			int iRet = -1;
			iRet = tempInforStd.maxDenseZone(result_rects, 2, 100);
			if(iRet != 0)
			{
				if(iReserve&1)
				{
					memset(buf_message, 0, 1024);
					sprintf(buf_message, "%s, 获取密集区失败", tempInforStd.m_strID.c_str());
					theApp.putLog(buf_message);
				}
				continue;
			}

			cv::Mat img_std_1, img_std_2;
			cv::Rect rect_std_1, rect_std_2;
			rect_std_1 = result_rects[0];
			rect_std_2 = result_rects[1];
			img_std_1 = tempInforStd.m_image(rect_std_1);
			img_std_2 = tempInforStd.m_image(rect_std_2);

			cv::Mat img_real_1, img_real_2;
			cv::Rect rect_real_1, rect_real_2;

			if(match_direction == 2)
			{
				bool bExits = false;
				for(int j=0; j<2; j++)
				{
					if(j == 0)
					{
						match_direction = 0;
					}
					else
					{
						match_direction = 1;
					}
					if(match_direction == 0)
					{
						//投影方向一致
						if(tempInforStd.m_direction_image == 0)
						{
							//模板横向	
							rect_real_1 = rect_std_1;
							rect_real_2 = rect_std_2;
						}
						else
						{
							//模板纵向
							rect_real_1.x = rect_std_1.y;
							rect_real_1.y = image_content.rows - rect_std_1.x - rect_std_1.width;

							rect_real_2.x = rect_std_2.y;
							rect_real_2.y = image_content.rows - rect_std_2.x - rect_std_2.width;
						}
					}
					else
					{
						//投影方向相反
						if(tempInforStd.m_direction_image == 0)
						{
							//模板横向	
							rect_real_1.x = image_content.cols - rect_std_1.x - rect_std_1.width;
							rect_real_1.y = image_content.rows - rect_std_1.y;

							rect_real_2.x = image_content.cols - rect_std_2.x - rect_std_2.width;
							rect_real_2.y = image_content.rows - rect_std_2.y;
						}
						else
						{
							//模板纵向
							rect_real_1.x = image_content.cols - rect_std_1.y -rect_std_1.width;
							rect_real_1.y = rect_std_1.x;

							rect_real_2.x = image_content.cols - rect_std_2.y -rect_std_2.width;
							rect_real_2.y = rect_std_2.x;
						}
					}

					rect_real_1.width = rect_std_1.width;
					rect_real_1.height = rect_std_1.height;
					rect_real_2.width = rect_std_2.width;
					rect_real_2.height = rect_std_2.height;

					img_real_1 = image_content(rect_real_1);
					img_real_2 = image_content(rect_real_2);

					int simValue= 0;
					iRet = RCN_MatchTextBlockForPreAdjustment(img_std_1, img_real_1, &simValue, iReserve);
					if(iReserve&1)
					{
						memset(buf_message, 0, 1024);
						sprintf(buf_message, "密集块0的分数: %d", simValue);
						theApp.putLog(buf_message);
					}
					if(iRet != 0 || simValue < 21)
					{
						continue;
						//return -13;
					}

					iRet = RCN_MatchTextBlockForPreAdjustment(img_std_2, img_real_2, &simValue, iReserve);
					if(iReserve&1)
					{
						memset(buf_message, 0, 1024);
						sprintf(buf_message, "密集块1的分数: %d", simValue);
						theApp.putLog(buf_message);
					}

					if(iRet != 0 || simValue < 21)
					{
						//return -13;
						continue;
					}
					bExits = true;
					break;
				}
				if(bExits)
					break;
				else
					continue;
			}
			else if(match_direction == 0)
			{
				//投影方向一致
				if(tempInforStd.m_direction_image == 0)
				{
					//模板横向	
					rect_real_1 = rect_std_1;
					rect_real_2 = rect_std_2;
				}
				else
				{
					//模板纵向
					rect_real_1.x = rect_std_1.y;
					rect_real_1.y = image_content.rows - rect_std_1.x - rect_std_1.width;

					rect_real_2.x = rect_std_2.y;
					rect_real_2.y = image_content.rows - rect_std_2.x - rect_std_2.width;
				}
			}
			else
			{
				//投影方向相反
				if(tempInforStd.m_direction_image == 0)
				{
					//模板横向	
					//rect_real_1.x = image_content.cols - rect_std_1.x - rect_std_1.width;
					//rect_real_1.y = image_content.rows - rect_std_1.y;

					//rect_real_2.x = image_content.cols - rect_std_2.x - rect_std_2.width;
					//rect_real_2.y = image_content.rows - rect_std_2.y;
					rect_real_1.x = image_content.cols - rect_std_1.x - rect_std_1.width;
					rect_real_1.y = image_content.rows - rect_std_1.y - rect_std_1.height;

					rect_real_2.x = image_content.cols - rect_std_2.x - rect_std_2.width;
					rect_real_2.y = image_content.rows - rect_std_2.y - rect_std_2.height;
				}
				else
				{
					//模板纵向
					rect_real_1.x = image_content.cols - rect_std_1.y -rect_std_1.width;
					rect_real_1.y = rect_std_1.x;

					rect_real_2.x = image_content.cols - rect_std_2.y -rect_std_2.width;
					rect_real_2.y = rect_std_2.x;
				}
			}

			rect_real_1.width = rect_std_1.width;
			rect_real_1.height = rect_std_1.height;
			rect_real_2.width = rect_std_2.width;
			rect_real_2.height = rect_std_2.height;

			img_real_1 = image_content(rect_real_1);
			img_real_2 = image_content(rect_real_2);

			int simValue= 0;
			iRet = RCN_MatchTextBlockForPreAdjustment(img_std_1, img_real_1, &simValue, iReserve);
			if(iReserve&1)
			{
				memset(buf_message, 0, 1024);
				sprintf(buf_message, "密集块0的分数: %d", simValue);
				theApp.putLog(buf_message);
			}
			if(iRet != 0 || simValue < 21)
			{
				continue;
				//return -13;
			}

			iRet = RCN_MatchTextBlockForPreAdjustment(img_std_2, img_real_2, &simValue, iReserve);
			if(iReserve&1)
			{
				memset(buf_message, 0, 1024);
				sprintf(buf_message, "密集块1的分数: %d", simValue);
				theApp.putLog(buf_message);
			}

			if(iRet != 0 || simValue < 21)
			{
				//return -13;
				continue;
			}
			break;
		}
	}

	//投影没有一致的，返回
	if(isSameProj == false || i == num_template)
	{
		return -12;
	}
	//换算盖章位置
	CTemplateInfor tempInforStdMatch = theApp.m_templateInforLists[match_id];	
	int width_tem = tempInforStdMatch.m_image.cols;
	int height_tem = tempInforStdMatch.m_image.rows;

	//cv::Point pt_seal = tempInforStdMatch.m_pt;	
	cv::Point pt_seal = theApp.m_ptSealTemplate;

	float scaleX = 1.0;
	float scaleY = 1.0;
	if(tempInforStdMatch.m_direction_image == 0)
	{
		//模板横向
		scaleX = (float)rcnData.iCutWidth/width_tem;
		scaleY = (float)rcnData.iCutHeight/height_tem;
		rcnData.iCheckX = pt_seal.x*scaleX;
		rcnData.iCheckY = pt_seal.y*scaleY;
	}
	else
	{
		//模板纵向
		scaleX = (float)rcnData.iCutWidth/height_tem;
		scaleY = (float)rcnData.iCutHeight/width_tem;
		rcnData.iCheckX = pt_seal.y*scaleX;
		rcnData.iCheckY = (width_tem - pt_seal.x)*scaleY;
	}

	if(match_direction != 0)
	{		
		//匹配相反
		rcnData.dAngle += 180;
	}
	RCN_GetCutPictureSealPoint(&rcnData);
	out_sealPoint.x = rcnData.iX + theApp.m_zonea4.x;
	out_sealPoint.y = rcnData.iY + theApp.m_zonea4.y;
	if(iReserve&1)
	{
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "印章位置:( %d, %d)", out_sealPoint.x, out_sealPoint.y);
		theApp.putLog(buf_message);
	}

	//识别出来后，将模板以及原图输出
	if (iReserve & 1)
	{	
		cv::Mat image_raw_1 = tempInforStdMatch.m_image_raw.clone();
		cv::circle(image_raw_1, pt_seal, 6, cv::Scalar(0, 255, 0), 3);
		SYSTEMTIME st;
		GetSystemTime(&st);
		CString sFileName;
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_模板图.jpg", (LPSTR)(LPCTSTR)theApp.m_pathPreAdj, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, image_raw_1);

		cv::circle(imageMat, cv::Point(out_sealPoint.x, out_sealPoint.y), 6, cv::Scalar(0, 255, 0), 3);
		GetSystemTime(&st);
		sFileName.Format("%s\\Image%04d%02d%02d_%02d%02d%02d_%03d_盖章位置图.jpg", (LPSTR)(LPCTSTR)theApp.m_pathPreAdj, st.wYear, st.wMonth, st.wDay, st.wHour + 8, st.wMinute, st.wSecond, st.wMilliseconds);
		cv::imwrite((LPSTR)(LPCTSTR)sFileName, imageMat);
	}
	DWORD t2 = GetTickCount() - t1;
	if(iReserve&1)
	{
		memset(buf_message, 0, 1024);
		sprintf(buf_message, "总耗时: %d ms", t2);
		theApp.putLog(buf_message);
	}

	return 0;
}



bool CmpByDenseValue(DENSEDATA d1, DENSEDATA d2)
{
	return (d1.dense_val > d2.dense_val);
}

int CTemplateInfor::maxDenseZone(std::vector<cv::Rect>& out_rects, int rect_num, int length_pix)
{

	rect_num = 2;
	length_pix = 100;

	//连续性，高投影分布的区域；	
	//数值分段,分段统计
	std::vector<DENSEDATA> denseVec; 
	int sum_proj = 0;
	int iCnt = (m_feature_proj.size() - length_pix)  / length_pix;
	for(int i=0; i<m_feature_proj.size(); i++)
	{
		if(i >= iCnt * length_pix)
			break;

		int val_proj = m_feature_proj[i];
		sum_proj += val_proj;
		if((i+1)%length_pix == 0)
		{		
			//denseMap[i] = sum_proj;
			denseVec.push_back(DENSEDATA(i+1-length_pix, sum_proj));
			sum_proj = 0;
		}
	}

	if(denseVec.size() < 2)
	{
		return -3;
	}

	//找到密度最大的
	std::sort(denseVec.begin(), denseVec.end(), CmpByDenseValue);

	//优化x短边方向的密集位置;
	int num_dark_2 = 0;
	int num_dark_3 = 0;
	int off_pos_dense = 10;
	for(int i=0; i<rect_num; i++)
	{
		cv::Rect tmp_rect;
		cv::Rect tmp_rect_1;
		DENSEDATA d1; 
		d1 = denseVec[i];
		if(m_direction_image == 1)
		{
			//按照朝上
			//tmp_rect.x = 0 + off_pos_dense;

			//优化x；
			cv::Mat image_block;
			cv::Rect rect_maxDenseLineBlock;
			rect_maxDenseLineBlock.x = 0;
			rect_maxDenseLineBlock.y = d1.index_pix;
			rect_maxDenseLineBlock.width = m_image.cols;
			rect_maxDenseLineBlock.height = length_pix;
			image_block = m_image(rect_maxDenseLineBlock);

			std::vector<int> array_proj_col;
			CalacMaxDenseLineProj(image_block, theApp.m_strPath, array_proj_col, 0);	
			//统计列方向的密集区;
			int sum_proj_cols_block = 0;
			std::vector<DENSEDATA> denseVecCols;
			for(int j=0; j<array_proj_col.size(); j++)
			{
				int val = array_proj_col[j];
				sum_proj_cols_block += val;
				if((j+1)%length_pix == 0)
				{
					denseVecCols.push_back(DENSEDATA(j+1-length_pix, sum_proj_cols_block));
					sum_proj_cols_block = 0;
				}
			}

			if(denseVecCols.size() < 2)
			{
				return -3;
			}
			std::sort(denseVecCols.begin(), denseVecCols.end(), CmpByDenseValue);		
			tmp_rect.x = denseVecCols[0].index_pix;
			tmp_rect.y = d1.index_pix;
			tmp_rect.width = length_pix;
			tmp_rect.height = length_pix;	
			out_rects.push_back(tmp_rect);	
			
			if(i==1)
			{
				num_dark_3 = denseVecCols[0].dense_val;
			}

			if(i == 0)
			{	
				num_dark_2 = denseVecCols[1].dense_val;
				tmp_rect_1.x = denseVecCols[1].index_pix;
				tmp_rect_1.y = d1.index_pix;
				tmp_rect_1.width = length_pix;
				tmp_rect_1.height = length_pix;
				out_rects.push_back(tmp_rect_1);
			}
		}
		else
		{
			//按照朝左；
			//预留，横向
			tmp_rect.x = d1.index_pix;
			tmp_rect.y = m_image.cols - length_pix - off_pos_dense;

			cv::Mat image_block;
			cv::Rect rect_maxDenseLineBlock;
			rect_maxDenseLineBlock.x = d1.index_pix;
			rect_maxDenseLineBlock.y = 0;
			rect_maxDenseLineBlock.width = length_pix;
			rect_maxDenseLineBlock.height = m_image.rows;
			image_block = m_image(rect_maxDenseLineBlock);

			std::vector<int> array_proj_col;
			CalacMaxDenseLineProj(image_block, theApp.m_strPath, array_proj_col, 0);	
			//统计列方向的密集区;
			int sum_proj_rows_block = 0;
			std::vector<DENSEDATA> denseVecCols;
			for(int j=0; j<array_proj_col.size(); j++)
			{
				int val = array_proj_col[j];
				sum_proj_rows_block += val;
				if((j+1)%length_pix == 0)
				{
					denseVecCols.push_back(DENSEDATA(j+1-length_pix, sum_proj_rows_block));
					sum_proj_rows_block = 0;
				}
			}

			if(denseVecCols.size() < 2)
			{
				return -3;
			}
			std::sort(denseVecCols.begin(), denseVecCols.end(), CmpByDenseValue);		
			tmp_rect.x = d1.index_pix;
			tmp_rect.y = denseVecCols[0].index_pix;
			tmp_rect.width = length_pix;
			tmp_rect.height = length_pix;	
			out_rects.push_back(tmp_rect);	

			if(i==1)
			{
				num_dark_3 = denseVecCols[0].dense_val;
			}

			if(i == 0)
			{	
				num_dark_2 = denseVecCols[1].dense_val;
				tmp_rect_1.x = d1.index_pix;
				tmp_rect_1.y = denseVecCols[1].index_pix;
				tmp_rect_1.width = length_pix;
				tmp_rect_1.height = length_pix;	
				out_rects.push_back(tmp_rect_1);
			}

		}	
	}

	if(out_rects.size() != 3)
	{
		return -3;
	}

	//out_rects 中存储3个. 再移除其中黑色数最少的.
	if(num_dark_2 > num_dark_3)
	{
		out_rects.erase(out_rects.begin()+2);
	}
	else
	{
		out_rects.erase(out_rects.begin()+1);
	}

	return 0;
}

