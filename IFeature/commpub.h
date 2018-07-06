#if !defined(AFX_COMMPUB_H__A7E91DB9_98F9_88AA_A888_734EB43039CA__INCLUDED_)
#define AFX_COMMPUB_H__A7E91DB9_98F9_88AA_A888_734EB43039CA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#pragma pack(push)
#pragma pack(1) 
typedef struct _RcnData_
{
	char szCheckTypeName[128];  //凭证类型名称
	char szCheckTypeID[128];    //凭证类型编号
	char szCheckNO[30];         //凭证编号
	char szInputName[128];      //输入的凭证类型名称。
	int iX;                     //原图盖章X坐标
	int iY;                     //原图盖章Y坐标
	int iCheckX;                //凭证盖章X坐标
	int iCheckY;                //凭证盖章Y坐标
	int iCutX;                  //裁剪矩形区域左上角X坐标值
	int iCutY;                  //裁剪矩形区域左上角Y坐标值 
	int iCutWidth;              //裁剪图的宽度
	int iCutHeight;             //裁剪图的高度
	int iYzmX;                  //验证码X坐标
	int iYzmY;                  //验证码Y坐标
	double dAngle;              //识别角度
	float fCenterX;             //中心点X坐标
	float fCenterY;             //中心点Y坐标
	BYTE bDirectionTitle;       //凭证的方向 0:上 1：下 2:左 3：右
	double dAngleSeal;          //盖章角度;
	int iUnitNum;
	char szInputNo[32];        //输入的凭证的编号

	_RcnData_()
	{
		memset(szCheckTypeName, 0, 128);
		memset(szCheckTypeID, 0, 128);
		memset(szCheckNO, 0, 30);
		memset(szInputName, 0, 128);
		memset(szInputNo, 0, 32);
		iX = 0;
		iY = 0;
		iCheckX = 0;
		iCheckY = 0;
		iCutX = 0;
		iCutY = 0;
		iCutWidth = 0;
		iCutHeight = 0;
		iYzmX = 0;
		iYzmY = 0;
		dAngle = 0.0;
		fCenterX = 0.0;
		fCenterY = 0.0;
		bDirectionTitle = 0;
		dAngleSeal = 0.0;
		iUnitNum = 0;
	}
}RCNDATA, *PRCNDATA;

typedef struct _SAlogrithmParam_
{
	int XScaleplate;          //校准卡X方向的宽度
	int YScaleplate;          //校准卡Y方向的宽度
	double PixNum;            //1mm的像素数	
	int Bright;               //用于裁剪的亮度调整;默认120
	int Threshold;            //裁剪相关的阈值，默认设置成100
	int Debug;                //预留调试接口,默认为0;
	char szPathModel[260];    //模板路径, 如果是空，读同级的; 否则读该路径
	char szModelNameHead[260];    //模板名， 总行的
	char szModelNameBranch[260];    //模板名， 分行的
	_SAlogrithmParam_()
	{
		memset(szPathModel, 0, 260);
		memset(szModelNameHead, 0, 260);
		memset(szModelNameBranch, 0, 260);
	}
}SALOGRITHMPARAM;

#pragma pack(pop)

extern "C" {
	__declspec(dllexport) int WINAPI RCN_GenImageTemplate(char *pFile, int &ivalue, int &iW, int &iH, int x1, int y1, int x2, int y2);
	__declspec(dllexport)  unsigned long WINAPI RCN_CreateImage(char *pcImg, int size);
	__declspec(dllexport)  void WINAPI RCN_ReleaseImage(DWORD dwImg);
	__declspec(dllexport)  int WINAPI RCN_CheckRecognise(DWORD dwImg, RCNDATA *pRcnData, int &iImgReversal, int iReserve);
	__declspec(dllexport)  int WINAPI RCN_OpenCamera(char *pszCameraName, int iLeft, int iTop, int iRight, int iBottom, double dFocus, double dExposure, int iRotate, int iSFlag);
	__declspec(dllexport)  int WINAPI RCN_CloseCamera();
	__declspec(dllexport)  int WINAPI RCN_Capture(char *pszData, int *pSize);
	__declspec(dllexport)  int WINAPI RCN_OpenPropertyDlg();
	__declspec(dllexport)  int WINAPI RCN_CorrectionDeviceCoordinate(DWORD dwImg, POINT OutArraySealCenter[3]);
	__declspec(dllexport)  int WINAPI RCN_GetCutPicture(DWORD dwImg, char *pID, char *pPath, int iImgReversal, BYTE bQulity, long xDpi, long yDpi);
	__declspec(dllexport)  int WINAPI RCN_GetOriginPicture(DWORD dwImg, char *pID, char *pPath, BYTE bQulity, long xDpi, long yDpi);
	__declspec(dllexport)  int WINAPI RCN_GetScalePicture(DWORD dwImg, char *pID, char *pPath, float fscale, BYTE bQulity, long xDpi, long yDpi);
	__declspec(dllexport)  int WINAPI RCN_VerifySealCodeBar(DWORD dwImg, char *szInVerifyCode, POINT point, int iImgReversal, int iReserve);
	__declspec(dllexport)  int WINAPI RCN_AdjustCameraParameter(DWORD dwParent);
	__declspec(dllexport)  int WINAPI RCN_AdjustCameraCutArea(int iLeft, int iTop, int iRight, int iBottom);
	__declspec(dllexport)  int WINAPI RCN_AdjustCameraFocusExposure(double dFocus, double dExposure);
	__declspec(dllexport)  int WINAPI RCN_GetCameraParameter(int &iLeft, int &iTop, int &iRight, int &iBottom, double &dFocus, double &dExposure);
	__declspec(dllexport)  int WINAPI RCN_GetCutPictureToMemory(DWORD dwImg, RCNDATA *pRcnData, char* bufImage, int* pLength);
	__declspec(dllexport)  int WINAPI RCN_GetCutPictureSealPoint(RCNDATA *pRcnData);
}
#endif // !defined(AFX_COMMPUB_H__A7E91DB9_98F9_88AA_A888_734EB43039CA__INCLUDED_)