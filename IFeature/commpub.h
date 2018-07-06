#if !defined(AFX_COMMPUB_H__A7E91DB9_98F9_88AA_A888_734EB43039CA__INCLUDED_)
#define AFX_COMMPUB_H__A7E91DB9_98F9_88AA_A888_734EB43039CA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#pragma pack(push)
#pragma pack(1) 
typedef struct _RcnData_
{
	char szCheckTypeName[128];  //ƾ֤��������
	char szCheckTypeID[128];    //ƾ֤���ͱ��
	char szCheckNO[30];         //ƾ֤���
	char szInputName[128];      //�����ƾ֤�������ơ�
	int iX;                     //ԭͼ����X����
	int iY;                     //ԭͼ����Y����
	int iCheckX;                //ƾ֤����X����
	int iCheckY;                //ƾ֤����Y����
	int iCutX;                  //�ü������������Ͻ�X����ֵ
	int iCutY;                  //�ü������������Ͻ�Y����ֵ 
	int iCutWidth;              //�ü�ͼ�Ŀ��
	int iCutHeight;             //�ü�ͼ�ĸ߶�
	int iYzmX;                  //��֤��X����
	int iYzmY;                  //��֤��Y����
	double dAngle;              //ʶ��Ƕ�
	float fCenterX;             //���ĵ�X����
	float fCenterY;             //���ĵ�Y����
	BYTE bDirectionTitle;       //ƾ֤�ķ��� 0:�� 1���� 2:�� 3����
	double dAngleSeal;          //���½Ƕ�;
	int iUnitNum;
	char szInputNo[32];        //�����ƾ֤�ı��

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
	int XScaleplate;          //У׼��X����Ŀ��
	int YScaleplate;          //У׼��Y����Ŀ��
	double PixNum;            //1mm��������	
	int Bright;               //���ڲü������ȵ���;Ĭ��120
	int Threshold;            //�ü���ص���ֵ��Ĭ�����ó�100
	int Debug;                //Ԥ�����Խӿ�,Ĭ��Ϊ0;
	char szPathModel[260];    //ģ��·��, ����ǿգ���ͬ����; �������·��
	char szModelNameHead[260];    //ģ������ ���е�
	char szModelNameBranch[260];    //ģ������ ���е�
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