#pragma once
#include "afxcmn.h"
#include "afxwin.h"
#include "resource.h"
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
// CCameraDlg 对话框

class CCameraDlg : public CDialog
{
	DECLARE_DYNAMIC(CCameraDlg)

public:
	CCameraDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CCameraDlg();
	CRect mypicrect;

// 对话框数据
	enum { IDD = IDD_CAMERA_DIALOG };

	

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持


	afx_msg void OnTimer(UINT nIDEvent);
	DECLARE_MESSAGE_MAP()
public:
	void OnDisplayImage(BYTE *buffer, DWORD size);
	void ShowPic(IplImage *img);
	virtual BOOL OnInitDialog();
	virtual BOOL DestroyWindow();

	afx_msg void OnClose();
	long CleanJpgBuffer();

public:
	BYTE* m_pJpgBuffer;
	int m_iLeft;
	int m_iTop;
	int m_iBottom;
	int m_iRight;
	void SetCutArea(int iLeft, int iTop, int iRight, int iBottom);
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
};
