// CamaraDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "CameraDlg.h"
#include "IFeature.h"
#include "CvvImage.h"
#include "commpub.h"

// CCameraDlg 对话框

IMPLEMENT_DYNAMIC(CCameraDlg, CDialog)
CCameraDlg::CCameraDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CCameraDlg::IDD, pParent)
{
	m_pJpgBuffer = NULL;
	m_iLeft = 0;
	m_iTop = 0;
	m_iRight = 0;
	m_iBottom = 0;
}

CCameraDlg::~CCameraDlg()
{
	if(m_pJpgBuffer)
	{
		delete[] m_pJpgBuffer;
		m_pJpgBuffer = NULL;
	}
}

void CCameraDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CCameraDlg, CDialog)
	ON_WM_TIMER()
	ON_WM_DESTROY()
	ON_WM_CLOSE()
	ON_BN_CLICKED(IDC_BUTTON1, &CCameraDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CCameraDlg::OnBnClickedButton2)
END_MESSAGE_MAP()


// CCameraDlg 消息处理程序
void CCameraDlg::SetCutArea(int iLeft, int iTop, int iRight, int iBottom)
{
	m_iLeft = iLeft;
	m_iTop = iTop;
	m_iRight = iRight;
	m_iBottom = iBottom;
}

long CCameraDlg::CleanJpgBuffer()
{
	//int iSize = 20480000;
	int iSize = 40960000;
	if(!m_pJpgBuffer)
	{
		m_pJpgBuffer = new BYTE [ iSize ];
		if (m_pJpgBuffer == NULL )
		{
			return -1;
		}
	}
	memset(m_pJpgBuffer, 0, iSize);
	return 0;
}

BOOL CCameraDlg::OnInitDialog()
{
	//int iSize = 20480000;
	int iSize = 40960000;
	m_pJpgBuffer = new BYTE [ iSize ];

	CDialog::OnInitDialog();
	
	CString sLeft, sTop, sBottom, sRight;
	sLeft.Format("%d", m_iLeft);
	sTop.Format("%d", m_iTop);
	sBottom.Format("%d", m_iBottom);
	sRight.Format("%d", m_iRight);

	GetDlgItem(IDC_EDIT1)->SetWindowText(sLeft);
	GetDlgItem(IDC_EDIT3)->SetWindowText(sTop);
	GetDlgItem(IDC_EDIT2)->SetWindowText(sRight);
	GetDlgItem(IDC_EDIT4)->SetWindowText(sBottom);

	SetTimer(1, 30, NULL);
	return TRUE; 
}

void CCameraDlg::ShowPic(IplImage *img)
{
	CDC* pDC = GetDlgItem(IDC_STATIC_VIDEO)->GetDC();
	HDC hDC = pDC->GetSafeHdc();
	CvvImage dcimg;
	dcimg.CopyOf(img, 1);
	dcimg.DrawToHDC(hDC, &mypicrect);
}

void CCameraDlg::OnDisplayImage(BYTE *buffer, DWORD size) 
{
	/*
	CFileDialog dialog(true,NULL,NULL,0,"图片文件|*.*");
	dialog.m_ofn.lpstrTitle="打开图片文件";
	if (dialog.DoModal()!=IDOK)
	{
		return;
	}
	*/
	unsigned long u = RCN_CreateImage((char *)buffer, size);
	IplImage *oriimg = (IplImage *)u; 
	//transimg = cvLoadImage(dialog.GetPathName(),0);
	GetDlgItem(IDC_STATIC_VIDEO)->GetClientRect(&mypicrect);
	/*
	if (oriimg->width > mypicrect.Width() )
	{
		if (oriimg->height <= mypicrect.Height())
		{
			mypicrect.bottom = mypicrect.top+mypicrect.Height() * mypicrect.Width() / oriimg->width;
		}
		else
		{
			if (oriimg->height / mypicrect.Height() > oriimg->width / mypicrect.Width())
			{
				mypicrect.right = mypicrect.left + mypicrect.Width() * mypicrect.Height() / oriimg->height;
			}
			else
			{
				mypicrect.bottom = mypicrect.top + mypicrect.Height() * mypicrect.Width() / oriimg->width;
			}
		}
	}
	else
	{
		if (oriimg->height > mypicrect.Height())
		{
			mypicrect.right = mypicrect.left + mypicrect.Width() * mypicrect.Height() / oriimg->height;
		}
		else
		{
			mypicrect.right = mypicrect.left + oriimg->width;
			mypicrect.bottom = mypicrect.top + oriimg->height;
		}
	}
	*/
	ShowPic(oriimg);
	RCN_ReleaseImage(u);
}

BOOL CCameraDlg::DestroyWindow()
{



	return CDialog::DestroyWindow();
}

void CCameraDlg::OnTimer(UINT nIDEvent)
{
	switch(nIDEvent)
	{
	case 1:
		{
			if(CleanJpgBuffer() == 0)
			{
				//int iOutLen = 20480000;
				int iOutLen = 40960000;
				int iRet = RCN_Capture((char *)m_pJpgBuffer, &iOutLen);
				OnDisplayImage(m_pJpgBuffer, iOutLen);
			}
		}
		break;
	default:
		break;
	}
	CDialog::OnTimer (nIDEvent);
}

void CCameraDlg::OnClose()
{
	// TODO: Add your message handler code here and/or call default
	KillTimer(1);
	CDialog::OnClose();
}


void CCameraDlg::OnBnClickedButton1()
{
	RCN_OpenPropertyDlg();
}


//设置区域
void CCameraDlg::OnBnClickedButton2()
{
	CString sLeft, sTop, sBottom, sRight;
	GetDlgItem(IDC_EDIT1)->GetWindowText(sLeft);
	GetDlgItem(IDC_EDIT3)->GetWindowText(sTop);
	GetDlgItem(IDC_EDIT2)->GetWindowText(sRight);
	GetDlgItem(IDC_EDIT4)->GetWindowText(sBottom);
	RCN_AdjustCameraCutArea(atoi((LPSTR)(LPCTSTR)sLeft), atoi((LPSTR)(LPCTSTR)sTop), atoi((LPSTR)(LPCTSTR)sRight), atoi((LPSTR)(LPCTSTR)sBottom));
}
