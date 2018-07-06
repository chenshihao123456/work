#if!defined CXRECTANGLE_H_2014_08_18
#define CXRECTANGLE_H_2014_08_18

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#include "opencv2/opencv.hpp"

class CxRectangle
{
public:
	CxRectangle(CvPoint i_pt[4]);
	CxRectangle(CvPoint pt1, CvPoint pt2, CvPoint pt3, CvPoint pt4);
	~CxRectangle();

private:
	CvPoint m_pt[4];
	void Calc(CvPoint i_pt[4]);

public:
	CvPoint GetTopLeftPoint() { return m_pt[0]; }
	CvPoint GetTopRightPoint() { return m_pt[1]; }
	CvPoint GetButtomLeftPoint() { return m_pt[2]; }
	CvPoint GetButtomRightPoint() { return m_pt[3]; }
};

#endif