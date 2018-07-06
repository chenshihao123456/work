#include "StdAfx.h"
#include "IFeature.h"
#include "CxRectangle.h"


void CxRectangle::Calc(CvPoint i_pt[4])
{
	int ic1 = 0, ic2 = 0, ic3 = 0, ic4 = 0;
	if((abs(i_pt[0].x - i_pt[1].x) < 3))    
	{
		if(i_pt[0].x < i_pt[2].x)         //×ó±ßµã
		{
			if(i_pt[0].y < i_pt[1].y)
			{
				m_pt[0] = i_pt[0];
				m_pt[2] = i_pt[1];
			}
			else
			{
				m_pt[0] = i_pt[1];
				m_pt[2] = i_pt[0];
			}

			if(i_pt[2].y < i_pt[3].y)
			{
				m_pt[1] = i_pt[2];
				m_pt[3] = i_pt[3];
			}
			else
			{
				m_pt[1] = i_pt[3];
				m_pt[3] = i_pt[2];
			}
		}
		else                               //ÓÒ±ßµã
		{
			if(i_pt[0].y < i_pt[1].y)
			{
				m_pt[1] = i_pt[0];
				m_pt[3] = i_pt[1];
			}
			else
			{
				m_pt[1] = i_pt[1];
				m_pt[3] = i_pt[0];
			}

			if(i_pt[2].y < i_pt[3].y)
			{
				m_pt[0] = i_pt[2];
				m_pt[2] = i_pt[3];
			}
			else
			{
				m_pt[0] = i_pt[3];
				m_pt[2] = i_pt[2];
			}
		}
	}
	else if((abs(i_pt[0].x - i_pt[2].x) < 3)) 
	{
		if(i_pt[0].x < i_pt[1].x)         //×ó±ßµã
		{
			if(i_pt[0].y < i_pt[2].y)
			{
				m_pt[0] = i_pt[0];
				m_pt[2] = i_pt[2];
			}
			else
			{
				m_pt[0] = i_pt[2];
				m_pt[2] = i_pt[0];
			}

			if(i_pt[1].y < i_pt[3].y)
			{
				m_pt[1] = i_pt[1];
				m_pt[3] = i_pt[3];
			}
			else
			{
				m_pt[1] = i_pt[3];
				m_pt[3] = i_pt[1];
			}
		}
		else                               //ÓÒ±ßµã
		{
			if(i_pt[0].y < i_pt[2].y)
			{
				m_pt[1] = i_pt[0];
				m_pt[3] = i_pt[2];
			}
			else
			{
				m_pt[1] = i_pt[2];
				m_pt[3] = i_pt[0];
			}

			if(i_pt[1].y < i_pt[3].y)
			{
				m_pt[0] = i_pt[1];
				m_pt[2] = i_pt[3];
			}
			else
			{
				m_pt[0] = i_pt[3];
				m_pt[2] = i_pt[1];
			}
		}
	}
	else
	{
		if(i_pt[0].x < i_pt[1].x)         //×ó±ßµã
		{
			if(i_pt[0].y < i_pt[3].y)
			{
				m_pt[0] = i_pt[0];
				m_pt[2] = i_pt[3];
			}
			else
			{
				m_pt[0] = i_pt[3];
				m_pt[2] = i_pt[0];
			}

			if(i_pt[1].y < i_pt[2].y)
			{
				m_pt[1] = i_pt[1];
				m_pt[3] = i_pt[2];
			}
			else
			{
				m_pt[1] = i_pt[2];
				m_pt[3] = i_pt[1];
			}
		}
		else                               //ÓÒ±ßµã
		{
			if(i_pt[0].y < i_pt[3].y)
			{
				m_pt[1] = i_pt[0];
				m_pt[3] = i_pt[3];
			}
			else
			{
				m_pt[1] = i_pt[3];
				m_pt[3] = i_pt[0];
			}

			if(i_pt[1].y < i_pt[2].y)
			{
				m_pt[0] = i_pt[1];
				m_pt[2] = i_pt[2];
			}
			else
			{
				m_pt[0] = i_pt[2];
				m_pt[2] = i_pt[1];
			}
		}
	}
}

CxRectangle::CxRectangle(CvPoint i_pt[4])
{
	Calc(i_pt);
}


CxRectangle::CxRectangle(CvPoint pt1, CvPoint pt2, CvPoint pt3, CvPoint pt4)
{
	CvPoint i_pt[4];
	i_pt[0] = pt1;
	i_pt[1] = pt2;
	i_pt[2] = pt3;
	i_pt[3] = pt4;
	Calc(i_pt);
}


CxRectangle::~CxRectangle()
{
}