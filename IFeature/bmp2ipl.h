//bmp2ipl.h
#ifndef BMP2IPL_H
#define BMP2IPL_H

#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"

class BMP 
{
public:
	BMP():bmpData(NULL) {
		memset(&biHeader, 0, sizeof(biHeader));
	}
	BMP(const BMP & img);
	BMP(const IplImage &img);
	BMP(int width, int height, int bitCount);
	~BMP(){ if(bmpData){ delete [] bmpData;	bmpData = NULL;	} }

	bool CreateImage(const BITMAPINFOHEADER &biHeader);

	//Export
	IplImage * BMP2Ipl();
	//void Show(HWND hWnd, int nID);
	//void Show(CDC *pDC,  CRect & rect);
	//void Show(HWND hWnd);

	void ReSize(int newW, int newH);

private:
	void CopyData(char *dest, const char *src, int dataByteSize,
		bool isConvert, int height);
	// isConvert=true 进行顶左和底左之间的转换(顶左到底左或底左到顶左)，否则不转换

	// biSizeImage may be set to zero for BI_RGB bitmaps, so calculate it.
	int ImgSize() {
		return ( biHeader.biHeight * ((biHeader.biWidth * biHeader.biBitCount / 8 + 3) & (-4)));
	}

	BOOL IsSupport(const BMP &img);
	BOOL IsSupport(const IplImage &img);

public:
	void Clear();
	BITMAPINFOHEADER biHeader;
	BITMAPFILEHEADER bfHeader;
	unsigned char * bmpData;
};
#endif