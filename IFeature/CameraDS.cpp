//////////////////////////////////////////////////////////////////////
// Video Capture using DirectShow
// Author: Shiqi Yu (shiqi.yu@gmail.com)
// Thanks to:
//		HardyAI@OpenCV China
//		flymanbox@OpenCV China (for his contribution to function CameraName, and frame width/height setting)
// Last modification: April 9, 2009
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// 使用说明：
//   1. 将CameraDS.h CameraDS.cpp以及目录DirectShow复制到你的项目中
//   2. 菜单 Project->Settings->Settings for:(All configurations)->C/C++->Category(Preprocessor)->Additional include directories
//      设置为 DirectShow/Include
//   3. 菜单 Project->Settings->Settings for:(All configurations)->Link->Category(Input)->Additional library directories
//      设置为 DirectShow/Lib
//////////////////////////////////////////////////////////////////////

// CameraDS.cpp: implementation of the CCameraDS class.
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "CameraDS.h"
#include "IFeature.h"

#pragma comment(lib,"Strmiids.lib") 
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////



HMODULE  DumpModuleFeature1()  
{  
	HMODULE hMoudle = NULL;  
	GetModuleHandleEx(  
		GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,  
		(PCTSTR)DumpModuleFeature1,  
		&hMoudle);  
	return hMoudle;
} 


CCameraDS::CCameraDS()
{
	m_bConnected = false;
	m_nWidth = 0;
	m_nHeight = 0;
	m_bLock = false;
	m_bChanged = false;
	m_pFrame = NULL;
	m_nBufferSize = 0;

	m_pNullFilter = NULL;
	m_pMediaEvent = NULL;
	m_pSampleGrabberFilter = NULL;
	m_pGraph = NULL;
	m_nCamID = 0;


	char szDir[1024];
	memset(szDir, '\0', 1024);
	TCHAR szPath[MAX_PATH];
#ifdef TESTDEMO
	GetModuleFileName( GetModuleHandle("IFeature.dll"), szPath, MAX_PATH );
#else
	//GetModuleFileName(GetModuleHandle("ABC.Dfjy.Device.STAMP.IABCFeature.dll"), szPath, MAX_PATH);
	HMODULE hModule;	
	hModule = DumpModuleFeature1();
	GetModuleFileName(hModule, szPath, MAX_PATH);
#endif
	GetLongPathName(szPath, szDir, MAX_PATH);
	PathRemoveFileSpec(szDir);		

	char winDir[1024];
	memset(winDir, 0, sizeof(winDir));
	sprintf_s(winDir, "%s\\Featurex\\esm.ini", szDir);

	m_CameraType = GetPrivateProfileInt("Camera", "CameraType", 0, winDir);

	CoInitialize(NULL);
}

CCameraDS::~CCameraDS()
{
	CloseCamera();
	CoUninitialize();
}

void CCameraDS::CloseCamera()
{
	if(m_bConnected)
		m_pMediaControl->Stop();

	m_pGraph = NULL;
	m_pDeviceFilter = NULL;
	m_pMediaControl = NULL;
	m_pSampleGrabberFilter = NULL;
	m_pSampleGrabber = NULL;
	m_pGrabberInput = NULL;
	m_pGrabberOutput = NULL;
	m_pCameraOutput = NULL;
	m_pMediaEvent = NULL;
	m_pNullFilter = NULL;
	m_pNullInputPin = NULL;


	if (m_pFrame)
		cvReleaseImage(&m_pFrame);

	m_bConnected = false;
	m_nWidth = 0;
	m_nHeight = 0;
	m_bLock = false;
	m_bChanged = false;
	m_nBufferSize = 0;
}

bool CCameraDS::OpenCamera(int nCamID, bool bDisplayProperties, int nWidth, int nHeight, int nExpose, double nfocusLen)
{
	m_nCamID = nCamID;
	HRESULT hr = S_OK;
	CoInitialize(NULL);
	// Create the Filter Graph Manager.
	hr = CoCreateInstance(CLSID_FilterGraph, NULL, CLSCTX_INPROC,
							IID_IGraphBuilder, (void **)&m_pGraph);

	hr = CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER, 
							IID_IBaseFilter, (LPVOID *)&m_pSampleGrabberFilter);

	hr = m_pGraph->QueryInterface(IID_IMediaControl, (void **) &m_pMediaControl);
	hr = m_pGraph->QueryInterface(IID_IMediaEvent, (void **) &m_pMediaEvent);

	hr = CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER,
							IID_IBaseFilter, (LPVOID*) &m_pNullFilter);


	hr = m_pGraph->AddFilter(m_pNullFilter, L"NullRenderer");
	hr = m_pSampleGrabberFilter->QueryInterface(IID_ISampleGrabber, (void**)&m_pSampleGrabber);
	
	AM_MEDIA_TYPE   mt;
	ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
	mt.majortype = MEDIATYPE_Video;

	if(m_CameraType == 0)
	{
		mt.subtype = MEDIASUBTYPE_RGB24;
	}
	else
	{
		mt.subtype = MEDIASUBTYPE_MJPG;// MEDIASUBTYPE_RGB24; MEDIASUBTYPE_MJPG
	}
		
	mt.formattype = FORMAT_VideoInfo; 
	hr = m_pSampleGrabber->SetMediaType(&mt);
	MYFREEMEDIATYPE(mt);

	m_pGraph->AddFilter(m_pSampleGrabberFilter, L"Grabber");
 
	// Bind Device Filter.  We know the device because the id was passed in
	BindFilter(nCamID, &m_pDeviceFilter);
	m_pGraph->AddFilter(m_pDeviceFilter, NULL);

	CComPtr<IEnumPins> pEnum;
	m_pDeviceFilter->EnumPins(&pEnum);
 
	hr = pEnum->Reset();
	hr = pEnum->Next(1, &m_pCameraOutput, NULL); 

	pEnum = NULL; 
	m_pSampleGrabberFilter->EnumPins(&pEnum);
	pEnum->Reset();
	hr = pEnum->Next(1, &m_pGrabberInput, NULL); 

	pEnum = NULL;
	m_pSampleGrabberFilter->EnumPins(&pEnum);
	pEnum->Reset();
	pEnum->Skip(1);
	hr = pEnum->Next(1, &m_pGrabberOutput, NULL); 

	pEnum = NULL;
	m_pNullFilter->EnumPins(&pEnum);
	pEnum->Reset();
	hr = pEnum->Next(1, &m_pNullInputPin, NULL);

	//SetCrossBar();

	if (bDisplayProperties) 
	{
		CComPtr<ISpecifyPropertyPages> pPages;
		HRESULT hr = m_pCameraOutput->QueryInterface(IID_ISpecifyPropertyPages, (void**)&pPages);
		if (SUCCEEDED(hr))
		{
			PIN_INFO PinInfo;
			m_pCameraOutput->QueryPinInfo(&PinInfo);

			CAUUID caGUID;
			pPages->GetPages(&caGUID);

			OleCreatePropertyFrame(NULL, 0, 0,
						L"Property Sheet", 1,
						(IUnknown **)&(m_pCameraOutput.p),
						caGUID.cElems,
						caGUID.pElems,
						0, 0, NULL);
			CoTaskMemFree(caGUID.pElems);
			PinInfo.pFilter->Release();
		}
		pPages = NULL;
	}
	else 
	{
		//设置相机属性
		PIN_INFO PinInfo;
		m_pCameraOutput->QueryPinInfo(&PinInfo);

		IAMVideoProcAmp * pAmp = NULL;
		hr = PinInfo.pFilter->QueryInterface(IID_IAMVideoProcAmp, (void**)&pAmp);	
		//if (pAmp)
		//{
		//	HRESULT hr = pAmp->Set(VideoProcAmp_Brightness, 0, 2);//亮度 
		//	pAmp->Release();
		//}	
		//IAMCameraContro_Exposure
		IAMCameraControl *m_pCtrl;
		PinInfo.pFilter->QueryInterface(IID_IAMCameraControl, (void **)&m_pCtrl );
		m_pCtrl->Set( CameraControl_Exposure, nExpose, CameraControl_Flags_Manual );
		m_pCtrl->Set( CameraControl_Focus, nfocusLen, CameraControl_Flags_Manual );
		PinInfo.pFilter->Release();


		//////////////////////////////////////////////////////////////////////////////
		// 加入由 lWidth和lHeight设置的摄像头的宽和高 的功能，默认320*240
		// 
		//////////////////////////////////////////////////////////////////////////////
	   int _Width = nWidth, _Height = nHeight;
	   IAMStreamConfig*   iconfig; 
	   iconfig = NULL;
	   hr = m_pCameraOutput->QueryInterface(IID_IAMStreamConfig,   (void**)&iconfig);   
      
	   AM_MEDIA_TYPE* pmt;    
	   if(iconfig->GetFormat(&pmt) !=S_OK) 
	   {
		  //printf("GetFormat Failed ! \n");
		  return   false;   
	   }
      
	   VIDEOINFOHEADER*   phead;
	   if ( pmt->formattype == FORMAT_VideoInfo)   
	   {   
			phead=( VIDEOINFOHEADER*)pmt->pbFormat;   
			phead->bmiHeader.biWidth = _Width;   
			phead->bmiHeader.biHeight = _Height;   
			if(( hr=iconfig->SetFormat(pmt)) != S_OK )   
			{
				return   false;
			}
		}   
		iconfig->Release();   
		iconfig=NULL;   
		MYFREEMEDIATYPE(*pmt);
	}

	hr = m_pGraph->Connect(m_pCameraOutput, m_pGrabberInput);
	hr = m_pGraph->Connect(m_pGrabberOutput, m_pNullInputPin);
	if (FAILED(hr))
	{
		switch(hr)
		{
			case VFW_S_NOPREVIEWPIN :
				break;
			case E_FAIL :
				break;
			case E_INVALIDARG :
				break;
			case E_POINTER :
				break;
		}
	}

	m_pSampleGrabber->SetBufferSamples(TRUE);
	m_pSampleGrabber->SetOneShot(TRUE);  
	hr = m_pSampleGrabber->GetConnectedMediaType(&mt);
	if(FAILED(hr))
		return false;

	VIDEOINFOHEADER *videoHeader;
	videoHeader = reinterpret_cast<VIDEOINFOHEADER*>(mt.pbFormat);
	m_nWidth = videoHeader->bmiHeader.biWidth;
	m_nHeight = videoHeader->bmiHeader.biHeight;
	m_bConnected = true;
	pEnum = NULL;
	return true;
}


bool CCameraDS::BindFilter(int nCamID, IBaseFilter **pFilter)
{
	if (nCamID < 0)
		return false;
 
    // enumerate all video capture devices
	CComPtr<ICreateDevEnum> pCreateDevEnum;
	HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
								IID_ICreateDevEnum, (void**)&pCreateDevEnum);
	if (hr != NOERROR)
	{
		return false;
	}

    CComPtr<IEnumMoniker> pEm;
    hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
        &pEm, 0);
    if (hr != NOERROR) 
	{
		return false;
    }

    pEm->Reset();
    ULONG cFetched;
    IMoniker *pM;
	int index = 0;
    while(hr = pEm->Next(1, &pM, &cFetched), hr==S_OK, index <= nCamID)
    {
		IPropertyBag *pBag;
		hr = pM->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pBag);
		if(SUCCEEDED(hr)) 
		{
			VARIANT var;
			var.vt = VT_BSTR;
			hr = pBag->Read(L"FriendlyName", &var, NULL);
			if (hr == NOERROR) 
			{
				if (index == nCamID)
				{
					pM->BindToObject(0, 0, IID_IBaseFilter, (void**)pFilter);
				}
				SysFreeString(var.bstrVal);
			}
			pBag->Release();
		}
		pM->Release();
		index++;
    }
	pCreateDevEnum = NULL;
	return true;
}



//将输入crossbar变成PhysConn_Video_Composite
void CCameraDS::SetCrossBar()
{
	int i;
	IAMCrossbar *pXBar1 = NULL;
	ICaptureGraphBuilder2 *pBuilder = NULL;
	HRESULT hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, NULL,
					CLSCTX_INPROC_SERVER, IID_ICaptureGraphBuilder2, 
					(void **)&pBuilder);

	if (SUCCEEDED(hr))
	{
		hr = pBuilder->SetFiltergraph(m_pGraph);
	}

	hr = pBuilder->FindInterface(&LOOK_UPSTREAM_ONLY, NULL, 
								m_pDeviceFilter,IID_IAMCrossbar, (void**)&pXBar1);
	if (SUCCEEDED(hr)) 
	{
  		long OutputPinCount;
		long InputPinCount;
		long PinIndexRelated;
		long PhysicalType;
		long inPort = 0;
		long outPort = 0;

		pXBar1->get_PinCounts(&OutputPinCount,&InputPinCount);
		for( i =0;i<InputPinCount;i++)
		{
			pXBar1->get_CrossbarPinInfo(TRUE,i,&PinIndexRelated,&PhysicalType);
			if(PhysConn_Video_Composite==PhysicalType) 
			{
				inPort = i;
				break;
			}
		}
		for( i =0;i<OutputPinCount;i++)
		{
			pXBar1->get_CrossbarPinInfo(FALSE,i,&PinIndexRelated,&PhysicalType);
			if(PhysConn_Video_VideoDecoder==PhysicalType) 
			{
				outPort = i;
				break;
			}
		}
  
		if(S_OK==pXBar1->CanRoute(outPort,inPort))
		{
			pXBar1->Route(outPort,inPort);
		}
		pXBar1->Release();  
	}
	pBuilder->Release();
}


/*
The returned image can not be released.
*/
IplImage* CCameraDS::QueryFrame()
{
	long evCode;
	long size = 0;

	m_pMediaControl->Run();
	m_pMediaEvent->WaitForCompletion(INFINITE, &evCode);
	m_pSampleGrabber->GetCurrentBuffer(&size, NULL);

	//if the buffer size changed
	if (size != m_nBufferSize)
	{
		if (m_pFrame)
			cvReleaseImage(&m_pFrame);

		m_nBufferSize = size;
		m_pFrame = cvCreateImage(cvSize(m_nWidth, m_nHeight), IPL_DEPTH_8U, 3);
	}

	m_pSampleGrabber->GetCurrentBuffer(&m_nBufferSize, (long*)m_pFrame->imageData);
	
	if(m_CameraType == 0)
		cvFlip(m_pFrame);

	return m_pFrame;
}

int CCameraDS::CameraCount()
{

	int count = 0;
 	CoInitialize(NULL);

   // enumerate all video capture devices
	CComPtr<ICreateDevEnum> pCreateDevEnum;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
									IID_ICreateDevEnum, (void**)&pCreateDevEnum);

    CComPtr<IEnumMoniker> pEm;
    hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
        &pEm, 0);
    if (hr != NOERROR) 
	{
		return count;
    }

    pEm->Reset();
    ULONG cFetched;
    IMoniker *pM;
    while(hr = pEm->Next(1, &pM, &cFetched), hr==S_OK)
    {
		count++;
    }

	pCreateDevEnum = NULL;
	pEm = NULL;
	return count;
}

int CCameraDS::CameraName(int nCamID, char* sName, int nBufferSize)
{
	int count = 0;
 	CoInitialize(NULL);

   // enumerate all video capture devices
	CComPtr<ICreateDevEnum> pCreateDevEnum;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER,
									IID_ICreateDevEnum, (void**)&pCreateDevEnum);

    CComPtr<IEnumMoniker> pEm;
    hr = pCreateDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
        &pEm, 0);
    if (hr != NOERROR) return 0;


    pEm->Reset();
    ULONG cFetched;
    IMoniker *pM;
    while(hr = pEm->Next(1, &pM, &cFetched), hr==S_OK)
    {
		if (count == nCamID)
		{
			IPropertyBag *pBag=0;
			hr = pM->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pBag);
			if(SUCCEEDED(hr))
			{
				VARIANT var;
				var.vt = VT_BSTR;
				hr = pBag->Read(L"FriendlyName", &var, NULL); //还有其他属性,像描述信息等等...
	            if(hr == NOERROR)
		        {
			        //获取设备名称			
					WideCharToMultiByte(CP_ACP,0,var.bstrVal,-1,sName, nBufferSize ,"",NULL);

	                SysFreeString(var.bstrVal);				
		        }
			    pBag->Release();
			}
			pM->Release();

			break;
		}
		count++;
    }

	pCreateDevEnum = NULL;
	pEm = NULL;

	return 1;
}

bool CCameraDS::isOpened()
{
	return m_bConnected;
}


int CCameraDS::OpenCameraVideoProperty()
{
	CComPtr<ISpecifyPropertyPages> pPages;
	HRESULT hr = m_pCameraOutput->QueryInterface(IID_ISpecifyPropertyPages, (void**)&pPages);
	if (SUCCEEDED(hr))
	{
		PIN_INFO PinInfo;
		m_pCameraOutput->QueryPinInfo(&PinInfo);

		CAUUID caGUID;
		pPages->GetPages(&caGUID);

		OleCreatePropertyFrame(NULL, 0, 0,
			L"Property Sheet", 1,
			(IUnknown **)&(m_pCameraOutput.p),
			caGUID.cElems,
			caGUID.pElems,
			0, 0, NULL);
		CoTaskMemFree(caGUID.pElems);
		PinInfo.pFilter->Release();
	}
	pPages = NULL;
	return 0;
}


HRESULT getDevice(IBaseFilter** gottaFilter, int deviceId, WCHAR * wDeviceName, char * nDeviceName)
{
	BOOL done = false;
	int deviceCounter = 0;

	// Create the System Device Enumerator.
	ICreateDevEnum *pSysDevEnum = NULL;
	HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (void **)&pSysDevEnum);
	if (FAILED(hr))
	{
		return hr;
	}

	// Obtain a class enumerator for the video input category.
	IEnumMoniker *pEnumCat = NULL;
	hr = pSysDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnumCat, 0);

	if (hr == S_OK)
	{
		// Enumerate the monikers.
		IMoniker *pMoniker = NULL;
		ULONG cFetched;
		while ((pEnumCat->Next(1, &pMoniker, &cFetched) == S_OK) && (!done))
		{
			if(deviceCounter == deviceId)
			{
				// Bind the first moniker to an object
				IPropertyBag *pPropBag;
				hr = pMoniker->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pPropBag);
				if (SUCCEEDED(hr))
				{
					// To retrieve the filter's friendly name, do the following:
					VARIANT varName;
					VariantInit(&varName);
					hr = pPropBag->Read(L"FriendlyName", &varName, 0);
					if (SUCCEEDED(hr))
					{

						//copy the name to nDeviceName & wDeviceName
						int count = 0;
						while( varName.bstrVal[count] != 0x00 ) {
							wDeviceName[count] = varName.bstrVal[count];
							nDeviceName[count] = (char)varName.bstrVal[count];
							count++;
						}

						// We found it, so send it back to the caller
						hr = pMoniker->BindToObject(NULL, NULL, IID_IBaseFilter, (void**)gottaFilter);
						done = true;
					}
					VariantClear(&varName);
					pPropBag->Release();
					pPropBag = NULL;
					pMoniker->Release();
					pMoniker = NULL;
				}
			}
			deviceCounter++;
		}
		pEnumCat->Release();
		pEnumCat = NULL;
	}
	pSysDevEnum->Release();
	pSysDevEnum = NULL;

	if (done) 
	{
		return hr;    // found it, return native error
	} 
	else 
	{
		return VFW_E_NOT_FOUND;    // didn't find it error
	}
}


HRESULT CCameraDS::ShowFilterPropertyPages(IBaseFilter *pFilter, CString sName)
{
	ISpecifyPropertyPages *pSpec;
	CAUUID cauuid;

	HRESULT hr = pFilter->QueryInterface(IID_ISpecifyPropertyPages,
		(void **)&pSpec);
	if(hr == S_OK)
	{
		hr = pSpec->GetPages(&cauuid);

		hr = OleCreatePropertyFrame(NULL, 30, 30, NULL, 1,
			(IUnknown **)&pFilter, cauuid.cElems,
			(GUID *)cauuid.pElems, 0, 0, NULL);

		CoTaskMemFree(cauuid.pElems);
		pSpec->Release();
	}
	/*
	ISpecifyPropertyPages *pProp;
	HRESULT hr = pFilter->QueryInterface(IID_ISpecifyPropertyPages, (void **)&pProp);
	if (SUCCEEDED(hr))
	{
		// Get the filter's name and IUnknown pointer.
		FILTER_INFO FilterInfo;
		hr = pFilter->QueryFilterInfo(&FilterInfo);
		USES_CONVERSION;
		//char *p = W2A(FilterInfo.achName);
		wchar_t *pw = A2W((LPSTR)(LPCTSTR)sName);
		//AfxMessageBox(p);
		IUnknown *pFilterUnk;
		pFilter->QueryInterface(IID_IUnknown, (void **)&pFilterUnk);

		// Show the page.
		CAUUID caGUID;
		pProp->GetPages(&caGUID);
		pProp->Release();
		OleCreatePropertyFrame(
			NULL,                   // Parent window
			0, 0,                   // Reserved
			pw,//FilterInfo.achName,     // Caption for the dialog box
			1,                      // Number of objects (just the filter)
			&pFilterUnk,            // Array of object pointers.
			caGUID.cElems,          // Number of property pages
			caGUID.pElems,          // Array of property page CLSIDs
			0,                      // Locale identifier
			0, NULL                 // Reserved
			);

		// Clean up.
		if(pFilterUnk)pFilterUnk->Release();
		if(FilterInfo.pGraph)FilterInfo.pGraph->Release();
		CoTaskMemFree(caGUID.pElems);
	}*/

	return hr;
}

int CCameraDS::OpenCameraProperty(CString sName)
{
	if(m_pDeviceFilter)
	{
		ShowFilterPropertyPages(m_pDeviceFilter, sName);
	}
	return 0;
}