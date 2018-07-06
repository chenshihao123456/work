#include "stdafx.h"
#include "ModelFile.h"


CModelFile::CModelFile(void)
{
	m_hFile = INVALID_HANDLE_VALUE;
}

CModelFile::~CModelFile(void)
{
	DeleteAllModelFeature();
	if (m_hFile != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hFile);
	}
}

int CModelFile::OpenModelFile(CString sFile)
{
	int iRet = -1;
	if(sFile.IsEmpty()) 
		return iRet;

	//FILE_FLAG_WRITE_THROUGH
	//m_hFile = CreateFile(sFile, GENERIC_READ|GENERIC_WRITE, FILE_SHARE_DELETE | FILE_SHARE_WRITE | FILE_SHARE_READ, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	m_hFile = CreateFile(sFile, GENERIC_READ|GENERIC_WRITE, FILE_SHARE_DELETE | FILE_SHARE_WRITE | FILE_SHARE_READ, 0, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, 0);
	if (m_hFile != INVALID_HANDLE_VALUE)
	{		
		iRet = 0;
	}
	return iRet;
}

int CModelFile::ReadModelFile()
{
	if (m_hFile == INVALID_HANDLE_VALUE)
		return -1;

	DWORD dwFileSize = 0;
	LARGE_INTEGER tIntSize; 
	GetFileSizeEx(m_hFile, &tIntSize); 
	dwFileSize = tIntSize.LowPart;         //模板文件大小，这个参数就够用了。
	int iBlockSize = sizeof(MMODELFILE);
	int icc = dwFileSize / iBlockSize;
	if(icc > 0)
	{
		int i=0;		
		while(i<icc)
		{	
			PMMODELFILE pBuffer = new MMODELFILE();

			//memset(pBuffer, '\0', dwFileSize + 1);
			memset(pBuffer, 0, iBlockSize);
			DWORD dwCount = 0;
			ReadFile(m_hFile, pBuffer, iBlockSize, &dwCount, NULL); 
			if(dwCount != iBlockSize)
			{
				delete pBuffer;
				pBuffer = NULL;
				break;
			}
			std::string modelNo = pBuffer->pFeature.strModNo;
			m_map.insert(pair<string, PMMODELFILE>(modelNo, pBuffer));
			i++;
		}
	}
	return 0;
}

int CModelFile::AddModelFeature(string szModNo, MMODELFILE *pData)
{
	if(!pData)
		return -1;

	ITERSTFILE iter = m_map.find(szModNo);
	if(iter != m_map.end())
	{
		return -2;		
	}
	else
	{
		MMODELFILE *pBuffer = new MMODELFILE();
		memcpy(pBuffer, pData, sizeof(MMODELFILE));
		m_map.insert(pair<string, PMMODELFILE>(szModNo, pBuffer));
	}
	return 0;
}

PMMODELFILE CModelFile::GetModelFeature(string szModNo)
{
	ITERSTFILE iter = m_map.find(szModNo);
	if(iter != m_map.end())
	{
		return iter->second;
	}
	return NULL;
}

int CModelFile::DeleteModelFeature(string szModNo)
{
	ITERSTFILE iter = m_map.find(szModNo);
	if(iter != m_map.end())
	{
		MMODELFILE *pBuffer = iter->second;
		m_map.erase(iter);
		if(pBuffer)
		{
			delete pBuffer;
			pBuffer = NULL;
		}	
	}
	else
		return -1;

	return 0;
}


int CModelFile::DeleteAllModelFeature()
{
	ITERSTFILE  iter1, iter2;
	for(iter1 = m_map.begin(); iter1 != m_map.end(); )
	{
		iter2 = iter1;
		iter1++;
		PMMODELFILE pBuffer = iter2->second;	
		m_map.erase(iter2);
		if(pBuffer)
		{
			delete pBuffer;
			pBuffer = NULL;
		}
	}
	return 0;
}

int CModelFile::SaveAllModelFile()
{
	if (m_hFile == INVALID_HANDLE_VALUE)
		return -1;

	LARGE_INTEGER tIntPos;
	tIntPos.QuadPart = 0;
	DWORD dwPtr = SetFilePointerEx(m_hFile, tIntPos, NULL, FILE_BEGIN);
	int iLen = sizeof(MMODELFILE);
	ITERSTFILE  iter;
	for(iter = m_map.begin(); iter != m_map.end(); iter++)
	{	
		PMMODELFILE pBuffer = iter->second;
		DWORD written = 0;
		BOOL res = WriteFile(m_hFile, (const char*)pBuffer, iLen, &written, 0);
		if (res && (written == iLen))
		{
			continue;
		}
		else
		{
			break;
		}
	}

	if(iter == m_map.end())
	{
		FlushFileBuffers(m_hFile);  //数据立即写入磁盘
		return 0;
	}

	return -2;
}

int CModelFile::GetModelCount()
{
	return m_map.size();
}