#pragma once
#include "commdef.h"

#include <map>
#include <string>

using namespace std;

typedef map<string, MMODELFILE*> MAPSTMODELFILE;
typedef map<string, MMODELFILE*>::iterator  ITERSTFILE;

class CModelFile
{
public:
	CModelFile(void);
	~CModelFile(void);

public:
	HANDLE m_hFile;
	MAPSTMODELFILE m_map;

public:
	int OpenModelFile(CString sFile);
	int ReadModelFile();
	int AddModelFeature(string szModNo, MMODELFILE *pData);
	PMMODELFILE GetModelFeature(string szModNo);
	int DeleteModelFeature(string szModNo);
	int DeleteAllModelFeature();
	int SaveAllModelFile();
	int GetModelCount();         //Ä£°å¸öÊý

};

