
// LocalPowerController_Updator.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CLocalPowerController_UpdatorApp:
// �� Ŭ������ ������ ���ؼ��� LocalPowerController_Updator.cpp�� �����Ͻʽÿ�.
//

class CLocalPowerController_UpdatorApp : public CWinApp
{
public:
	CLocalPowerController_UpdatorApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CLocalPowerController_UpdatorApp theApp;