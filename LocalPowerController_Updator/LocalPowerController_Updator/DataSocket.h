#pragma once
class CLocalPowerController_UpdatorDlg;
// CDataSocket ��� ����Դϴ�.

class CDataSocket : public CSocket
{
public:
	CDataSocket();
	virtual ~CDataSocket();
	CDataSocket(CLocalPowerController_UpdatorDlg* pDlg);
	virtual void OnReceive(int nErrorCode);
	CLocalPowerController_UpdatorDlg* m_pDlg;
};


