// DataSocket.cpp : ���� �����Դϴ�.
//

#include "stdafx.h"
#include "LocalPowerController_Updator.h"
#include "DataSocket.h"
#include "LocalPowerController_UpdatorDlg.h"


// CDataSocket

CDataSocket::CDataSocket()
	: m_pDlg(NULL)
{
}

CDataSocket::~CDataSocket()
{
}


// CDataSocket ��� �Լ�


CDataSocket::CDataSocket(CLocalPowerController_UpdatorDlg* pDlg)
{
	m_pDlg = pDlg;
}


void CDataSocket::OnReceive(int nErrorCode)
{
	// TODO: ���⿡ Ư��ȭ�� �ڵ带 �߰� ��/�Ǵ� �⺻ Ŭ������ ȣ���մϴ�.
	int Length = Receive(m_pDlg->mRxBuf, RX_BUF_SIZE);
	if (Length > 0)
	{
		m_pDlg->ReceiveParsing(m_pDlg->mRxBuf, Length);
	}

	CSocket::OnReceive(nErrorCode);
}
