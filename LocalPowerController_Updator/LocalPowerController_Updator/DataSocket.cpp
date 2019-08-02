// DataSocket.cpp : 구현 파일입니다.
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


// CDataSocket 멤버 함수


CDataSocket::CDataSocket(CLocalPowerController_UpdatorDlg* pDlg)
{
	m_pDlg = pDlg;
}


void CDataSocket::OnReceive(int nErrorCode)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	int Length = Receive(m_pDlg->mRxBuf, RX_BUF_SIZE);
	if (Length > 0)
	{
		m_pDlg->ReceiveParsing(m_pDlg->mRxBuf, Length);
	}

	CSocket::OnReceive(nErrorCode);
}
