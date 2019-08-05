
// LocalPowerController_UpdatorDlg.h : ��� ����
//

#pragma once
#include "afxwin.h"
class CDataSocket;


// CLocalPowerController_UpdatorDlg ��ȭ ����
class CLocalPowerController_UpdatorDlg : public CDialogEx
{
// �����Դϴ�.
public:
	CLocalPowerController_UpdatorDlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_LOCALPOWERCONTROLLER_UPDATOR_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.


// �����Դϴ�.
protected:
	HICON m_hIcon;

	// ������ �޽��� �� �Լ�
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	BYTE mRxBuf[RX_BUF_SIZE];
	BYTE mTxBuf[TX_BUF_SIZE];
	BYTE mRxDataField[RX_BUF_SIZE];
	BYTE mParsingState;
	UINT16 mRxCks;
	UINT16 mRxDataLength;
	BOOL bIsFileOpened;
	BYTE mRdBuf[240 * 1024];
	BYTE mTotalPage;
	BYTE mErasePage;
	BYTE mPageCnt;
	BYTE mPageNum;
	UINT16 mUpdateFile_FwVer;
	UINT16 mUpdateFile_Crc16;
	CEdit m_cEdit_Log;
	CComboBox m_cCombo_ip;
	void ReceiveParsing(BYTE* pReceiveBuf, int Length);
	void LogAdd(CString EditStr);
	void SendWithCks(BYTE* pData, UINT16 Length);
	CString StrWithTime(CString StrOut, BYTE nOption);
	UINT16 Get_Crc16(BYTE* pCrcData, UINT Length);
	CString mStrCombo_ip;
	CDataSocket* m_pDataSocket;
	afx_msg void OnBnClickedBtnFileOpen();
	afx_msg void OnCbnSelchangeCmbIp();
	afx_msg void OnBnClickedBtnConnect();
	afx_msg void OnBnClickedBtnReset();
	afx_msg void OnBnClickedBtnGetVer();
	afx_msg void OnBnClickedBtnClearLog();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	UINT16 mRxDataCnt;
	BYTE mRxCmd;
};
