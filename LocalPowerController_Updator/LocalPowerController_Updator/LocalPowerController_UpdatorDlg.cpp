
// LocalPowerController_UpdatorDlg.cpp : ���� ����
//

#include "stdafx.h"
#include "LocalPowerController_Updator.h"
#include "LocalPowerController_UpdatorDlg.h"
#include "afxdialogex.h"
#include "DataSocket.h"	// bySeo

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

UINT16 Crc16Table[256] = {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

// ���� ���α׷� ������ ���Ǵ� CAboutDlg ��ȭ �����Դϴ�.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

// �����Դϴ�.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CLocalPowerController_UpdatorDlg ��ȭ ����



CLocalPowerController_UpdatorDlg::CLocalPowerController_UpdatorDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_LOCALPOWERCONTROLLER_UPDATOR_DIALOG, pParent)
	, mParsingState(0)
	, bIsFileOpened(FALSE)
	, mTotalPage(0)
	, mErasePage(0)
	, mPageCnt(0)
	, mPageNum(0)
	, mUpdateFile_FwVer(0)
	, mUpdateFile_Crc16(0)
	, mStrCombo_ip(_T(""))
	, m_pDataSocket(NULL)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CLocalPowerController_UpdatorDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT_LOG, m_cEdit_Log);
	DDX_Control(pDX, IDC_CMB_IP, m_cCombo_ip);
	DDX_CBString(pDX, IDC_CMB_IP, mStrCombo_ip);
}

BEGIN_MESSAGE_MAP(CLocalPowerController_UpdatorDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_FILE_OPEN, &CLocalPowerController_UpdatorDlg::OnBnClickedBtnFileOpen)
	ON_CBN_SELCHANGE(IDC_CMB_IP, &CLocalPowerController_UpdatorDlg::OnCbnSelchangeCmbIp)
	ON_BN_CLICKED(IDC_BTN_CONNECT, &CLocalPowerController_UpdatorDlg::OnBnClickedBtnConnect)
	ON_BN_CLICKED(IDC_BTN_RESET, &CLocalPowerController_UpdatorDlg::OnBnClickedBtnReset)
	ON_BN_CLICKED(IDC_BTN_GET_VER, &CLocalPowerController_UpdatorDlg::OnBnClickedBtnGetVer)
	ON_BN_CLICKED(IDC_BTN_CLEAR_LOG, &CLocalPowerController_UpdatorDlg::OnBnClickedBtnClearLog)
END_MESSAGE_MAP()


// CLocalPowerController_UpdatorDlg �޽��� ó����

BOOL CLocalPowerController_UpdatorDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// �ý��� �޴��� "����..." �޴� �׸��� �߰��մϴ�.

	// IDM_ABOUTBOX�� �ý��� ��� ������ �־�� �մϴ�.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// �� ��ȭ ������ �������� �����մϴ�.  ���� ���α׷��� �� â�� ��ȭ ���ڰ� �ƴ� ��쿡��
	//  �����ӿ�ũ�� �� �۾��� �ڵ����� �����մϴ�.
	SetIcon(m_hIcon, TRUE);			// ū �������� �����մϴ�.
	SetIcon(m_hIcon, FALSE);		// ���� �������� �����մϴ�.

	// TODO: ���⿡ �߰� �ʱ�ȭ �۾��� �߰��մϴ�.
	BYTE Cnt;
	CString Str;

	SetWindowPos(NULL, 10, 10, 0, 0, SWP_NOSIZE);
	m_cEdit_Log.SetWindowTextA(_T("����������ġ �߿��� ������Ʈ\r\n"));
	for (Cnt = 201; Cnt <= 203; Cnt++)
	{
		Str.Format(_T("192.168.0.%d"), Cnt);
		m_cCombo_ip.AddString(Str);
	}
	mStrCombo_ip = _T("192.168.0.201");
	UpdateData(FALSE);

	return TRUE;  // ��Ŀ���� ��Ʈ�ѿ� �������� ������ TRUE�� ��ȯ�մϴ�.
}

void CLocalPowerController_UpdatorDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// ��ȭ ���ڿ� �ּ�ȭ ���߸� �߰��� ��� �������� �׸�����
//  �Ʒ� �ڵ尡 �ʿ��մϴ�.  ����/�� ���� ����ϴ� MFC ���� ���α׷��� ��쿡��
//  �����ӿ�ũ���� �� �۾��� �ڵ����� �����մϴ�.

void CLocalPowerController_UpdatorDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // �׸��⸦ ���� ����̽� ���ؽ�Ʈ�Դϴ�.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Ŭ���̾�Ʈ �簢������ �������� ����� ����ϴ�.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// �������� �׸��ϴ�.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// ����ڰ� �ּ�ȭ�� â�� ���� ���ȿ� Ŀ���� ǥ�õǵ��� �ý��ۿ���
//  �� �Լ��� ȣ���մϴ�.
HCURSOR CLocalPowerController_UpdatorDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CLocalPowerController_UpdatorDlg::ReceiveParsing(BYTE* ReceiveBuf, int Length)
{
	UINT16 Cnt1;
	UINT16 Cnt2;
	BYTE ReadData = 0;
	CString Str1;
	CString Str2;
	UINT16 CheckSum;
	BYTE EndParsing = 0;

	Str1 = _T("RX :\r\n");
	for (Cnt1 = 0; Cnt1 < Length; Cnt1++)
	{
		Str2.Format(_T(" %02X"), ReceiveBuf[Cnt1]);
		Str1 += Str2;
	}
	Str1 += _T("\r\n");
	LogAdd(StrWithTime(Str1, 0));
}


void CLocalPowerController_UpdatorDlg::OnBnClickedBtnFileOpen()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	TCHAR sFilter[] = _T("*.bin, *.BIN | *.bin;*BIN ||");
	CFileDialog dlg(TRUE, NULL, NULL, 0, sFilter);
	CFile file;
	UINT FileSize, Remains, TotalSize;
	CString Str1 = _T("");
	CString Str2;

	if (dlg.DoModal() == IDOK)
	{
		if (file.Open(dlg.GetPathName(), CFile::modeRead) == 0)
		{
			LogAdd(StrWithTime(_T("���� ���� ����\r\n"), 0));
			bIsFileOpened = FALSE;
		}
		else
		{
			Str2 = _T("���� ���� ����\r\n");
			Str1 += Str2;
			Str2.Format(_T("%s\r\n"), dlg.GetPathName());
			Str1 += Str2;
			FileSize = file.GetLength();
			Remains = FileSize % 1024;
			if (Remains)
			{
				TotalSize = FileSize + 1024 - Remains;	// To make an align with 1024 block. It'll fill area of (1024 - remains) with 0x00
			}
			else
			{
				TotalSize = FileSize;
			}
			for (UINT Cnt = 0; Cnt < TotalSize; Cnt++)
			{
				mRdBuf[Cnt] = 0;
			}
			file.Read(mRdBuf, FileSize);
			file.Close();
			if (mRdBuf[2048] == 0x7E && mRdBuf[2049] == 0x81 && mRdBuf[2050] == 0x96 && mRdBuf[2051] == 0xA5 && mRdBuf[2052] == 0x64 && mRdBuf[2053] == 0x42)
			{
				bIsFileOpened = TRUE;
				mTotalPage = TotalSize / 1024;
				mUpdateFile_FwVer = (UINT16)mRdBuf[2054] << 8;
				mUpdateFile_FwVer |= mRdBuf[2055];
				mUpdateFile_Crc16 = Get_Crc16(mRdBuf, 1024 * mTotalPage);
				mErasePage = 134;
				Str2.Format(_T("File Size\t\t: %d\r\n"), FileSize);
				Str1 += Str2;
				Str2.Format(_T("Remains\t\t: %d\r\n"), Remains);
				Str1 += Str2;
				Str2.Format(_T("1024-Remains\t: %d\r\n"), 1024 - Remains);
				Str1 += Str2;
				Str2.Format(_T("TotalSize\t\t: %d\r\n"), TotalSize);
				Str1 += Str2;
				Str2.Format(_T("Pages\t\t: %d\r\n"), mTotalPage);
				Str1 += Str2;
				Str2.Format(_T("Ver\t\t: 0x%X\r\n"), mUpdateFile_FwVer);
				Str1 += Str2;
				Str2.Format(_T("CRC16\t\t: 0x%X\r\n"), mUpdateFile_Crc16);
				Str1 += Str2;
			}
			else
			{
				Str2 = _T("����������ġ �߿��� ������ �ƴ�!\r\n");
				Str1 += Str2;
			}
			LogAdd(StrWithTime(Str1, 0));
		}
	}
}

CString CLocalPowerController_UpdatorDlg::StrWithTime(CString StrOut, BYTE nOption)
{
	SYSTEMTIME st;
	CTime timer;
	CString str, strMilli;

#ifdef DEBUG_LOG
	timer = CTime::GetCurrentTime();
	GetSystemTime(&st);
	if (nOption == 0)
	{
		str = timer.Format("[%H:%M:%S");
	}
	else
	{
		str = timer.Format("[%Y-%m-%d]\r\n[%H:%M:%S");
	}
	strMilli.Format(_T(".%03d]"), st.wMilliseconds);
	str += strMilli;
	str += StrOut;
#else
	str = _T("");
#endif

	return str;
}

void CLocalPowerController_UpdatorDlg::LogAdd(CString EditStr)
{
	int Length = m_cEdit_Log.GetWindowTextLengthA();
	m_cEdit_Log.SetSel(Length, Length);
	m_cEdit_Log.ReplaceSel(EditStr);
}

UINT16 CLocalPowerController_UpdatorDlg::Get_Crc16(BYTE* pCrcData, UINT Length)
{
	UINT Cnt16;
	UINT16 Crc16Result;

	Crc16Result = 0;
	for (Cnt16 = 0; Cnt16 < Length; Cnt16++)
	{
		Crc16Result = (Crc16Result << 8) ^ Crc16Table[((Crc16Result >> 8) ^ *pCrcData) & 0x00FF];
		pCrcData++;
	}

	return Crc16Result;
}


void CLocalPowerController_UpdatorDlg::OnCbnSelchangeCmbIp()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	UpdateData(TRUE);
}


void CLocalPowerController_UpdatorDlg::OnBnClickedBtnConnect()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	CString Str;
	CString Str_ip;

	if (m_pDataSocket == NULL)
	{
		Str_ip = mStrCombo_ip;
		m_pDataSocket = new CDataSocket(this);
		if (m_pDataSocket->Create())
		{
			Str.Format(_T("����������ġ(%s) ���� ��\r\n"), Str_ip);
			LogAdd(StrWithTime(Str, 0));
			if (m_pDataSocket->Connect(Str_ip, 5000))
			{
				GetDlgItem(IDC_BTN_CONNECT)->SetWindowTextA(_T("Disconnect"));
				Str.Format(_T("%s ���� ����\r\n"), Str_ip);
			}
			else
			{
				m_pDataSocket = NULL;
				Str.Format(_T("%s ���� ����\r\n"), Str_ip);				
			}			
		}
		else
		{
			Str.Format(_T("Socket ���� ����\r\n"));
			m_pDataSocket = NULL;
		}
		LogAdd(StrWithTime(Str, 0));
	}
	else
	{
		m_pDataSocket->Close();
		m_pDataSocket = NULL;
		GetDlgItem(IDC_BTN_CONNECT)->SetWindowTextA(_T("Connect"));
		LogAdd(StrWithTime(_T("���� ����\r\n"), 0));
	}
}

void CLocalPowerController_UpdatorDlg::SendWithCks(BYTE* pData, UINT16 Length)
{
	if (m_pDataSocket != NULL)
	{
		UINT16 Cnt1 = 0;
		UINT16 CheckSum = 0;
		CString Str1, Str2;

		pData[0] = 0x7E;	// STX
		pData[1] = 0x01;	// Destination Address(Target Board)
		pData[2] = 0x00;	// Source Address(PC)
		*(UINT16*)(pData + 3) = Length;
		for (Cnt1 = 0; Cnt1 < (Length+6); Cnt1++)
		{
			CheckSum += pData[Cnt1];
		}
		*(UINT16*)(pData + Length + 6) = CheckSum;
		pData[Length + 8] = 0x81;	// END
		m_pDataSocket->Send(pData, Length + 9);
		Str1 = _T("TX :\r\n");
		for (Cnt1 = 0; Cnt1 < (Length + 9); Cnt1++)
		{
			Str2.Format(_T(" %02X"), pData[Cnt1]);
			Str1 += Str2;
			if (((Cnt1 + 1) % 16) == 0)
			{
				Str1 += _T("\r\n");
			}
		}
		Str1 += _T("\r\n");
		LogAdd(StrWithTime(Str1, 0));
	}
	else
	{
		LogAdd(StrWithTime(_T("���� �ȵǾ���\r\n"), 0));
	}
}

void CLocalPowerController_UpdatorDlg::OnBnClickedBtnReset()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	BYTE TxData[11] = { 0, };

	TxData[1] = 0x01;	// DA
	TxData[2] = 0x00;	// SA
	TxData[5] = 0xA1;	// CMD
	*(UINT16*)(TxData + 6) = 5;	// Data(TimeOut)
	SendWithCks(TxData, 2);
}

void CLocalPowerController_UpdatorDlg::OnBnClickedBtnGetVer()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	BYTE TxData[10] = { 0, };

	TxData[1] = 0x01;	// DA
	TxData[2] = 0x00;	// SA
	TxData[5] = 0xA0;	// CMD
	*(UINT16*)(TxData + 6) = 0;	// Data
	SendWithCks(TxData, 1);
}


void CLocalPowerController_UpdatorDlg::OnBnClickedBtnClearLog()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_cEdit_Log.SetWindowTextA(_T(""));
	LogAdd(StrWithTime(_T("\r\n"), 0));
}
