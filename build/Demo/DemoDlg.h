
// DemoDlg.h : ͷ�ļ�
//

#pragma once
#include "pthread.h"
#include "fifo.h"
#include "usb_control.h"

// CDemoDlg �Ի���
class CDemoDlg : public CDialogEx
{
// ����
public:
	CDemoDlg(CWnd* pParent = NULL);	// ��׼���캯��
	virtual ~CDemoDlg();	// �Զ�������������

// �Ի�������
	enum { IDD = IDD_DEMO_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��


// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonPlay();
	afx_msg void OnBnClickedRadioLocal();
	afx_msg void OnBnClickedRadioNetwork();
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnEnChangeEditPhi1();
	afx_msg void OnEnChangeEditPhi2();
	afx_msg void OnEnChangeEditY0();
	afx_msg void OnEnChangeEditCh0();
	afx_msg void OnEnChangeEditCh1();
	afx_msg void OnEnChangeEditCv0();
	afx_msg void OnEnChangeEditVanish();
	afx_msg void OnBnClickedButtonSaveConfig();
	afx_msg void OnBnClickedCheckDetrack();
	afx_msg void OnBnClickedCheckDetped();
	afx_msg void OnBnClickedOk();
	afx_msg void OnEnChangeEditInstallHeight();
	afx_msg void OnLButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnGetMinMaxInfo(MINMAXINFO* lpMMI);
public:
	CDC *pDC;
	BOOL bFullScreen;
	CRect rectFullScreen;
	WINDOWPLACEMENT m_struOldWndpl;		// ��������Ļ�ϵ�λ��
	WINDOWPLACEMENT m_struOldWndpPic;	// ͼ��ؼ�����Ļ�ϵ�λ��
	int datasource;
	enum {LOCAL = 1,NETWORK};
	CString filename;
	int width;		// ͼ��ˮƽ�ֱ���
	int height;		// ͼ��ֱ�ֱ���
	Fifo *rgbring;	// ��ɫѹ��ͼ��Ļ��ζ���
	enum {FRAME_RESOLUTION = 16, PIXEL_FORMAT = 24};
	pthread_t getid;	// ��ȡԭʼ�����̺߳�
	int CreateRawImageFetchThread();
	pthread_t viewtid;	// ������Ƶ�̺߳�
	int CreateMonitorOutputViewThread();
	enum {STOP, PLAY};
	int play;	// ����״̬
	int run;	// APP�Ƿ�����
	float fx;		// ���򽹾�
	float fy;		// ���򽹾�
	float cx;		// ���ĺ�����
	float cy;		// ����������
	float h;		// �������װ�߶�
	double phi1;	// ƫ����
	double phi2;	// ������
	double y0;		// ����ƫ��
	double b;		// ��׼���
	double ch0;		// ˮƽ����
	double ch1;		// ˮƽ���ʱ仯��
	double cv0;		// ��ֱ����
	int vanish;		// ���������
	int marktimer;	// ��������ģ�͵ļ�ʱ��
	int InitConfig(const char *path);
	int EditConfigFile(const char *filename, char *key, char *value);
	int SolveQuadraticEquation(double p1, double p2, double p3, double *sol);
	void ProjectLaneStateModel();
	int *pattern;	// ����ģ��ͶӰ����
	enum {PHI1_CHAN = 1, PHI2_CHAN, Y0_CHAN, CH0_CHAN, CH1_CHAN, CV0_CHAN, VANISH_CHAN,
		DETRACK_CHAN, DETPEDE_CHAN, INSTALLH_CHAN, NUM_PARAMS};
	int ischanged[128];	// �����Ƿ񱻸Ķ���״̬��
	int detrack;	// �������
	int detpede;	// �������
	enum {BAUD_RATE = 9600, BYTE_SIZE = 8, PARITY = 0, STOP_BITS = 1};
	int port;		// ����
	HANDLE device;	// USB�豸
	int duration;	// ��������ʱ��
	int clean_frames;	// �ɾ�֡������
	int total_frames;	// ��Ƶ�ļ���֡��
	int auto_run;	// �Ƿ񿪻��Զ�����
	unsigned long num_frames;
	bool restart;
};
