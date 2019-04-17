
// DemoDlg.h : 头文件
//

#pragma once
#include "pthread.h"
#include "fifo.h"
#include "usb_control.h"

// CDemoDlg 对话框
class CDemoDlg : public CDialogEx
{
// 构造
public:
	CDemoDlg(CWnd* pParent = NULL);	// 标准构造函数
	virtual ~CDemoDlg();	// 自定义虚析构函数

// 对话框数据
	enum { IDD = IDD_DEMO_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
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
	WINDOWPLACEMENT m_struOldWndpl;		// 窗口在屏幕上的位置
	WINDOWPLACEMENT m_struOldWndpPic;	// 图像控件在屏幕上的位置
	int datasource;
	enum {LOCAL = 1,NETWORK};
	CString filename;
	int width;		// 图像水平分辨率
	int height;		// 图像垂直分辨率
	Fifo *rgbring;	// 彩色压缩图像的环形队列
	enum {FRAME_RESOLUTION = 16, PIXEL_FORMAT = 24};
	pthread_t getid;	// 获取原始数据线程号
	int CreateRawImageFetchThread();
	pthread_t viewtid;	// 播放视频线程号
	int CreateMonitorOutputViewThread();
	enum {STOP, PLAY};
	int play;	// 播放状态
	int run;	// APP是否运行
	float fx;		// 横向焦距
	float fy;		// 纵向焦距
	float cx;		// 光心横坐标
	float cy;		// 光心纵坐标
	float h;		// 摄像机安装高度
	double phi1;	// 偏航角
	double phi2;	// 俯仰角
	double y0;		// 横向偏移
	double b;		// 标准轨距
	double ch0;		// 水平曲率
	double ch1;		// 水平曲率变化率
	double cv0;		// 垂直曲率
	int vanish;		// 天际线坐标
	int marktimer;	// 叠加铁轨模型的计时器
	int InitConfig(const char *path);
	int EditConfigFile(const char *filename, char *key, char *value);
	int SolveQuadraticEquation(double p1, double p2, double p3, double *sol);
	void ProjectLaneStateModel();
	int *pattern;	// 铁轨模型投影坐标
	enum {PHI1_CHAN = 1, PHI2_CHAN, Y0_CHAN, CH0_CHAN, CH1_CHAN, CV0_CHAN, VANISH_CHAN,
		DETRACK_CHAN, DETPEDE_CHAN, INSTALLH_CHAN, NUM_PARAMS};
	int ischanged[128];	// 参数是否被改动的状态表
	int detrack;	// 检测铁轨
	int detpede;	// 检测行人
	enum {BAUD_RATE = 9600, BYTE_SIZE = 8, PARITY = 0, STOP_BITS = 1};
	int port;		// 串口
	HANDLE device;	// USB设备
	int duration;	// 报警持续时间
	int clean_frames;	// 干净帧的数量
	int total_frames;	// 视频文件总帧数
	int auto_run;	// 是否开机自动运行
	unsigned long num_frames;
	bool restart;
};
