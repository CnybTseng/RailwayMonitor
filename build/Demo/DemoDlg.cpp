
// DemoDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "Demo.h"
#include "DemoDlg.h"
#include "afxdialogex.h"
#include "mmsystem.h"
#include "json/json.h"
#include "CvvImage.h"
#include "RDC.h"
#include "list.h"
#include "frame.h"
#include "railwaymonitor.h"
#include <stdio.h>
#include <queue>
#include <audioclient.h>
#include <mmdeviceapi.h>
#include <endpointvolume.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define ROUND_UP_IMAGE_SIZE	1048576
#define MIN_PHI1	-1.0f
#define MAX_PHI1	1.0f
#define MIN_PHI2	-1.0f
#define MAX_PHI2	1.0f
#define MIN_Y0		-1.0f
#define MAX_Y0		1.0f
#define MIN_CH0		-1e-3
#define MAX_CH0		1e-3
#define MIN_CH1		-1e-6
#define MAX_CH1		1e-6
#define MIN_CV0		-1e-3
#define MAX_CV0		1e-3
#define KEEPTIMER	10

// CDemoDlg 对话框

static void PrintSimpleLog(const char *msg);
static void RestartProcess();

CDemoDlg::CDemoDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDemoDlg::IDD, pParent)
	, datasource(NETWORK)
	, width(640)
	, height(480)
	, rgbring(NULL)
	, play(STOP)
	, run(1)
	, phi1(0)
	, phi2(0)
	, fx(1000 / 17.0f * 150)
	, fy(fx)
	, cx(width / 2.0f)
	, cy(height / 2.0f)
	, h(1.75f)
	, y0(0)
	, b(1.435)
	, ch0(0)
	, ch1(0)
	, cv0(0)
	, vanish(240)
	, marktimer(0)
	, detrack(0)
	, detpede(0)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	
	// 分配原始图像环形队列存储空间
	rgbring = fifo_alloc(ROUND_UP_IMAGE_SIZE << 2);
	if (!rgbring) 
	{
		PrintSimpleLog("分配原始图像环形队列存储空间失败\n");
		exit(-1);
	}
	else
	{
		PrintSimpleLog("分配原始图像环形队列存储空间成功\n");
	}
	
	// 初始化原始图像格式转换模块
	RDC_Init(PIXEL_FORMAT, FRAME_RESOLUTION);
	
	// 分配铁轨模型投影坐标的存储空间
	pattern = (int *)malloc((height << 1) * sizeof(int));
	if (!pattern)
	{
		PrintSimpleLog("分配铁轨模型投影坐标的存储空间失败\n");
		exit(-1);
	}
	else
	{
		PrintSimpleLog("分配铁轨模型投影坐标的存储空间成功\n");
	}
	
	// 从配置文件加载参数
	InitConfig("RailwayMonitor.json");
	
	// 初始化参数是否被改变的状态表
	for (int i = 0; i < NUM_PARAMS; i++) ischanged[i] = 0;
	
	// 创建算法模块实例
	RailwayMonitorCreate(width, height, 150);
	
	// 启动算法模块各个线程
	if (-1 == RailwayMonitorBegin()) 
	{
		PrintSimpleLog("启动算法模块各个线程失败\n");
		exit(-1);
	}
	else
	{
		PrintSimpleLog("启动算法模块各个线程成功\n");
	}
	
	// 打开USB设备
	device = open_usb_device(port, BAUD_RATE, BYTE_SIZE, PARITY, STOP_BITS);
	restart = false;
}

CDemoDlg::~CDemoDlg()
{	
	// 等待原始数据接收线程退出
	while (1)
	{
		int ret = pthread_kill(getid, 0);
		if (ESRCH == ret) break;
		else if (EINVAL == ret) break;
		else
		{	
			Sleep(1);
			continue;
		}
	}
	
	// 等待算法模块各个线程退出
	RailwayMonitorEnd();
	
	// 等待视频播放线程退出
	while (1)
	{
		int ret = pthread_kill(viewtid, 0);
		if (ESRCH == ret) break;
		else if (EINVAL == ret) break;
		else 
		{	
			Sleep(1);
			continue;
		}
	}
	
	// 销毁算法模块实例
	RailwayMonitorDestroy();

	if (rgbring) fifo_delete(rgbring);
	
	if (pattern)
	{
		free(pattern);
		pattern = NULL;
	}
	
	if (device != INVALID_HANDLE_VALUE) close_usb_device(device);
	
	if (restart)
	{
		RestartProcess();
	}
}

void CDemoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CDemoDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_PLAY, &CDemoDlg::OnBnClickedButtonPlay)
	ON_BN_CLICKED(IDC_RADIO_LOCAL, &CDemoDlg::OnBnClickedRadioLocal)
	ON_BN_CLICKED(IDC_RADIO_NETWORK, &CDemoDlg::OnBnClickedRadioNetwork)
	ON_WM_HSCROLL()
	ON_EN_CHANGE(IDC_EDIT_PHI1, &CDemoDlg::OnEnChangeEditPhi1)
	ON_EN_CHANGE(IDC_EDIT_PHI2, &CDemoDlg::OnEnChangeEditPhi2)
	ON_EN_CHANGE(IDC_EDIT_Y0, &CDemoDlg::OnEnChangeEditY0)
	ON_EN_CHANGE(IDC_EDIT_CH0, &CDemoDlg::OnEnChangeEditCh0)
	ON_EN_CHANGE(IDC_EDIT_CH1, &CDemoDlg::OnEnChangeEditCh1)
	ON_EN_CHANGE(IDC_EDIT_CV0, &CDemoDlg::OnEnChangeEditCv0)
	ON_EN_CHANGE(IDC_EDIT_VANISH, &CDemoDlg::OnEnChangeEditVanish)
	ON_BN_CLICKED(IDC_BUTTON_SAVE_CONFIG, &CDemoDlg::OnBnClickedButtonSaveConfig)
	ON_BN_CLICKED(IDC_CHECK_DETRACK, &CDemoDlg::OnBnClickedCheckDetrack)
	ON_BN_CLICKED(IDC_CHECK_DETPED, &CDemoDlg::OnBnClickedCheckDetped)
	ON_BN_CLICKED(IDOK, &CDemoDlg::OnBnClickedOk)
	ON_EN_CHANGE(IDC_EDIT_INSTALL_HEIGHT, &CDemoDlg::OnEnChangeEditInstallHeight)
	ON_WM_LBUTTONDBLCLK()
	ON_WM_GETMINMAXINFO()
END_MESSAGE_MAP()


// CDemoDlg 消息处理程序

BOOL CDemoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	m_hIcon = AfxGetApp()->LoadIcon(IDI_ICON1);
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// 根据ID获得窗口指针再获取与该窗口关联的上下文指针
	pDC = GetDlgItem(IDC_STATIC)->GetDC();

	// 初始化选中“网络”单选按钮
	CButton *button = (CButton *)GetDlgItem(IDC_RADIO_NETWORK);
	button->SetCheck(TRUE);

	// 从配置文件加载参数
	InitConfig("RailwayMonitor.json");

	// 设置偏航角滚动条取值范围
	CScrollBar *scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_PHI1);
	scrollbar->SetScrollRange(-10000, 10000);
	scrollbar->SetScrollPos(0);
	CString value;
	value.Format(L"%.15lf", phi1);
	SetDlgItemTextW(IDC_EDIT_PHI1, value);
	
	// 设置俯仰角滚动条取值范围
	scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_PHI2);
	scrollbar->SetScrollRange(-10000, 10000);
	scrollbar->SetScrollPos(0);
	value.Format(L"%.15lf", phi2);
	SetDlgItemTextW(IDC_EDIT_PHI2, value);
	
	// 设置横向偏移滚动条取值范围
	scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_Y0);
	scrollbar->SetScrollRange(-10000, 10000);
	scrollbar->SetScrollPos(0);
	value.Format(L"%.15lf", y0);
	SetDlgItemTextW(IDC_EDIT_Y0, value);
	
	// 设置水平曲率滚动条取值范围
	scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_CH0);
	scrollbar->SetScrollRange(-10000, 10000);
	scrollbar->SetScrollPos(0);
	value.Format(L"%.15e", ch0);
	SetDlgItemTextW(IDC_EDIT_CH0, value);
	
	// 设置水平曲率变化率滚动条取值范围
	scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_CH1);
	scrollbar->SetScrollRange(-10000, 10000);
	scrollbar->SetScrollPos(0);
	value.Format(L"%.15e", ch1);
	SetDlgItemTextW(IDC_EDIT_CH1, value);
	
	// 设置垂直曲率滚动条取值范围
	scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_CV0);
	scrollbar->SetScrollRange(-10000, 10000);
	scrollbar->SetScrollPos(0);
	value.Format(L"%.15e", cv0);
	SetDlgItemTextW(IDC_EDIT_CV0, value);
	
	// 设置天际线滚动条取值范围
	scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_VANISH);
	scrollbar->SetScrollRange(0, height - 1);
	scrollbar->SetScrollPos(vanish);
	value.Format(L"%d", vanish);
	SetDlgItemTextW(IDC_EDIT_VANISH, value);
	
	// 初始化铁轨检测复选框
	button = (CButton *)GetDlgItem(IDC_CHECK_DETRACK);
	button->SetCheck(detrack);
	
	// 初始化行人检测复选框
	button = (CButton *)GetDlgItem(IDC_CHECK_DETPED);
	button->SetCheck(detpede);
	
	// 设置安装高度文本框
	value.Format(L"%.2f", h);
	SetDlgItemTextW(IDC_EDIT_INSTALL_HEIGHT, value);
	
	// 初始化参数是否被改变的状态表
	for (int i = 0; i < NUM_PARAMS; i++) ischanged[i] = 0;
	
	// 启动原始视频预览线程
	if (-1 == CreateMonitorOutputViewThread())
	{
		PrintSimpleLog("启动原始视频预览线程失败\n");
		exit(-1);
	}
	else
	{
		PrintSimpleLog("启动原始视频预览线程成功\n");
	}
	
	// 启动原始数据接收线程
	if (-1 == CreateRawImageFetchThread()) 
	{
		PrintSimpleLog("启动原始数据接收线程失败\n");
		exit(-1);
	}
	else
	{
		PrintSimpleLog("启动原始数据接收线程成功\n");
	}
	
	// 自动播放
	if (auto_run)
	{
		this->PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_PLAY, BN_CLICKED));
		HWND hWnd = ::FindWindow(NULL, _T("Demo"));
		if (hWnd)
		{
			CRect rc;
			::GetWindowRect(hWnd, &rc);
			CPoint pt(rc.CenterPoint());
			::PostMessage(hWnd, WM_LBUTTONDBLCLK, MK_LBUTTON, MAKELPARAM(pt.x, pt.y));
		}
	}
				
	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CDemoDlg::OnPaint()
{	
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
	
	CRect rect;
	GetClientRect(&rect);
	
	// 初始化图像控件背景为黑色
	FillRect(pDC->GetSafeHdc(), &rect, CBrush(RGB(0, 0, 0)));
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CDemoDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

// 原始数据获取线程
void *RawImageFetchThread(void *s)
{
	CDemoDlg *dlg = (CDemoDlg *)s;
	
	FILE *fp = NULL;
	frame_receiver *graber = NULL;
	char *recvbuf = NULL;
		
	// 分配原始图像存储空间
	char *rawimag = (char *)malloc(ROUND_UP_IMAGE_SIZE);
	if (!rawimag)
	{
		PrintSimpleLog("分配原始图像存储空间失败\n");
		exit(-1);
	}
	else
	{
		PrintSimpleLog("分配原始图像存储空间成功\n");
	}
	
	// 分配彩色压缩图像存储空间
	char *rgbimag = (char *)malloc(ROUND_UP_IMAGE_SIZE);
	if (!rgbimag)
	{
		PrintSimpleLog("运行原始数据获取线程失败\n");
		exit(-1);
	}
	else
	{
		PrintSimpleLog("运行原始数据获取线程成功\n");
	}
	
	while (dlg->run)
	{
		if (dlg->STOP == dlg->play)
		{
			Sleep(1);
			continue;
		}
		
		// 获取原始图像
		if (dlg->LOCAL == dlg->datasource)
		{
			if (NULL == fp)
			{
				USES_CONVERSION;
				fp = fopen(T2A(dlg->filename), "rb");
				if (!fp) exit(-1);
			}
			
			size_t read = fread(rawimag, 1, (dlg->width * dlg->height) << 1, fp);
			if (0 == read)
			{
				if (fp)
				{	
					fclose(fp);
					fp = NULL;
				}
				dlg->PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_PLAY, BN_CLICKED));
				Sleep(1);	// 等待播放状态被设置为停止
				continue;
			}
		}
		else
		{
			if (NULL == graber)
			{
				graber = new frame_receiver;
				if (!graber)
				{
					PrintSimpleLog("创建原始帧抓取器失败\n");
					exit(-1);
				}
				else
				{
					PrintSimpleLog("创建原始帧抓取器成功\n");
				}
				
				graber->init(UDP_SERVER_USE, 32345);
				if (!graber->open()) 
				{
					PrintSimpleLog("初始化原始帧抓取器失败\n");
					exit(-1);
				}
				else
				{
					PrintSimpleLog("初始化原始帧抓取器成功\n");
				}
				
				if (!graber->run()) 
				{
					PrintSimpleLog("运行原始帧抓取器失败\n");
					exit(-1);
				}
				else
				{
					PrintSimpleLog("运行原始帧抓取器成功\n");
				}
				
				recvbuf = (char *)malloc(((dlg->width * dlg->height) << 1) * sizeof(char));
				if (!recvbuf)
				{
					PrintSimpleLog("分配原始帧缓冲区失败\n");
					exit(-1);
				}
				else
				{
					PrintSimpleLog("分配原始帧缓冲区成功\n");
				}
			}
			
			unsigned short width, height;
			if (graber->get(1, recvbuf, width, height))
			{
				graber->recombine_raw_data(recvbuf, (uint16_t *)rawimag, (width * height) << 1);
				graber->fill_zero_lines((uint16_t *)rawimag, width, height);
			}
			else
			{
				Sleep(1);
				continue;
			}
		}
		
		// 将原始图像放入算法模块的环形队列
		if (RailwayMonitorPutData((unsigned char *)rawimag, ROUND_UP_IMAGE_SIZE))
		{
			PrintSimpleLog("将原始帧放入算法模块失败\n");
			dlg->restart = true;
			dlg->PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_PLAY, BN_CLICKED));
			dlg->PostMessage(WM_COMMAND, MAKEWPARAM(IDOK, BN_CLICKED));
			break;
		}
		
		// 原始图像格式转换为RGB		
		unsigned int rol;
		RDC_SendRawData((unsigned char *)rawimag, (dlg->width * dlg->height) << 1);
		RDC_GetFrame((unsigned char *)rgbimag, &rol);
		
		// 将彩色压缩图像放入UI的环形队列
		int counter = 100;
		while (counter--)
		{
			int write = fifo_put(dlg->rgbring, rgbimag, ROUND_UP_IMAGE_SIZE);
			if (write == ROUND_UP_IMAGE_SIZE)
			{
				break;
			}
			Sleep(1);
		}
		
		if (counter < 0)
		{
			PrintSimpleLog("将彩色帧放入显示模块失败\n");
			dlg->restart = true;
			dlg->PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_PLAY, BN_CLICKED));
			dlg->PostMessage(WM_COMMAND, MAKEWPARAM(IDOK, BN_CLICKED));
			break;
		}
	}
	
	if (fp)
	{	
		fclose(fp);
		fp = NULL;
	}

	if (graber)
	{
		graber->stop();
		Sleep(1000);
		delete graber;
	}
	
	if (recvbuf)
	{
		free(recvbuf);
		recvbuf = NULL;
	}
	
	if (rawimag)
	{
		free(rawimag);
		rawimag = NULL;
	}
	
	if (rgbimag)
	{
		free(rgbimag);
		rgbimag = NULL;
	}
	
	return (void *)(0);
}

// 创建原始数据获取线程
int CDemoDlg::CreateRawImageFetchThread()
{
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	int ret = pthread_create(&getid, &attr, RawImageFetchThread, this);
	if (0 != ret)
	{
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}

	pthread_attr_destroy(&attr);

	return 0;
}

// 视频播放控制
void CDemoDlg::OnBnClickedButtonPlay()
{	
	if (STOP == play)
	{
		play = PLAY;			
		CButton *button = (CButton *)GetDlgItem(IDC_BUTTON_PLAY);
		button->SetWindowTextW(_T("停止"));
		num_frames = 0;
	}
	else
	{
		play = STOP;		
		CButton *button = (CButton *)GetDlgItem(IDC_BUTTON_PLAY);
		button->SetWindowTextW(_T("播放"));
	}
}

// 报警视频压缩线程
void *AlarmVideoCompressThread(void *s)
{
	char filename[128];
	time_t t = time(NULL);
	struct tm *lt = localtime(&t);
	sprintf_s(filename, "%04d年%02d月%02d日-%02d时%02d分%02d秒.avi", 1900 + lt->tm_year,
		1 + lt->tm_mon, lt->tm_mday, lt->tm_hour, lt->tm_min, lt->tm_sec);
	std::queue<cv::Mat> *images = (std::queue<cv::Mat> *)s;
	cv::VideoWriter vw(filename, CV_FOURCC('M', 'P', '4', '2'), 25.0, cv::Size(images->front().cols, images->front().rows));
	
	while (images->size())
	{
		vw << images->front();
		images->pop();
	}
	
	delete images;
	vw.release();
	return (void *)(0);
}

// 生成报警录像
void MakeAlarmVideo(IplImage *frame, int flag, int clean_frames, int total_frames)
{
	static int timer = 0;
	static int num_alarm = 0;
	static std::queue<cv::Mat> images;
	cv::Mat image = cv::Mat(frame->height, frame->width, CV_8UC3);
	memcpy(image.data, frame->imageData, frame->height * frame->width * 3);

	if (flag) ++num_alarm;
	
	if (0 == flag && timer < clean_frames + 1)
	{
		if (timer < clean_frames)
		{
			++timer;
			images.push(image);
		}
		else
		{
			images.pop();
			images.push(image);
		}
	}
	else
	{
		++timer;
		images.push(image);
		if (timer > total_frames - 1)
		{	
			if (num_alarm / (float)total_frames < 0.5f)
			{
				while (images.size())
				{
					images.pop();
				}
				
				timer = 0;
				num_alarm = 0;
				return;
			}
			
			std::queue<cv::Mat> *images_backup = new std::queue<cv::Mat>;
			while (images.size())
			{
				images_backup->push(images.front().clone());
				images.pop();
			}
			
			pthread_t tid;
			pthread_attr_t attr;
			pthread_attr_init(&attr);
			pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
			
			int ret = pthread_create(&tid, &attr, AlarmVideoCompressThread, images_backup);
			if (0 != ret)
			{
				PrintSimpleLog("create AlarmVideoCompressThread fail\n");
			}

			pthread_attr_destroy(&attr);
			timer = 0;
			num_alarm = 0;
		}
	}
}

BOOL ChangeVolume(float nVolume)
{
	HRESULT hr = NULL;
	IMMDeviceEnumerator *deviceEnumerator = NULL;
	hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL, CLSCTX_INPROC_SERVER,
		__uuidof(IMMDeviceEnumerator), (LPVOID *)&deviceEnumerator);
	if (FAILED(hr))
		return FALSE;

	IMMDevice *defaultDevice = NULL;
	hr = deviceEnumerator->GetDefaultAudioEndpoint(eRender, eConsole, &defaultDevice);
	deviceEnumerator->Release();
	if (FAILED(hr))
		return FALSE;

	IAudioEndpointVolume *endpointVolume = NULL;
	hr = defaultDevice->Activate(__uuidof(IAudioEndpointVolume),
		CLSCTX_INPROC_SERVER, NULL, (LPVOID *)&endpointVolume);
	defaultDevice->Release();
	if (FAILED(hr))
		return FALSE;

	hr = endpointVolume->SetMasterVolumeLevelScalar(nVolume, NULL);
	endpointVolume->Release();

	return SUCCEEDED(hr);
}

// 监视器输出显示线程
void *MonitorOutputViewThread(void *s)
{
	CDemoDlg *dlg = (CDemoDlg *)s;
	HDC hdc = dlg->pDC->GetSafeHdc();	// 获取设备上下文句柄

	CRect rect;

	char *rgbimag = (char *)malloc(ROUND_UP_IMAGE_SIZE);
	if (!rgbimag)
	{
		PrintSimpleLog("运行监视器输出显示线程失败\n");
		exit(-1);
	}
	else
	{
		PrintSimpleLog("运行监视器输出显示线程成功\n");
	}
	
	CvSize sz(dlg->width, dlg->height);
	IplImage *frame = cvCreateImage(sz, IPL_DEPTH_8U, 3);
	unsigned char *data = (unsigned char *)frame->imageData;
	int chanles = frame->nChannels;
	int step = frame->widthStep;
	CvvImage cimg;
	int sound_alarm_timer = 0;
	
	// 测试帧率
	LARGE_INTEGER frequency;
	LARGE_INTEGER start;
	LARGE_INTEGER end; 
	float frame_rate;
	dlg->num_frames = 0;
	
	QueryPerformanceFrequency(&frequency); 
		
	while (dlg->run)
	{
		if (dlg->STOP == dlg->play)
		{
			Sleep(1);
			continue;
		}
		
		int read = fifo_get(dlg->rgbring, rgbimag, ROUND_UP_IMAGE_SIZE);
		if (read != ROUND_UP_IMAGE_SIZE)
		{
			Sleep(1);
			continue;
		}
		
		memcpy(frame->imageData, rgbimag, dlg->width * dlg->height * 3);
		
		// 从算法模块读取检测结果
		RailwayMonitorOutput output;
		ListInit(&output.track);
		ListInit(&output.alarm);
		if (RailwayMonitorGetData(&output))
		{
			PrintSimpleLog("从算法模块获取检测结果失败\n");
			dlg->restart = true;
			dlg->PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_PLAY, BN_CLICKED));
			dlg->PostMessage(WM_COMMAND, MAKEWPARAM(IDOK, BN_CLICKED));
			break;
		}
		
		if (dlg->num_frames == 0)
		{
			QueryPerformanceCounter(&start);
		}
		
		// 更新和叠加帧率
		++dlg->num_frames;
		QueryPerformanceCounter(&end);
		frame_rate = dlg->num_frames / ((end.QuadPart - start.QuadPart) / (double)frequency.QuadPart + 1e-6f);
		{
			char text[128];
			sprintf(text, "%.0f fps", frame_rate);
			CvFont font;
			cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 0, 1, 8);
			cvPutText(frame, text, CvPoint(16, 16), &font, CvScalar(0, 255, 255));
		}
		
		// 叠加检测到的铁轨
		Node *tracknode = output.track.head;
		while (tracknode)
		{
			// 铁轨.
			if (((struct Track *)tracknode->val)->px >= 0 && ((struct Track *)tracknode->val)->px < dlg->width)
			{
				*(data + ((struct Track *)tracknode->val)->py * step + ((struct Track *)tracknode->val)->px * chanles) = 255;
				*(data + ((struct Track *)tracknode->val)->py * step + ((struct Track *)tracknode->val)->px * chanles + 1) = 255;
				*(data + ((struct Track *)tracknode->val)->py * step + ((struct Track *)tracknode->val)->px * chanles + 2) = 255;
			}
			
			// 行人警戒线.
			if (((struct Track *)tracknode->val)->pwx >= 0 && ((struct Track *)tracknode->val)->pwx < dlg->width)
			{
				*(data + ((struct Track *)tracknode->val)->py * step + ((struct Track *)tracknode->val)->pwx * chanles) = 0;
				*(data + ((struct Track *)tracknode->val)->py * step + ((struct Track *)tracknode->val)->pwx * chanles + 1) = 255;
				*(data + ((struct Track *)tracknode->val)->py * step + ((struct Track *)tracknode->val)->pwx * chanles + 2) = 255;
			}
			
			tracknode = tracknode->next;
		}
		
		// 报警日志
		FILE *fp = fopen("alarm_log.txt", "a+");
		int counter = 1;
		int flag = 0;
		float score = 0;
		
		// 叠加报警信息
		Node *alarmnode = output.alarm.head;
		while (alarmnode)
		{
			struct Alarm *alarm = (struct Alarm *)alarmnode->val;
			if (alarm->isdangerous)
			{
				cvRectangle(frame, CvPoint(alarm->left, alarm->top),
					CvPoint(alarm->right, alarm->bottom), CvScalar(0, 0, 255), 1);
			}
			else
			{
				alarmnode = alarmnode->next;
				continue;
			}
			
			char text[128];
			sprintf(text, "%.2f", alarm->score);
			int ytext = alarm->top >= 2 ? alarm->top - 2 : 0;
			CvFont font;
			cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 0, 1, 8);
			cvPutText(frame, text, CvPoint(alarm->left, ytext), &font, CvScalar(255, 255, 255));
			
			struct tm *lt = localtime(&alarm->time);
			char alarm_time[64];
			sprintf_s(alarm_time, "%04d年%02d月%02d日 %02d:%02d:%02d", 1900 + lt->tm_year,
				1 + lt->tm_mon, lt->tm_mday, lt->tm_hour, lt->tm_min, lt->tm_sec);
			fprintf(fp, "[%02d]报警时间：%s\n", counter++, alarm_time);
			
			if (alarm->type == RAILWAY_ALARM_TRACK_PEDESTRIAN)
			{
				fprintf(fp, "    报警类型：行人入侵！\n");
			}
			else if (alarm->type == RAILWAY_ALARM_TRACK_OBSTACLE)
			{
				fprintf(fp, "    报警类型：异物入侵！\n");
			}
			else
			{
				fprintf(fp, "    报警类型：未知入侵！\n");
			}
			
			// fprintf(fp, "    报警区域：[左=%d，右=%d，上=%d，下=%d]\n", alarm->left, alarm->right,
			// 	alarm->top, alarm->bottom);
			// 
			// fprintf(fp, "    报警置信度：%.1f\n", alarm->score);
			
			if (alarm->score > score)
			{
				score = alarm->score;
			}
						
			alarmnode = alarmnode->next;

			flag = 1;			
		}

		if (flag)
		{
			fprintf(fp, "///////////////////////////////////////////////////////////////////\n\n");
		}
		
		fclose(fp);
		ListDelAll(&output.track);
		ListDelAll(&output.alarm);
		
		// 启动外部声光报警器
		if (flag && dlg->device != INVALID_HANDLE_VALUE)
		{
			char open_cmd[] = {0x55, 0x02, 0x00, dlg->duration, 0x00};
			write_to_usb_device(dlg->device, open_cmd, sizeof(open_cmd));
		}
		
		if (flag && sound_alarm_timer == 0)
		{
			CoInitialize(NULL);
			ChangeVolume(2 * (score - 0.5));
			CoUninitialize();
			PlaySound(L"alarm.wav", NULL, SND_FILENAME | SND_ASYNC | SND_NOSTOP);
		}
		
		if (flag)
		{
			sound_alarm_timer++;
			if (sound_alarm_timer == dlg->total_frames)
			{
				sound_alarm_timer = 0;
			}
		}

		MakeAlarmVideo(frame, flag, dlg->clean_frames, dlg->total_frames);

		// 叠加参数设置时机器反馈结果
		if (dlg->marktimer > 0)
		{			
			// 叠加铁轨模型投影到画面上
			for (int i = 0; i < dlg->height - dlg->vanish; i++)
			{
				// 左轨
				if (dlg->pattern[2 * i] >= 0 && dlg->pattern[2 * i] < dlg->width)
				{
					*(data + (dlg->height - 1 - i) * step + dlg->pattern[2 * i] * chanles) = 0;
					*(data + (dlg->height - 1 - i) * step + dlg->pattern[2 * i] * chanles + 1) = 255;
					*(data + (dlg->height - 1 - i) * step + dlg->pattern[2 * i] * chanles + 2) = 255;
				}
				
				// 右轨
				if (dlg->pattern[2 * i + 1] >= 0 && dlg->pattern[2 * i + 1] < dlg->width)
				{
					*(data + (dlg->height - 1 - i) * step + dlg->pattern[2 * i + 1] * chanles) = 0;
					*(data + (dlg->height - 1 - i) * step + dlg->pattern[2 * i + 1] * chanles + 1) = 255;
					*(data + (dlg->height - 1 - i) * step + dlg->pattern[2 * i + 1] * chanles + 2) = 255;
				}
			}
			
			// 叠加天际线到画面上
			cvLine(frame, cvPoint(0, dlg->vanish), cvPoint(dlg->width - 1, dlg->vanish), cvScalar(0, 255, 255));
			
			dlg->marktimer--;
		}
				
		cimg.CopyOf(frame, frame->nChannels);
		dlg->GetDlgItem(IDC_STATIC)->GetClientRect(&rect);	//获取客户区矩形框
		cimg.DrawToHDC(hdc, &rect);
		// Sleep(20);
	}

	if (rgbimag)
	{
		free(rgbimag);
		rgbimag = NULL;
	}

	cvReleaseImage(&frame);
	
	return (void *)(0);
}

// 创建监视器输出显示线程
int CDemoDlg::CreateMonitorOutputViewThread()
{
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	int ret = pthread_create(&viewtid, &attr, MonitorOutputViewThread, this);
	if (0 != ret)
	{
		fprintf(stderr, "pthread_create fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}

	pthread_attr_destroy(&attr);

	return 0;
}

// 选择播放本地文件
void CDemoDlg::OnBnClickedRadioLocal()
{
	CFileDialog filedlg(TRUE, NULL, NULL, OFN_HIDEREADONLY | OFN_FILEMUSTEXIST, L"JPG Files(*.dat)|*.dat||", this);
	if (filedlg.DoModal())
	{
		filename = filedlg.GetPathName();
		datasource = LOCAL;
	}
}

// 选择播放网络原始数据
void CDemoDlg::OnBnClickedRadioNetwork()
{
	datasource = NETWORK;
}

// 滚动条事务处理
void CDemoDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{	
	// 获取滚动条取值范围
	int minpos, maxpos;
	pScrollBar->GetScrollRange(&minpos, &maxpos);

	// 获取滚动条当前位置
	int curpos = pScrollBar->GetScrollPos();

	switch (nSBCode)
	{
	case SB_LEFT:
		curpos = minpos;
		break;
	case SB_RIGHT:
		curpos = maxpos;
		break;
	case SB_ENDSCROLL:
		break;
	case SB_LINELEFT:
		if (curpos > minpos) curpos--;
		break;
	case SB_LINERIGHT:
		if (curpos < maxpos) curpos++;
		break;
	case SB_PAGELEFT:
		break;
	case SB_PAGERIGHT:
		break;
	case SB_THUMBPOSITION:
		curpos = nPos;
		break;
	case SB_THUMBTRACK:
		curpos = nPos;
		break;
	default:
		break;
	}
	
	if (pScrollBar == (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_PHI1))
	{
		phi1 = (MAX_PHI1 - MIN_PHI1) * (curpos - minpos) / (maxpos - minpos) + MIN_PHI1;
		CString value;
		value.Format(L"%.15lf", phi1);
		SetDlgItemTextW(IDC_EDIT_PHI1, value);
		marktimer = KEEPTIMER;
		ischanged[PHI1_CHAN] = 1;
	}
	else if (pScrollBar == (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_PHI2))
	{
		phi2 = (MAX_PHI2 - MIN_PHI2) * (curpos - minpos) / (maxpos - minpos) + MIN_PHI2;
		CString value;
		value.Format(L"%.15lf", phi2);
		SetDlgItemTextW(IDC_EDIT_PHI2, value);
		marktimer = KEEPTIMER;
		ischanged[PHI2_CHAN] = 1;
	}
	else if (pScrollBar == (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_Y0))
	{
		y0 = (MAX_Y0 - MIN_Y0) * (curpos - minpos) / (maxpos - minpos) + MIN_Y0;
		CString value;
		value.Format(L"%.15lf", y0);
		SetDlgItemTextW(IDC_EDIT_Y0, value);
		marktimer = KEEPTIMER;
		ischanged[Y0_CHAN] = 1;
	}
	else if (pScrollBar == (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_CH0))
	{
		ch0 = (MAX_CH0 - MIN_CH0) * (curpos - minpos) / (maxpos - minpos) + MIN_CH0;
		CString value;
		value.Format(L"%.15e", ch0);
		SetDlgItemTextW(IDC_EDIT_CH0, value);
		marktimer = KEEPTIMER;
		ischanged[CH0_CHAN] = 1;
	}
	else if (pScrollBar == (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_CH1))
	{
		ch1 = (MAX_CH1 - MIN_CH1) * (curpos - minpos) / (maxpos - minpos) + MIN_CH1;
		CString value;
		value.Format(L"%.15e", ch1);
		SetDlgItemTextW(IDC_EDIT_CH1, value);
		marktimer = KEEPTIMER;
		ischanged[CH1_CHAN] = 1;
	}
	else if (pScrollBar == (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_CV0))
	{
		cv0 = (MAX_CV0 - MIN_CV0) * (curpos - minpos) / (maxpos - minpos) + MIN_CV0;
		CString value;
		value.Format(L"%.15e", cv0);
		SetDlgItemTextW(IDC_EDIT_CV0, value);
		marktimer = KEEPTIMER;
		ischanged[CV0_CHAN] = 1;
	}
	else
	{
		vanish = curpos;
		CString value;
		value.Format(L"%d", vanish);
		SetDlgItemTextW(IDC_EDIT_VANISH, value);
		marktimer = KEEPTIMER;
		ischanged[VANISH_CHAN] = 1;
	}

	pScrollBar->SetScrollPos(curpos);
	CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}

// 编辑偏航角文本框
void CDemoDlg::OnEnChangeEditPhi1()
{
	// 获取文本框值
	CString value;
	GetDlgItem(IDC_EDIT_PHI1)->GetWindowTextW(value);
	phi1 = _ttof(value);

	// 取值合法性判断
	if (phi1 < MIN_PHI1)
	{	
		phi1 = MIN_PHI1;
		value.Format(L"%.15lf", phi1);
		SetDlgItemTextW(IDC_EDIT_PHI1, value);
	}
	else if (phi1 > MAX_PHI1)
	{	
		phi1 = MAX_PHI1;
		value.Format(L"%.15lf", phi1);
		SetDlgItemTextW(IDC_EDIT_PHI1, value);
	}
	
	// 更新滚动条位置
	CScrollBar *scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_PHI1);
	int minpos, maxpos;
	scrollbar->GetScrollRange(&minpos, &maxpos);
	scrollbar->SetScrollPos((int)((maxpos - minpos) * (phi1 - MIN_PHI1) / (MAX_PHI1 - MIN_PHI1) + minpos));
	scrollbar->UpdateWindow();
	
	// 计算铁轨模型投影点
	ProjectLaneStateModel();
}

// 编辑俯仰角文本框
void CDemoDlg::OnEnChangeEditPhi2()
{
	// 获取文本框值
	CString value;
	GetDlgItem(IDC_EDIT_PHI2)->GetWindowTextW(value);
	phi2 = _ttof(value);

	// 取值合法性判断
	if (phi2 < MIN_PHI2)
	{	
		phi2 = MIN_PHI2;
		value.Format(L"%.15lf", phi2);
		SetDlgItemTextW(IDC_EDIT_PHI2, value);
	}
	else if (phi2 > MAX_PHI2)
	{	
		phi2 = MAX_PHI2;
		value.Format(L"%.15lf", phi2);
		SetDlgItemTextW(IDC_EDIT_PHI2, value);
	}
	
	// 更新滚动条位置
	CScrollBar *scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_PHI2);
	int minpos, maxpos;
	scrollbar->GetScrollRange(&minpos, &maxpos);
	scrollbar->SetScrollPos((int)((maxpos - minpos) * (phi2 - MIN_PHI2) / (MAX_PHI2 - MIN_PHI2) + minpos));
	scrollbar->UpdateWindow();
	
	// 计算铁轨模型投影点
	ProjectLaneStateModel();
}

// 编辑横向偏移文本框
void CDemoDlg::OnEnChangeEditY0()
{
	// 获取文本框值
	CString value;
	GetDlgItem(IDC_EDIT_Y0)->GetWindowTextW(value);
	y0 = _ttof(value);

	// 取值合法性判断
	if (y0 < MIN_Y0)
	{	
		y0 = MIN_Y0;
		value.Format(L"%.15lf", y0);
		SetDlgItemTextW(IDC_EDIT_Y0, value);
	}
	else if (y0 > MAX_Y0)
	{	
		y0 = MAX_Y0;
		value.Format(L"%.15lf", y0);
		SetDlgItemTextW(IDC_EDIT_Y0, value);
	}
	
	// 更新滚动条位置
	CScrollBar *scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_Y0);
	int minpos, maxpos;
	scrollbar->GetScrollRange(&minpos, &maxpos);
	scrollbar->SetScrollPos((int)((maxpos - minpos) * (y0 - MIN_Y0) / (MAX_Y0 - MIN_Y0) + minpos));
	scrollbar->UpdateWindow();
	
	// 计算铁轨模型投影点
	ProjectLaneStateModel();
}

// 编辑水平曲率文本框
void CDemoDlg::OnEnChangeEditCh0()
{
	// 获取文本框值
	CString value;
	GetDlgItem(IDC_EDIT_CH0)->GetWindowTextW(value);
	ch0 = _ttof(value);

	// 取值合法性判断
	if (ch0 < MIN_CH0)
	{	
		ch0 = MIN_CH0;
		value.Format(L"%.15e", ch0);
		SetDlgItemTextW(IDC_EDIT_CH0, value);
	}
	else if (ch0 > MAX_CH0)
	{	
		ch0 = MAX_CH0;
		value.Format(L"%.15e", ch0);
		SetDlgItemTextW(IDC_EDIT_CH0, value);
	}
	
	// 更新滚动条位置
	CScrollBar *scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_CH0);
	int minpos, maxpos;
	scrollbar->GetScrollRange(&minpos, &maxpos);
	scrollbar->SetScrollPos((int)((maxpos - minpos) * (ch0 - MIN_CH0) / (MAX_CH0 - MIN_CH0) + minpos));
	scrollbar->UpdateWindow();
	
	// 计算铁轨模型投影点
	ProjectLaneStateModel();
}

// 编辑水平曲率变化率文本框
void CDemoDlg::OnEnChangeEditCh1()
{
	// 获取文本框值
	CString value;
	GetDlgItem(IDC_EDIT_CH1)->GetWindowTextW(value);
	ch1 = _ttof(value);

	// 取值合法性判断
	if (ch1 < MIN_CH1)
	{	
		ch1 = MIN_CH1;
		value.Format(L"%.15e", ch1);
		SetDlgItemTextW(IDC_EDIT_CH1, value);
	}
	else if (ch1 > MAX_CH1)
	{	
		ch1 = MAX_CH1;
		value.Format(L"%.15e", ch1);
		SetDlgItemTextW(IDC_EDIT_CH1, value);
	}
	
	// 更新滚动条位置
	CScrollBar *scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_CH1);
	int minpos, maxpos;
	scrollbar->GetScrollRange(&minpos, &maxpos);
	scrollbar->SetScrollPos((int)((maxpos - minpos) * (ch1 - MIN_CH1) / (MAX_CH1 - MIN_CH1) + minpos));
	scrollbar->UpdateWindow();
	
	// 计算铁轨模型投影点
	ProjectLaneStateModel();
}

// 编辑垂直曲率文本框
void CDemoDlg::OnEnChangeEditCv0()
{
	// 获取文本框值
	CString value;
	GetDlgItem(IDC_EDIT_CV0)->GetWindowTextW(value);
	cv0 = _ttof(value);

	// 取值合法性判断
	if (cv0 < MIN_CV0)
	{	
		cv0 = MIN_CV0;
		value.Format(L"%.15e", cv0);
		SetDlgItemTextW(IDC_EDIT_CV0, value);
	}
	else if (cv0 > MAX_CV0)
	{	
		cv0 = MAX_CV0;
		value.Format(L"%.15e", cv0);
		SetDlgItemTextW(IDC_EDIT_CV0, value);
	}
	
	// 更新滚动条位置
	CScrollBar *scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_CV0);
	int minpos, maxpos;
	scrollbar->GetScrollRange(&minpos, &maxpos);
	scrollbar->SetScrollPos((int)((maxpos - minpos) * (cv0 - MIN_CV0) / (MAX_CV0 - MIN_CV0) + minpos));
	scrollbar->UpdateWindow();
	
	// 计算铁轨模型投影点
	ProjectLaneStateModel();
}

// 编辑天际线文本框
void CDemoDlg::OnEnChangeEditVanish()
{	
	// 获取文本框值
	CString value;
	GetDlgItem(IDC_EDIT_VANISH)->GetWindowTextW(value);
	vanish = _ttoi(value);

	// 取值合法性判断
	if (vanish < 0)
	{	
		vanish = 0;
		value.Format(L"%d", vanish);
		SetDlgItemTextW(IDC_EDIT_VANISH, value);
	}
	else if (vanish > height - 1)
	{	
		vanish = height - 1;
		value.Format(L"%d", vanish);
		SetDlgItemTextW(IDC_EDIT_VANISH, value);
	}
	
	// 更新滚动条位置
	CScrollBar *scrollbar = (CScrollBar *)GetDlgItem(IDC_SCROLLBAR_VANISH);
	scrollbar->SetScrollPos(vanish);
	scrollbar->UpdateWindow();
}

// 从配置文件加载参数
int CDemoDlg::InitConfig(const char *path)
{
	std::ifstream ifs(path, std::ios::binary);

	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root))
	{
		if (root["cameraInstallHeight"].isDouble())
		{
			h = root["cameraInstallHeight"].asFloat();
		}
		else
		{
			fprintf(stderr, "'cameraInstallHeight' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}

		if (root["laneInitState"]["yawAngle"].isDouble())
		{
			phi1 = root["laneInitState"]["yawAngle"].asDouble();
		}
		else
		{
			fprintf(stderr, "'yawAngle' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["pitchAngle"].isDouble())
		{
			phi2 = root["laneInitState"]["pitchAngle"].asDouble();
		}
		else
		{
			fprintf(stderr, "'pitchAngle' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["lateralOffset"].isDouble())
		{
			y0 = root["laneInitState"]["lateralOffset"].asDouble();
		}
		else
		{
			fprintf(stderr, "'lateralOffset' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["horizontalCurvature"].isDouble())
		{
			ch0 = root["laneInitState"]["horizontalCurvature"].asDouble();
		}
		else
		{
			fprintf(stderr, "'horizontalCurvature' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["alterationOfHorizontalCurvature"].isDouble())
		{
			ch1 = root["laneInitState"]["alterationOfHorizontalCurvature"].asDouble();
		}
		else
		{
			fprintf(stderr, "'alterationOfHorizontalCurvature' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["laneInitState"]["verticalCurvature"].isDouble())
		{
			cv0 = root["laneInitState"]["verticalCurvature"].asDouble();
		}
		else
		{
			fprintf(stderr, "'verticalCurvature' isn't double[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["maxSumScanLineInter"].isInt())
		{
			vanish = height - root["maxSumScanLineInter"].asInt();
		}
		else
		{
			fprintf(stderr, "'maxSumScanLineInter' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["trackerOnOff"].isInt())
		{
			detrack = root["trackerOnOff"].asInt();
		}
		else
		{
			fprintf(stderr, "'trackerOnOff' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["pedestrianDetectorOnOff"].isInt())
		{
			detpede = root["pedestrianDetectorOnOff"].asInt();
		}
		else
		{
			fprintf(stderr, "'pedestrianDetectorOnOff' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["usbPort"].isInt())
		{
			port = root["usbPort"].asInt();
		}
		else
		{
			fprintf(stderr, "'usbPort' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["duration"].isInt())
		{
			duration = root["duration"].asInt();
		}
		else
		{
			fprintf(stderr, "'duration' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["cleanFrames"].isInt())
		{
			clean_frames = root["cleanFrames"].asInt();
		}
		else
		{
			fprintf(stderr, "'cleanFrames' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["totalFrames"].isInt())
		{
			total_frames = root["totalFrames"].asInt();
		}
		else
		{
			fprintf(stderr, "'totalFrames' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
		
		if (root["autoRun"].isInt())
		{
			auto_run = root["autoRun"].asInt();
		}
		else
		{
			fprintf(stderr, "'autoRun' isn't integer[%s:%d].\n", __FILE__, __LINE__);
			return -1;
		}
	}
	else
	{
		fprintf(stderr, "Json::Reader.parse fail[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	return 0;
}

// 修改算法模块配置文件
int CDemoDlg::EditConfigFile(const char *filename, char *key, char *value)
{
	FILE *fp = NULL;
	FILE *newfp = NULL;
	
	fp = fopen(filename, "r");
	if (!fp)
	{
		fprintf(stderr, "fopen[%s:%d].\n", __FILE__, __LINE__);
		return -1;
	}
	
	newfp = fopen("temp.json", "w");
	if (!newfp)
	{
		fprintf(stderr, "fopen[%s:%d].\n", __FILE__, __LINE__);
		if (fp) fclose(fp);
		return -1;
	}
	
	while (!feof(fp))
	{
		char line[1024];
		fgets(line, sizeof(line), fp);
		
		char *p = strstr(line, key);
		if (!p)
		{
			fputs(line, newfp);
		}
		else
		{
			char *q = strpbrk(p, ",\n");
			if (q)
			{
				char newline[1024];
				size_t len = p - line + strlen(key);
				memcpy(newline, line, len);
				strcpy(newline + len, value);
				strcat(newline, q);
				fputs(newline, newfp);
			}
			else
			{
				fprintf(stderr, "End of the line is invalid[%s:%d]!", __FILE__, __LINE__);
				if (fp) fclose(fp);
				if (newfp) fclose(newfp);
				return -1;
			}
		}
	}
	
	if (fp) fclose(fp);
	if (newfp) fclose(newfp);
	
	char cmd[128];
	sprintf(cmd, "move temp.json %s", filename);
	system(cmd);
	
	return 0;
}

// 解一元二次方程
int CDemoDlg::SolveQuadraticEquation(double p1, double p2, double p3, double *sol)
{
	double _sol;
	if (fabs(p1) > 1e-10)
	{
		double delta = p2 * p2 - 4 * p1 * p3;
		if (delta > 0) _sol = (-p2 - sqrt(delta)) / (2 * p1);
		else return -1;
	}
	else
	{
		if (fabs(p2) > 1e-30) _sol = -p3 / p2;
		else return -1;
	}
	
	*sol = _sol;
	
	return 0;
}

// 投影铁轨状态模型
void CDemoDlg::ProjectLaneStateModel()
{
	int cnt = 0;
	for (int yb = height - 1; yb >= vanish; yb--)
	{
		double p1 = -fy * cv0 / 2;
		double p3 = fy * h;
		double p2 = fy * phi2 + cy - yb;
		
		double dist = -1;
		if (SolveQuadraticEquation(p1, p2, p3, &dist) || dist < 0) continue;
		
		double temp1 = ch0 * pow(dist, 2) / 2 + ch1 * pow(dist, 3) / 6;
		double yl = -b / 2 + temp1;
		double temp2 = fx * (-y0 + phi1 * dist) / dist + cx;
		int xb = (int)(fx * yl / dist + temp2 + 0.5f);
		pattern[cnt++] = xb;
		
		yl = b / 2 + temp1;
		xb = (int)(fx * yl / dist + temp2 + 0.5f);
		pattern[cnt++] = xb;
	}
}

// 保存系统设置
void CDemoDlg::OnBnClickedButtonSaveConfig()
{
	const char filename[] = "RailwayMonitor.json";
	
	if (ischanged[PHI1_CHAN])
	{
		char key[] = "\"yawAngle\":";
		char value[128];
		sprintf(value, "%.15lf", phi1);
		EditConfigFile(filename, key, value);
	}
	
	if (ischanged[PHI2_CHAN])
	{
		char key[] = "\"pitchAngle\":";
		char value[128];
		sprintf(value, "%.15lf", phi2);
		EditConfigFile(filename, key, value);
	}
	
	if (ischanged[Y0_CHAN])
	{
		char key[] = "\"lateralOffset\":";
		char value[128];
		sprintf(value, "%.15lf", y0);
		EditConfigFile(filename, key, value);
	}
	
	if (ischanged[CH0_CHAN])
	{
		char key[] = "\"horizontalCurvature\":";
		char value[128];
		sprintf(value, "%.15e", ch0);
		EditConfigFile(filename, key, value);
	}
	
	if (ischanged[CH1_CHAN])
	{
		char key[] = "\"alterationOfHorizontalCurvature\":";
		char value[128];
		sprintf(value, "%.15e", ch1);
		EditConfigFile(filename, key, value);
	}
	
	if (ischanged[CV0_CHAN])
	{
		char key[] = "\"verticalCurvature\":";
		char value[128];
		sprintf(value, "%.15e", cv0);
		EditConfigFile(filename, key, value);
	}
	
	if (ischanged[VANISH_CHAN])
	{
		char key[] = "\"maxSumScanLineInter\":";
		char value[128];
		sprintf(value, "%d", height - vanish);
		EditConfigFile(filename, key, value);
	}
	
	if (ischanged[DETRACK_CHAN])
	{
		char key[] = "\"trackerOnOff\":";
		char value[128];
		sprintf(value, "%d", detrack);
		EditConfigFile(filename, key, value);
	}
	
	if (ischanged[DETPEDE_CHAN])
	{
		char key[] = "\"pedestrianDetectorOnOff\":";
		char value[128];
		sprintf(value, "%d", detpede);
		EditConfigFile(filename, key, value);
	}
	
	if (ischanged[INSTALLH_CHAN])
	{
		char key[] = "\"cameraInstallHeight\":";
		char value[128];
		sprintf(value, "%.2f", h);
		EditConfigFile(filename, key, value);
	}
	
	for (int i = 0; i < NUM_PARAMS; i++) ischanged[i] = 0;
}

// 通过复选框选择启用或关闭铁轨检测
void CDemoDlg::OnBnClickedCheckDetrack()
{
	CButton *button = (CButton *)GetDlgItem(IDC_CHECK_DETRACK);
	if (button->GetCheck()) detrack = 1;
	else detrack = 0;
	ischanged[DETRACK_CHAN] = 1;
}

// 通过复选框选择启用或关闭行人检测
void CDemoDlg::OnBnClickedCheckDetped()
{
	CButton *button = (CButton *)GetDlgItem(IDC_CHECK_DETPED);
	if (button->GetCheck()) detpede = 1;
	else detpede = 0;
	ischanged[DETPEDE_CHAN] = 1;
}

// 退出对话框应用程序
void CDemoDlg::OnBnClickedOk()
{
	if (play == PLAY)
	{
		this->PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_PLAY, BN_CLICKED));
	}
	
	// 等待绘图线程释放掉占用的资源
	run = 0;
	while (1)
	{
		int ret = pthread_kill(viewtid, 0);
		if (ESRCH == ret) break;
		else if (EINVAL == ret) break;
		else 
		{	
			Sleep(1);
			continue;
		}
	}
	
	CDialogEx::OnOK();
}

// 编辑安装高度文本框
void CDemoDlg::OnEnChangeEditInstallHeight()
{
	// 获取文本框值
	CString value;
	GetDlgItem(IDC_EDIT_INSTALL_HEIGHT)->GetWindowTextW(value);
	h = _ttof(value);
		
	// 计算铁轨模型投影点
	ProjectLaneStateModel();

	marktimer = KEEPTIMER;
	ischanged[INSTALLH_CHAN] = 1;
}

// 对话框双击全屏或退出全屏消息处理
void CDemoDlg::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	if (!bFullScreen)
	{
		bFullScreen = true;

		// 获取系统屏幕宽高
		int g_iCurScreenWidth = GetSystemMetrics(SM_CXSCREEN);
		int g_iCurScreenHeight = GetSystemMetrics(SM_CYSCREEN);

		// 用m_struOldWndpl得到当前窗口的显示状态和窗体位置，以供退出全屏后使用
		GetWindowPlacement(&m_struOldWndpl);
		GetDlgItem(IDC_STATIC)->GetWindowPlacement(&m_struOldWndpPic);

		// 计算出窗口全屏显示客户端所应该设置的窗口大小,主要为了将不需要显示的窗体边框等部分排除在屏幕外
		CRect rectWholeDlg;
		CRect rectClient;
		GetWindowRect(&rectWholeDlg);	// 得到当前窗体的总的相对于屏幕的坐标
		RepositionBars(0, 0xffff, AFX_IDW_PANE_FIRST, reposQuery, &rectClient);		// 得到客户区窗口坐标
		ClientToScreen(&rectClient);	// 将客户区相对窗体的坐标转为相对屏幕坐标

		rectFullScreen.left = rectWholeDlg.left - rectClient.left;
		rectFullScreen.top = rectWholeDlg.top - rectClient.top;
		rectFullScreen.right = rectWholeDlg.right + g_iCurScreenWidth - rectClient.right;
		rectFullScreen.bottom = rectWholeDlg.bottom + g_iCurScreenHeight - rectClient.bottom;

		// 设置窗口对象参数,为全屏做好准备并进入全屏状态
		WINDOWPLACEMENT struWndpl;
		struWndpl.length = sizeof(WINDOWPLACEMENT);
		struWndpl.flags = 0;
		struWndpl.showCmd = SW_SHOWNORMAL;
		struWndpl.rcNormalPosition = rectFullScreen;
		SetWindowPlacement(&struWndpl);		// 该函数设置指定窗口的显示状态和显示大小位置等

		// 将PICTURE控件的坐标设为全屏大小
		GetDlgItem(IDC_STATIC)->MoveWindow(CRect(0, 0, g_iCurScreenWidth, g_iCurScreenHeight));

		// 隐藏其他控件
		GetDlgItem(IDC_STATIC_SETS)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_STATIC_PHI1)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SCROLLBAR_PHI1)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_EDIT_PHI1)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_STATIC_PHI2)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SCROLLBAR_PHI2)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_EDIT_PHI2)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_STATIC_Y0)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SCROLLBAR_Y0)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_EDIT_Y0)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_STATIC_CH0)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SCROLLBAR_CH0)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_EDIT_CH0)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_STATIC_CH1)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SCROLLBAR_CH1)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_EDIT_CH1)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_STATIC_CV0)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SCROLLBAR_CV0)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_EDIT_CV0)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_STATIC_VANISH)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_SCROLLBAR_VANISH)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_EDIT_VANISH)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_STATIC_INSTALL_HEIGHT)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_EDIT_INSTALL_HEIGHT)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_CHECK_DETRACK)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_CHECK_DETPED)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_RADIO_LOCAL)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_RADIO_NETWORK)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_BUTTON_PLAY)->ShowWindow(SW_HIDE);
		GetDlgItem(IDC_BUTTON_SAVE_CONFIG)->ShowWindow(SW_HIDE);
		GetDlgItem(IDOK)->ShowWindow(SW_HIDE);
	}
	else
	{
		GetDlgItem(IDC_STATIC)->SetWindowPlacement(&m_struOldWndpPic);
		SetWindowPlacement(&m_struOldWndpl);
		bFullScreen = false;

		GetDlgItem(IDC_STATIC_SETS)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_STATIC_PHI1)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_SCROLLBAR_PHI1)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_EDIT_PHI1)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_STATIC_PHI2)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_SCROLLBAR_PHI2)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_EDIT_PHI2)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_STATIC_Y0)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_SCROLLBAR_Y0)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_EDIT_Y0)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_STATIC_CH0)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_SCROLLBAR_CH0)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_EDIT_CH0)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_STATIC_CH1)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_SCROLLBAR_CH1)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_EDIT_CH1)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_STATIC_CV0)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_SCROLLBAR_CV0)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_EDIT_CV0)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_STATIC_VANISH)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_SCROLLBAR_VANISH)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_EDIT_VANISH)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_STATIC_INSTALL_HEIGHT)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_EDIT_INSTALL_HEIGHT)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_CHECK_DETRACK)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_CHECK_DETPED)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_RADIO_LOCAL)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_RADIO_NETWORK)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_BUTTON_PLAY)->ShowWindow(SW_SHOW);
		GetDlgItem(IDC_BUTTON_SAVE_CONFIG)->ShowWindow(SW_SHOW);
		GetDlgItem(IDOK)->ShowWindow(SW_SHOW);
	}

	CDialogEx::OnLButtonDblClk(nFlags, point);
}

// 重载此函数以使得窗体大小可以大于屏幕大小
void CDemoDlg::OnGetMinMaxInfo(MINMAXINFO* lpMMI)
{
	if (bFullScreen)
	{
		lpMMI->ptMaxSize.x = rectFullScreen.Width();
		lpMMI->ptMaxSize.y = rectFullScreen.Height();
		lpMMI->ptMaxPosition.x = rectFullScreen.left;
		lpMMI->ptMaxPosition.y = rectFullScreen.top;
		lpMMI->ptMaxTrackSize.x = rectFullScreen.Width();
		lpMMI->ptMaxTrackSize.y = rectFullScreen.Height();
	}

	CDialogEx::OnGetMinMaxInfo(lpMMI);
}

void PrintSimpleLog(const char *msg)
{
	FILE *fp = fopen("log.txt", "a");
	fprintf(fp, msg);
	fclose(fp);
}

void RestartProcess()
{
	PrintSimpleLog("重启进程\n");
	LPWSTR lpFileName = new WCHAR[MAX_PATH];
	GetModuleFileName(NULL, lpFileName, MAX_PATH);

	STARTUPINFO startupinfo;
	PROCESS_INFORMATION proc_info;

	memset(&startupinfo, 0, sizeof(STARTUPINFO));
	startupinfo.cb = sizeof(STARTUPINFO);

	::CreateProcess(lpFileName, NULL, NULL, NULL, FALSE, NORMAL_PRIORITY_CLASS, NULL, NULL, &startupinfo, &proc_info);
	delete [] lpFileName;
}