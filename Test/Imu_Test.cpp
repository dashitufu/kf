#include "common.h"
#include "Matrix.h"
#include "Image.h"

#include "imu_gnss.h"
#include "math.h"

double w_Noise[100][2] = { 
	-2.0425727 , -3.742799 ,
-3.1529799 , 2.5782993 ,
2.2438998 , 3.5713084 ,
-1.879869 , -3.4510992 ,
-5.1766987 , 5.45802 ,
1.53741 , 2.5911455 ,
-5.7953496 , -3.426506 ,
6.621353 , -4.529073 ,
-0.9315346 , -3.7946482 ,
-1.383895 , -3.8110704 ,
0.26977393 , 5.861996 ,
1.1606426 , 1.0666039 ,
-1.1137025 , 1.8911375 ,
-1.3794477 , 0.66100466 ,
1.078775 , -4.5403256 ,
4.628414 , 4.3489223 ,
1.3508705 , 5.2859015 ,
-0.38334155 , -0.3851174 ,
1.1998563 , 4.833365 ,
1.0413339 , 5.570896 ,
-2.4204412 , -6.27098 ,
-3.2189412 , 5.445211 ,
1.9118257 , 5.1716366 ,
0.30714518 , -0.05377425 ,
0.8494765 , -0.11138775 ,
-2.1643054 , -5.4566793 ,
2.0342522 , 3.3313322 ,
5.960482 , -2.8354194 ,
-2.3120012 , -0.75957376 ,
2.925482 , -3.3556578 ,
-1.109053 , -3.3213575 ,
9.929465 , 3.6910286 ,
4.671547 , 3.9050386 ,
-3.1419878 , 4.056049 ,
-4.3942876 , 6.554394 ,
-1.9564805 , 3.7364872 ,
4.9230285 , 7.991681 ,
2.8633795 , 2.7273335 ,
-3.482349 , -4.1655774 ,
-2.0697832 , -1.4984388 ,
-4.0280576 , 0.55899876 ,
2.897898 , 0.18546216 ,
-3.529054 , 0.6379767 ,
-2.5711327 , -1.9140121 ,
-3.2280736 , 0.40580696 ,
-1.7313997 , 5.1817636 ,
-4.8057294 , 0.78091705 ,
-3.1204362 , 3.8121862 ,
3.5081294 , -2.6747315 ,
0.17491032 , 5.045933 ,
-3.9163122 , 1.8371578 ,
4.518792 , -2.8148787 ,
-4.399894 , -3.6213675 ,
1.3166544 , -0.47843984 ,
-4.267264 , 1.6239445 ,
-2.5526898 , 2.900715 ,
2.9331572 , 1.8051926 ,
0.22733918 , 2.5250337 ,
10.779189 , 3.1435351 ,
0.4279759 , 3.3522992 ,
-4.2551208 , 6.133679 ,
-4.3889227 , -1.930222 ,
-4.0063963 , 0.53440887 ,
8.304249 , 0.16538855 ,
-0.5363019 , 4.4550676 ,
1.2903812 , 0.36126235 ,
-0.49793532 , -0.5998032 ,
0.35565877 , 6.1696124 ,
0.95648193 , -2.0067563 ,
3.0626512 , -3.277883 ,
-1.2270161 , 1.1668913 ,
-4.2369804 , 2.088774 ,
-2.613425 , 1.022891 ,
-3.543464 , 2.2032564 ,
-1.0819701 , 3.6521287 ,
-6.367238 , -1.6605325 ,
3.3980932 , -2.3494177 ,
-1.992478 , 2.0835776 ,
-0.28432223 , -3.3765194 ,
3.1618953 , -6.422771 ,
0.28011382 , 2.7628915 ,
-9.494754 , 2.0892756 ,
-2.6566732 , 0.7419872 ,
0.6295633 , -0.5223675 ,
3.4491067 , 2.59325 ,
1.3629328 , 4.49465 ,
-0.86488664 , -3.3748372 ,
-1.1220727 , -0.60645276 ,
3.8588836 , 4.9633055 ,
-7.359572 , -7.020124 ,
-5.8174453 , -2.4407618 ,
0.5498342 , 0.7439036 ,
-0.8005535 , -0.0060780523 ,
1.7810051 , -4.609944 ,
-1.7828717 , 3.1267772 ,
-2.953032 , 1.9035128 ,
5.637464 , -3.1163316 ,
4.966491 , -0.6210026 ,
1.2693408 , -0.55710334 ,
-2.0348744 , 2.0592241 ,
};
double v_Noise[100][2] = {
	192.94925 , 6.681399 ,
	-134.09369 , 191.06369 ,
	-113.954796 , 30.905539 ,
	36.67022 , -38.767773 ,
	78.509 , -83.66603 ,
	155.69778 , 48.367767 ,
	-133.23662 , 44.165047 ,
	98.82411 , -29.748081 ,
	-14.268207 , -174.67772 ,
	-61.00664 , -20.34529 ,
	-61.482727 , 9.115138 ,
	-240.35747 , -100.84649 ,
	57.30266 , 21.565208 ,
	40.601727 , -11.233297 ,
	71.92493 , 102.24461 ,
	-188.52765 , 37.914047 ,
	93.34785 , -98.64822 ,
	-130.35155 , 16.741796 ,
	57.910145 , -51.208492 ,
	-39.14262 , -15.3558235 ,
	-1.790494 , -77.209045 ,
	-86.00121 , 44.153576 ,
	62.387836 , -34.984604 ,
	117.43066 , -247.17645 ,
	-61.421005 , 77.45139 ,
	42.213074 , -190.72002 ,
	61.55555 , -4.9620748 ,
	45.74792 , 135.51035 ,
	7.3904834 , 112.57976 ,
	-76.05542 , -29.49243 ,
	-60.802757 , 140.8083 ,
	-8.754285 , -51.46984 ,
	-117.52081 , 58.496113 ,
	49.388832 , -193.3437 ,
	120.00751 , 0.74625945 ,
	-226.7795 , 27.54754 ,
	121.27918 , 130.71661 ,
	-194.4749 , -23.443964 ,
	50.94354 , -39.855373 ,
	-68.365814 , 72.345314 ,
	-23.343258 , 77.01434 ,
	88.28087 , -39.376495 ,
	-173.97426 , -8.891878 ,
	-19.811646 , 137.9706 ,
	-28.112375 , 131.90092 ,
	-38.934723 , 1.8815314 ,
	-76.05122 , 29.73679 ,
	42.374466 , -167.45424 ,
	22.136507 , -107.59794 ,
	-62.456814 , -188.28938 ,
	-9.443471 , 82.15335 ,
	43.710957 , 114.12888 ,
	-88.01377 , -48.726982 ,
	64.56744 , 101.45162 ,
	-223.48529 , 126.94322 ,
	-40.615993 , 70.710075 ,
	-79.86979 , -156.70348 ,
	-0.67313117 , 81.37209 ,
	2.5585384 , 53.278095 ,
	-63.62606 , 194.07819 ,
	-55.131058 , -52.33486 ,
	22.007515 , -25.851906 ,
	-113.60571 , 130.99428 ,
	-128.21384 , 46.14773 ,
	-19.202658 , 7.438185 ,
	1.9394095 , 37.319096 ,
	-23.230415 , -23.814455 ,
	-55.09008 , -31.789099 ,
	18.154818 , -17.971209 ,
	200.70074 , -50.18636 ,
	-115.0117 , -151.0142 ,
	-163.18378 , 99.158806 ,
	123.47303 , 118.77007 ,
	80.335365 , 163.09074 ,
	-11.3505535 , -59.112663 ,
	-51.23848 , -109.58555 ,
	-72.177055 , -13.663014 ,
	30.03972 , -51.014748 ,
	45.77348 , -49.91431 ,
	-13.4431305 , -63.007683 ,
	-12.185923 , 37.88855 ,
	-47.289165 , -205.3884 ,
	5.9159527 , -65.16769 ,
	42.498146 , 51.381283 ,
	-111.89794 , -220.37987 ,
	-11.783521 , -28.258133 ,
	-56.73934 , 123.13334 ,
	-21.922697 , 95.70157 ,
	-44.10149 , -69.914856 ,
	50.825 , 43.76366 ,
	-0.30118275 , -34.695984 ,
	123.62237 , -89.11965 ,
	42.720047 , 137.51445 ,
	-193.19757 , -92.63379 ,
	130.503 , -22.229517 ,
	-88.70013 , -8.994515 ,
	28.883629 , 37.819492 ,
	-57.12555 , 91.538475 ,
	129.86293 , -102.49474 ,
	-102.62325 , 51.91321 
};

template<typename _T>struct ESKF_Param {
	const _T init_time_seconds_ = 10,
		gravity_norm_ = 9.81,
		max_static_gyro_var = 0.5,
		max_static_acce_var = 0.05,

		bias_gyro_var_ = 1e-6,  // 陀螺零偏游走标准差
		bias_acce_var_ = 1e-4,  // 加计零偏游走标准差

		// 里程计参数
		odom_var_ = 0.5,
		odom_span_ = 0.1,        // 里程计测量间隔
		wheel_radius_ = 0.155,   // 轮子半径
		circle_pulse_ = 1024.0,  // 编码器每圈脉冲数

		/// RTK 观测参数
		gnss_pos_noise_ = 0.1,                   // GNSS位置噪声
		gnss_height_noise_ = 0.1,                // GNSS高度噪声
		gnss_ang_noise_ = 1.0 * PI/180;  // GNSS旋转噪声
	//IMU噪声项
	_T w_var_ = 1e-5,       // 陀螺测量标准差
		a_var_ = 1e-2;       // 加计测量标准差

	_T antenna_pos[2] = { (_T)-0.17, (_T)-0.20 }, antenna_angle = (_T)12.06;
	_T init_bg_[3], init_ba_[3];
		
	_T	odom_noise_[3 * 3],	//里程计噪声？
		gnss_noise_[6 * 6];	//卫星数据噪声
};

template<typename _T>struct ESKF {		//搞个卡尔曼滤波环境
	ESKF_Param<_T> m_oParam;
	_T Origin[3];	//整个过程的原点的世界坐标
	_T R[3 * 3], Rotation_Vector[4], p[3],
		v[3]={},
		m_fCur_Time;

	_T ba[3], bg[3], g[3];

	_T Var_a, Var_w;

	_T dx[18] = {};		//排列顺序：p,v,q(旋转向量),bg,ba,g

	_T P[18 * 18];		//这个很有可能就是P矩阵
	
	_T Q_[18 * 18];			//一个大矩阵，描述噪声？暂时不明
};

static void Imu_Test_1()
{//这个已经与参考程序一致
	typedef double _T;		//由于数据精度要求太高，只能用double
	_T g[4] = { 0,0/*,-9.8f*/ },								//此处是否用重力加速度存疑，按照加速度性质，最后一步非常大
		bg[4] = { 00.000224886, -7.61038e-05, -0.000742259 },	//做死g偏移
		ba[4] = { -0.165205, 0.0926887, 0.0058049 },			//做死加速度零偏
		R[3 * 3], p[3] = { 0 }, v[3] = { 0 },					//p,v,q参数
		fPre_Time=0;		
	Gen_I_Matrix(R, 3, 3);	//初始化为单位矩阵

	unsigned long long tStart = iGet_Tick_Count();
	FILE* pFile = fopen("D:\\software\\3rdparty\\slam_in_autonomous_driving-master\\data\\ch3\\10.txt", "rb");
	if (!pFile) { printf("Fail to open file\n"); return; };

	//搞团点云看点
	Point_Cloud<_T> oPC;
	Init_Point_Cloud(&oPC, 300000);

	char Line[256];
	int i, iCount = 10;
	
	for (i = 0;i<iCount && iRead_Line(pFile, Line, 256);/* i++*/)
	{
		if (strstr(Line, "IMU "))
		{
			IMU<_T> oImu;
			sscanf(Line, "IMU %lf %lf %lf %lf %lf %lf %lf ", &oImu.m_fTime,
				&oImu.a[0], &oImu.a[1], &oImu.a[2],	&oImu.w[0], &oImu.w[1], &oImu.w[2]);

			if (fPre_Time == 0)
			{
				fPre_Time = oImu.m_fTime;
				continue;
			}
			Update_pvq(oImu, p, v, R, ba, bg, g,oImu.m_fTime-fPre_Time);
			fPre_Time = oImu.m_fTime;
			printf("i:%d Time:%lf %.7ef %.7ef %.7ef\n", i,oImu.m_fTime, p[0], p[1], p[2]);
			i++;

			//画出位置
			if(i%10==0)
				Draw_Point(&oPC, p[0], p[1], p[2]);
		}
	}
	printf("%lld %.7ef %.7ef %.7ef\n", iGet_Tick_Count() - tStart,p[0],p[1],p[2]);

	//存个盘看看
	bSave_PLY("c:\\tmp\\1.ply", oPC);
	
	Free_Point_Cloud(&oPC);
	if (pFile)
		fclose(pFile);
	return;
}

static void Gnss_Test_1()
{
	typedef double _T;
	unsigned long long tStart = iGet_Tick_Count();
	FILE* pFile = fopen("D:\\software\\3rdparty\\slam_in_autonomous_driving-master\\data\\ch3\\10.txt", "rb");
	if (!pFile) { printf("Fail to open file\n"); return; };

	char Line[256];
	int i, iCount = 390000;

	Point_Cloud<_T> oPC;
	Init_Point_Cloud(&oPC, 40000);

	UTM_Param<_T> oParam;
	utm<_T> oUtm;
	_T Map_Origin[3] = { 0 };

	for (i = 0; i < iCount && iRead_Line(pFile, Line, 256);)
	{
		if (strstr(Line, "GNSS "))
		{
			GNSS<_T> oGnss;
			//if(typeid(_T) == typeid(float)))
			sscanf(Line, "GNSS %lf %lf %lf %lf %lf %d", &oGnss.m_fTime,
				&oGnss.GPS[0], &oGnss.GPS[1], &oGnss.GPS[2],
				&oGnss.m_fHeading, &oGnss.m_bHeading_Valid);

			//以下代码存疑，既然不考虑天线了，那传进去的参数没用，直接用
			// utm的数据才准确，此处充其量只是对齐数据
			//将GNSS经纬度转换为一种方便的坐标
			bLat_Long_2_utm_ang(oGnss,(_T) - 0.17,(_T) -0.20, (_T)12.06, Map_Origin, &oParam, &oUtm);
			
			//显示数据
			printf("%f %f %f\n", oUtm.x, oUtm.y, oUtm.z);
			i++;
			Draw_Point(&oPC, oUtm.x, oUtm.y, oUtm.z);
		}
	}
	printf("%lld %.7ef %.7ef %.7ef\n", iGet_Tick_Count() - tStart, oUtm.x, oUtm.y, oUtm.z);

	bSave_PLY("c:\\tmp\\1.ply", oPC);	
	Free_Point_Cloud(&oPC);
	if (pFile)
		fclose(pFile);
	return;
}

static void Gnss_Test_2()
{//感觉这个才对
	typedef double _T;
	unsigned long long tStart = iGet_Tick_Count();
	FILE* pFile = fopen("D:\\software\\3rdparty\\slam_in_autonomous_driving-master\\data\\ch3\\10.txt", "rb");
	if (!pFile) { printf("Fail to open file\n"); return; };

	char Line[256];
	int i, iCount = 390000;

	Point_Cloud<_T> oPC;
	Init_Point_Cloud(&oPC, 40000);

	UTM_Param<_T> oParam;
	utm<_T> oUtm;
	_T Map_Origin[3] = { 0 };

	for (i = 0; i < iCount && iRead_Line(pFile, Line, 256);)
	{
		if (strstr(Line, "GNSS "))
		{
			GNSS<_T> oGnss;
			//if(typeid(_T) == typeid(float)))
			sscanf(Line, "GNSS %lf %lf %lf %lf %lf %d", &oGnss.m_fTime,
				&oGnss.GPS[0], &oGnss.GPS[1], &oGnss.GPS[2],
				&oGnss.m_fHeading, &oGnss.m_bHeading_Valid);

			oGnss.GPS[0] *= (_T)(PI / 180.f);
			oGnss.GPS[1] *= (_T)(PI / 180.f);

			//将GNSS经纬度转换为一种方便的坐标
			if (!bLat_Long_2_utm(oGnss.GPS[0], oGnss.GPS[1], &oParam, &oUtm.m_fNorth, &oUtm.m_fEast, &oUtm.m_iZone, &oUtm.m_bNorth))
				return;
			oUtm.m_fHeight = oGnss.GPS[2];
			
			if (i == 0)	//设置原点
				Map_Origin[0] = oUtm.m_fEast, Map_Origin[1] = oUtm.m_fNorth, Map_Origin[2] = oUtm.m_fHeight;

			//所有位置改成相对于原点位置
			oUtm.x = oUtm.m_fEast-Map_Origin[0], oUtm.y = oUtm.m_fNorth - Map_Origin[1], oUtm.z = oUtm.m_fHeight - Map_Origin[2];
			//printf("%f %f %f\n", oUtm.x, oUtm.y, oUtm.z);
			i++;
			Draw_Point(&oPC, oUtm.x, oUtm.y, oUtm.z);
		}
	}

	printf("%lld %.7ef %.7ef %.7ef\n", iGet_Tick_Count() - tStart, oUtm.x, oUtm.y, oUtm.z);

	bSave_PLY("c:\\tmp\\1.ply", oPC);	
	Free_Point_Cloud(&oPC);
	if (pFile)
		fclose(pFile);
	return;
}

template<typename _T>void Get_Imu(FILE *pFile,IMU<_T>** ppImu, int* piCount, _T init_time_seconds_)
{//读一大段imu数据
	char Line[256];
	int i = 0, iCount;
	_T init_start_time_;

	//假定imu每秒100条以上
	iCount = (int)(init_time_seconds_ * 101);
	IMU<_T>* pImu = (IMU<_T>*)pMalloc( iCount * sizeof(IMU<_T>));

	for (i=0;i<iCount && iRead_Line(pFile, Line, 256);)
	{
		if (strstr(Line, "IMU "))
		{
			IMU<_T> oImu;
			sscanf(Line, "IMU %lf %lf %lf %lf %lf %lf %lf ", &oImu.m_fTime,
				&oImu.a[0], &oImu.a[1], &oImu.a[2],	&oImu.w[0], &oImu.w[1], &oImu.w[2]);

			if (i == 0)
				init_start_time_ = oImu.m_fTime;

			pImu[i++] = oImu;

			if (oImu.m_fTime - init_start_time_ > init_time_seconds_)
				break;	//够数据了
		}
	}

	*piCount=iCount = i;
	Shrink(pImu, iCount * sizeof(IMU<_T>));
	*ppImu = pImu;
	return;
}

template<typename _T>void Build_Noise(ESKF<_T> *poEskf)
{//难道确定噪声的初值？
	ESKF_Param<_T>* poParam = &poEskf->m_oParam;
	_T Var_a_2 = poEskf->Var_a;  // * ev;
	_T Var_w_2 = poEskf->Var_w;  // * et;
	_T bw_2 = poParam->bias_gyro_var_;  // * eg;
	_T ba_2 = poParam->bias_acce_var_;  // * ea;

	memset(poEskf->Q_, 0, sizeof(poEskf->Q_));
	for (int i = 0; i < 3; i++)
	{
		int j = i + 3;
		poEskf->Q_[j * 18 + j] = Var_a_2;
		j = i + 6;
		poEskf->Q_[j * 18 + j] = Var_w_2;
		j = i + 9;
		poEskf->Q_[j * 18 + j] = bw_2;
		j = i + 12;
		poEskf->Q_[j * 18 + j] = ba_2;
	}
	//Disp(poEskf->Q_, 18, 18, "Q_");

	//后面的用不上
	//memset(poParam->odom_noise_, 0,sizeof(poParam->odom_noise_));
	//poParam->odom_noise_[0] = poParam->odom_noise_[4] =
	//	poParam->odom_noise_[8] =  poParam->odom_var_ * poParam->odom_var_;

	//memset(poParam->gnss_noise_, 0, sizeof(poParam->gnss_noise_));
	//// 设置GNSS状态
	//_T gp2 = poParam->gnss_pos_noise_ * poParam->gnss_pos_noise_;
	//_T gh2 = poParam->gnss_height_noise_ * poParam->gnss_height_noise_;
	//_T ga2 = poParam->gnss_ang_noise_ * poParam->gnss_ang_noise_;
	//poParam->gnss_noise_[0] = poParam->gnss_noise_[1 * 6 + 1] = gp2;
	//poParam->gnss_noise_[2 * 6 + 2] = gh2;
	//poParam->gnss_noise_[3 * 6 + 3] = poParam->gnss_noise_[4 * 6 + 4] =
	//	poParam->gnss_noise_[5 * 6 + 5] = ga2;
	////Disp(poParam->gnss_noise_, 6, 6, "gnss_noise_");
	return;
}

template<typename _T>int bCal_E_Cov(ESKF<_T> *poEskf, IMU<_T> Imu[], int iCount)
{
	int i;
	_T E_a[3] = {}, E_w[3]= {}, Var_a[3]= {}, Var_w[3]= {};
	ESKF_Param<_T>* poParam = &poEskf->m_oParam;
	for (i = 0; i < iCount; i++)
	{
		IMU<_T> oImu = Imu[i];
		E_a[0] += oImu.a[0], E_a[1] += oImu.a[1], E_a[2] += oImu.a[2];
		E_w[0] += oImu.w[0], E_w[1] += oImu.w[1], E_w[2] += oImu.w[2];
	}
	Vector_Multiply(E_a, 3, (_T)1. / iCount, E_a);
	Vector_Multiply(E_w, 3, (_T)1. / iCount, E_w);
	//Disp(E_a, 1, 3, "E_a");
	//Disp(E_w, 1, 3, "E_w");

	//算方差
	for (i = 0; i < iCount; i++)
	{
		IMU<_T> oImu = Imu[i];
		Var_a[0] += (oImu.a[0] - E_a[0]) * (oImu.a[0] - E_a[0]);
		Var_a[1] += (oImu.a[1] - E_a[1]) * (oImu.a[1] - E_a[1]);
		Var_a[2] += (oImu.a[2] - E_a[2]) * (oImu.a[2] - E_a[2]);

		Var_w[0] += (oImu.w[0] - E_w[0]) * (oImu.w[0] - E_w[0]);
		Var_w[1] += (oImu.w[1] - E_w[1]) * (oImu.w[1] - E_w[1]);
		Var_w[2] += (oImu.w[2] - E_w[2]) * (oImu.w[2] - E_w[2]);
	}

	//此处有异议，参考程序要-1，暂时对齐
	_T fRecip = (_T) 1./(iCount - 1);
	Vector_Multiply(Var_a, 3, fRecip, Var_a);
	Vector_Multiply(Var_w, 3, fRecip, Var_w);
	//Disp(Cov_a, 1, 3, "Cov_a");
	//Disp(Cov_w, 1, 3, "Cov_w");

	//对a的期望做某种规格化
	_T g[3],fValue = fGet_Mod(E_a, 3);
	Vector_Multiply(E_a, 3, -poParam->gravity_norm_/fValue, g);
	Vector_Add(E_a, g, 3, E_a);

	//判断协方差是否偏离太大（噪声太大）
	if ( (fValue=fGet_Mod(Var_w,3)) > poParam->max_static_gyro_var) 
	{
		printf("陀螺仪测量噪声太大%f", fValue);
		return 0;
	}	
	if ( (fValue=fGet_Mod(Var_a,3))> poParam->max_static_acce_var) 
	{
		printf("加计测量噪声太大%f\n",fValue);
		return 0;
	}

	//方差能很好反映噪声特征，算个方差
	poEskf->Var_a = sqrt(Var_a[0]);
	poEskf->Var_w = sqrt(Var_w[0]);
	poEskf->m_fCur_Time =Imu[iCount-1].m_fTime;
	//printf("%.7ef %.7ef\n", poEskf->Var_a,poEskf->Var_w);

	//将样本的期望作为零偏，有道理，但是bg为什么是角速度的期望，待考
	memcpy(poParam->init_ba_, E_a, 3 * sizeof(_T));
	memcpy(poParam->init_bg_, E_w, 3 * sizeof(_T));

	Build_Noise(poEskf);
	memcpy(poEskf->ba, poParam->init_ba_, sizeof(poEskf->ba));
	memcpy(poEskf->bg, poParam->init_bg_, sizeof(poEskf->bg));
	memcpy(poEskf->g, g, sizeof(g));

	memset(poEskf->P, 0, sizeof(poEskf->P));
	Add_I_Matrix(poEskf->P, 18, 1e-4);
	//Disp(poEskf->P, 18, 18, "P");

	return 1;
}
template<typename _T>int bInit_Imu(FILE* pFile, ESKF<_T> *poEskf)
{//从以往数据分析处各种初值
	IMU<_T>* pImu;		//数据集
	ESKF_Param<_T>* poParam = &poEskf->m_oParam;
	int bRet=0, iCount;			//数据集大小
	Get_Imu(pFile,&pImu, &iCount, poEskf->m_oParam.init_time_seconds_);

	//开始初始化
	if (iCount < 10)
	{
		printf("Insufficient imu count in queue\n");
		goto END;
	}
	if (!bCal_E_Cov(poEskf, pImu, iCount))
		goto END;

	bRet = 1;
END:
	if (pImu)
		Free(pImu);
	return bRet;
}

template<typename _T>int bInit_GNSS(FILE* pFile, ESKF<_T>* poEskf)
{
	GNSS<_T> oGnss;
	ESKF_Param<_T>* poParam = &poEskf->m_oParam;
	char Line[256];
	int iResult;

	//移动到吓一条GNSS
	while (iRead_Line(pFile, Line, 256))
	{
		if (strstr(Line, "GNSS"))
		{
			iResult = sscanf(Line, "GNSS %lf %lf %lf %lf %lf %d", &oGnss.m_fTime,
				&oGnss.GPS[0], &oGnss.GPS[1], &oGnss.GPS[2],
				&oGnss.m_fHeading, &oGnss.m_bHeading_Valid);
			oGnss.m_bHeading_Valid = 1;
			break;
		}
	}

	//将GNSS纬经高转换为某种x,y,h坐标，以后用东北天坐标
	_T map_origin[3] = {};
	UTM_Param<_T> oUTM_Param;
	utm<_T> oUtm;
	if (!bLat_Long_2_utm_ang(oGnss, poParam->antenna_pos[0], poParam->antenna_pos[1],
		poParam->antenna_angle, map_origin, &oUTM_Param, &oUtm))
		return 0;

	Get_R_t(oUtm.m_Pose, poEskf->R, poEskf->Origin);
	Rotation_Matrix_2_Vector_3(poEskf->R, poEskf->Rotation_Vector);
	memset(poEskf->p, 0, sizeof(poEskf->p));
	poEskf->m_fCur_Time = oGnss.m_fTime;

	return 1;
}
template<typename _T>void Disp_Eskf(ESKF<_T> oEskf)
{
	Disp(oEskf.p, 1, 3, "p");
	Disp(oEskf.v, 1, 3, "v");
	Disp(oEskf.R, 3, 3, "q");
	Disp(oEskf.ba, 1, 3, "ba");
	Disp(oEskf.bg, 1, 3, "bg");
	Disp(oEskf.g, 1, 3, "g");
}
template<typename _T>void Predict(ESKF<_T>* poEskf, IMU<_T>oImu)
{//ESKF的预测阶段
	_T dt = oImu.m_fTime - poEskf->m_fCur_Time;
	Update_pvq(oImu, poEskf->p, poEskf->v, poEskf->R, poEskf->ba, poEskf->bg, poEskf->g, dt);
	//Disp_Eskf(*poEskf);


	return;
}
void Eskf_Test_1()
{//经典误差卡尔曼滤波器
	typedef double _T;
	FILE* pFile = fopen("D:\\software\\3rdparty\\slam_in_autonomous_driving-master\\data\\ch3\\10.txt", "rb");
	if (!pFile)	{printf("Fail to open file\n");	return;	}

	char Line[256];			//装一行数据
	int i, iCount = 100;	//共装多少数据

	//首先对imu进行初始化
	ESKF<_T> oEskf;
	ESKF_Param<_T> oParam;

	if (!bInit_Imu(pFile, &oEskf))
		return;

	if (!bInit_GNSS(pFile, &oEskf))
		return;

	//Disp(oEskf.R, 3, 3, "R");
	for (i = 1;i<iCount && iRead_Line(pFile, Line, 256); i++)
	{
		if (strstr(Line, "IMU "))
		{
			IMU<_T> oImu;
			sscanf(Line, "IMU %lf %lf %lf %lf %lf %lf %lf ", &oImu.m_fTime,
				&oImu.a[0], &oImu.a[1], &oImu.a[2], &oImu.w[0], &oImu.w[1], &oImu.w[2]);
			Predict(&oEskf, oImu);
		}
	}

	if (pFile)
		fclose(pFile);
	return;
}

void Kf_Test_1()
{//这个跟随参考程序
	typedef double _T;
	_T xk[2];	//x包括两项，位置+速度
	_T p0 = 0, v0 = 5, a = 0.6;
	_T P[4] = { 0.1,0,		//先验误差协方差矩阵
		0,	0.1 };
	_T F[4] = { 1,	1,		//状态转移矩阵
		0,	1 };
	_T B[4] = { 0.5, 1 };
	_T H[4] = { 1,	0,		//显然，这个是废的，没有起到挑选分量的作用
		0,	1 };
	_T Q[4] = { 10,	0,		//过程噪声协方差，各维之间互相独立，协方差为0
		0,	10 };
	_T R[4] = { 10000,	0,	//观测噪声协方差，各维之间互相独立，协方差为0
		0,	10000 };
	_T K[4];

	_T real_positions[100], real_speeds[100];
	_T z[100][2];

	//先造数据,为了和参考程序对数据，样本抄自参考程序
	xk[0] = 0, xk[1] = v0;
	for (int i = 0; i < 100; i++)
	{
		_T Temp[4];
		//xk = F * xk_1 + B * a + w
		Matrix_Multiply(F, 2, 2, xk, 1, xk);	// F * xk
		Matrix_Multiply(B, 2, 1, a, Temp);		// B * a
		Vector_Add(xk, Temp, 2, xk);			// F * xk + B * a
		Vector_Add(xk, w_Noise[i], 2, xk);		// xk = F * xk_1 + B * a + w
		real_positions[i] = xk[0];
		real_speeds[i] = xk[1];

		Matrix_Multiply(H, 2, 2, xk, 1, Temp);
		Vector_Add(Temp, v_Noise[i], 2, z[i]);
		//Disp(z[i], 1, 2, "zk");
	}

	//以下是真正的滤波过程
	//xk的最初值
	xk[0] = p0, xk[1] = 0;
	for (int i = 0; i < 100; i++)
	{
		_T Temp[4],Ht[4];

		//xk = F*xk_1 + B*uk
		Matrix_Multiply(F, 2, 2, xk, 1, xk);	//F*xk_1
		Vector_Multiply(B, 2, a, Temp);			//B*uk
		Vector_Add(xk, Temp, 2, xk);			//F*xk_1 + B*uk
		//Disp(xk, 1, 2, "xk");

		//Pk = F * P * F' + Q	先验误差协方差矩阵
		Matrix_Transpose(F, 2, 2, Temp);			//P'
		Matrix_Multiply(P, 2, 2, Temp, 2, Temp);	//P*F'
		Matrix_Multiply(F, 2, 2, Temp, 2, Temp);	//F*P*P'
		Matrix_Add(Temp, Q, 2, P);					//F*P*P' + Q
		//Disp(P, 2, 2, "P");

		//计算K = P * H' *  (H * P * H' + R)^(-1)
		Matrix_Transpose(H, 2, 2, Ht);				//H'
		Matrix_Multiply(P, 2, 2, Ht, 2, Temp);		//P*H'
		Matrix_Multiply(H, 2, 2, Temp, 2, Temp);	//H*P*H'
		Matrix_Add(Temp, R, 2, Temp);				//H * P * H' + R		
		Get_Inv_Matrix_Row_Op(Temp, Temp, 2);		//(H * P * H' + R)^-1
		Matrix_Multiply(Ht, 2, 2, Temp, 2, Temp);	//H' *  (H * P * H' + R)^(-1)
		Matrix_Multiply(P, 2, 2, Temp, 2, K);		//P * H' *  (H * P * H' + R)^(-1)
		//Disp(K, 2, 2, "K");

		
		//xk	= xk + K * (zk - H * xk)
		////		= K*zk + (I - K)*xk
		//{		//xk= K*zk + (I - K)*xk
		//	_T Temp1[4];
		//	Matrix_Multiply(K, 2, 2, z[i], 1, Temp);
		//	//Disp(Temp, 1, 2, "K*zk");
		//	Matrix_Multiply(K, 2, 2, (_T)-1, Temp1);
		//	Add_I_Matrix(Temp1, 2);
		//	Disp(K, 2, 2, "K");
		//	Disp(Temp1, 2, 2, "I - K");
		//	Matrix_Multiply(Temp1, 2, 2, xk, 1, Temp1);
		//	Disp(Temp1, 1, 2, "(I - K)*xk");
		//	Vector_Add(Temp, Temp1, 2, Temp1);
		//	Disp(Temp1, 1, 2, "xk");
		//}
		
		//xk	= xk + K * (zk - H * xk)
		Matrix_Multiply(H, 2, 2, xk, 1, Temp);		//H * xk
		Vector_Minus(z[i], Temp, 2, Temp);			//zk - H * xk
		//Disp(z[i], 1, 2, "zk");
		Matrix_Multiply(K, 2, 2, Temp, 1, Temp);	//K * (zk - H * xk)
		Vector_Add(xk, Temp, 2, xk);				//xk = xk + K * (zk - H * xk)
		//Disp(K, 2, 2, "K");
		Disp(xk, 1, 2, "xk");

		//P = (I - K*H ) * P	后验误差协方差矩阵
		Matrix_Multiply(K, 2, 2, H, 2, Temp);			//K*H
		Matrix_Multiply(Temp, 2, 2, (_T) - 1, Temp);	//-K*H
		Add_I_Matrix(Temp, 2);							//I - K*H
		Matrix_Multiply(Temp, 2, 2, P, 2, P);			//(I - K*H ) * P
		//Disp(P, 2, 2, "P");
	}

	//Disp(xk, 1, 2, "xk_Optim");
	return;
}

template<typename _T>void Get_Cov(_T Sample[][2], int iSample_Count, _T Cov[4])
{//通过若干个样本计算其协方差矩阵
	static int iCount = 0;
	int i;
	_T E[2] = {}, Cov_1[4] = {};
	_T fRecip = (_T)1.f / iSample_Count;

	for (i = 0; i < iSample_Count; i++)
	{
		/*if(iCount==4)
		Disp(Sample[i], 1, 2, "xk");*/
		Vector_Add(Sample[i], E, 2, E);
	}
	E[0] *=fRecip, E[1] *=fRecip;
	/*if (iCount == 4)
	Disp(E, 1, 2, "E");*/
	for (i = 0; i < iSample_Count; i++)
	{
		Cov_1[0] += (Sample[i][0] - E[0]) * (Sample[i][0] - E[0]);
		Cov_1[3] += (Sample[i][1] - E[1]) * (Sample[i][1] - E[1]);
		Cov_1[1]= (Cov_1[2]+= (Sample[i][0] - E[0]) *(Sample[i][1] - E[1]));
	}	
	Vector_Multiply(Cov_1, 4, fRecip, Cov);
	//Disp(Cov, 2, 2, "Cov");
	iCount++;
}

void Kf_Test_2()
{//不断更新Q,R以反映协方差变化
	typedef double _T;
	const int iWin_Size=20,		//滑动窗口大小，用于计算Q,R
		iSample_Count=100;		//一共仿真多少组样本
	_T xk[2],	//xk真实值，x包括两项，位置+速度
		xk_Pred[2],		//预测值
		xk_Optim[2];	//最优值

	_T zk[2];	//观测值
	_T p0 = 0, v0 = 5, a = 0.00;
	_T P[4] = { 0.1,0,		//先验误差协方差矩阵
		0,	0.1 };
	_T F[4] = { 1,	1,		//状态转移矩阵
		0,	1 };
	_T B[4] = { 0.5, 1 };
	_T H[4] = { 1,	0,		//显然，这个是废的，没有起到挑选分量的作用
		0,	1 };
	_T Q[4] = {};			//xk过程噪声协方差，各维之间互相独立，协方差为0
	_T R[4] = {};			//zk观测噪声协方差，各维之间互相独立，协方差为0

	_T K[4];				//权值矩阵，决定预测值与观测值的比例
	//_T xk_Optim_All[iWin_Size + iSample_Count][2];
	//_T zk_All[iWin_Size + iSample_Count][2];
	_T(*xk_Optim_All)[2] = (_T(*)[2])pMalloc((iSample_Count + iWin_Size) * 2 * sizeof(_T));
	_T(*zk_All)[2] = (_T(*)[2])pMalloc((iSample_Count + iWin_Size) * 2 * sizeof(_T));

	int i;
	//*************第一步，为滑动窗口做数据*********************
	xk[0] = 0, xk[1] = v0;
	_T fVar_xk = sqrt(10), fVar_zk = sqrt(10000);
	for (i = 0; i < iWin_Size; i++)
	{
		_T Temp[2];
		//造zk = 
		Matrix_Multiply(F, 2, 2, xk, 1, xk);	// F * xk
		Matrix_Multiply(B, 2, 1, a, Temp);		// B * a
		Vector_Add(xk, Temp, 2, xk);			// F * xk + B * a
		//Disp(xk, 1, 2, "xk");
		xk_Optim_All[i][0] = xk[0] + fGet_Random_No((_T)0, fVar_xk);	// xk = F * xk_1 + B * a + w
		xk_Optim_All[i][1] = xk[1] + fGet_Random_No((_T)0, fVar_xk);	//w ~ N(0, sqrt(10))

		//造zk
		zk_All[i][0] = xk[0] + fGet_Random_No((_T)0, fVar_zk);
		zk_All[i][1] = xk[1] + fGet_Random_No((_T)0, fVar_zk);
	}

	Get_Cov(xk_Optim_All,iWin_Size, Q);
	Get_Cov(zk_All, iWin_Size, R);
	//Disp(Q, 2, 2, "Q");
	//Disp(R, 2, 2, "R");
	//*************第一步，为滑动窗口做数据*********************

	//**********************初始化图****************************
	Image oImage;
	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP, 24);
	//**********************初始化图****************************

	//**********************第二部，卡尔曼滤波**********************
	memcpy(xk_Optim, xk_Optim_All[iWin_Size - 1], 2 * sizeof(_T));
	for (i = 0; i < iSample_Count; i++)
	{
		_T Temp[4], Ht[4];

		//真实值 xk = F*xk_1 + B*uk
		Matrix_Multiply(F, 2, 2, xk, 1, xk);	//F*xk_1
		Vector_Multiply(B, 2, a, Temp);			//B*uk
		Vector_Add(xk, Temp, 2, xk);			//F*xk_1 + B*uk

		//预测值 xk_Pred = F * xk_1_Optim + B * uk
		Matrix_Multiply(F, 2, 2, xk_Optim, 1, xk_Pred);	//F*xk_1
		Vector_Multiply(B, 2, a, Temp);					//B*uk
		Vector_Add(xk_Pred, Temp, 2, xk_Pred);			//F*xk_1 + B*uk

		//造zk数据
		zk[0] = xk[0] + fGet_Random_No((_T)0, fVar_zk);
		zk[1] = xk[1] + fGet_Random_No((_T)0, fVar_zk);

		//计算先验协方差 Pk = F * P * F' + Q	先验误差协方差矩阵
		Matrix_Transpose(F, 2, 2, Temp);			//P'
		Matrix_Multiply(P, 2, 2, Temp, 2, Temp);	//P*F'
		Matrix_Multiply(F, 2, 2, Temp, 2, Temp);	//F*P*P'
		Matrix_Add(Temp, Q, 2, P);					//F*P*P' + Q
		//if(i==0)
		//Disp(Q, 2, 2, "Q");
		//计算K = P * H' *  (H * P * H' + R)^(-1)
		Matrix_Transpose(H, 2, 2, Ht);				//H'
		Matrix_Multiply(P, 2, 2, Ht, 2, Temp);		//P*H'
		Matrix_Multiply(H, 2, 2, Temp, 2, Temp);	//H*P*H'
		Matrix_Add(Temp, R, 2, Temp);				//H * P * H' + R		
		Get_Inv_Matrix_Row_Op(Temp, Temp, 2);		//(H * P * H' + R)^-1
		Matrix_Multiply(Ht, 2, 2, Temp, 2, Temp);	//H' *  (H * P * H' + R)^(-1)
		Matrix_Multiply(P, 2, 2, Temp, 2, K);		//P * H' *  (H * P * H' + R)^(-1)

		//计算最优值 xk	= xk_Pred + K * (zk - H * xk_Pred)
		Matrix_Multiply(H, 2, 2, xk_Pred, 1, Temp);		//H * xk
		Vector_Minus(zk, Temp, 2, Temp);				//zk - H * xk
		Matrix_Multiply(K, 2, 2, Temp, 1, Temp);	//K * (zk - H * xk)
		Vector_Add(xk_Pred, Temp, 2, xk_Optim);				//xk = xk + K * (zk - H * xk)
		//Disp(K, 2, 2, "K");
		//Disp(xk_Optim, 1, 2, "xk_Optim");

		//P = (I - K*H ) * P	后验误差协方差矩阵
		Matrix_Multiply(K, 2, 2, H, 2, Temp);			//K*H
		Matrix_Multiply(Temp, 2, 2, (_T) - 1, Temp);	//-K*H
		Add_I_Matrix(Temp, 2);							//I - K*H
		Matrix_Multiply(Temp, 2, 2, P, 2, P);			//(I - K*H ) * P

		//更新Q,R
		memcpy(xk_Optim_All[i+iWin_Size], xk_Optim, 2 * sizeof(_T));
		memcpy(zk_All[i + iWin_Size], zk, 2 * sizeof(_T));
		Get_Cov((_T(*)[2])xk_Optim_All[i+1], iWin_Size, Q);
		Get_Cov((_T(*)[2])zk_All[i+1], iWin_Size, R);
		//Disp(Q, 2, 2, "Q");
		//Disp(xk, 1, 2, "xk_Real");
		Disp(xk_Optim, 1, 2, "xk_Optim");
		//Disp(zk, 1, 2, "zk");
		//printf("\n");

		//画图 红，真实值，绿：最优值，蓝：观察值
		//画位置偏移
		//Draw_Point(oImage, xk[0],i*10,2,255,0,0);
		//Draw_Point(oImage, xk_Optim[0],i*10, 2, 0, 255, 0);
		//Draw_Point(oImage, zk[0],i*10, 2, 0, 0, 255);

		////画速度偏移
		//Draw_Point(oImage, xk[1]+960,i*10,2,255,0,0);
		//Draw_Point(oImage, xk_Optim[1]+960,i*10, 2, 0, 255, 0);
		//Draw_Point(oImage, zk[1]+960,i*10, 2, 0, 0, 255);
	}

	//**********************第二部，卡尔曼滤波**********************
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	//Disp(Q, 2, 2, "Q");
	//Disp(R, 2, 2, "R");

	Free_Image(&oImage);
	Free(xk_Optim_All);
	Free(zk_All);
	return;
}

template<typename _T>struct KF {	//将状态，Q，F滑动窗口，观察全搞里头
	
	_T p0 = 0, v0 = 5, a = 0.00,
		fVar_xk = sqrt(10), fVar_zk = sqrt(10000);
	int iWin_Size = 20;			//滑动窗口大小，用于计算Q,R

	_T	xk[2],
		xk_Pred[2],			//预测值
		xk_Optim[2],		//最优值
		zk[2];

	_T P[4] = { 0.1,0,		//先验误差协方差矩阵
		0,	0.1 };
	_T F[4] = { 1,	1,		//状态转移矩阵
		0,	1 };
	_T B[4] = { 0.5, 1 };
	_T H[4] = { 1,	0,		//显然，这个是废的，没有起到挑选分量的作用
		0,	1 };
	_T Q[4] = {};			//xk过程噪声协方差，各维之间互相独立，协方差为0
	_T R[4] = {};			//zk观测噪声协方差，各维之间互相独立，协方差为0

	_T K[4];				//权值矩阵，决定预测值与观测值的比例
};

template<typename _T>struct KF_Win_Queue{
	_T m_fSigma_xi[2],				//Sigma(xi)
	m_fSigma_xi_Minus_E[2],			//Sigma(xi-E)
	//m_fSigma_xi_Minus_E_Sqr[2],	//Sigma (xi-E)^2
	m_fSigma_Cov_x_N[4],			//Sigma(xp-Ep)*(xq-Eq)
	E[2],							//期望
	//D[2],							//方差
	Cov[4];							//协方差
	_T (*m_pBuffer)[2];
};
template<typename _T>struct KF_Win {
	KF_Win_Queue<_T> Q,			//xk过程噪声协方差，各维之间互相独立，协方差为0
		R;						//zk观测噪声协方差，各维之间互相独立，协方差为0
	int m_iHead;	//队头
	int m_iEnd;		//队尾
	int m_iCount;	//当前队列的元素个数
	int m_iBuffer_Size;
};

template<typename _T>void Init_Queue(KF_Win<_T>* poQueue, int iBuffer_Size)
{
	poQueue->m_iBuffer_Size = iBuffer_Size;
	//poQueue->m_iCount = poQueue->m_iEnd = poQueue->m_iHead = 0;
	poQueue->m_iEnd = 0;
	poQueue->Q.m_pBuffer= (_T(*)[2])pMalloc(sizeof(_T) * 2 * 2 * iBuffer_Size);	//为了省地方，只存索引，不存内容
	poQueue->R.m_pBuffer = poQueue->Q.m_pBuffer + iBuffer_Size;
	return;
}
template<typename _T>void Free_Queue(KF_Win<_T>* poQueue)
{
	Free(poQueue->Q.m_pBuffer);
}

template<typename _T>void Init_Kf_Win(KF_Win<_T> *poWin_Queue, KF<_T> *poKf)
{
	KF_Win<_T> oWin_Queue = {};
	_T xk[2], zk[2];
	int i;
	xk[0] = 0, xk[1] = poKf->v0;;
	Init_Queue(&oWin_Queue, poKf->iWin_Size);
	_T fRecip = (_T)1.f / oWin_Queue.m_iBuffer_Size;

	for (i = 0; i < oWin_Queue.m_iBuffer_Size; i++)
	{
		_T Temp[4];
		Matrix_Multiply(poKf->F, 2, 2, xk, 1, xk);	// F * xk
		Matrix_Multiply(poKf->B, 2, 1, poKf->a, Temp);		// B * a
		Vector_Add(xk, Temp, 2, xk);			// F * xk + B * a

		oWin_Queue.Q.m_pBuffer[i][0] = xk[0] + fGet_Random_No((_T)0, poKf->fVar_xk);	// xk = F * xk_1 + B * a + w
		oWin_Queue.Q.m_pBuffer[i][1] = xk[1] + fGet_Random_No((_T)0, poKf->fVar_xk);	//w ~ N(0, sqrt(10))
		oWin_Queue.Q.E[0] += oWin_Queue.Q.m_pBuffer[i][0], oWin_Queue.Q.E[1] += oWin_Queue.Q.m_pBuffer[i][1];
		//Disp(oWin_Queue.Q.m_pBuffer[i], 1, 2, "xk");

		//造zk
		oWin_Queue.R.m_pBuffer[i][0] = zk[0]=xk[0] + fGet_Random_No((_T)0, poKf->fVar_zk);
		oWin_Queue.R.m_pBuffer[i][1] = zk[1]=xk[1] + fGet_Random_No((_T)0, poKf->fVar_zk);
		oWin_Queue.R.E[0] += zk[0], oWin_Queue.R.E[1] += zk[1];
	}
	//∑xi
	oWin_Queue.Q.m_fSigma_xi[0] = oWin_Queue.Q.E[0],oWin_Queue.Q.m_fSigma_xi[1] = oWin_Queue.Q.E[1];
	oWin_Queue.R.m_fSigma_xi[0] = oWin_Queue.R.E[0],oWin_Queue.R.m_fSigma_xi[1] = oWin_Queue.R.E[1];

	//E
	//oWin_Queue.Q.E[0] /= poKf->iWin_Size, oWin_Queue.Q.E[1] /= poKf->iWin_Size;
	//oWin_Queue.R.E[0] /= poKf->iWin_Size, oWin_Queue.R.E[1] /= poKf->iWin_Size;
	Vector_Multiply(oWin_Queue.Q.E, 2, fRecip,oWin_Queue.Q.E);
	Vector_Multiply(oWin_Queue.R.E, 2, fRecip,oWin_Queue.R.E);
	//Disp(oWin_Queue.Q.E, 1, 2, "E");

	//∑(xi - E)
	oWin_Queue.Q.m_fSigma_xi_Minus_E[0] = oWin_Queue.Q.m_fSigma_xi[0] - oWin_Queue.Q.E[0];
	oWin_Queue.Q.m_fSigma_xi_Minus_E[1] = oWin_Queue.Q.m_fSigma_xi[1] - oWin_Queue.Q.E[1];
	oWin_Queue.R.m_fSigma_xi_Minus_E[0] = oWin_Queue.R.m_fSigma_xi[0] - oWin_Queue.R.E[0];
	oWin_Queue.R.m_fSigma_xi_Minus_E[1] = oWin_Queue.R.m_fSigma_xi[1] - oWin_Queue.R.E[1];
	
	//算个∑(xi - E)*(xi - E)
	for (i = 0; i < oWin_Queue.m_iBuffer_Size; i++)
	{
		_T dx0=oWin_Queue.Q.m_pBuffer[i][0] - oWin_Queue.Q.E[0], 
			dx1=oWin_Queue.Q.m_pBuffer[i][1] - oWin_Queue.Q.E[1];
		
		oWin_Queue.Q.m_fSigma_Cov_x_N[0] += dx0 * dx0;
		oWin_Queue.Q.m_fSigma_Cov_x_N[3] += dx1 * dx1;
		oWin_Queue.Q.m_fSigma_Cov_x_N[1] += dx0 * dx1;

		dx0=oWin_Queue.R.m_pBuffer[i][0] - oWin_Queue.R.E[0], 
			dx1=oWin_Queue.R.m_pBuffer[i][1] - oWin_Queue.R.E[1];
		oWin_Queue.R.m_fSigma_Cov_x_N[0] += dx0 * dx0;
		oWin_Queue.R.m_fSigma_Cov_x_N[3] += dx1 * dx1;
		oWin_Queue.R.m_fSigma_Cov_x_N[1] += dx0 * dx1;
	}
	oWin_Queue.Q.m_fSigma_Cov_x_N[2] = oWin_Queue.Q.m_fSigma_Cov_x_N[1];
	oWin_Queue.R.m_fSigma_Cov_x_N[2] = oWin_Queue.R.m_fSigma_Cov_x_N[1];
	//Disp(oWin_Queue.Q.m_fSigma_Cov_x_N, 2, 2, "Cov_x_N");

	Vector_Multiply(oWin_Queue.Q.m_fSigma_Cov_x_N, 4,fRecip, oWin_Queue.Q.Cov);
	Vector_Multiply(oWin_Queue.R.m_fSigma_Cov_x_N, 4, fRecip, oWin_Queue.R.Cov);
	//Disp(oWin_Queue.Q.Cov, 2, 2, "Q");
	memcpy(poKf->Q, oWin_Queue.Q.Cov, 4 * sizeof(_T));
	memcpy(poKf->R, oWin_Queue.R.Cov, 4 * sizeof(_T));

	memcpy(poKf->xk, xk, 2 * sizeof(_T));
	memcpy(poKf->zk, zk, 2 * sizeof(_T));

	*poWin_Queue = oWin_Queue;
	return;
}
template<typename _T>void Update_Kf(KF<_T> *poKf_1)
{
	static int iCount = 0;
	KF<_T> oKf = *poKf_1;
	_T Temp[4], Ht[4];

	//预测值 xk_Pred = F * xk_1_Optim + B * uk
	Matrix_Multiply(oKf.F, 2, 2, oKf.xk_Optim, 1, oKf.xk_Pred);	//F*xk_1
	Vector_Multiply(oKf.B, 2, oKf.a, Temp);						//B*uk
	Vector_Add(oKf.xk_Pred, Temp, 2, oKf.xk_Pred);				//F*xk_1 + B*uk
	
	//计算先验协方差 Pk = F * P * F' + Q	先验误差协方差矩阵
	Matrix_Transpose( oKf.F, 2, 2, Temp);			//P'
	Matrix_Multiply( oKf.P, 2, 2, Temp, 2, Temp);	//P*F'
	Matrix_Multiply( oKf.F, 2, 2, Temp, 2, Temp);	//F*P*P'
	Matrix_Add(Temp,  oKf.Q, 2,  oKf.P);			//F*P*P' + Q
	//if(iCount==0)
		//Disp(oKf.Q, 2, 2, "Q");

	//计算K = P * H' *  (H * P * H' + R)^(-1)
	Matrix_Transpose(oKf.H, 2, 2, Ht);				//H'
	Matrix_Multiply(oKf.P, 2, 2, Ht, 2, Temp);		//P*H'
	Matrix_Multiply(oKf.H, 2, 2, Temp, 2, Temp);	//H*P*H'
	Matrix_Add(Temp, oKf.R, 2, Temp);				//H * P * H' + R		
	Get_Inv_Matrix_Row_Op(Temp, Temp, 2);			//(H * P * H' + R)^-1
	Matrix_Multiply(Ht, 2, 2, Temp, 2, Temp);		//H' *  (H * P * H' + R)^(-1)
	Matrix_Multiply(oKf.P, 2, 2, Temp, 2, oKf.K);	//P * H' *  (H * P * H' + R)^(-1)
	

	//计算最优值 xk	= xk_Pred + K * (zk - H * xk_Pred)
	Matrix_Multiply(oKf.H, 2, 2, oKf.xk_Pred, 1, Temp);		//H * xk
	Vector_Minus(oKf.zk, Temp, 2, Temp);					//zk - H * xk
	Matrix_Multiply(oKf.K, 2, 2, Temp, 1, Temp);			//K * (zk - H * xk)
	Vector_Add(oKf.xk_Pred, Temp, 2, oKf.xk_Optim);			//xk = xk + K * (zk - H * xk)
	//Disp(oKf.xk_Optim, 1, 2, "xk_Optim");

	//P = (I - K*H ) * P	后验误差协方差矩阵
	Matrix_Multiply(oKf.K, 2, 2, oKf.H, 2, Temp);			//K*H
	Matrix_Multiply(Temp, 2, 2, (_T) - 1, Temp);	//-K*H
	Add_I_Matrix(Temp, 2);							//I - K*H
	Matrix_Multiply(Temp, 2, 2, oKf.P, 2, oKf.P);			//(I - K*H ) * P

	*poKf_1 = oKf;
	iCount++;
	return;
}
template<typename _T>void Update_Cov(_T E[2], _T Cov[4], _T Sigma_xi[2], _T Sigma_Cov_x_N[4],
	_T xk[2], _T xk_n[2],int n)
{
	static int iCount = 0;
	_T fRecip = (_T)1.f / n;

	//更新 ∑'xj = ∑xi - xk + xk_n
	_T Sigma_xj[2] = { Sigma_xi[0] - xk[0] + xk_n[0], Sigma_xi[1] - xk[1] + xk_n[1] };
	//更新期望E
	_T E1[2] = { Sigma_xj[0] * fRecip,Sigma_xj[1] * fRecip };
	_T xk_minus_E[2] = { xk[0] - E[0], xk[1] - E[1] };
	_T xk_n_minus_E[2] = { xk_n[0] - E[0],xk_n[1] - E[1] };

	//_T Sigma_Cov_x_N_1[4];
	_T de[2] = { E[0] - E1[0],E[1] - E1[1] };
	_T de_sqr[2] = { de[0] * de[0],de[1] * de[1] };

	Sigma_Cov_x_N[0] = Sigma_Cov_x_N[0] - xk_minus_E[0] * xk_minus_E[0] +
		xk_n_minus_E[0] * xk_n_minus_E[0] + 2*de[0]*(xk_n_minus_E[0] - xk_minus_E[0])  + n *de_sqr[0];
	//printf("%lf\n", Sigma_Cov_x_N[0] - xk_minus_E[0] * xk_minus_E[0] + 
	//	xk_n_minus_E[0] * xk_n_minus_E[0] +  2*de[0]*(xk_n_minus_E[0] - xk_minus_E[0])  + n *de_sqr[0]);
	Sigma_Cov_x_N[3] = Sigma_Cov_x_N[3] - xk_minus_E[1] * xk_minus_E[1] +
		xk_n_minus_E[1] * xk_n_minus_E[1] + 2*de[1]*(xk_n_minus_E[1] - xk_minus_E[1])  + n * de_sqr[1];

	Sigma_Cov_x_N[1] = Sigma_Cov_x_N[2] = Sigma_Cov_x_N[1] - xk_minus_E[0] * xk_minus_E[1] + xk_n_minus_E[0] * xk_n_minus_E[1] +	//第一部分
		de[1] * (Sigma_xj[0] -Sigma_xi[0]) +		//第二部分
		de[0] * (Sigma_xj[1] - Sigma_xi[1]) +		//第三部分
		n*de[0] * de[1];
	memcpy(E, E1, 2 * sizeof(_T));
	memcpy(Sigma_xi, Sigma_xj, 2 * sizeof(_T));

	Vector_Multiply(Sigma_Cov_x_N, 4, fRecip, Cov);
	//Disp(Sigma_xi, 1, 2, "Sigma_xi");
	iCount++;
}
template<typename _T>void Update_Q_R(KF<_T>* poKf, KF_Win<_T> *poWin)
{
	KF_Win<_T> oWin = *poWin;

	Update_Cov(oWin.Q.E, oWin.Q.Cov, oWin.Q.m_fSigma_xi, oWin.Q.m_fSigma_Cov_x_N, oWin.Q.m_pBuffer[oWin.m_iHead], poKf->xk_Optim,oWin.m_iBuffer_Size);
	Update_Cov(oWin.R.E, oWin.R.Cov, oWin.R.m_fSigma_xi, oWin.R.m_fSigma_Cov_x_N, oWin.R.m_pBuffer[oWin.m_iHead], poKf->zk,oWin.m_iBuffer_Size);
	//Disp(oWin.Q.Cov,2,2,"Q");
		//xk_n_minus_E[0] * xk_n_minus_E[1];
	//把窗口位置移动一格
	memcpy(oWin.Q.m_pBuffer[oWin.m_iHead], poKf->xk_Optim, 2 * sizeof(_T));
	memcpy(oWin.R.m_pBuffer[oWin.m_iHead], poKf->zk, 2 * sizeof(_T));

	memcpy(poKf->Q, oWin.Q.Cov, 4 * sizeof(_T));
	memcpy(poKf->R, oWin.R.Cov, 4 * sizeof(_T));

	oWin.m_iHead = (oWin.m_iHead + 1) % oWin.m_iBuffer_Size;
	*poWin = oWin;
	return;
}
void Kf_Test_3()
{
	typedef double _T;
	const int iSample_Count = 1000000;		//一共仿真多少组样本
	_T xk[2], 
		zk[2];						//xk真实值，x包括两项，位置+速度
	KF<_T> oKf;

	//初始化滑动窗口
	KF_Win<_T> oWin_Queue;
	Init_Kf_Win(&oWin_Queue, &oKf);
	//Disp(oKf.Q, 2, 2, "Q");
	//Disp(oKf.R, 2, 2, "R");

	memcpy(oKf.xk_Optim, oWin_Queue.Q.m_pBuffer[oWin_Queue.m_iBuffer_Size-1], 2 * sizeof(_T));
	for (int i = 0; i < iSample_Count; i++)
	{
		/*****************************造数据*********************/
		_T Temp[4];
		//真实值 xk = F*xk_1 + B*uk		此处属于造数据
		Matrix_Multiply(oKf.F, 2, 2, oKf.xk, 1, oKf.xk);	//F*xk_1
		Vector_Multiply(oKf.B, 2, oKf.a, Temp);				//B*uk
		Vector_Add(oKf.xk, Temp, 2, oKf.xk);				//F*xk_1 + B*uk
				
		//造zk数据
		oKf.zk[0] = oKf.xk[0] + fGet_Random_No((_T)0, oKf.fVar_zk);
		oKf.zk[1] = oKf.xk[1] + fGet_Random_No((_T)0, oKf.fVar_zk);
		/*****************************造数据*********************/
		
		Update_Kf(&oKf);
		//Disp(oKf.xk_Optim, 1, 2, "xk_Optim");
		//Disp(oKf.K, 2, 2, "K");
		//Disp((_T*)oWin_Queue.Q.m_pBuffer, oWin_Queue.m_iBuffer_Size, 2, "xk");
		//Disp(oKf.Q, 2, 2, "Q");
		Update_Q_R(&oKf, &oWin_Queue);
	}
	Disp(oKf.xk_Optim,1,2,"xk_Optim");

	Free_Queue(&oWin_Queue);
}

int main()
{
	Init_Env();
	//Imu_Test_1();			//只用imu数据进行轨迹推导，画出轨迹
	//Gnss_Test_1();		//仅用gnss数据，画出轨迹
	//Gnss_Test_2();		//修正一下，用相对位置
	
	Kf_Test_1();			//经典卡尔曼滤波器，写死Q,R
	//Kf_Test_2();			//沿路更新Q,R，引入滑动窗口
	//Kf_Test_3();			//优化Q,R滑动窗口的性能，改用自搓滑动窗口一加一减法

	//Eskf_Test_1();			//ESKF推算航迹
	Free_Env();
#ifdef WIN32
	_CrtDumpMemoryLeaks();
#endif
	return 0;
}