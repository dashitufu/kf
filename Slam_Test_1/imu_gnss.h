#pragma once

/*WGS84����ģ�Ͳ���
NOTE:���ʹ����������ģ����Ҫ�޸�������� */
const double WGS84_WIE = 7.2921151467E-5;       /* ������ת���ٶ�*/
const double WGS84_F   = 0.0033528106647474805; /* ���� */
const double WGS84_RA  = 6378137.0000000000;    /* ������a */
const double WGS84_RB  = 6356752.3142451793;    /* �̰���b */
const double WGS84_GM0 = 398600441800000.00;    /* ������������ */
const double WGS84_E1  = 0.0066943799901413156; /* ��һƫ����ƽ�� */
const double WGS84_E2  = 0.0067394967422764341; /* �ڶ�ƫ����ƽ�� */

//һ��imu����
//IMU 1624426287.22854877 0.000680678408277777028 -0.000532325421858261482 0.000656243798749856877 -0.605724081666666581 0.0254972899999999988 9.80681344416666789
template<typename _T> struct IMU {
	_T m_fTime;	//ʱ�䣬Ϊʲô�ø�������
	_T a[3];	//���ٶȣ�x,y,z���������϶��м��ٶ�
	_T w[3];	//���ٶȣ������������н��ٶ�
	_T Reserve;	//��ʱδ����
	_T Delta_Time;		//����Time��ȥ��һ��Time? Delta Time?
};

template<typename _T>struct GNSS {
	_T m_fTime;	//ʱ�䣬Ϊʲô�ø�������
	
	union {
		_T blh[3];
		struct {
			_T GPS[3];	//GPS��γ����
			_T m_fHeading;
			int m_bHeading_Valid;
		};		
	};

	_T std[3];
};

template<typename _T> struct utm {
	_T m_Pose[4 * 4];
	//union {
		struct {_T m_fEast, m_fNorth,m_fHeight;};	//�����Զ����Ա����߳�
		struct {_T x, y, z;};						//��ʱx,y,z����
	//};	
	int m_iZone;		//λ���ĸ���
	char m_bNorth;		//λ���ϻ��Ǳ�
};

template<typename _T>struct UTM_Param {
	_T UTM_a = (_T)6378137.0;         /* Semi-major axis of ellipsoid in meters  */
	_T UTM_f = (_T)(1 / 298.257223563); /* Flattening of ellipsoid                 */
	long UTM_Override = 0;            /* Zone override flag                      */

	/* Transverse_Mercator projection Parameters */
	_T TranMerc_Origin_Lat = (_T)0.0;     /* Latitude of origin in radians */
	_T TranMerc_Origin_Long = (_T)0.0;    /* Longitude of origin in radians */
	_T TranMerc_False_Northing = (_T)0.0; /* False northing in meters */
	_T TranMerc_False_Easting = (_T)0.0;  /* False easting in meters */
	_T TranMerc_Scale_Factor =(_T) 1.0;   /* Scale factor  */

	/* Ellipsoid Parameters, default to WGS 84  */
	_T TranMerc_a = (_T)6378137.0;              /* Semi-major axis of ellipsoid in meters */
	_T TranMerc_f = (_T)(1 / 298.257223563);      /* Flattening of ellipsoid  */
	_T TranMerc_es = (_T)0.0066943799901413800; /* Eccentricity (0.08181919084262188000) squared */
	_T TranMerc_ebs = (_T)0.0067394967565869;   /* Second Eccentricity squared */

	/* Isometeric to geodetic latitude parameters, default to WGS 84 */
	_T TranMerc_ap = (_T)6367449.1458008;
	_T TranMerc_bp = (_T)16038.508696861;
	_T TranMerc_cp = (_T)16.832613334334;
	_T TranMerc_dp = (_T)0.021984404273757;
	_T TranMerc_ep = (_T)3.1148371319283e-005;

	/* Maximum variance for easting and northing values for WGS 84. */
	_T TranMerc_Delta_Easting = 40000000.0;
	_T TranMerc_Delta_Northing = 40000000.0;
};

//blh <-> e
template<typename _T>void Coord_blh_2_e(_T fLatitude, _T fLongitude, _T H, _T RE, _T* px, _T* py, _T* pz);
template<typename _T>void Coord_e_2_blh(_T x, _T y, _T z, _T RE, _T* pfLatitude, _T* pfLongitude, _T* H);

//������ <->e
template<typename _T>void Coord_enu_2_e(_T xEast, _T yNorth, _T zUp, _T fLatitude_0, _T fLongitude_0, _T h_0, _T* px, _T* py, _T* pz);
template<typename _T>void Coord_e_2_enu(_T x, _T y, _T z, _T fLatitude_0, _T fLongitude_0, _T h_0, _T* pfxEast, _T* pfyNorth, _T* pfzUp);

//������ <->e
template<typename _T>void Coord_ned_2_e(_T xNorth, _T xEast, _T zUp, _T fLatitude_0, _T fLongitude_0, _T h_0, _T* px, _T* py, _T* pz);
template<typename _T>void Coord_e_2_enu(_T x, _T y, _T z, _T fLatitude_0, _T fLongitude_0, _T h_0, _T* pfxEast, _T* pfyNorth, _T* pfzUp);

//��γ��->utm
template<typename _T>int bLat_Long_2_utm(_T fLatitude, _T fLongitude, UTM_Param<_T>* poParam, _T* pfNorth, _T* pfEast, int* piZone, char* pcNorth);
template<typename _T>int bLat_Long_2_utm_rad(GNSS<_T> oGnss, _T fAntenna_x, _T fAntenna_y, _T fAntenna_Angle, _T Map_Origin[3], UTM_Param<_T>* poParam, utm<_T>* poUtm);
template<typename _T>int bLat_Long_2_utm_ang(GNSS<_T> oGnss, _T fAntenna_x, _T fAntenna_y, _T fAntenna_Angle, _T Map_Origin[3], UTM_Param<_T>* poParam, utm<_T>* poUtm);

//����������ת
template<typename _T>void Rotate_x(_T Source[3], _T phi, _T Dest[2]);
template<typename _T>void Rotate_y(_T Source[3], _T theta, _T Dest[3]);
template<typename _T>void Rotate_z(_T Source[3], _T psi, _T Dest[3]);

//��IMU����
template<typename _T>int bRead_IMU(const char* pcFile, IMU<_T>** ppIMU, int* piCount);

//����һ��imu����������pvq(R)ֵ
template<typename _T>void Update_pvq(IMU<_T> oImu, _T p[3], _T v[3], _T R[3 * 3], _T ba[3], _T bg[3], _T g[3], _T dt);

