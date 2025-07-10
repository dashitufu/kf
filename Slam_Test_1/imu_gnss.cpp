//#include "Reconstruct.
#include "stdio.h"
#include "imu_gnss.h"
#include "Common.h"
#include "Matrix.h"
#include "Image.h"

template<typename _T>struct IMU_Error {	//IMU的误差
	_T a_bias[3], w_bias[3], a_scale[3], w_scale[3];
};
template<typename _T>struct GNSS_Param{
	//初始欧拉角
	_T Init_rpy[3] = {  0.85421502, -2.03480295, 185.70235133};	//roll->X	pitch->Y	yaw->Z
	//初始位置，还是个角度，这就奇怪了
	//猜测： 纬度 经度 高程
	_T Init_Pos[3] = { 30.4447873701, 114.4718632047, 20.899 };
	//初始速度
	_T Init_v[3] = { 0,0,0 };

	//零偏
	_T Init_w_bias[3] = {0,0,0};	//deg/h
	_T Init_a_bias[3] = {0,0,0};	//mGal
	//比例银子
	_T Init_w_scale[3] = {0,0,0};	//ppm
	_T Init_a_scale[3] = {0,0,0};	//ppm

	// 初始位置标准差
	_T Init_Pos_std[3] = { 0.005, 0.004, 0.008 };
	//初始姿态标准差, 横滚、俯仰、航向角标准差
	_T Init_rpy_std[3] = { 0.003, 0.003, 0.023 };
	//初始速度标准差, 导航坐标系下北向、东向和垂向速度
	_T Init_v_std[3] = { 0.003, 0.004, 0.004 };


	//开始时间
	_T m_fStart_Time = 456300.00000000000,
		m_fEnd_Time = 459664.62061101903;

	// 读取IMU噪声参数,先不管
};
template<typename _T>struct Transaction {
	IMU<_T> m_oImu_Pre, m_oImu_Cur;
	IMU_Error<_T> m_oImu_Error;
	GNSS<_T> m_oGnss_Data;
	//以下先纬后经	[0]:纬度	[1]:经度
	_T pvapre_Pos[3], pvapre_v[3], pvapre_a[3];
	_T pvacur_Pos[3], pvacur_v[3], pvacur_a[3];

};

void SB_imu_gnss()
{
	bRead_IMU(NULL, (IMU<double>**)NULL, NULL);
	bRead_IMU(NULL, (IMU<float>**)NULL, NULL);

	bLat_Long_2_utm_rad(GNSS<double>{}, 0., 0., 0., (double*)NULL, (UTM_Param<double>*)NULL, (utm<double>*)NULL);
	bLat_Long_2_utm_rad(GNSS<float>{}, 0.f, 0.f, 0.f, (float*)NULL, (UTM_Param<float>*)NULL, (utm<float>*)NULL);

	bLat_Long_2_utm_ang(GNSS<double>{}, 0., 0., 0., (double*)NULL, (UTM_Param<double>*)NULL, (utm<double>*)NULL);
	bLat_Long_2_utm_ang(GNSS<float>{}, 0.f, 0.f, 0.f, (float*)NULL, (UTM_Param<float>*)NULL, (utm<float>*)NULL);

	Predict_pvq(IMU<double>{}, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, (double*)NULL, 0.);
	Predict_pvq(IMU<float>{}, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL, (float*)NULL,(float)0.f);

	return;
}

//int iRead_Line(FILE* pFile, char Line[], int iLine_Size = 256)
//{//从文件读入一行,返回一行大小
//	int iResult;
//	while (iResult=(int)fread((void*)Line, 1, 1, pFile))
//		if (Line[0] != '\r' && Line[0] != '\n' && Line[0] != '\t' && Line[0] != ' ')
//			break;
//	int i = 0;
//	while (iResult=(int)fread((void*)&Line[++i], 1, 1, pFile))
//	{
//		if (Line[i] == '\r' || Line[i] == '\n')
//			break;
//		if (i > iLine_Size)
//			return 0;
//	}
//	if(i<iLine_Size)
//		Line[i] = 0;
//	return iResult?i:0;
//}
template<typename _T>int bRead_IMU(const char* pcFile, IMU<_T>** ppIMU, int *piCount)
{//获取所有IMU结果
	int bRet = 1;
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
		return 0;
	const int LINE_SIZE = 256,
			iMax_Count = 100;
	char Line[LINE_SIZE];
	int iResult, iCount=0;
		
	IMU<_T>* pImu = (IMU<_T>*)pMalloc(iMax_Count * sizeof(IMU<_T>));
	if (!pImu)
	{
		bRet = 0;
		goto END;
	}

	while ( (iResult=iRead_Line(pFile,Line, LINE_SIZE)) && iResult<=LINE_SIZE && iCount<iMax_Count)
		if (bRead_IMU_Line(Line, &pImu[iCount]))
			iCount++;
	
	if (iResult && iResult < LINE_SIZE)
	{
		bRet = 1;
		if (iCount < iMax_Count)
			Shrink(pImu, iCount * sizeof(IMU<_T>));
		*piCount = iCount;
		*ppIMU = pImu;
	}		
END:
	fclose(pFile);
	if (!bRet && pImu)
		Free(pImu);	
	return bRet;
}
template<typename _T>int bRead_IMU_Line(const char* pcLine, IMU<_T>*poIMU)
{
	IMU<_T> oIMU;
	int iResult;
	if(sizeof(_T)==8)
		iResult = sscanf(pcLine, "IMU %lf %lf %lf %lf %lf %lf %lf", (double*)&oIMU.m_fTime,(double*) &oIMU.a[0], (double*)&oIMU.a[1], (double*)&oIMU.a[2],(double*) &oIMU.w[0], (double*)&oIMU.w[1], (double*)&oIMU.w[2]);
	else
		iResult = sscanf(pcLine, "IMU %f %f %f %f %f %f %f", (float*)&oIMU.m_fTime, (float*)&oIMU.a[0], (float*)&oIMU.a[1], (float*)&oIMU.a[2],(float*) &oIMU.w[0], (float*)&oIMU.w[1], (float*)&oIMU.w[2]);

	*poIMU = oIMU;
	return iResult == 7;
}
template<typename _T>void Disp_GNSS(GNSS<_T> oGnss)
{
	printf("t: %.10f\n", oGnss.m_fTime);
	printf("blh: %.10f\t%.10f\t%.10f\n", oGnss.blh[0], oGnss.blh[1], oGnss.blh[2]);
	printf("std: %.10f\t%.10f\t%.10f\n", oGnss.std[0], oGnss.std[1], oGnss.std[2]);
}
template<typename _T>void Disp_IMU(IMU<_T> oIMU)
{
	printf("t: %.10f Delta t:%.10f\n", oIMU.m_fTime,oIMU.Delta_Time);
	printf("w: %.10f\t%.10f\t%.10f\n", oIMU.w[0], oIMU.w[1], oIMU.w[2]);
	printf("a: %.10f\t%.10f\t%.10f\n", oIMU.a[0], oIMU.a[1], oIMU.a[2]);
}

template<typename _T>void Exp(_T w[3], _T theta, _T B[3 * 3])
{
	//e^(theta*w^) = cos(theta)* I + (1-cos(theta)) ωω' + sin(theta)ω^
	_T fCos_theta = cos(theta),
		fSin_theta = sin(theta);
	_T w_Hat[3 * 3], wwt[3 * 3];
	Hat(w,w_Hat);
	Matrix_Multiply(w_Hat,3,3, fSin_theta, w_Hat);

	Transpose_Multiply(w, 3, 1, wwt);
	Matrix_Multiply(wwt, 3, 3, 1.f - fCos_theta, wwt);

	Gen_I_Matrix(B, 3, 3);
	Matrix_Multiply(B, 3,3,fCos_theta, B);

	Matrix_Add(B, wwt,3, B);
	Matrix_Add(B, w_Hat, 3, B);

	return;
}
template<typename _T>void Get_R_t1(_T R_t0[3 * 3], _T w[3],_T R_t1[3*3])
{//想当然根据IMU读数的w值与t0时刻的R，计算R_t1
//假设delta t为时间片=1， w已经根据 delta t与一秒之间的比例进行调整
	_T w_Exp[3 * 3];
	//先算exp(w%);
	_T w1[4];
	//此处关键！先得将w分解为 4元旋转向量
	Rotation_Vector_3_2_4(w, w1);
	Exp(w1, (_T)w1[3], w_Exp);
	//Disp(w_Exp, 3, 3, "w_Exp");

	////以下用参考算法验算，可见，数值已经很接近
	//_T w_Hat[3 * 3], w_Exp_1[3*3];
	//Hat(w, w_Hat);
	//Exp_Ref(w_Hat, 3, w_Exp_1,1e-10);
	//Disp(w_Exp_1, 3, 3, "w_Exp_1");
	//Vector_Minus(w_Exp, w_Exp_1, 3 * 3, w_Exp_1);
	//printf("%e\n", fGet_Mod(w_Exp_1, 9));

	//R(t1) = R(to)*exp(w^delta_t)	
	Matrix_Multiply(R_t0, 3, 3, w_Exp,3, R_t1);

	return;
}

template<typename _T>void Rotation_Vector_4_2_w(_T Rotation_Vector_4[3], _T w[3])
{//未搞定
	_T J[3*3];
	Get_J_by_Rotation_Vector(Rotation_Vector_4, J);
	Matrix_Multiply(J, 3, 3, Rotation_Vector_4, 1, w);
	Matrix_Multiply(w, 1, 3, Rotation_Vector_4[3], w);
	return;
}
template<typename _T>void Rotation_Vector_3_2_w(_T Rotation_Vector_3[3], _T w[3])
{//未搞定
	_T Rotation_Vector_4[4];
	Rotation_Vector_3_2_4(Rotation_Vector_3, Rotation_Vector_4);

	//要对旋转向量求导
	Rotation_Vector_4_2_w(Rotation_Vector_4, w);
	return;
}
template<typename _T>void w_2_Rotation_Vector_3(_T w[3], _T Rotation_Vector_3[3])
{//未搞定
	_T w_4[4];
	union {
		_T J[3 * 3];
		_T J_Inv[3 * 3];
	};
	Rotation_Vector_3_2_4(w, w_4);
	Get_J_by_Rotation_Vector(w_4, J);
	int iResult;
	Get_Inv_Matrix_Row_Op_2(J, J_Inv, 3, &iResult);
	Matrix_Multiply(J_Inv, 3, 3, w, 1, Rotation_Vector_3);

	return;
}
template<typename _T>void Get_Derive_R_t(_T R[3*3], _T w[3], _T R_Deriv[3*3])
{//求旋转对时间t的求导 dR/dt = R * w^
	_T w_Hat[3 * 3];
	Hat(w, w_Hat);
	Matrix_Multiply(R, 3, 3, w_Hat, 3, R_Deriv);
	return;
}

template<typename _T>void Get_Deriv_T_t(_T T[4 * 4], _T w[3], _T v[3],_T T_Deriv[4*4])
{//用方法1： dT/dt =	Rw^	v
//						0	0
	union {
		_T R[3 * 3];
		_T R_Deriv[3 * 3];
	};
	Get_R_t(T, R);
	_T w_Hat[3 * 3];
	Hat(w, w_Hat);
	Matrix_Multiply(R, 3, 3, w_Hat, 3, R_Deriv);
	Copy_Matrix_Partial(R, 3, 3, T_Deriv, 4, 0, 0);
	T_Deriv[3] = v[0];
	T_Deriv[7] = v[1];
	T_Deriv[11] = v[2];
	T_Deriv[12] = T_Deriv[13] = T_Deriv[14] = T_Deriv[15]=0;
	return;
}
template<typename _T>void Exp_Phi_Delta_Phi_3(_T Phi[3], _T Delta_Phi[3], _T R[3*3])
{///计算 R = exp( (φ+delta_φ)^), 返回 φ+delta_φ对应的旋转矩阵
	_T Sum[3];
	Vector_Add(Phi, Delta_Phi, 3, Sum);
	Rotation_Vector_3_2_Matrix(Sum, R);
	return;
}

template<typename _T>void Get_Delta_R(_T Phi[3], _T Delta_Phi[3], _T Delta_R[3 * 3])
{//计算 Delta_R = exp( (Jl(φ)* delta_φ)^)
	_T Phi_4[4];
	_T Jl[3 * 3];
	_T V[3];

	Rotation_Vector_3_2_4(Phi, Phi_4);
	Get_J_by_Rotation_Vector(Phi_4, Jl);
	Matrix_Multiply(Jl, 3, 3, Delta_Phi, 1, V);
	Rotation_Vector_3_2_Matrix(V, Delta_R);
	return;
}

static void Test_1()
{
	typedef double _T;
	IMU<_T> oIMU;
	int iResult=bRead_IMU_Line("IMU 1624426287.22854877 0.000680678408277777028 -0.000532325421858261482 0.000656243798749856877 -0.605724081666666581 0.0254972899999999988 9.80681344416666789",  &oIMU);
	//Disp_IMU(oIMU);

	{	//第一个试验，根据t0时刻的旋转矩阵与角速度w，求t1时刻的旋转矩阵
		_T I[3 * 3],R_t1[3*3],R_Deriv[3*3];
		Gen_I_Matrix(I, 3, 3);
		Get_R_t1(I, oIMU.w, R_t1);

		Get_Derive_R_t(I, oIMU.w, R_Deriv);
		//Disp(R_Deriv, 3, 3, "R_Deriv");
	}

	{//验算罗德里格斯公式
		_T V[4] = { 1,2,3 }, V_Hat[3 * 3];
		_T R[3 * 3];
		//方法1，按定义展开
		Hat(V, V_Hat);
		Exp_Ref(V_Hat, 3, R);
		//Disp(R, 3, 3, "R");

		//罗德里格斯公式
		Rotation_Vector_3_2_4(V, V);
		Rotation_Vector_4_2_Matrix(V, R);
		//Disp(R, 3, 3, "R");
	}

	{//求dT/dt, 位姿对时间的导数
		_T V[4] = { 1,2,3 }, R[3 * 3], t[3] = { 10,20,30 },T[4*4];
		Rotation_Vector_3_2_4(V, V);
		Rotation_Vector_4_2_Matrix(V, R);
		Gen_Homo_Matrix(R, t, T);
		_T w[] = { 2,3,4 }, v[] = {100,200,300}, T_Deriv[4 * 4];
		Get_Deriv_T_t(T, w, v, T_Deriv);
		//Disp(T_Deriv, 4, 4, "T_Deriv");
	}

	{//验证BCH，表明BCH公式只是近似
		_T RV_3[3], Delta_RV_3[3], RV1_3[3], R1[3*3];
		Get_Random_Norm_Vec(RV_3, 3);
		Get_Random_Norm_Vec(Delta_RV_3, 3);
		Vector_Multiply(RV_3, 3, 1.5, RV_3);
		Vector_Multiply(Delta_RV_3, 3, 0.1, Delta_RV_3);

		//先算exp( (A+B)^)
		Vector_Add(RV_3, Delta_RV_3, 3, RV1_3);
		Rotation_Vector_3_2_Matrix(RV1_3, R1);
		Disp(R1, 3, 3, "R_1");
		Disp(RV1_3, 1, 3, "RV1_3");

		////一步到位看看
		//Exp_Phi_Delta_Phi_3(RV_3, Delta_RV_3, R1);
		//Disp(R1, 3, 3, "R_1");

		//先将delta_R_V进行 exp( (Jl(R_V)*delta_R_V)^)
		_T RV_4[4], Delta_RV_4[4];
		Rotation_Vector_3_2_4(RV_3, RV_4);
		Rotation_Vector_3_2_4(Delta_RV_3, Delta_RV_4);

		//exp( (Jl(φ)*Δφ)^)
		_T J[3 * 3],Temp[4*4];
		Get_J_by_Rotation_Vector(RV_4, J);
		Matrix_Multiply(J, 3, 3, Delta_RV_3,1, Temp);
		Rotation_Vector_3_2_Matrix(Temp, Temp);	//Delta_R
		//Disp(Temp, 3, 3, "Delta_R");

		Rotation_Vector_3_2_Matrix(RV_3, R1);
		Matrix_Multiply(Temp, 3, 3, R1, 3, R1);
		Disp(R1, 3, 3, "R_1");
		Rotation_Matrix_2_Vector(R1, Temp);
		Rotation_Vector_4_2_3(Temp, Temp);
		Disp(Temp, 1, 3, "RV1_3");

		////用Get_Delta_R
		//Rotation_Vector_3_2_Matrix(RV_3, R1);
		//Get_Delta_R(RV_3, Delta_RV_3, Temp);
		//Disp(Temp, 3, 3, "Delta_R");
		//Matrix_Multiply(Temp, 3, 3, R1, 3, R1);
		//Disp(R1, 3, 3, "R_1");
	}

	{//用BCH方法再试更新 T = Delta_T * T
		_T RV_3[3], Delta_RV_3[3];
		_T t[3] = { 100,200,300 }, Delta_t[3] = { 0.1,0.2,0.3 };

		Get_Random_Norm_Vec(RV_3, 3);
		Get_Random_Norm_Vec(Delta_RV_3, 3);	//充当ksi
		Vector_Multiply(RV_3, 3, 1.5, RV_3);
		Vector_Multiply(Delta_RV_3, 3, 0.1, Delta_RV_3);

		//先算RV + Delta_RV对应什么新的位姿，显然se3上直接相加与其他方法差太远
		_T Ksi[6], T[4 * 4];
		Vector_Add(RV_3, Delta_RV_3, 3, &Ksi[3]);
		Vector_Add(t, Delta_t, 3, Ksi);
		Gen_Homo_Matrix_1(&Ksi[3], Ksi, T);
		Disp(T, 4, 4, "T");

		//算Delta_Ksi, 对应Delta_T, 再加到原来的T
		Gen_Homo_Matrix_1(RV_3, t, T);
		_T Delta_T[4 * 4], T1[4 * 4];
		Gen_Homo_Matrix_1(Delta_RV_3, Delta_t, Delta_T);
		Matrix_Multiply(Delta_T, 4, 4, T, 4, T1);
		Disp(T1, 4, 4, "T");

		//用se3_2_SE3的防范转换Delta_Ksi
		memcpy(Ksi, Delta_t, 3 * sizeof(_T));
		memcpy(&Ksi[3], Delta_RV_3, 3 * sizeof(_T));
		se3_2_SE3(Ksi, Delta_T);
		Matrix_Multiply(Delta_T, 4, 4, T, 4, T1);
		Disp(T1, 4, 4, "T");
	}

	return;
}
template<typename _T>void Init_GNSS_Param(GNSS_Param<_T>* poParam)
{
#define FACTOR PI/180.f

	//更新Pos，此处存疑，Pos如果表示原始位置，为什么和弧度有关？
	for (int i = 0; i < 2; i++)
		poParam->Init_Pos[i] *= FACTOR;
	for (int i = 0; i < 2; i++)
	{
		poParam->Init_rpy[i] = poParam->Init_rpy[i] * FACTOR;
		poParam->Init_rpy_std[i]/=PI;
		poParam->Init_w_bias[i] /= PI * 2;
	}

	return;
}

template<typename _T>int bRead_GNSS_File(const char* pcFile,GNSS<_T> **ppGnss, _T tStart_Time, int *piCount)
{
	int i,iCount, bRet = 1;
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
		return 0;
	if (*piCount > 0)
		iCount = *piCount;
	else
		iCount = 1000000;
	GNSS<_T>* pGnss = (GNSS<_T>*)pMalloc(iCount * sizeof(GNSS<_T>));
	GNSS<_T> oGnss = {};
	for (i = 0;; i++)
	{
		_T fPre_Time = oGnss.m_fTime;
		int iResult = fscanf(pFile, "%lf %lf %lf %lf %lf %lf %lf\r\n", &oGnss.m_fTime, &oGnss.blh[0], &oGnss.blh[1], &oGnss.blh[2], &oGnss.std[0], &oGnss.std[1], &oGnss.std[2]);
		//oGnss.blh[0] *= (PI / 180.f);
		//oGnss.blh[1] *= (PI / 180.f);
		//Disp_GNSS(oGnss);
		if (oGnss.m_fTime >= tStart_Time)
			break;
		if (iResult < 7)
		{
			bRet = 0;
			goto END;
		}
	}

	pGnss[0] = oGnss;
	//Disp_IMU(oImu);
	for (i = 1; i < iCount; i++)
	{
		int iResult = fscanf(pFile, "%lf %lf %lf %lf %lf %lf %lf\r\n", &oGnss.m_fTime, &oGnss.blh[0], &oGnss.blh[1], &oGnss.blh[2], &oGnss.std[0], &oGnss.std[1], &oGnss.std[2]);
		oGnss.blh[0] *= (PI / 180.f);
		oGnss.blh[1] *= (PI / 180.f);
		pGnss[i] = oGnss;
		if (iResult == -1)
			break;
		else if (iResult < 7)
		{
			bRet = 0;
			break;
		}
	}
END:
	iCount = i;
	if (!bRet)
		Free(pGnss), * ppGnss = NULL, * piCount = 0;
	else
	{
		*ppGnss = pGnss, * piCount = iCount;
		Shrink(pGnss, iCount * sizeof(IMU<_T>));
	}
	return bRet;
}
template<typename _T>int bRead_Imu_File(const char* pcFile, IMU<_T> **ppImu, _T tStart_Time, int *piCount)
{
	int i,iCount, bRet = 1;
	FILE* pFile = fopen(pcFile, "rb");
	if (!pFile)
		return 0;
	if (*piCount > 0)
		iCount = *piCount;
	else
		iCount = 1000000;

	IMU<_T>* pImu = (IMU<_T>*)pMalloc(iCount * sizeof(IMU<_T>));
	//先移动到Start_Time
	IMU<_T> oImu = {};
	for (i = 0;; i++)
	{
		_T fPre_Time = oImu.m_fTime;
		int iResult = fscanf(pFile, "%lf %lf %lf %lf %lf %lf %lf\r\n", &oImu.m_fTime, &oImu.w[0], &oImu.w[1], &oImu.w[2], &oImu.a[0], &oImu.a[1], &oImu.a[2]);
		oImu.Delta_Time = oImu.m_fTime - fPre_Time;
		if (oImu.m_fTime >= tStart_Time)
			break;
		if (iResult < 7)
		{
			bRet = 0;
			goto END;
		}
	}
	pImu[0] = oImu;
	//Disp_IMU(oImu);
	for (i = 1; i < iCount; i++)
	{
		int iResult=fscanf(pFile, "%lf %lf %lf %lf %lf %lf %lf\r\n",&oImu.m_fTime, &oImu.a[0], &oImu.a[1], &oImu.a[2], &oImu.w[0], &oImu.w[1], &oImu.w[2]);
		oImu.Delta_Time = oImu.m_fTime - pImu[i - 1].m_fTime;
		pImu[i] = oImu;
		if (iResult == -1)
			break;
		else if (iResult < 7)
		{
			bRet = 0;
			break;
		}
	}
END:
	iCount = i;
	if (!bRet)
		Free(pImu), * ppImu = NULL, * piCount = 0;
	else
	{
		*ppImu = pImu, * piCount = iCount;
		Shrink(pImu, iCount * sizeof(IMU<_T>));
	}
		
	return bRet;
}

template<typename _T>void Imu_Compensate(Transaction<_T>* poTrans, IMU<_T>* poImu)
{//补偿什么？
	//imu.dtheta -= imuerror_.gyrbias * imu.dt;
	//imu.dvel -= imuerror_.accbias * imu.dt;
	int i;
	//Disp_IMU(*poImu);
	

	//感觉以下有点绕，理论上应该相当于 w = w_scale*w + w_bias
	//w = w- bias * dt;
	//scale = 1 - scale;
	//w = w / scale = (w - bias * dt) / (1 - scale);

	for (i = 0; i < 3; i++)
	{
		poImu->w[i] = (poImu->w[i] - poTrans->m_oImu_Error.w_bias[i] * poImu->Delta_Time) / (1 + poTrans->m_oImu_Error.w_scale[i]);
		poImu->a[i] = (poImu->a[i] - poTrans->m_oImu_Error.a_bias[i] * poImu->Delta_Time) / (1 + poTrans->m_oImu_Error.a_scale[i]);
	}

	//_T w_scale[3]={1,2,3}, a_scale[3];
	//for (i = 0; i < 3; i++)
	//{
	//	poImu->w[i] -= poTrans->m_oImu_Error.w_bias[i] * poImu->Delta_Time;
	//	poImu->a[i] -= poTrans->m_oImu_Error.a_bias[i] * poImu->Delta_Time;
	//	//w_scale[i] = 1 - poTrans->m_oImu_Error.w_scale[i];
	//	a_scale[i] = 1 - poTrans->m_oImu_Error.a_scale[i];

	//	poImu->w[i] *= 1.f / w_scale[i];
	//	poImu->a[i] *= 1.f / a_scale[i];
	//}	
	//Disp_IMU(*poImu);

	return;
}
template<typename _T>void Add_Gnss_Data(Transaction<_T>* poTrans, GNSS<_T> oGnss)
{
	poTrans->m_oGnss_Data = oGnss;
	return;
}
template<typename _T>void Add_Imu_Data(Transaction<_T>* poTrans, IMU<_T>* poImu, int bCompensate=0)
{
	poTrans->m_oImu_Pre = poTrans->m_oImu_Cur;
	poTrans->m_oImu_Cur = *poImu;
	if(bCompensate)
		Imu_Compensate(poTrans,&poTrans->m_oImu_Cur);
	return;
}
template<typename _T>void Get_Earth_r(_T Pos[],_T r[2])
{//在Pos处算出个子午圈半径和卯酉圈半径
//注意，这只是得到两个半径，并没有得到其他位置信息

	//由于推导麻烦，暂时用结论，日后再补证明
	_T fSin = sin(Pos[0]);
	//1- e^2 * sin^2(经度)
	_T fPart_1 = 1.f - WGS84_E1*fSin * fSin;
	_T fSqrt_Part_1 = sqrt(fPart_1);
	//卯酉圈半径 = 长半轴a/ [1- e^2 * sin^2(经度) ]
	r[1] = WGS84_RA / fSqrt_Part_1;
	//子午圈半径 = 长半轴a * (1- e^2) / √[1- e^2 * sin^2(经度)] * [1- e^2 * sin^2(经度)]
	r[0] = WGS84_RA * (1 - WGS84_E1) / (fSqrt_Part_1 * fPart_1);
	//Disp(r, 1, 2, "r");

	return;
}

void r_Follow_Test_1()
{//牵连试验，画一个地球绕太阳转，月球绕地球转试验。用牵连位移搞
	typedef float _T;
	//先开一张图
	char File[256];
	Image oImage;
	const int iWidth = 1000, iHeight = 1000;
	Init_Image(&oImage, iWidth, iHeight, Image::IMAGE_TYPE_BMP, 8);

	int x_Screen,y_Screen;  //屏幕坐标
	//K系参数，太阳
	_T o[] = { 0, 0 };		//K系为太阳坐标系，o为日心
	_T r = 200;				//K系半径为200
	_T c = 100;				//K系周期为100秒
	_T a = (_T)(2 * PI / c);		//周期函数cos(at)的a
	_T x, y;

	//K1系
	_T o1[2];
	_T r1 = 50;          //K1系的半径，地心到月心距离
	_T c1 = 10;          //K1系周期
	_T a1 = (_T)(2 * PI / c1);
	_T x1, y1;

	for (int t = 0; t < c*2; t++)
	{
		Set_Color(oImage);  //涂黑

		//画处日心位置
		Rect_2_Screen_Coordinate(o[0], o[1], &x_Screen, &y_Screen,iWidth,iHeight);
		Draw_Point(oImage, x_Screen, y_Screen);

		//画出地球，o1就是地心
		//K1系o1位置 o1 = o1(t)
		o1[0] = (_T)(r * cos(a * t));
		o1[1] = (_T)(r * sin(a * t));
		Rect_2_Screen_Coordinate(o1[0], o1[1], &x_Screen, &y_Screen,iWidth,iHeight);
		Draw_Point(oImage, x_Screen, y_Screen);

		//将月心视为质点, r(t)为t时刻，月心相对于日心位移
		//r'(t)为月心相对于地心位移。r0(t)为地心相对于日心位置
		//r(t) = r'(t) + r0(t)
		x1 = (_T)(r1 * cos(a1 * t));
		y1 =(_T)( r1 * sin(a1 * t));
		x = x1 + o1[0];
		y = y1 + o1[1];
		Rect_2_Screen_Coordinate(x, y, &x_Screen, &y_Screen, iWidth, iHeight);
		Draw_Point(oImage, x_Screen, y_Screen);

		sprintf(File, "c:\\tmp\\temp\\%03d.bmp", t);
		bSave_Image(File, oImage);        
	}

	Free_Image(&oImage);
	return;
}

void v_Follow_Test()
{//牵连速度实验	v(t) = v'(t) + u(t)
	//其中，v'(t)为K1系相对于o1的速度矢量。u(t)为k1系原点o1相对于K系o的速率
	//v(r)就是最终质点相对于K系原点o的速度矢量
	typedef float _T;
	int t;
	//K系参数，太阳
	_T o[] = { 0, 0 };		//K系为太阳坐标系，o为日心
	_T r = 200;				//K系半径为200
	_T c = 100;				//K系周期为100秒
	_T a = (_T)(2 * PI / c);		//周期函数cos(at)的a
	_T x, y,v[2];

	//K1系
	_T r1 = 50;          //K1系的半径，地心到月心距离
	_T c1 = 10;          //K1系周期
	_T a1 = (_T)(2 * PI / c1);
	_T v1[2];

	_T u[2];

	{//看看瞬间速度delta v与理论速度之间的差别
		//从以下实验可以看出，误差还是很大的，感觉用中间速度可能会准一些
		_T Pre_Pos[2];
		Pre_Pos[0] =(_T)( r * cos(a * 0));
		Pre_Pos[1] =(_T)( r * sin(a * 0));
		for(t=1;t<c;t++)
		{
			//算delta_r/t
			x = (_T)(r * cos(a * t));
			y =(_T)( r * sin(a * t));
			_T Temp[2] = { x - Pre_Pos[0],y - Pre_Pos[1] };
			printf("v = delta Pos/t (%f,%f)/s ", Temp[0] / 1.f, Temp[1] / 1.f);
			Pre_Pos[0] = x, Pre_Pos[1] = y;

			//求dr/dt，理论速度
			//dx/dt = -r*sin(at)*a
			v[0] =(_T)( -r * sin(a * t) * a);
			//dy/dt = r*cos(at)*a
			v[1] = (_T)(r * cos(a * t) * a);
			printf(" dr/dt=(%f,%f) ", v[0], v[1]);
			printf("|v|:%f %f\n", fGet_Mod(Temp, 2), fGet_Mod(v, 2));
		}
	}
	
	{//算牵连速度 v(t) = v'(t) + u(t)
		for (t = 0; t < c; t++)
		{
			//x1= r1 * cos(a1 * t) y1 = r1 * sin(a1 * t);
			//dx1/dt = -r1*sin(a1*t)*a1
			v1[0] = (_T)(-r1 * sin(a1 * t) * a1);
			//dy1/dt = r1*cos(a1*r)*a1
			v1[1] = (_T)(r1 * cos(a1 * t) * a1);

			//u(t)就是dr0/dt
			//r0x(t) = r * cos(at)	r0y =  r * sin(at);
			//u[0] = dr0x/dt = -r*sin(at)*a
			u[0] = (_T)(-r * sin(a * t) * a);
			//u[1] = dr0y/dt = r*cos(at)*a
			u[1] = (_T)(r * cos(a*t) * a);

			v[0] = v1[0] + u[0];
			v[1] = v1[1] + u[1];
			printf("v = (%f,%f)/s\n", v[0], v[1]);
		}		
	}
	return;
}
void a_Follow_Test_1()
{//牵连加速度实验 a = a' + ar
	typedef float _T;
	int t;
	_T v1[2],u[2],acce_1[2],ar[2],acce[2];

	_T o[] = { 0, 0 };		//K系为太阳坐标系，o为日心
	_T r = 200;				//K系半径为200
	_T c = 100;				//K系周期为100秒
	_T a = (_T)(2 * PI / c);		//周期函数cos(at)的a

	//K1系
	_T r1 = 50;          //K1系的半径，地心到月心距离
	_T c1 = 10;          //K1系周期
	_T a1 = (_T)(2 * PI / c1);

	for (t = 0; t < c; t++)
	{
		//dx1/dt = -r1*sin(a1*t)*a1
		v1[0] = (_T)(-r1 * sin(a1 * t) * a1);
		//dy1/dt = r1*cos(a1*r)*a1
		v1[1] =(_T)( r1 * cos(a1 * t) * a1);

		//a1 = dv1/dt
		acce_1[0] =(_T)( -r1 * a1 * cos(a1 * t) * a1);
		acce_1[1] = (_T)(-r1 * a1 * sin(a1 * t) * a1);
		
		//u(t)就是dr0/dt
		//r0x(t) = r * cos(at)	r0y =  r * sin(at);
		//u[0] = dr0x/dt = -r*sin(at)*a
		u[0] = (_T)(-r * sin(a * t) * a);
		//u[1] = dr0y/dt = r*cos(at)*a
		u[1] =(_T)( r * cos(a*t) * a);
		
		//ar = du/dt
		ar[0] =(_T)(- r * a * cos(a * t) * a);
		ar[1] =(_T)( -r * a * sin(a * t) * a);

		acce[0] = acce_1[0] + ar[0];
		acce[1] = acce_1[1] + ar[1];
		printf("a: (%f,%f)/t^2\n", acce[0], acce[1]);
	}
}
template<typename _T>_T fGet_g(_T Pos[3])
{//通过经纬高程计算重力加速度，这个没啥好算的，应该是个国际标准
	_T sinphi = sin(Pos[0]);
	_T sin2   = sinphi * sinphi;
	_T sin4   = sin2 * sin2;

	// normal gravity at equator, 赤道处正常重力
	_T gamma_a = 9.7803267715;
	// series expansion of normal gravity at given latitude, 给定纬度处正常重力的级数展开
	_T gamma_0 = gamma_a * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin4 + 0.0000001262 * sin2 * sin4 +
		0.0000000007 * sin4 * sin4);
	// changes of normal gravity with height, 正常重力随高度变化
	_T gamma = gamma_0 - (3.0877e-6 - 4.3e-9 * sin2) * Pos[2] + 0.72e-12 * Pos[2] * Pos[2];

	return gamma;
}

template<typename _T>void blh_2_Q(_T fLongitude, _T fLatitude, _T H, _T Q[4])
{//blh系转换为4元数，fLongitude:经度 fLatitude:维度 H:高程
	//Q的排列是 w, x, y, z	w尚未知意义
	_T lamda = fLongitude, phi = fLatitude;
	_T fPart_1 = -PI / 4.f - phi / 2.f;
	_T fPart_2 = lamda / 2.f;
	_T sin_Part_1 = sin(fPart_1),
		cos_Part_1 = cos(fPart_1),
		sin_Part_2 = sin(fPart_2),
		cos_Part_2 = cos(fPart_2);
	Q[0] = cos_Part_1 * cos_Part_2;
	Q[1] = -sin_Part_1 * sin_Part_2;
	Q[2] = sin_Part_1 * cos_Part_2;
	Q[3] = cos_Part_1 * sin_Part_2;
	//Disp(Q, 1, 4, "Q");
	_T R[3 * 3];
	Quaternion_2_Rotation_Matrix(Q, R);
	//Disp(R, 3, 3, "R");
	return;
}

template<typename _T>void blh_2_R(_T fLongitude, _T fLatitude, _T H, _T R[3*3])
{///blh系转换为旋转矩阵，fLongitude:经度 fLatitude:维度 H:高程
	//此处不用用z,y的旋转合成，第一次绕z是一样，但是第二次绕的不是y
	_T lamda = fLongitude, phi = fLatitude;
	_T sin_phi = sin(phi), cos_phi = cos(phi),
		sin_lamda = sin(lamda), cos_lamda = cos(lamda);
	R[0] = -sin_phi * cos_lamda;		//与武大不同
	R[1] = -sin_lamda;
	R[2] = -cos_phi * cos_lamda;

	R[3] = -sin_phi * sin_lamda;
	R[4] = cos_lamda;
	R[5] = -cos_phi * sin_lamda;

	R[6] = cos_phi;
	R[7] = 0;
	R[8] = -sin_phi;
	//Disp(R, 3, 3, "R");

	_T Q[4];
	Rotation_Matrix_2_Quaternion(R, Q);
	//Disp(Q, 1, 4, "Q");
	return;
}

template<typename _T>void Blh2Xyz(_T &x, _T &y, _T &z)
{//一个网上参考层序, x：经度 y:纬度 z:高程
	const _T epsilon = 0.000000000000001;
	const _T pi = 3.14159265358979323846;
	const _T d2r = pi / 180;
	const _T r2d = 180 / pi;

	const _T a = 6378137.0;		//椭球长半轴
	const _T f_inverse = 298.257223563;			//扁率倒数
	const _T b = a - a / f_inverse;
	//const _T b = 6356752.314245;			//椭球短半轴

	const _T e = sqrt(a * a - b * b) / a;

	//_T L = x * d2r;		//x: Longitude
	//_T B = y * d2r;		//y: Latitude
	_T L = x;
	_T B = y;
	_T H = z;

	_T N = a / sqrt(1 - e * e * sin(B) * sin(B));
	x = (N + H) * cos(B) * cos(L);
	y = (N + H) * cos(B) * sin(L);
	z = (N * (1 - e * e) + H) * sin(B);
}

template<typename _T>void Coord_blh_2_e(_T fLatitude, _T fLongitude, _T H, _T RE,_T* px, _T* py, _T* pz)
{//blh系转换为e系	B:纬度 L:经度 H:高程	Re: 子午圈半径 
	_T RE_Plus_H = RE + H;
	//x =	(RE + H) * cos(经度)*cos(纬度)
	_T sin_B = sin(fLatitude), cos_B = cos(fLatitude), cos_L = cos(fLongitude), sin_L = sin(fLongitude);

	*px = RE_Plus_H * cos_B * cos_L;

	//y = (RE + H) * cos(经度)*sin(纬度)
	*py = RE_Plus_H * cos_B *sin_L;

	//z	[RE*(1- e^2) + H]*sin(纬度)
	*pz = (RE * (1 - WGS84_E1) + H) * sin_B;
	return;
}
template<typename _T>void Coord_e_2_blh(_T x, _T y, _T z, _T RE, _T *pfLatitude, _T *pfLongitude, _T *H)
{//
	_T p = x * x + y * y;
	*pfLongitude = atan2(y, x);

	_T calB = atan2(z, sqrt(x * x + y * y)); 
	int counter = 0;
	_T eps = 1e-10;
	_T curB = 0;
	_T N;

	while (abs(curB - calB)> eps  && counter < 25)
	{//此处慢慢搞搞，到底是泰勒展开还是牛顿法什么的
		curB = calB;
		N = WGS84_RA / sqrt(1 - WGS84_E1 * sin(curB) * sin(curB));
		calB = atan2(z + N * WGS84_E1 * sin(curB), sqrt(x * x + y * y));
		counter++;	
	} 	   

	*pfLatitude = atan2(y, x);
	*pfLongitude = curB;
	*H = z / sin(curB) - N * (1 - WGS84_E1);	
}
template<typename _T>void Coord_enu_2_e(_T xEast, _T yNorth, _T zUp, _T fLatitude_0, _T fLongitude_0, _T h_0, _T *px, _T *py, _T *pz)
{//东北天到e系转换，给定一个东北天坐标(xEast, yNorth, zUp), 东北天原点的(纬，经，高)，求其在e系中的(x,y,z)坐标
 //_T a = 6378137;
 //_T b = 6356752.3142;
	_T f = (WGS84_RA - WGS84_RB) / WGS84_RA;	//这个经度能几乎全对
	_T e_sq = f * (2 - f);
	//_T pi = 3.14159265359;
	//_T lamb = pi / 180 * (lat0);//角度转弧度
	//_T phi = pi / 180 * (lon0);

	_T lamb = fLatitude_0;
	_T phi = fLongitude_0;

	_T s = sin(lamb);
	_T N = WGS84_RA / sqrt(1 - e_sq * s * s);

	_T sin_lambda = sin(lamb);
	_T cos_lambda = cos(lamb);
	_T sin_phi = sin(phi);
	_T cos_phi = cos(phi);

	_T x0 = (h_0 + N) * cos_lambda * cos_phi;
	_T y0 = (h_0 + N) * cos_lambda * sin_phi;
	_T z0 = (h_0 + (1 - e_sq) * N) * sin_lambda;

	_T t = cos_lambda * zUp - sin_lambda * yNorth;
	_T zd = sin_lambda * zUp + cos_lambda * yNorth;
	_T xd = cos_phi * t - sin_phi * xEast;
	_T yd = sin_phi * t + cos_phi * xEast;

	*px = xd + x0;
	*py = yd + y0;
	*pz = zd + z0;	
}
template<typename _T>void Coord_e_2_enu(_T x, _T y, _T z, _T fLatitude_0, _T fLongitude_0, _T h_0, _T* pfxEast, _T* pfyNorth, _T* pfzUp)
{//e系转换为d东北天坐标系， 给定一个e系坐标(x,y,z), 一个东北天原点的纬经高，求其东北天坐标
	//const _T a = 6378137;
	//const _T b = 6356752.3142;
	const _T f = (WGS84_RA - WGS84_RB) / WGS84_RA;
	const _T e_sq = f * (2 - f);

	//const _T lamb = lat0 / 180.0 * CV_PI;
	//const _T phi = lon0 / 180.0 * CV_PI;

	_T lamb = fLatitude_0;
	_T phi = fLongitude_0;

	const _T s = sin(lamb);
	const _T N = WGS84_RA / sqrt(1 - e_sq * s * s);

	const _T sin_lambda = sin(lamb);
	const _T cos_lambda = cos(lamb);
	const _T sin_phi = sin(phi);
	const _T cos_phi = cos(phi);

	const _T x0 = (h_0 + N) * cos_lambda * cos_phi;
	const _T y0 = (h_0 + N) * cos_lambda * sin_phi;
	const _T z0 = (h_0 + (1 - e_sq) * N) * sin_lambda;

	const _T xd = x - x0;
	const _T yd = y - y0;
	const _T zd = z - z0;

	const _T t = -cos_phi * xd - sin_phi * yd;

	*pfxEast = -sin_phi * xd + cos_phi * yd;
	*pfyNorth = t * sin_lambda + cos_lambda * zd;
	*pfzUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
}
template<typename _T>void Coord_ned_2_e(_T xNorth, _T xEast, _T zUp,_T fLatitude_0, _T fLongitude_0, _T h_0, _T *px, _T *py, _T *pz )
{//北东地->e系	轴		x	y	z		右手系
 //			方向	北	东	地
 //先将北东地转换为东北天
	Coord_enu_2_e(xNorth, xEast, -zUp, fLatitude_0, fLongitude_0, h_0, px, py, pz);
}

template<typename _T>void Coord_e_2_ned(_T x, _T y, _T z, _T fLatitude_0, _T fLongitude_0, _T h_0, _T* pfyNorth,_T* pfxEast, _T* pfzDown)
{
	Coord_e_2_enu(x, y, z, fLatitude_0, fLongitude_0, h_0, pfyNorth, pfxEast, pfzDown);
	*pfzDown = -*pfzDown;
}

template<typename _T>void Xyz2Blh(_T &x, _T &y, _T &z)
{//网上参考程序
	const _T epsilon = 0.000000000000001;
	const _T pi = 3.14159265358979323846;
	const _T d2r = pi / 180;
	const _T r2d = 180 / pi;

	const _T a = 6378137.0;		//椭球长半轴
	const _T f_inverse = 298.257223563;			//扁率倒数
	const _T b = a - a / f_inverse;
	const double e = sqrt(a * a - b * b) / a;

	_T tmpX =  x;
	_T temY = y ;
	_T temZ = z;

	_T curB = 0;
	_T N = 0; 
	_T calB = atan2(temZ, sqrt(tmpX * tmpX + temY * temY)); 

	int counter = 0;
	while (abs(curB - calB) * r2d > epsilon  && counter < 25)
	{
		curB = calB;
		N = a / sqrt(1 - e * e * sin(curB) * sin(curB));
		calB = atan2(temZ + N * e * e * sin(curB), sqrt(tmpX * tmpX + temY * temY));
		counter++;	
	} 	   

	//x = atan2(temY, tmpX) * r2d;
	//y = curB * r2d;
	x = atan2(temY, tmpX);
	y = curB;
	z = temZ / sin(curB) - N * (1 - e * e);	

}

template<typename _T>void Update_v(Transaction<_T>* poTrans, IMU<_T> oPre, IMU<_T>* poCur)
{
	_T r[2];
	Get_Earth_r(poTrans->pvapre_Pos, r);
	//Disp(r, 1, 2, "R");
	//临时代码
	_T x, y, z, * pPos = poTrans->pvapre_Pos;	// , R[3 * 3];	//e系的x,y,z
	Coord_blh_2_e(poTrans->pvapre_Pos[0], poTrans->pvapre_Pos[1], poTrans->pvapre_Pos[2],
		r[1], &x, &y, &z);
	/*printf("%f %f %f\n", x, y, z);
	x = poTrans->pvapre_Pos[1], y = poTrans->pvapre_Pos[0], z=poTrans->pvapre_Pos[2];
	Blh2Xyz(x, y, z);
	printf("%f %f %f\n", x, y, z);*/

	////Xyz2Blh(x, y, z);
	//printf("org:%f %f %f\n", poTrans->pvapre_Pos[0], poTrans->pvapre_Pos[1], poTrans->pvapre_Pos[2]);
	//memset(poTrans->pvapre_Pos, 0, 3 * sizeof(_T));
	//Coord_e_2_blh(x, y, z,r[1], &poTrans->pvapre_Pos[1],  &poTrans->pvapre_Pos[0], &poTrans->pvapre_Pos[2]);
	//printf("%f %f %f\n", poTrans->pvapre_Pos[0],poTrans->pvapre_Pos[1],poTrans->pvapre_Pos[2]);
	
	////东北天系转e系
	//printf("org: %f %f %f\n", x, y, z);
	//Coord_enu_2_e((_T)0.f,(_T) 0.f, (_T)0.f, pPos[0], pPos[1], pPos[2], &x, &y, &z);
	//printf(" %f %f %f\n", x, y, z);

	////e系转换为东北天系
	//Coord_e_2_enu(x, y, z, pPos[0], pPos[1], pPos[2], &x, &y, &z);
	//printf(" %f %f %f\n", x, y, z);

	//北东地<->e系
	//Coord_ned_2_e((_T)10.f, (_T)5.f, (_T)1.f, pPos[0], pPos[1], pPos[2], &x, &y, &z);
	//Coord_e_2_ned(x, y, z, pPos[0], pPos[1], pPos[2], &x, &y, &z);
	
	//WGS84_WIE 地球自转角速度
	//又，按照地球自传得性质，精度变，纬度不变
	
	//如何理解？将角速度w分解为 w*θ，先将w忽略，θ=WGS84_WIE，感觉是wie * r， 而r用了单位园
	_T wie_n[3] = { WGS84_WIE * cos(poTrans->pvapre_Pos[0]),0,-WGS84_WIE * sin(poTrans->pvapre_Pos[0]) };
	//Disp(wie_n, 1, 3, "win_n");
	_T wen_n[3] = {	 poTrans->pvapre_v[1] / (r[1] + poTrans->pvapre_Pos[2]),
		-poTrans->pvapre_v[0] / (r[0] +  poTrans->pvapre_Pos[2]),
		-poTrans->pvapre_v[1] * tan(poTrans->pvapre_Pos[0]) / (r[1] + poTrans->pvapre_Pos[2])	};
	//Disp(wen_n, 1, 3, "wen_n");
	_T gravity = fGet_g(poTrans->pvapre_Pos);
	return;
}

void GNSS_Test()
{
	typedef double _T;
	GNSS_Param<_T> oParam;
	IMU<_T>* pImu,oImu_Cur;
	GNSS<_T>* pGnss;
	Transaction<_T> oTrans = {};
	int i,j,iResult,iImu_Count = 200,iGnss_Count=200;

	Init_GNSS_Param(&oParam);
	memcpy(oTrans.pvapre_Pos, oParam.Init_Pos, 3 * sizeof(_T));
	
	//_T r[2];
	//Get_Earth_r(oParam.Init_Pos,(_T*)r);

	//Disp(oParam.Init_Pos, 3, 1, "Pos");
	iResult=bRead_Imu_File("C:\\Users\\admin\\Desktop\\KF-GINS-main\\dataset\\Leador-A15.txt",&pImu,oParam.m_fStart_Time,&iImu_Count);
	iResult = bRead_GNSS_File("C:\\Users\\admin\\Desktop\\KF-GINS-main\\dataset\\GNSS-RTK.txt",&pGnss,oParam.m_fStart_Time,&iGnss_Count);

	Disp_IMU(pImu[0]);
	oImu_Cur = pImu[0];
	GNSS<_T> oGnss = pGnss[0];
	Add_Imu_Data(&oTrans, &pImu[0],1);
	Add_Gnss_Data(&oTrans, pGnss[0]);

	for(i=j=1;i<iImu_Count;)
	{
		if (oGnss.m_fTime < oImu_Cur.m_fTime)
			oGnss = pGnss[j++];
		oImu_Cur = pImu[i++];		
		Add_Imu_Data(&oTrans, &oImu_Cur);

		Update_v(&oTrans, oImu_Cur, &oImu_Cur);
	}
}
template<typename _T>void Rotate_z(_T Source[3], _T psi, _T Dest[3])
{
	_T R[3 * 3] = { (_T)cos(psi),(_T)sin(psi),0,
				-(_T)sin(psi),(_T)cos(psi),0,
					0,0,1 };
	/*_T v[] = { 0,0,1,psi };
	Rotation_Vector_4_2_Matrix(v, R);*/
	Matrix_Multiply_3x1(R, Source, Dest);
	return;
}
template<typename _T>void Rotate_y(_T Source[3], _T theta, _T Dest[3])
{
	_T R[3 * 3] = { (_T)cos(theta),0,-(_T)sin(theta),
				0,1,0,
		(_T)sin(theta),0,(_T)cos(theta) };
	/*_T v[] = { 0,1,0,theta };
	Rotation_Vector_4_2_Matrix(v, R);*/
	Matrix_Multiply_3x1(R, Source, Dest);
	return;
}
template<typename _T>void Rotate_x(_T Source[3], _T phi, _T Dest[2])
{
	_T R[3 * 3] = { 1,0,0,
					0,(_T)cos(phi),(_T)sin(phi),
					0,-(_T)sin(phi),(_T)cos(phi) };
	Matrix_Multiply_3x1(R, Source, Dest);
	return;
}
template<typename _T>void Euler_Rotation(_T z, _T y, _T x,_T R[3*3])
{//z: 绕z转角度； y:绕y轴转角度；x:绕x转角度
	////武大书上方法，用的是xzy右手系
	//_T R_z[]={ cos(z),sin(z),0,
	//		-sin(z),cos(z),0,
	//		0,0,1 },
	//	R_y[]= { cos(y),0,-sin(y),
	//		0,1,0,
	//		sin(y),0,cos(y) },
	//	R_x[]={ 1,0,0,
	//		0,cos(x),sin(x),
	//		0,-sin(x),cos(x) };

	//Matrix_Multiply_3x3(R_y, R_z, R);
	//Matrix_Multiply_3x3(R_x, R, R);
	//Disp(R, 3, 3, "R");

	////武大书上方法2，直接算，一样结果
	//_T sin_x = (_T)sin(x), sin_y = (_T)sin(y), sin_z = (_T)sin(z),
	//	cos_x = (_T)cos(x), cos_y = (_T)cos(y), cos_z = (_T)cos(z);
	//R[0] = cos_y * cos_z;	R[1] = cos_y * sin_z;	R[2] = -sin_y;
	//R[3] = -cos_x*sin_z + sin_x * sin_y * cos_z;
	//R[4] = cos_x * cos_z + sin_x * sin_y * sin_z;
	//R[5] = sin_x * cos_y;
	//R[6] = sin_x * sin_z + cos_x * sin_y * cos_z;
	//R[7] = -sin_x * cos_z + cos_x * sin_y * sin_z;
	//R[8] = cos_x * cos_y;
	////Disp(R, 3, 3, "R");

	//14讲方法. 用的是xyz右手系
	_T v_z[] = { 0,0,1,z },
		v_y[] = { 0,1,0,y },
		v_x[] = { 1,0,0,x };
	_T R_Temp[3 * 3];
	Rotation_Vector_4_2_Matrix(v_z, R);
	Rotation_Vector_4_2_Matrix(v_y, R_Temp);
	Matrix_Multiply_3x3(R_Temp, R, R);
	Rotation_Vector_4_2_Matrix(v_x, R_Temp);
	Matrix_Multiply_3x3(R_Temp, R, R);
	//Disp(R, 3, 3, "R");
	return;
}
void Euler_Angle_Test()
{//书上是右手系，xzy, 拇指指向x, 其余4指先z后y
	//即x向左，y向上，z向里
	typedef float _T;
	//用武大教材
	{//绕z轴转
		_T Pos[] = { 1,0,0 };
		Rotate_z(Pos, (_T)(PI / 4), Pos);
		Disp(Pos, 1, 3, "Pos");
	}
	{//绕y轴转
		_T Pos[] = { 1,0,0 };
		Rotate_y(Pos,(_T)(PI / 4), Pos);
		Disp(Pos, 1, 3, "Pos");
	}
	{//绕x轴转
		_T Pos[] = { 0,1,0 };
		Rotate_x(Pos,(_T)(PI / 4), Pos);
		Disp(Pos, 1, 3, "Pos");
	}

	//14讲
	{//绕z轴转
		_T v[] = { 0,0,1,(_T)(PI / 4.f) }, R[3 * 3], Pos[] = { 1,0,0 };
		Rotation_Vector_4_2_Matrix(v, R);
		Matrix_Multiply_3x1(R, Pos, Pos);
		Disp(Pos, 1, 3, "Pos");
	}
	{//绕y轴转
		_T v[]={0,1,0,(_T)(PI/4.f)}, R[3 * 3], Pos[] = { 1,0,0 };
		Rotation_Vector_4_2_Matrix(v, R);
		Matrix_Multiply_3x1(R, Pos, Pos);
		Disp(Pos, 1, 3, "Pos");
	}
	{//绕x轴
		_T v[]={1,0,0,(_T)(PI/4.f)}, R[3 * 3], Pos[] = { 0,1,0 };
		Rotation_Vector_4_2_Matrix(v, R);
		Matrix_Multiply_3x1(R, Pos, Pos);
		Disp(Pos, 1, 3, "Pos");
	}

	{
		_T R[3 * 3], Pos[] = { 1,2,3 };
		Euler_Rotation(30.f, 20.f, 10.f, R);
		Disp(R, 3, 3, "R");
		Matrix_Multiply_3x1(R, Pos, Pos);
		Disp(Pos, 1, 3, "Pos");
	}
	return;
}
template<typename _T>void Coord_i_2_e(_T delta_t, _T R[3*3])
{//i系与e系同一个结构。e系经过时间delta_t后，
	_T theta = (_T)(WGS84_WIE * delta_t);		//地球经过delta_t转过的角度
	R[4] =R[0] = (_T)(cos(theta));
	R[1] =(_T)(sin(theta));
	R[3] = -R[1];
	R[2] = R[5] = R[6] = R[7] = 0;
	R[8] = 1;
	return;
}
template<typename _T>void Coord_e_2_i(_T delta_t, _T R[3*3])
{//是i->e的逆向。对于旋转矩阵来说，R必然正交，故此 R(-t) = R'，转置即可，其实交换[1][3]即可
	_T theta = (_T)(WGS84_WIE * delta_t);		//地球经过delta_t转过的角度
	R[4] =R[0] = (_T)(cos(theta));
	R[3] =(_T)(sin(theta));
	R[1] = -R[3];
	R[2] = R[5] = R[6] = R[7] = 0;
	R[8] = 1;
	return;
}
void Coord_Test()
{//搞搞各种坐标转换
	typedef float _T;
	{//i系转e系，i不动，e动
		_T R[3 * 3], Pos[] = { 1000,1000,1000 };
		Coord_i_2_e(100.f, R);
		Disp(R, 3, 3, "R");
		
		Coord_e_2_i(100.f, R);
		Disp(R, 3, 3, "R");

		Matrix_Multiply_3x1(R, Pos, Pos);
		Disp(Pos, 1, 3, "Pos");
	}
	return;
}

static void Imu_Test_1()
{//最简Imu更新轨迹
	typedef double _T;
	int iResult, iCount=100;
	IMU<_T>* pImu;
	_T fStart_Time = 456300.00000000000;
	_T R[3 * 3];
	_T Pos[3] = { 0,0,0 };	//起始位置
	_T v[3] = { 0,0,0 };	//开始速度

	Gen_I_Matrix(R, 3, 3);	//表示正向
	iResult=bRead_Imu_File("C:\\Users\\admin\\Desktop\\KF-GINS-main\\dataset\\Leador-A15.txt",&pImu,fStart_Time,&iCount);

	_T w_x_Delta_t[3] = { pImu[0].w[0] * pImu[0].Delta_Time,pImu[0].w[1] * pImu[0].Delta_Time,pImu[0].w[2] * pImu[0].Delta_Time };
	_T Delta_R[3 * 3], Next_R[3*3];
	Rotation_Vector_3_2_Matrix(w_x_Delta_t, Delta_R);
	Matrix_Multiply_3x3(Delta_R, R, Next_R);

	return;
}

void IMU_Test_Main()
{
	//Coord_Test();
	//r_Follow_Test_1();		//牵连位移实验
	//v_Follow_Test();			//牵连速度实验
	//a_Follow_Test_1();		//牵连加速度实验
	//Euler_Angle_Test();			//欧拉角旋转变换
	Imu_Test_1();
	//GNSS_Test();
	return;
}

#define MIN_LAT ((-80.5 * PI) / 180.0) /* -80.5 degrees in radians    */
#define MAX_LAT ((84.5 * PI) / 180.0)  /* 84.5 degrees in radians     */
#define MIN_EASTING 100000
#define MAX_EASTING 900000
#define MIN_NORTHING 0
#define MAX_NORTHING 10000000

#define UTM_NO_ERROR 0x0000
#define UTM_LAT_ERROR 0x0001
#define UTM_LON_ERROR 0x0002
#define UTM_EASTING_ERROR 0x0004
#define UTM_NORTHING_ERROR 0x0008
#define UTM_ZONE_ERROR 0x0010
#define UTM_HEMISPHERE_ERROR 0x0020
#define UTM_ZONE_OVERRIDE_ERROR 0x0040
#define UTM_A_ERROR 0x0080
#define UTM_INV_F_ERROR 0x0100

#define TRANMERC_NO_ERROR 0x0000
#define TRANMERC_LAT_ERROR 0x0001
#define TRANMERC_LON_ERROR 0x0002
#define TRANMERC_EASTING_ERROR 0x0004
#define TRANMERC_NORTHING_ERROR 0x0008
#define TRANMERC_ORIGIN_LAT_ERROR 0x0010
#define TRANMERC_CENT_MER_ERROR 0x0020
#define TRANMERC_A_ERROR 0x0040
#define TRANMERC_INV_F_ERROR 0x0080
#define TRANMERC_SCALE_FACTOR_ERROR 0x0100
#define TRANMERC_LON_WARNING 0x0200

#define MAX_DELTA_LONG ((PI * 90) / 180.0) /* 90 degrees in radians */
#define PI_OVER_2 (PI / 2.0e0)             /* PI over 2 */

#define MIN_SCALE_FACTOR 0.3
#define MAX_SCALE_FACTOR 3.0

#define SPHTMD(Latitude)                                                                                         \
    ((double)(oParam.TranMerc_ap * Latitude - oParam.TranMerc_bp * sin(2.e0 * Latitude) + oParam.TranMerc_cp * sin(4.e0 * Latitude) - \
              oParam.TranMerc_dp * sin(6.e0 * Latitude) + oParam.TranMerc_ep * sin(8.e0 * Latitude)))

#define SPHSN(Latitude) ((double)(oParam.TranMerc_a / sqrt(1.e0 - oParam.TranMerc_es * pow(sin(Latitude), 2))))

#define SPHSR(Latitude) ((double)(oParam.TranMerc_a * (1.e0 - oParam.TranMerc_es) / pow(DENOM(Latitude), 3)))

#define DENOM(Latitude) ((double)(sqrt(1.e0 - TranMerc_es * pow(sin(Latitude), 2))))

template<typename _T>int bConvert_Geodetic_To_Transverse_Mercator(_T Latitude, _T Longitude, _T* Easting, _T* Northing,UTM_Param<_T> *poParam)
{
	_T c; /* Cosine of latitude                          */
	_T c2;
	_T c3;
	_T c5;
	_T c7;
	_T dlam; /* Delta longitude - Difference in Longitude       */
	_T eta;  /* constant - TranMerc_ebs *c *c                   */
	_T eta2;
	_T eta3;
	_T eta4;
	_T s;  /* Sine of latitude                        */
	_T sn; /* Radius of curvature in the prime vertical       */
	_T t;  /* Tangent of latitude                             */
	_T tan2;
	_T tan3;
	_T tan4;
	_T tan5;
	_T tan6;
	_T t1;   /* Term in coordinate conversion formula - GP to Y */
	_T t2;   /* Term in coordinate conversion formula - GP to Y */
	_T t3;   /* Term in coordinate conversion formula - GP to Y */
	_T t4;   /* Term in coordinate conversion formula - GP to Y */
	_T t5;   /* Term in coordinate conversion formula - GP to Y */
	_T t6;   /* Term in coordinate conversion formula - GP to Y */
	_T t7;   /* Term in coordinate conversion formula - GP to Y */
	_T t8;   /* Term in coordinate conversion formula - GP to Y */
	_T t9;   /* Term in coordinate conversion formula - GP to Y */
	_T tmd;  /* True Meridional distance                        */
	_T tmdo; /* True Meridional distance for latitude of origin */
	long Error_Code = TRANMERC_NO_ERROR;
	_T temp_Origin;
	_T temp_Long;
	UTM_Param<_T> oParam = *poParam;

	if ((Latitude < (_T)(- ((PI * 89.99) / 180.0))) || (Latitude >(_T) (((PI * 89.99) / 180.0))))
	{ /* Latitude out of range */
		/*printf("%f\n", -((PI * 89.99) / 180.0));
		printf("%f\n", (PI * 89.99) / 180.0);*/
		printf("TRANMERC_LAT_ERROR\n");
		return 0;	// Error_Code |= TRANMERC_LAT_ERROR;
	}
	if (Longitude > PI)
		Longitude -= (_T)(2 * PI);
	if ((Longitude < (poParam->TranMerc_Origin_Long - ((PI * 90) / 180.0))) ||
		(Longitude > (poParam->TranMerc_Origin_Long + ((PI * 90) / 180.0))))
	{
		if (Longitude < 0)
			temp_Long = (_T)(Longitude + 2 * PI);
		else
			temp_Long = Longitude;

		if (oParam.TranMerc_Origin_Long < 0)
			temp_Origin =(_T)( oParam.TranMerc_Origin_Long + 2 * PI);
		else
			temp_Origin = oParam.TranMerc_Origin_Long;
		if ((temp_Long < (temp_Origin - ((PI * 90) / 180.0))) || (temp_Long > (temp_Origin + ((PI * 90) / 180.0))))
		{
			printf("TRANMERC_LON_ERROR\n");
			return 0;	//Error_Code |= TRANMERC_LON_ERROR;
		}
	}
	dlam = Longitude - oParam.TranMerc_Origin_Long;

	if (fabs(dlam) > (9.0 * PI /180)) { /* Distortion will result if Longitude is more than 9 degrees from the Central Meridian */
		Error_Code |= TRANMERC_LON_WARNING;
	}

	if (dlam > PI)
		dlam -=(_T)(2 * PI);
	if (dlam < -PI)
		dlam +=(_T)( (2 * PI));
	if (fabs(dlam) < 2.e-10) 
		dlam = 0.0;

	s = (_T)(sin(Latitude));
	c = (_T)(cos(Latitude));
	c2 = c * c;
	c3 = c2 * c;
	c5 = c3 * c2;
	c7 = c5 * c2;
	t = (_T)(tan(Latitude));
	tan2 = t * t;
	tan3 = tan2 * t;
	tan4 = tan3 * t;
	tan5 = tan4 * t;
	tan6 = tan5 * t;
	eta = oParam.TranMerc_ebs * c2;
	eta2 = eta * eta;
	eta3 = eta2 * eta;
	eta4 = eta3 * eta;

	/* radius of curvature in prime vertical */
	sn = (_T)(SPHSN(Latitude));

	/* True Meridianal Distances */
	tmd = (_T)(SPHTMD(Latitude));	//转成float丢精度暂时无解

	/*  Origin  */
	tmdo = (_T)(SPHTMD(oParam.TranMerc_Origin_Lat));

	/* northing */
	t1 = (tmd - tmdo) * oParam.TranMerc_Scale_Factor;
	t2 = (_T)(sn * s * c * oParam.TranMerc_Scale_Factor / 2.e0);
	t3 = (_T)(sn * s * c3 * oParam.TranMerc_Scale_Factor * (5.e0 - tan2 + 9.e0 * eta + 4.e0 * eta2) / 24.e0);

	t4 = (_T)(sn * s * c5 * oParam.TranMerc_Scale_Factor *
		(61.e0 - 58.e0 * tan2 + tan4 + 270.e0 * eta - 330.e0 * tan2 * eta + 445.e0 * eta2 + 324.e0 * eta3 -
			680.e0 * tan2 * eta2 + 88.e0 * eta4 - 600.e0 * tan2 * eta3 - 192.e0 * tan2 * eta4) /
		720.e0);

	t5 = (_T)(sn * s * c7 * oParam.TranMerc_Scale_Factor * (1385.e0 - 3111.e0 * tan2 + 543.e0 * tan4 - tan6) / 40320.e0);

	*Northing = (_T)(oParam.TranMerc_False_Northing + t1 + pow(dlam, 2.e0) * t2 + pow(dlam, 4.e0) * t3 + pow(dlam, 6.e0) * t4 +
		pow(dlam, 8.e0) * t5);

	/* Easting */
	t6 = sn * c * oParam.TranMerc_Scale_Factor;
	t7 =(_T)( sn * c3 * oParam.TranMerc_Scale_Factor * (1.e0 - tan2 + eta) / 6.e0);
	t8 = (_T)(sn * c5 * oParam.TranMerc_Scale_Factor *
		(5.e0 - 18.e0 * tan2 + tan4 + 14.e0 * eta - 58.e0 * tan2 * eta + 13.e0 * eta2 + 4.e0 * eta3 -
			64.e0 * tan2 * eta2 - 24.e0 * tan2 * eta3) / 120.e0);
	t9 = (_T)(sn * c7 * oParam.TranMerc_Scale_Factor * (61.e0 - 479.e0 * tan2 + 179.e0 * tan4 - tan6) / 5040.e0);

	*Easting =(_T)(oParam.TranMerc_False_Easting + dlam * t6 + pow(dlam, 3.e0) * t7 + pow(dlam, 5.e0) * t8 + pow(dlam, 7.e0) * t9);
	//printf("%lf\n", *Easting);
	*poParam = oParam;

	return 1;
}
template<typename _T>int bSet_Transverse_Mercator_Parameters(_T a, _T f, _T Origin_Latitude, _T Central_Meridian,
	_T False_Easting, _T False_Northing, _T Scale_Factor,UTM_Param<_T> *poParam)
{
	UTM_Param<_T> oParam = *poParam;
	_T tn; /* True Meridianal distance constant  */
	_T tn2;
	_T tn3;
	_T tn4;
	_T tn5;
	_T dummy_northing;
	_T TranMerc_b; /* Semi-minor axis of ellipsoid, in meters */
	_T inv_f = 1 / f;
	long Error_Code = TRANMERC_NO_ERROR;

	if (a <= 0.0) 
	{ /* Semi-major axis must be greater than zero */
		printf("TRANMERC_A_ERROR\n");
		return 0;//		Error_Code |= TRANMERC_A_ERROR;
	}

	if ((inv_f < 250) || (inv_f > 350)) 
	{ /* Inverse flattening must be between 250 and 350 */
		printf("TRANMERC_INV_F_ERROR\n");
		return 0;	//Error_Code |= TRANMERC_INV_F_ERROR;
	}

	if ((Origin_Latitude < -PI_OVER_2) || (Origin_Latitude > PI_OVER_2)) { /* origin latitude out of range */
		printf("TRANMERC_ORIGIN_LAT_ERROR\n");
		return 0;	//Error_Code |= TRANMERC_ORIGIN_LAT_ERROR;
	}
	if ((Central_Meridian < -PI) || (Central_Meridian > (2 * PI))) 
	{ /* origin longitude out of range */
		printf("TRANMERC_CENT_MER_ERROR\n");
		return 0;	// Error_Code |= TRANMERC_CENT_MER_ERROR;
	}

	if ((Scale_Factor < MIN_SCALE_FACTOR) || (Scale_Factor > MAX_SCALE_FACTOR)) 
	{
		printf("TRANMERC_SCALE_FACTOR_ERROR\n");
		return 0;	//Error_Code |= TRANMERC_SCALE_FACTOR_ERROR;
	}

	if (!Error_Code) 
	{ /* no errors */
		oParam.TranMerc_a = a;
		oParam.TranMerc_f = f;
		oParam.TranMerc_Origin_Lat = Origin_Latitude;
		if (Central_Meridian > PI) Central_Meridian -= (_T)((2 * PI));
		oParam.TranMerc_Origin_Long = Central_Meridian;
		oParam.TranMerc_False_Northing = False_Northing;
		oParam.TranMerc_False_Easting = False_Easting;
		oParam.TranMerc_Scale_Factor = Scale_Factor;

		/* Eccentricity Squared */
		oParam.TranMerc_es = 2 * oParam.TranMerc_f - oParam.TranMerc_f * oParam.TranMerc_f;
		/* Second Eccentricity Squared */
		oParam.TranMerc_ebs = (1 / (1 - oParam.TranMerc_es)) - 1;

		TranMerc_b = oParam.TranMerc_a * (1 - oParam.TranMerc_f);
		/*True meridianal constants  */
		tn = (oParam.TranMerc_a - TranMerc_b) / (oParam.TranMerc_a + TranMerc_b);
		tn2 = tn * tn;
		tn3 = tn2 * tn;
		tn4 = tn3 * tn;
		tn5 = tn4 * tn;

		oParam.TranMerc_ap = (_T)(oParam.TranMerc_a * (1.e0 - tn + 5.e0 * (tn2 - tn3) / 4.e0 + 81.e0 * (tn4 - tn5) / 64.e0));
		oParam.TranMerc_bp = (_T)(3.e0 * oParam.TranMerc_a * (tn - tn2 + 7.e0 * (tn3 - tn4) / 8.e0 + 55.e0 * tn5 / 64.e0) / 2.e0);
		oParam.TranMerc_cp =(_T)( 15.e0 * oParam.TranMerc_a * (tn2 - tn3 + 3.e0 * (tn4 - tn5) / 4.e0) / 16.0);
		oParam.TranMerc_dp = (_T)(35.e0 * oParam.TranMerc_a * (tn3 - tn4 + 11.e0 * tn5 / 16.e0) / 48.e0);
		oParam.TranMerc_ep = (_T)(315.e0 * oParam.TranMerc_a * (tn4 - tn5) / 512.e0);
		//printf("%10.10lf\n", PI );
		_T TranMerc_Delta_Easting, TranMerc_Delta_Northing;
		bConvert_Geodetic_To_Transverse_Mercator((_T)((PI * 89.99) / 180.0), (_T)(((PI * 90) / 180.0) + Central_Meridian), &TranMerc_Delta_Easting,
			&TranMerc_Delta_Northing,&oParam);
		oParam.TranMerc_Delta_Easting = TranMerc_Delta_Easting;
		oParam.TranMerc_Delta_Northing = TranMerc_Delta_Northing;
		bConvert_Geodetic_To_Transverse_Mercator((_T)0.f,(_T)(((PI * 90) / 180.0) + Central_Meridian), &TranMerc_Delta_Easting,
			&dummy_northing,&oParam);
		oParam.TranMerc_Delta_Easting = TranMerc_Delta_Easting;

		oParam.TranMerc_Delta_Northing++;
		oParam.TranMerc_Delta_Easting++;

		*poParam = oParam;
	} /* END OF if(!Error_Code) */
	return 1;
}

template<typename _T>int bLat_Long_2_utm(_T fLatitude, _T fLongitude,UTM_Param<_T> *poParam, _T *pfNorth,_T *pfEast, int *piZone, char *pcNorth)
{//给定一个纬经度，求落在哪个utm区域
 //纬度经度弧度
	long Lat_Degrees;
	long Long_Degrees;
	long temp_zone;
	int iRet = 0;
	_T Origin_Latitude = 0;
	_T Central_Meridian = 0;
	_T False_Easting = 500000;		//一区之中间？
	_T False_Northing = 0;
	_T Scale = (_T)0.9996;

	if ((fLatitude < MIN_LAT) || (fLatitude > MAX_LAT)) 
	{ /* Latitude out of range */
		printf("Latitude must between %f and %f, check if you use radian or angle\n", MIN_LAT, MAX_LAT);
		return 0;
		//iRet |= UTM_LAT_ERROR;
	}

	if ((fLongitude < -PI) || (fLongitude > PI)) 
	{ /* Longitude out of range */
		printf("Longitude must between %f and %f, check if you use radian or angle\n", -PI, PI);
		//iRet |= UTM_LON_ERROR;
		return 0;
	}

	if ((fLatitude > -1.0e-9) && (fLatitude < 0)) 
		fLatitude = 0.0;	//在赤道附近小于eps, Clip为0

	if (fLongitude < 0)
		fLongitude += (_T)((2 * PI) + 1.0e-10);	//大概利用周期性，但是加上个eps什么意思？

	//又换回角度，而且是整数
	Lat_Degrees = (long)(fLatitude * 180.0 / PI);
	Long_Degrees = (long)(fLongitude * 180.0 / PI);

	//以下从经度判断所属区间
	if (fLongitude < PI)
		temp_zone = (long)(31 + ((fLongitude * 180.0 / PI) / 6.0));
	else
		temp_zone = (long)(((fLongitude * 180.0 / PI) / 6.0) - 29);
	//似乎 1-60，而非 0-60
	if (temp_zone > 60)
		temp_zone = 1;

	//utm自己定义的算法
	if ((Lat_Degrees > 55) && (Lat_Degrees < 64) && (Long_Degrees > -1) && (Long_Degrees < 3)) 
		temp_zone = 31;
	if ((Lat_Degrees > 55) && (Lat_Degrees < 64) && (Long_Degrees > 2) && (Long_Degrees < 12))
		temp_zone = 32;
	if ((Lat_Degrees > 71) && (Long_Degrees > -1) && (Long_Degrees < 9)) 
		temp_zone = 31;
	if ((Lat_Degrees > 71) && (Long_Degrees > 8) && (Long_Degrees < 21))
		temp_zone = 33;
	if ((Lat_Degrees > 71) && (Long_Degrees > 20) && (Long_Degrees < 33))
		temp_zone = 35;
	if ((Lat_Degrees > 71) && (Long_Degrees > 32) && (Long_Degrees < 42))
		temp_zone = 37;

	if (poParam->UTM_Override) 
	{
		if ((temp_zone == 1) && (poParam->UTM_Override == 60))
			temp_zone = poParam->UTM_Override;
		else if ((temp_zone == 60) && (poParam->UTM_Override == 1))
			temp_zone = poParam->UTM_Override;
		else if ((Lat_Degrees > 71) && (Long_Degrees > -1) && (Long_Degrees < 42)) 
		{
			if (((temp_zone - 2) <= poParam->UTM_Override) && (poParam->UTM_Override <= (temp_zone + 2))) 
				temp_zone = poParam->UTM_Override;
			else
			{
				//Error_Code = UTM_ZONE_OVERRIDE_ERROR;
				printf("UTM_ZONE_OVERRIDE_ERROR\n");
				return 0;
			}

		} else if (((temp_zone - 1) <= poParam->UTM_Override) && (poParam->UTM_Override <= (temp_zone + 1)))
			temp_zone = poParam->UTM_Override;
		else
		{
			//Error_Code = UTM_ZONE_OVERRIDE_ERROR;
			printf("UTM_ZONE_OVERRIDE_ERROR\n");
			return 0;
		}			
	}

	if (temp_zone >= 31) 
		Central_Meridian = (_T)((6 * temp_zone - 183) * PI / 180.0);
	else
		Central_Meridian = (_T)((6 * temp_zone + 177) * PI / 180.0);
	int Zone = temp_zone;
	char Hemisphere;

	if (fLatitude < 0)
	{
		False_Northing = 10000000;
		Hemisphere = 'S';
	} else
		Hemisphere = 'N';

	_T Easting, Northing;

	bSet_Transverse_Mercator_Parameters(poParam->UTM_a, poParam->UTM_f, Origin_Latitude, Central_Meridian, False_Easting,
		False_Northing, Scale,poParam);

	bConvert_Geodetic_To_Transverse_Mercator(fLatitude, fLongitude, &Easting, &Northing,poParam);
	if ((Easting < MIN_EASTING) || (Easting > MAX_EASTING))
	{
		printf("UTM_EASTING_ERROR\n");
		return 0;	//Error_Code = UTM_EASTING_ERROR;
	}

	if ((Northing < MIN_NORTHING) || (Northing > MAX_NORTHING))
	{
		printf("UTM_NORTHING_ERROR\n");
		return 0;	//Error_Code |= UTM_NORTHING_ERROR;
	}

	*pfNorth = Northing;
	*pfEast = Easting;
	*piZone = Zone;
	*pcNorth = Hemisphere;
	//printf("%lf\n", Easting);
	return 1;
}

template<typename _T>int bLat_Long_2_utm_rad(GNSS<_T> oGnss, _T fAntenna_x,_T fAntenna_y,_T fAntenna_Angle, _T Map_Origin[3],UTM_Param<_T> *poParam,utm<_T> *poUtm)
{
	_T fNorth, fEast;
	int iZone;
	char cNorth;
	if (!bLat_Long_2_utm(oGnss.GPS[0], oGnss.GPS[1], poParam, &fNorth, &fEast, &iZone, &cNorth))
		return 0;
	//printf("%lf %lf\n", fEast,fNorth );
	poUtm->m_fEast = fEast, poUtm->m_fNorth = fNorth, poUtm->m_fHeight = oGnss.GPS[2];

	_T fHeading;
	if (oGnss.m_bHeading_Valid)
		fHeading = (_T)((90 - oGnss.m_fHeading) * PI / 180.);  // 北东地转到东北天
	else
		fHeading = 0;

	union {	_T TBG[4 * 4],TGB[4 * 4];};
	{
		_T v[3] = { 0,0,(_T)(fAntenna_Angle * PI / 180.) }, t[3] = { fAntenna_x,fAntenna_y,0 };
		//TBG位姿，位于(x,y,0), 面向(0,0,angle)，感觉斜向上
		Gen_Homo_Matrix_1(v, t, TBG);
	}

	//取其逆
	Get_Inv_Matrix_Row_Op(TBG, TGB,4);
	//Disp(TGB, 4, 4, "TGB");

	union {	_T TWG[4 * 4], TWB[4 * 4];	};
	{
		_T v[3] = { 0,0,fHeading }, t[3] = { fEast - Map_Origin[0],fNorth- Map_Origin[1],oGnss.GPS[2]- Map_Origin[2]};
		Gen_Homo_Matrix_1(v, t, TWG);
	}
	//printf("%lf\n", fEast);
	//Disp(TWG, 4, 4, "TWG");
	Matrix_Multiply(TWG,4,4, TGB,4, TWB);
	//Disp(TWB, 4, 4, "TWB");

	poUtm->x = TWB[3];
	poUtm->y = TWB[7];
	poUtm->z = TWB[11];

	if (oGnss.m_bHeading_Valid)
		memcpy(poUtm->m_Pose, TWB, sizeof(TWB));

	return 1;
}
template<typename _T>int bLat_Long_2_utm_ang(GNSS<_T> oGnss, _T fAntenna_x, _T fAntenna_y, _T fAntenna_Angle, _T Map_Origin[3], UTM_Param<_T>* poParam, utm<_T>* poUtm)
{
	GNSS<_T> oGnss_1 = oGnss;
	oGnss_1.GPS[0] *= (_T)(PI / 180.f);
	oGnss_1.GPS[1] *= (_T)(PI / 180.f);
	//Disp(oGnss_1.GPS, 1, 2);
	return bLat_Long_2_utm_rad(oGnss_1, fAntenna_x, fAntenna_y, fAntenna_Angle, Map_Origin, poParam, poUtm);
}

template<typename _T>void Predict_pvq(IMU<_T> oImu,_T p[3], _T v[3], _T R[3 * 3],_T ba[3], _T bg[3],_T g[3],_T dt)
{//这个不知能否做成统一函数，目前只是最简形式。一旦上无人机得上满血版
//状态量改变了p,v,q		未改变的是bg, ba ,g
 //		a1 = [R * (a + ba) + g] * dt
 //p1 = p0 + (v0 + 0.5 * a1 * dt) * dt
 //v1 = v0 + a1 * dt
 //a1 = [R * (a + ba) + g] * dt


	_T a1[3];		//a1 = R * (a - ba) + g
	_T a1_dt[3];	//a1 * dt
	_T Temp[3];

	//先算 a1 = R * (a - ba) + g
	Vector_Minus(oImu.a, ba, 3, a1);		//a1 - ba
	Matrix_Multiply(R, 3, 3, a1, 1, a1);	//R * (a - ba)
	Vector_Add(a1, g, 3, a1);				//a1 = R * (a - ba) + g

	//计算 a1*dt
	Vector_Multiply(a1, 3, dt, a1_dt);

	//第一部分，更新p, 计算	p1 = p0 + (v0 + 0.5 * a1 * dt) * dt
	//其中，					a1 = R * (a - ba) + g
	Vector_Multiply(a1_dt, 3,(_T)0.5, Temp);	//0.5 * a1 * dt
	Vector_Add(v, Temp, 3, Temp);				//v0 + 0.5 * a1 * dt
	Vector_Multiply(Temp, 3, dt, Temp);			//(v0 + 0.5 * a1 * dt) * dt
	Vector_Add(p, Temp, 3, p);					//p0 + (v0 + 0.5 * a1 * dt) * dt

	//第二部分，更新v,	v1 = p0 + (v0 + 1/2*a1*dt) * dt
	// 其中				a1 = [R * (a + ba) + g] * dt
	Vector_Add(v, a1_dt, 3, v);				//v1 = v0 + a1*dt

	//第三部分，更新q, R1 = R0 * Exp[ (w + bg) * dt ]
	Vector_Minus(oImu.w, bg, 3, Temp);		// w + bg
	Vector_Multiply(Temp, 3, dt, Temp);		//(w + bg) * dt 

	_T Exp[3 * 3];
	Rotation_Vector_3_2_Matrix(Temp, Exp);
	Matrix_Multiply(R, 3, 3, Exp, 3, R);	//R即是q
}
