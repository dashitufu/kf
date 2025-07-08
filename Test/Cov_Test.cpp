//验证一下协方差，相关系数
#include "stdio.h"
#include "Common.h"
#include "Matrix.h"

#include <random>

using namespace std;


/************************一组二维统计函数****************************/
template<typename _T>void Get_E_2(_T Point[][2], int iCount, _T E[])
{//计算期望
	E[0] = E[1] = 0;
	for (int i = 0; i < iCount; i++)
	{
		E[0] += Point[i][0];
		E[1] += Point[i][1];
	}
	E[0] /= iCount,	E[1] /= iCount;
}
template<typename _T>float fGet_Cov_2(_T Point[][2], int iCount, _T E[2])
{//利用期望与样本计算协方差
	_T fTotal = 0;
	for (int i = 0; i < iCount; i++)	//由点与期望(0,0)围成一个正方形，累加面积
		fTotal += (Point[i][0] - E[0]) * (Point[i][1] - E[1]);
	return fTotal / iCount;
}
template<typename _T>void Get_Var_2(_T Point[][2], int iCount, _T E[2],_T Var[2])
{//算方差
	Var[0] = Var[1] = 0;
	for (int i = 0; i < iCount; i++)
	{
		Var[0] += (Point[i][0] - E[0]) * (Point[i][0] - E[0]);
		Var[1] += (Point[i][1] - E[1]) * (Point[i][1] - E[1]);
	}
	Var[0] /= iCount;
	Var[1] /= iCount;
}

template<typename _T>_T fGet_Corr_Coef_2(_T Point[][2], int iCount)
{
	_T E[2], Var[2];
	Get_E_2(Point, iCount, E);
	_T fCov = fGet_Cov_2(Point, iCount, E);
	Get_Var_2(Point, iCount, E, Var);

	//相关系数
	return (_T)(fCov / sqrt(Var[0] * Var[1]));
}
template<typename _T>void Gen_Cov_Matrix_2(_T Point[][2], int iCount, _T A[2 * 2])
{
	_T E[2], Var[2];
	Get_E_2(Point, iCount, E);
	_T fCov = fGet_Cov_2(Point, iCount, E);
	Get_Var_2(Point, iCount, E, Var);
	
	A[0] = Var[0], A[3] = Var[1];
	A[1] = A[2] = fCov;
}
template<typename _T>_T fGet_Mah_Dist_2(_T Cov[2*2], _T Point_1[2],_T Point_2[2])
{//一点与一个点集的关系
	_T Delta[2] = { Point_2[0] - Point_1[0],Point_2[1] - Point_1[1] };
	Matrix_Multiply(Cov, 2, 2,Delta,1,Delta);

	_T fDist = fDot(Delta, Delta, 2);
	return (_T)sqrt(fDist);
}
/************************一组二维统计函数****************************/

static void Test_1()
{//协方差感性认识
	typedef float _T;
	Line_1 oLine;
	const int iCount = 100;
	int i;
	_T x, y, Point[iCount][2],E[2];

	{//第一条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 100);	//做一条斜率为1的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Get_E_2(Point, iCount, E);
		_T fCov = fGet_Cov_2(Point, iCount, E);
		printf("Covar:%f\n", fCov);

		_T Var[2];
		Get_Var_2(Point, iCount, E, Var);
		Disp(Var, 1, 2, "Var");
	}

	{//第2条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 200);	//做一条斜率为2的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Get_E_2(Point, iCount, E);
		_T fCov = fGet_Cov_2(Point, iCount, E);
		printf("Covar:%f\n", fCov);

		_T Var[2];
		Get_Var_2(Point, iCount, E, Var);
		Disp(Var, 1, 2, "Var");
	}

	{//第3条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 50);	//做一条斜率为1/2的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Get_E_2(Point, iCount, E);
		_T fCov = fGet_Cov_2(Point, iCount, E);
		printf("Covar:%f\n", fCov);

		_T Var[2];
		Get_Var_2(Point, iCount, E, Var);
		Disp(Var, 1, 2, "Var");
	}
	//从上面看出，斜率越大，协方差也越大（大致）
	return;
}
static void Test_2()
{//搞搞相关系数
	typedef float _T;
	Line_1 oLine;
	const int iCount = 100;
	int i;
	_T x, y, Point[iCount][2],E[2];

	{//第一条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 100);	//做一条斜率为1的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Get_E_2(Point, iCount, E);
		_T fCov = fGet_Cov_2(Point, iCount, E);
		//printf("Covar:%f\n", fCov);

		_T Var[2];
		Get_Var_2(Point, iCount, E, Var);
		//Disp(Var, 1, 2, "Var");

		//相关系数
		_T rho = (_T)(fCov / sqrt(Var[0] * Var[1]));
		printf("rho:%f\n", rho);		

		rho = fGet_Corr_Coef_2(Point, iCount);
		printf("rho:%f\n", rho);	
	}

	{//第2条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 200);	//做一条斜率为2的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Get_E_2(Point, iCount, E);
		_T fCov = fGet_Cov_2(Point, iCount, E);
		//printf("Covar:%f\n", fCov);

		_T Var[2];
		Get_Var_2(Point, iCount, E, Var);
		//Disp(Var, 1, 2, "Var");

		//相关系数
		_T rho = (_T)(fCov / sqrt(Var[0] * Var[1]));
		printf("rho:%f\n", rho);		

		rho = fGet_Corr_Coef_2(Point, iCount);
		printf("rho:%f\n", rho);	
	}

	{//第3条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 50);	//做一条斜率为1/2的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Get_E_2(Point, iCount, E);
		_T fCov = fGet_Cov_2(Point, iCount, E);
		//printf("Covar:%f\n", fCov);

		_T Var[2];
		Get_Var_2(Point, iCount, E, Var);
		//Disp(Var, 1, 2, "Var");

		//相关系数
		_T rho = (_T)(fCov / sqrt(Var[0] * Var[1]));
		printf("rho:%f\n", rho);		

		rho = fGet_Corr_Coef_2(Point, iCount);
		printf("rho:%f\n", rho);	
	}
	//可见，相关系数与斜率并不一定有关。不过水平与垂直另当别论
}

static void Test_3()
{//求个协方差矩阵
	typedef float _T;
	Line_1 oLine;
	const int iCount = 100;
	int i;
	_T x, y, Point[iCount][2],A[2*2];

	{//第一条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 100);	//做一条斜率为1的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Gen_Cov_Matrix_2(Point, iCount, A);
		Disp(A, 2, 2, "A");
	}

	{//第2条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 200);	//做一条斜率为2的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Gen_Cov_Matrix_2(Point, iCount, A);
		Disp(A, 2, 2, "A");
	}

	{//第3条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 50);	//做一条斜率为1/2的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Gen_Cov_Matrix_2(Point, iCount, A);
		Disp(A, 2, 2, "A");
	}

	return;
}

static void Test_4()
{//搞个马氏距离试一下
	typedef float _T;
	Line_1 oLine;
	const int iCount = 100;
	int i;
	_T x, y, Point[iCount][2], E[2];
	union {
		_T A[2 * 2];
		_T Sigma_1[2 * 2];
	};

	{//第一条直线，求x,y之间的协方差
		Cal_Line(&oLine, 0, 0, 100, 213);	//做一条斜率为1的直线
		for (i = 0; i < iCount; i++)
		{
			x = (_T)i;
			y = fGet_Line_y(&oLine, x);
			Point[i][0] = x;
			Point[i][1] = y;
		}
		Get_E_2(Point, iCount, E);
		Gen_Cov_Matrix_2(Point, iCount, A);
		Get_Inv_Matrix_Row_Op(A, Sigma_1, 2);

		//造一个新点
		_T Point_1[2] = { 102,fGet_Line_y(&oLine,102) };
		_T fDist = fGet_Mah_Dist_2(Sigma_1,E,Point_1);
		printf("Dist:%f\n", fDist);

		fDist = fGet_Distance(Point_1, E, 2);
		printf("Dist:%f\n", fDist);

		//再逐个比较
		for (i = 0; i < iCount; i++)
		{
			fDist = fGet_Mah_Dist_2(Sigma_1, Point[i], Point_1);
			printf("Dist:%f\n", fDist);
		}
	}
	//可见，同一团点距离就很近

	return;
}

template<typename _T>_T fGet_Norm_Dist(_T x)
{//返回标准正太分布（积分）
	return (_T)(0.5 * erfc(-x * sqrt(0.5)));
}

template<typename _T>_T fGet_Norm_Dist(_T e, _T sigma, _T x)
{//对于一般正太分布，返回积分值
//基于一个原理，若 x ~ N(e,sigma^2) 则
//F(x) = Phi( (x-e)/sigma )
	return fGet_Norm_Dist((x - e) / sigma);
}


void Norm_Dist_Test_1()
{//正太分布实验
	typedef float _T;
	//{//标准正太分布的密度函数
	//	//phi(x) = 1/sqrt(2PI) * e^(-x*x/2)
	//	for (_T x = 0; x < 1; x += 0.001)
	//		printf("%llf\n", exp(-x * x / 2) / sqrt(2 * PI));
	//}

	{//标准正太分布函数
		for (_T x = 100; x < 200; x += 1)
			printf("%lf\n", (_T)fGet_Norm_Dist((_T)100.,(_T)10.,x));
	}

	return;
}
int main1()
{
	Init_Env();
	//Test_1();	//最简协方差
	//Test_2();	//相关系数，就是将协方差归一化
	//Test_3();	//协方差矩阵
	//Test_4();	//协方差矩阵的应用，马氏距离，注意，要求逆

	Norm_Dist_Test_1();
	Free_Env();

#ifdef WIN32
	_CrtDumpMemoryLeaks();
#endif
	return 0;
}