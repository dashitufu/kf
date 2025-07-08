//��֤һ��Э������ϵ��
#include "stdio.h"
#include "Common.h"
#include "Matrix.h"

#include <random>

using namespace std;


/************************һ���άͳ�ƺ���****************************/
template<typename _T>void Get_E_2(_T Point[][2], int iCount, _T E[])
{//��������
	E[0] = E[1] = 0;
	for (int i = 0; i < iCount; i++)
	{
		E[0] += Point[i][0];
		E[1] += Point[i][1];
	}
	E[0] /= iCount,	E[1] /= iCount;
}
template<typename _T>float fGet_Cov_2(_T Point[][2], int iCount, _T E[2])
{//������������������Э����
	_T fTotal = 0;
	for (int i = 0; i < iCount; i++)	//�ɵ�������(0,0)Χ��һ�������Σ��ۼ����
		fTotal += (Point[i][0] - E[0]) * (Point[i][1] - E[1]);
	return fTotal / iCount;
}
template<typename _T>void Get_Var_2(_T Point[][2], int iCount, _T E[2],_T Var[2])
{//�㷽��
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

	//���ϵ��
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
{//һ����һ���㼯�Ĺ�ϵ
	_T Delta[2] = { Point_2[0] - Point_1[0],Point_2[1] - Point_1[1] };
	Matrix_Multiply(Cov, 2, 2,Delta,1,Delta);

	_T fDist = fDot(Delta, Delta, 2);
	return (_T)sqrt(fDist);
}
/************************һ���άͳ�ƺ���****************************/

static void Test_1()
{//Э���������ʶ
	typedef float _T;
	Line_1 oLine;
	const int iCount = 100;
	int i;
	_T x, y, Point[iCount][2],E[2];

	{//��һ��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 100);	//��һ��б��Ϊ1��ֱ��
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

	{//��2��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 200);	//��һ��б��Ϊ2��ֱ��
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

	{//��3��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 50);	//��һ��б��Ϊ1/2��ֱ��
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
	//�����濴����б��Խ��Э����ҲԽ�󣨴��£�
	return;
}
static void Test_2()
{//������ϵ��
	typedef float _T;
	Line_1 oLine;
	const int iCount = 100;
	int i;
	_T x, y, Point[iCount][2],E[2];

	{//��һ��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 100);	//��һ��б��Ϊ1��ֱ��
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

		//���ϵ��
		_T rho = (_T)(fCov / sqrt(Var[0] * Var[1]));
		printf("rho:%f\n", rho);		

		rho = fGet_Corr_Coef_2(Point, iCount);
		printf("rho:%f\n", rho);	
	}

	{//��2��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 200);	//��һ��б��Ϊ2��ֱ��
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

		//���ϵ��
		_T rho = (_T)(fCov / sqrt(Var[0] * Var[1]));
		printf("rho:%f\n", rho);		

		rho = fGet_Corr_Coef_2(Point, iCount);
		printf("rho:%f\n", rho);	
	}

	{//��3��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 50);	//��һ��б��Ϊ1/2��ֱ��
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

		//���ϵ��
		_T rho = (_T)(fCov / sqrt(Var[0] * Var[1]));
		printf("rho:%f\n", rho);		

		rho = fGet_Corr_Coef_2(Point, iCount);
		printf("rho:%f\n", rho);	
	}
	//�ɼ������ϵ����б�ʲ���һ���йء�����ˮƽ�봹ֱ������
}

static void Test_3()
{//���Э�������
	typedef float _T;
	Line_1 oLine;
	const int iCount = 100;
	int i;
	_T x, y, Point[iCount][2],A[2*2];

	{//��һ��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 100);	//��һ��б��Ϊ1��ֱ��
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

	{//��2��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 200);	//��һ��б��Ϊ2��ֱ��
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

	{//��3��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 50);	//��һ��б��Ϊ1/2��ֱ��
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
{//������Ͼ�����һ��
	typedef float _T;
	Line_1 oLine;
	const int iCount = 100;
	int i;
	_T x, y, Point[iCount][2], E[2];
	union {
		_T A[2 * 2];
		_T Sigma_1[2 * 2];
	};

	{//��һ��ֱ�ߣ���x,y֮���Э����
		Cal_Line(&oLine, 0, 0, 100, 213);	//��һ��б��Ϊ1��ֱ��
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

		//��һ���µ�
		_T Point_1[2] = { 102,fGet_Line_y(&oLine,102) };
		_T fDist = fGet_Mah_Dist_2(Sigma_1,E,Point_1);
		printf("Dist:%f\n", fDist);

		fDist = fGet_Distance(Point_1, E, 2);
		printf("Dist:%f\n", fDist);

		//������Ƚ�
		for (i = 0; i < iCount; i++)
		{
			fDist = fGet_Mah_Dist_2(Sigma_1, Point[i], Point_1);
			printf("Dist:%f\n", fDist);
		}
	}
	//�ɼ���ͬһ�ŵ����ͺܽ�

	return;
}

template<typename _T>_T fGet_Norm_Dist(_T x)
{//���ر�׼��̫�ֲ������֣�
	return (_T)(0.5 * erfc(-x * sqrt(0.5)));
}

template<typename _T>_T fGet_Norm_Dist(_T e, _T sigma, _T x)
{//����һ����̫�ֲ������ػ���ֵ
//����һ��ԭ���� x ~ N(e,sigma^2) ��
//F(x) = Phi( (x-e)/sigma )
	return fGet_Norm_Dist((x - e) / sigma);
}


void Norm_Dist_Test_1()
{//��̫�ֲ�ʵ��
	typedef float _T;
	//{//��׼��̫�ֲ����ܶȺ���
	//	//phi(x) = 1/sqrt(2PI) * e^(-x*x/2)
	//	for (_T x = 0; x < 1; x += 0.001)
	//		printf("%llf\n", exp(-x * x / 2) / sqrt(2 * PI));
	//}

	{//��׼��̫�ֲ�����
		for (_T x = 100; x < 200; x += 1)
			printf("%lf\n", (_T)fGet_Norm_Dist((_T)100.,(_T)10.,x));
	}

	return;
}
int main1()
{
	Init_Env();
	//Test_1();	//���Э����
	//Test_2();	//���ϵ�������ǽ�Э�����һ��
	//Test_3();	//Э�������
	//Test_4();	//Э��������Ӧ�ã����Ͼ��룬ע�⣬Ҫ����

	Norm_Dist_Test_1();
	Free_Env();

#ifdef WIN32
	_CrtDumpMemoryLeaks();
#endif
	return 0;
}