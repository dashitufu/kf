// Sophus_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "sophus/so3.hpp"
#include <iostream>
using namespace std;
using namespace Sophus;
#define PI 3.1415926

int main()
{
	//先造一个三维旋转向量
	Vector3d so3(1, 2, 3);
	cout << so3 << endl;

	//将旋转向量转换为旋转矩阵
	SO3d R = SO3d::exp(so3);
	cout << R.matrix() << endl;

}
