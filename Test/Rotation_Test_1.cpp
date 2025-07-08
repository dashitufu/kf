//本文件用于演示旋转矩阵的各种应用
#include "Common.h"
#include "Matrix.h"
#include "Image.h"

void Test_1()
{//生成一个二维旋转矩阵
	typedef float _T;
	_T R[2 * 2], Point_1[2] = { 100,200 },Point_2[2], theta = PI / 6;
	//做一个绕原点旋转theta的矩阵
	Gen_Rotation_Matrix_2D(R, theta);

	//求Point_2 = R * Point_1
	Matrix_Multiply(R, 2, 2, Point_1, 1, Point_2);
	Disp(Point_1, 2, 1, "Point_1");
	Disp(Point_2, 2, 1, "Point_2 = R * Point_1");

	//为了感性认识，可以画张图
	Image oImage;
	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP,24);
	Set_Color(oImage);

	int Screen_Pos_1[2],	//Point_1的屏幕坐标 
		Screen_Pos_2[2],	//Point_2的屏幕坐标
		Org[2];				//原点
	//注意，屏幕坐标与直角坐标系不一样，故此要转换一下
	Rect_2_Screen_Coordinate(Point_1[0], Point_1[1], &Screen_Pos_1[0], &Screen_Pos_1[1]);
	Rect_2_Screen_Coordinate(Point_2[0], Point_2[1], &Screen_Pos_2[0], &Screen_Pos_2[1]);
	Rect_2_Screen_Coordinate(0, 0, &Org[0], &Org[1]);

	//画线，便于观察
	Draw_Line(oImage, Screen_Pos_1[0], Screen_Pos_1[1], Org[0], Org[1]);
	Draw_Line(oImage, Screen_Pos_2[0], Screen_Pos_2[1], Org[0], Org[1]);
		
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	
	Free_Image(&oImage);
}
void Test_2()
{//旋转向量到旋转矩阵
	typedef float _T;       //后续的精度可以改为double
	{   //第一个例子，旋转矩阵到旋转矩阵
		_T Rotation_Vector[] = { 1,2,3 };    //绕z轴旋转45度
		_T Rotation_Matrix[3 * 3];
		Rotation_Vector_3_2_Matrix(Rotation_Vector, Rotation_Matrix);
		Disp(Rotation_Matrix, 3, 3, "Rotaion Matrix");

		//假设有个点，落在x轴上
		_T Point[] = { 1, 0, 0 };
		//旋转变换
		Matrix_Multiply(Rotation_Matrix, 3, 3, Point, 1, Point);
		Disp(Point, 3, 1, "Point");
	}

}

int main()
{
	Init_Env();
	Test_2();
	Free_Env();
	return 0;
}
