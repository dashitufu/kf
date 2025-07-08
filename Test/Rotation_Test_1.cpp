//���ļ�������ʾ��ת����ĸ���Ӧ��
#include "Common.h"
#include "Matrix.h"
#include "Image.h"

void Test_1()
{//����һ����ά��ת����
	typedef float _T;
	_T R[2 * 2], Point_1[2] = { 100,200 },Point_2[2], theta = PI / 6;
	//��һ����ԭ����תtheta�ľ���
	Gen_Rotation_Matrix_2D(R, theta);

	//��Point_2 = R * Point_1
	Matrix_Multiply(R, 2, 2, Point_1, 1, Point_2);
	Disp(Point_1, 2, 1, "Point_1");
	Disp(Point_2, 2, 1, "Point_2 = R * Point_1");

	//Ϊ�˸�����ʶ�����Ի���ͼ
	Image oImage;
	Init_Image(&oImage, 1920, 1080, Image::IMAGE_TYPE_BMP,24);
	Set_Color(oImage);

	int Screen_Pos_1[2],	//Point_1����Ļ���� 
		Screen_Pos_2[2],	//Point_2����Ļ����
		Org[2];				//ԭ��
	//ע�⣬��Ļ������ֱ������ϵ��һ�����ʴ�Ҫת��һ��
	Rect_2_Screen_Coordinate(Point_1[0], Point_1[1], &Screen_Pos_1[0], &Screen_Pos_1[1]);
	Rect_2_Screen_Coordinate(Point_2[0], Point_2[1], &Screen_Pos_2[0], &Screen_Pos_2[1]);
	Rect_2_Screen_Coordinate(0, 0, &Org[0], &Org[1]);

	//���ߣ����ڹ۲�
	Draw_Line(oImage, Screen_Pos_1[0], Screen_Pos_1[1], Org[0], Org[1]);
	Draw_Line(oImage, Screen_Pos_2[0], Screen_Pos_2[1], Org[0], Org[1]);
		
	bSave_Image("c:\\tmp\\1.bmp", oImage);
	
	Free_Image(&oImage);
}
void Test_2()
{//��ת��������ת����
	typedef float _T;       //�����ľ��ȿ��Ը�Ϊdouble
	{   //��һ�����ӣ���ת������ת����
		_T Rotation_Vector[] = { 1,2,3 };    //��z����ת45��
		_T Rotation_Matrix[3 * 3];
		Rotation_Vector_3_2_Matrix(Rotation_Vector, Rotation_Matrix);
		Disp(Rotation_Matrix, 3, 3, "Rotaion Matrix");

		//�����и��㣬����x����
		_T Point[] = { 1, 0, 0 };
		//��ת�任
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
