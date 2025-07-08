// Slam_Test_1.cpp : ���ļ����� "main" ����������ִ�н��ڴ˴���ʼ��������
//
#include <iostream>
#include "Common.h"
#include "Matrix.h"

void Test_1()
{
    typedef float _T;       //�����ľ��ȿ��Ը�Ϊdouble
    {   //��һ�����ӣ���ת������ת����
        _T Rotation_Vector[] = { 0,0,1,PI / 4 };    //��z����ת45��
        _T Rotation_Matrix[3 * 3];
        Rotation_Vector_4_2_Matrix(Rotation_Vector, Rotation_Matrix);
        Disp(Rotation_Matrix, 3, 3, "Rotaion Matrix");

        //�����и��㣬����x����
        _T Point[] = { 1, 0, 0 };
        //��ת�任
        Matrix_Multiply(Rotation_Matrix, 3, 3, Point, 1, Point);
        Disp(Point, 3, 1, "Point");
    }

    {//�ڶ������ӣ�ͼʾ��ת������ת�Ƕ�
        _T Rotation_Vector[4] = { 10,20,30 };  //ע�⣬�˴��ȶ���ת�᷽�򣬲�����ת�Ƕ�
        _T R[3 * 3];
        //����һ������
        Point_Cloud<_T> oPC;
        //�������10000��
        Init_Point_Cloud(&oPC, 10000, 1);
        //������ת��
        Draw_Line(&oPC,  -Rotation_Vector[0], -Rotation_Vector[1], -Rotation_Vector[2], Rotation_Vector[0], Rotation_Vector[1], Rotation_Vector[2]);
        
        //����ת���һ��
        Normalize(Rotation_Vector, 3, Rotation_Vector);

        //��һ����������ת����ת
        float i,fTheta = PI;  //��ת180��
        for (i = 0; i < fTheta; i += PI / 100)
        {
            _T Point_3[] = { 15,16,17 };
            Rotation_Vector[3] = i;
            Rotation_Vector_4_2_Matrix(Rotation_Vector, R);
            Matrix_Multiply(R, 3, 3, Point_3,1, Point_3);
            if(i==0)
                Draw_Point(&oPC, Point_3[0], Point_3[1], Point_3[2], 255, 255, 255);
            else
                Draw_Point(&oPC, Point_3[0], Point_3[1], Point_3[2], 255, 0, 0);
        }
        
        bSave_PLY("c:\\tmp\\1.ply", oPC);
        Free_Point_Cloud(&oPC);
    }
}

int main()
{   
    //�����ڴ档���������������ù����У�����Ҫ���Լ����ڴ����
    Init_Env();
    
    Test_1();

    //�ͷ��ڴ�
    Free_Env();

#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
    return 0;
}
