// Slam_Test_1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include "Common.h"
#include "Matrix.h"

void Test_1()
{
    typedef float _T;       //后续的精度可以改为double
    {   //第一个例子，旋转矩阵到旋转矩阵
        _T Rotation_Vector[] = { 0,0,1,PI / 4 };    //绕z轴旋转45度
        _T Rotation_Matrix[3 * 3];
        Rotation_Vector_4_2_Matrix(Rotation_Vector, Rotation_Matrix);
        Disp(Rotation_Matrix, 3, 3, "Rotaion Matrix");

        //假设有个点，落在x轴上
        _T Point[] = { 1, 0, 0 };
        //旋转变换
        Matrix_Multiply(Rotation_Matrix, 3, 3, Point, 1, Point);
        Disp(Point, 3, 1, "Point");
    }

    {//第二个例子，图示旋转轴与旋转角度
        _T Rotation_Vector[4] = { 10,20,30 };  //注意，此处先定旋转轴方向，不管旋转角度
        _T R[3 * 3];
        //声明一个点云
        Point_Cloud<_T> oPC;
        //分配点云10000点
        Init_Point_Cloud(&oPC, 10000, 1);
        //画出旋转轴
        Draw_Line(&oPC,  -Rotation_Vector[0], -Rotation_Vector[1], -Rotation_Vector[2], Rotation_Vector[0], Rotation_Vector[1], Rotation_Vector[2]);
        
        //将旋转轴归一化
        Normalize(Rotation_Vector, 3, Rotation_Vector);

        //用一个点绕着旋转轴旋转
        float i,fTheta = PI;  //旋转180度
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
    //分配内存。在所有真正的商用工程中，必须要有自己的内存管理
    Init_Env();
    
    Test_1();

    //释放内存
    Free_Env();

#ifdef WIN32
    _CrtDumpMemoryLeaks();
#endif
    return 0;
}
