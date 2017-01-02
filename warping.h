#pragma warning(disable:4996)
#pragma warning(disable:4244)
#pragma warning(disable:4819)

#ifndef _WARPING_H
#define _WARPING_H
#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <iostream>
#include <fstream>
//#include <omp.h>

#pragma comment(lib, "cv.lib")
#pragma comment(lib, "cxcore.lib")
#pragma comment(lib, "highgui.lib")
#pragma comment(lib, "cvaux.lib")

typedef unsigned char uchar;
using namespace std;
#define cvmMul( src1, src2, dst )       cvMatMulAdd( src1, src2, 0, dst )

#endif

CvPoint2D32f projUVZtoXY(CvMat* p, int u, int v, double z, int height);
void cvexSetRotMatrix(CvMat* rot,double pitch, double roll, double yaw);
void erodebound(IplImage* bound, int flag);
void cvexSetCameraParam(char* filename, CvMat* inMat1, CvMat* exMat1, CvMat* inMat2, CvMat* exMat2, CvMat* inMat3, CvMat* exMat3, char* cam_posi1, char* cam_posi2, char* cam_posi3);
IplImage* cvexLoadImage(char* filename, int iscolor = 1);
void cvexMedian(IplImage* src);
void cvexBilateral(IplImage* dst, int sigma_d, int sigma_c);
void cvexConvertCameraParam(CvMat* inMat, CvMat* exMat, int height);
IplImage* viewsynthesis(IplImage* src_L, IplImage* src_R, IplImage* depth_L, IplImage* depth_R, CvMat* inMat_L, CvMat* exMat_L, CvMat* inMat_R, CvMat* exMat_R, CvMat* inMat_V, CvMat* exMat_V, CvMat* inMat_L_cpy, CvMat* exMat_L_cpy, CvMat* inMat_R_cpy, CvMat* exMat_R_cpy, CvMat* inMat_V_cpy, CvMat* exMat_V_cpy, float Z_near_L, float Z_far_L, float Z_near_R, float Z_far_R, int flag);
