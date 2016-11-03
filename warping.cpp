
#include "log.h"
#include "warping.h"

#include <cv.h>
#include <cvaux.h>
#include <cvwimage.h>
#include <cxcore.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#ifndef NUM_THREADS
#define NUM_THREADS 4
#endif

int IMG_WIDTH;
int IMG_HEIGHT;
IplImage* depth_right;
IplImage* depth_left;
IplImage* udepth_left;
IplImage* udepth_right;

IplImage* src_left_g;
IplImage* src_right_g;
IplImage* dst_left_g;
IplImage* dst_right_g;

CvMat* homog_LV[256];
CvMat* homog_RV[256];
CvMat* homog_VL[256];
CvMat* homog_VR[256];

void* calcVirtImage_thread(void* _tid);
void* calcVirtDepth_thread(void* _tid);
void calcVirtualDepth(uchar* udepth, uchar* udepth2, IplImage* depth_L, IplImage* depth_R,
			CvMat** H_LV, CvMat** H_RV, int width, int height)
{
    /*****  udepth loop 1 *****/
    // update udepth and udepth2 values based on corresponding left/right depth
    // It scans all depth values of left image, looks up corresponding H_lv matrix and
    // update the projected location in udepth
    //
    // It scans all depth values of right image, looks up corresponding H_rv matrix and
    // update the projected location in udepth2
    // TODO: We may optimize the follwoing loop by making average on the fly.
    // Therefore we can save memory by not storing intermediate data to udepth2 and dst2
    int64_t time;
    static float total_t = 0;
    static int count = 1;
#ifdef PTHREAD_V1
    time = cv::getTickCount();
    pthread_t thread[NUM_THREADS];
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    long t;
    void *status;
    int rc;
    for(t=0; t<NUM_THREADS; t++) {
        info_print("Main: creating thread %ld\n", t);
        rc = pthread_create(&thread[t], &attr, calcVirtDepth_thread, (void *)t);
        if (rc) {
            error_print("return code from pthread_create() is %d\n", rc);
            exit(-1);
        }
    }

    /* Free attribute and wait for the other threads */
    pthread_attr_destroy(&attr);
    for(t=0; t<NUM_THREADS; t++) {
        rc = pthread_join(thread[t], &status);
        if (rc) {
            error_print("return code from pthread_join() is %d\n", rc);
            exit(-1);
        }
        info_print("Main: completed join with thread %ld having a status of %ld\n", t, (long)status);
    }

    udepth = (uchar*)udepth_left->imageData;
    udepth2 = (uchar*)udepth_right->imageData;
    total_t +=(float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0;

    timing_print("Udepth loop1 took %f msec, ave %f over %d frames\n",
                (float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0,
		total_t/count,
		count);

    count++;
#else
    // sequential version
    time = cv::getTickCount();
    //uchar* udepth = (uchar*)udepth_left->imageData;
    //uchar* udepth2 = (uchar*)udepth_right->imageData;
    for(int j = 0; j < height; j++)
    {
        CvMat* m = cvCreateMat(3, 1, CV_64F);
        CvMat* mv = cvCreateMat(3, 1, CV_64F);
        for(int i = 0; i < width; i++)
        {
            int pt = i + j * width;
            cvmSet(m, 0, 0, i);
            cvmSet(m, 1, 0, j);
            cvmSet(m, 2, 0, 1);
            uchar val = (uchar)depth_L->imageData[pt];
            cvmMul(H_LV[val], m, mv);
            int u = mv->data.db[0] / mv->data.db[2];
            int v = mv->data.db[1] / mv->data.db[2];
            u = abs(u) % width; // boundary check
            v = abs(v) % height;
            int ptv = u + v * width;
            udepth[ptv] = (udepth[ptv] > val) ? udepth[ptv] : val;

            val = (uchar)depth_R->imageData[pt];
            cvmMul(H_RV[val], m, mv);
            u = mv->data.db[0] / mv->data.db[2];
            v = mv->data.db[1] / mv->data.db[2];
            u = abs(u) % width;
            v = abs(v) % height;
            ptv = u + v * width;
            udepth2[ptv] = (udepth2[ptv] > val) ? udepth2[ptv] : val;
        }
        cvReleaseMat(&m);
        cvReleaseMat(&mv);
    }
    total_t +=(float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0;
    timing_print("Udepth loop1 took %f msec, ave %f over %d frames\n",
                (float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0,
		total_t/count,
		count);
    count++;
#endif
    /******************************************************/
}
// compute virtual image kernel
void* calcVirtDepth_thread(void* _tid) {

    int i, j;
    long tid = (long)_tid;
    info_print("Thread %ld starting...\n", tid);

    int local_h = (int)ceil(IMG_HEIGHT/NUM_THREADS + 1); // row block dim
    int start_row = local_h * tid;                   // start row given tid
    int end_row = std::min(IMG_HEIGHT, (int)(tid+1)*local_h);      // end row given tid (boundary checked if end_row > height)

    uchar* udepth = (uchar*)udepth_left->imageData;
    uchar* udepth2 = (uchar*)udepth_right->imageData;
    CvMat* m = cvCreateMat(3, 1, CV_64F);
    CvMat* mv = cvCreateMat(3, 1, CV_64F);
    for(j = start_row; j < end_row; j++) {
        for(int i = 0; i < IMG_WIDTH; i++) {
            int pt = i + j * IMG_WIDTH;
            cvmSet(m, 0, 0, i);
            cvmSet(m, 1, 0, j);
            cvmSet(m, 2, 0, 1);
            uchar val = (uchar)depth_left->imageData[pt];
            cvmMul(homog_LV[val], m, mv);
            int u = mv->data.db[0] / mv->data.db[2];
            int v = mv->data.db[1] / mv->data.db[2];
            u = abs(u) % IMG_WIDTH; // boundary check
            v = abs(v) % IMG_HEIGHT;
            int ptv = u + v * IMG_WIDTH;
            udepth[ptv] = (udepth[ptv] > val) ? udepth[ptv] : val;

            val = (uchar)depth_right->imageData[pt];
            cvmMul(homog_RV[val], m, mv);
            u = mv->data.db[0] / mv->data.db[2];
            v = mv->data.db[1] / mv->data.db[2];
            u = abs(u) % IMG_WIDTH;
            v = abs(v) % IMG_HEIGHT;
            ptv = u + v * IMG_WIDTH;
            udepth2[ptv] = (udepth2[ptv] > val) ? udepth2[ptv] : val;
        }
    }

    cvReleaseMat(&m);
    cvReleaseMat(&mv);
    pthread_exit((void*) _tid);
}

void calcVirtualImage(IplImage* dst, IplImage* dst2, IplImage* src_L, IplImage* src_R,
                      uchar* udepth, uchar* udepth2,
                      CvMat** H_VL, CvMat** H_VR, int width, int height)
{
    // This loop updates the dst and dst2 that store RGB values of virtual view
    // It uses udepth to find H_vl and use projected location to update dst RGB value
    // It uses udepth2 to find H_vr and use projected location to update dst2 RGB value
    int64_t time;
    static float total_t = 0;
    static int count = 1;
#ifdef PTHREAD_V1
    time = cv::getTickCount();
    pthread_t thread[NUM_THREADS];
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    long t;
    void *status;
    int rc;
    for(t=0; t<NUM_THREADS; t++) {
        info_print("Main: creating thread %ld\n", t);
        rc = pthread_create(&thread[t], &attr, calcVirtImage_thread, (void *)t);
        if (rc) {
            error_print("return code from pthread_create() is %d\n", rc);
            exit(-1);
        }
    }

    /* Free attribute and wait for the other threads */
    pthread_attr_destroy(&attr);
    for(t=0; t<NUM_THREADS; t++) {
        rc = pthread_join(thread[t], &status);
        if (rc) {
            error_print("return code from pthread_join() is %d\n", rc);
            exit(-1);
        }
        info_print("Main: completed join with thread %ld having a status of %ld\n", t, (long)status);
    }

    total_t +=(float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0;

    timing_print("calcVirtImage loop2 took %f msec, ave %f over %d frames\n",
                (float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0,
		total_t/count,
		count);

    count++;
#else
    // sequential version
    time = cv::getTickCount();
    for(int j = 0; j < height; j++)
    {
    CvMat* m = cvCreateMat(3, 1, CV_64F);
    CvMat* mv = cvCreateMat(3, 1, CV_64F);
        for(int i = 0; i < width; i++)
        {
            int ptv = i + j * width;
            mv->data.db[0] = i;
            mv->data.db[1] = j;
            mv->data.db[2] = 1;
            cvmMul(H_VL[udepth[ptv]], mv, m);
            int u = m->data.db[0] / m->data.db[2];
            int v = m->data.db[1] / m->data.db[2];
            u = abs(u) % width;
            v = abs(v) % height;
            int pt = u + v * width;
            for(int k = 0; k < 3; k++){
                dst->imageData[ptv * 3 + k] = src_L->imageData[pt * 3 + k];
            }
            cvmMul(H_VR[udepth2[ptv]], mv, m);
            u = m->data.db[0] / m->data.db[2];
            v = m->data.db[1] / m->data.db[2];
            u = abs(u) % width;
            v = abs(v) % height;
            pt = u + v * width;
            for(int k = 0; k < 3; k++){
                dst2->imageData[ptv * 3 + k] = src_R->imageData[pt * 3 + k];
            }
        }
    cvReleaseMat(&m);
    cvReleaseMat(&mv);
    }
    total_t +=(float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0;
    timing_print("UImage loop2 took %f msec, ave %f over %d frames\n",
                (float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0,
		total_t/count,
		count);
    count++;
#endif
    /******************************************************/
}
// compute virtual depth kernel
void* calcVirtImage_thread(void* _tid) {

    long tid = (long)_tid;
    info_print("Thread %ld starting...\n", tid);

    int local_h = (int)ceil(IMG_HEIGHT/NUM_THREADS + 1); // row block dim
    int start_row = local_h * tid;                   // start row given tid
    int end_row = std::min(IMG_HEIGHT, (int)(tid+1)*local_h);      // end row given tid (boundary checked if end_row > height)

    uchar* udepth = (uchar*)udepth_left->imageData;
    uchar* udepth2 = (uchar*)udepth_right->imageData;
    CvMat* m = cvCreateMat(3, 1, CV_64F);
    CvMat* mv = cvCreateMat(3, 1, CV_64F);
    for(int j = start_row; j < end_row; j++) {
        for(int i = 0; i < IMG_WIDTH; i++) {

            int ptv = i + j * IMG_WIDTH;
            mv->data.db[0] = i;
            mv->data.db[1] = j;
            mv->data.db[2] = 1;
            cvmMul(homog_VL[udepth[ptv]], mv, m);
            int u = m->data.db[0] / m->data.db[2];
            int v = m->data.db[1] / m->data.db[2];
            u = abs(u) % IMG_WIDTH;
            v = abs(v) % IMG_HEIGHT;
            int pt = u + v * IMG_WIDTH;
            for(int k = 0; k < 3; k++)
                dst_left_g->imageData[ptv * 3 + k] = src_left_g->imageData[pt * 3 + k];

            cvmMul(homog_VR[udepth2[ptv]], mv, m);
            u = m->data.db[0] / m->data.db[2];
            v = m->data.db[1] / m->data.db[2];
            u = abs(u) % IMG_WIDTH;
            v = abs(v) % IMG_HEIGHT;
            pt = u + v * IMG_WIDTH;
            for(int k = 0; k < 3; k++)
                dst_right_g->imageData[ptv * 3 + k] = src_right_g->imageData[pt * 3 + k];
        }
    }
    cvReleaseMat(&m);
    cvReleaseMat(&mv);
    pthread_exit((void*) _tid);
}

//convert 3D point to 2D point
CvPoint2D32f projUVZtoXY(CvMat* p, int u, int v, double z, int height)
{
    CvPoint2D32f pt;
    double c0, c1, c2;
    v = height - v - 1;
    c0 = z * cvmGet(p, 0, 2) + cvmGet(p, 0, 3);
    c1 = z * cvmGet(p, 1, 2) + cvmGet(p, 1, 3);
    c2 = z * cvmGet(p, 2, 2) + cvmGet(p, 2, 3);
    pt.y = u * (c1 * cvmGet(p, 2, 0) - cvmGet(p, 1, 0) * c2) +
        v * (c2 * cvmGet(p, 0, 0) - cvmGet(p, 2, 0) * c0) +
        cvmGet(p, 1, 0) * c0 - c1 * cvmGet(p, 0, 0);
    pt.y /= v * (cvmGet(p, 2, 0) * cvmGet(p, 0, 1) - cvmGet(p, 2, 1) * cvmGet(p, 0, 0)) +
        u * (cvmGet(p, 1, 0) * cvmGet(p, 2, 1) - cvmGet(p, 1, 1) * cvmGet(p, 2, 0)) +
        cvmGet(p, 0, 0) * cvmGet(p, 1, 1) - cvmGet(p, 1, 0) * cvmGet(p, 0, 1);
    pt.x = pt.y * (cvmGet(p, 0, 1) - cvmGet(p, 2, 1) * u) + c0 - c2 * u;
    pt.x /= cvmGet(p, 2, 0) * u - cvmGet(p, 0, 0);
    return pt;
}


//rotation matrix
void cvexSetRotMatrix(CvMat* rot,double pitch, double roll, double yaw)
{
    double phi = roll / 180.0 * CV_PI;
    double theta = pitch / 180.0 * CV_PI;
    double pusai = yaw / 180.0 * CV_PI;

    rot->data.db[0] = cos(phi) * cos(theta);
    rot->data.db[1] = cos(phi) * sin(theta) * sin(pusai) - sin(phi) * cos(pusai);
    rot->data.db[2] = cos(phi) * sin(theta) * cos(pusai) + sin(phi) * sin(pusai);

    rot->data.db[3] = sin(phi) * cos(theta);
    rot->data.db[4] = sin(phi) * sin(theta) * sin(pusai) + cos(phi) * cos(pusai);
    rot->data.db[5] = sin(phi) * sin(theta) * cos(pusai) - cos(phi) * sin(pusai);

    rot->data.db[6] = -sin(theta);
    rot->data.db[7] = cos(theta) * sin(pusai);
    rot->data.db[8] = cos(theta) * cos(pusai);
}

//intrinsic and extrinsic parameters
void cvexSetCameraParam(char* filename, CvMat* inMat1, CvMat* exMat1, CvMat* inMat2, CvMat* exMat2, CvMat* inMat3, CvMat* exMat3, char* cam_posi1, char* cam_posi2, char* cam_posi3)
{
    ifstream param(filename);
    if(!param.is_open())
    {
        cout << "can't open " << filename << endl;
        exit(1);
    }

    double val;
    char s1[40], s2[40];
    char* p;

/////////////
    sprintf(s1, "%s", cam_posi1);
    while((p=strstr(s1, s2)) == NULL || (strlen(s1) != strlen(s2)) ){
        param >> s2;
    }

    for(int i = 0; i < 9; i++)
        param >> inMat1->data.db[i];

    param >> val;
    param >> val;

    for(int i = 0; i < 12; i++)
        param >> exMat1->data.db[i];

    param.close();

    param.open(filename);

/////////////
    sprintf(s1, "%s", cam_posi2);
    while((p=strstr(s1, s2)) == NULL || (strlen(s1) != strlen(s2)) ){
        param >> s2;
    }

    for(int i = 0; i < 9; i++)
        param >> inMat2->data.db[i];

    param >> val;
    param >> val;

    for(int i = 0; i < 12; i++)
        param >> exMat2->data.db[i];

    param.close();

    param.open(filename);

/////////////
    sprintf(s1, "%s", cam_posi3);
    while((p=strstr(s1, s2)) == NULL || (strlen(s1) != strlen(s2)) ){
        param >> s2;
    }

    for(int i = 0; i < 9; i++)
        param >> inMat3->data.db[i];

    param >> val;
    param >> val;

    for(int i = 0; i < 12; i++)
        param >> exMat3->data.db[i];
}


IplImage* cvexLoadImage(char* filename, int iscolor)
{
    IplImage* image = cvLoadImage(filename, iscolor);
    if(image == NULL)
    {
        cout << "can't open " << filename << endl;
        exit(1);
    }
    return image;
}

void cvexMedian(IplImage* dst)
{
    IplImage* buf = cvCloneImage(dst);
    cvSmooth(buf, dst, CV_MEDIAN);
    cvReleaseImage(&buf);
}

void cvexBilateral(IplImage* dst, int sigma_d, int sigma_c)
{
    IplImage* buf = cvCloneImage(dst);
    cvSmooth(buf, dst, CV_BILATERAL, sigma_d, sigma_c);
    cvReleaseImage(&buf);
}


void erodebound(IplImage* bound, int flag)
{
    int width = bound->width;
    int height = bound->height;
    uchar *ub = (uchar *)bound->imageData;

    if(flag)
    {
        for(int j = 0; j < height; j++)
        {
            for(int i = 0; i < width; i++)
            {
                int l = i + j * width;
                if(ub[l] == 255)
                {
                    int ll = l;
                    while((ub[ll] == 255) && (i < width))
                    {
                        ub[ll] = 0;
                        ll++;
                        i++;
                    }
                    ub[ll - 1] = 255;
                }
            }
        }

    }
    else
    {
        for(int j = 0; j < height; j++)
        {
            for(int i = 0; i < width; i++)
            {
                int l = i + j * width;
                if(ub[l] == 255)
                {
                    int ll = l;
                    while((ub[ll] == 255) && (i < width))
                    {
                        ub[ll] = 0;
                        ll++;
                        i++;
                    }
                    ub[l] = 255;
                }
            }
        }
    }
}


void cvexConvertCameraParam(CvMat* inMat, CvMat* exMat, int height)
{
    inMat->data.db[4] = -inMat->data.db[4];
    inMat->data.db[5] = height - inMat->data.db[5];
    CvMat* R = cvCreateMat(3, 3, CV_64F);

    for(int j = 0; j < 3; j++)
        for(int i = 0; i < 3; i++)
            R->data.db[i + j * 3] = exMat->data.db[i + j * 4];

    CvMat* t = cvCreateMat(3, 1, CV_64F);

    for(int i = 0; i < 3; i++)
        t->data.db[i] = exMat->data.db[3 + i * 4];

    CvMat* Rt = cvCreateMat(3, 3, CV_64F);
    cvTranspose(R, Rt);
    CvMat* t2 = cvCreateMat(3, 1, CV_64F);
    cvmMul(Rt, t, t2);


    for(int i = 0; i < 3; i++)
        t2->data.db[i] = -t2->data.db[i];

    for(int j = 0; j < 3; j++)
        for(int i = 0; i < 3; i++)
            exMat->data.db[i + j * 4] = Rt->data.db[i + j * 3];

    for(int i = 0; i < 3; i++)
        exMat->data.db[3 + i * 4] = t2->data.db[i];

    cvReleaseMat(&R);
    cvReleaseMat(&t);
    cvReleaseMat(&Rt);
    cvReleaseMat(&t2);
}

void init_synthesis(int width, int height) {
    IMG_WIDTH = width;
    IMG_HEIGHT = height;
    depth_left = cvCreateImage(cvSize(width, height), 8, 1);
    depth_right= cvCreateImage(cvSize(width, height), 8, 1);
    udepth_left = cvCreateImage(cvSize(width, height), 8, 1);
    udepth_right= cvCreateImage(cvSize(width, height), 8, 1);

    src_left_g = cvCreateImage(cvSize(width, height), 8, 3);
    src_right_g = cvCreateImage(cvSize(width, height), 8, 3);
    dst_left_g = cvCreateImage(cvSize(width, height), 8, 3);
    dst_right_g = cvCreateImage(cvSize(width, height), 8, 3);

    for(int i = 0; i < 256; i++) {
        homog_LV[i] = cvCreateMat(3, 3, CV_64F);
        homog_RV[i] = cvCreateMat(3, 3, CV_64F);
        homog_VL[i] = cvCreateMat(3, 3, CV_64F);
        homog_VR[i] = cvCreateMat(3, 3, CV_64F);
    }

    // initialize with 0
    uchar* udepth = (uchar*)udepth_left->imageData;
    uchar* udepth2 = (uchar*)udepth_right->imageData;
    for(int ptv = 0; ptv < width * height; ptv++){
        udepth[ptv]  = 0;
        udepth2[ptv] = 0;
    }
}

IplImage* viewsynthesis(IplImage* src_L, IplImage* src_R, IplImage* depth_L, IplImage* depth_R, CvMat* inMat_L, CvMat* exMat_L, CvMat* inMat_R, CvMat* exMat_R, CvMat* inMat_V, CvMat* exMat_V, CvMat* inMat_L_cpy, CvMat* exMat_L_cpy, CvMat* inMat_R_cpy, CvMat* exMat_R_cpy, CvMat* inMat_V_cpy, CvMat* exMat_V_cpy, float Z_near_L, float Z_far_L, float Z_near_R, float Z_far_R, int flag)
{
    static float total_t = 0;
    static int count = 1;
    int64_t time = cv::getTickCount();
    int width = src_L->width;
    int height = src_R->height;
    init_synthesis(width, height);

    depth_left = cvCloneImage(depth_L);
    depth_right = cvCloneImage(depth_R);
    src_left_g = cvCloneImage(src_L);
    src_right_g = cvCloneImage(src_R);

    IplImage* dst = cvCreateImage(cvSize(width, height), 8, 3);
    IplImage* dst2 = cvCreateImage(cvSize(width, height), 8, 3);
    IplImage* depth_V = cvCreateImage(cvSize(width, height), 8, 1);
    IplImage* depth_V2 = cvCreateImage(cvSize(width, height), 8, 1);

    for (int i=0; i<9; i++)
        inMat_L_cpy->data.db[i] = inMat_L->data.db[i];

    for (int i=0; i<12; i++)
        exMat_L_cpy->data.db[i] = exMat_L->data.db[i];

    for (int i=0; i<9; i++)
        inMat_R_cpy->data.db[i] = inMat_R->data.db[i];

    for (int i=0; i<12; i++)
        exMat_R_cpy->data.db[i] = exMat_R->data.db[i];

    for (int i=0; i<9; i++)
        inMat_V_cpy->data.db[i] = inMat_V->data.db[i];

    for (int i=0; i<12; i++)
        exMat_V_cpy->data.db[i] = exMat_V->data.db[i];

    cvexConvertCameraParam(inMat_L_cpy, exMat_L_cpy, height);
    cvexConvertCameraParam(inMat_R_cpy, exMat_R_cpy, height);
    cvexConvertCameraParam(inMat_V_cpy, exMat_V_cpy, height);

    CvMat* P_L = cvCreateMat(3, 4, CV_64F);
    CvMat* P_R = cvCreateMat(3, 4, CV_64F);
    CvMat* P_V = cvCreateMat(3, 4, CV_64F);

    cvmMul(inMat_R_cpy, exMat_R_cpy, P_R);  // P_R = inMat_R * exMat_R
    cvmMul(inMat_L_cpy, exMat_L_cpy, P_L);  // P_L = inMat_L * exMat_L
    cvmMul(inMat_V_cpy, exMat_V_cpy, P_V);  // P_V = inMat_V * exMat_V

    CvMat* H_LV[256];
    CvMat* H_RV[256];
    CvMat* H_VL[256];
    CvMat* H_VR[256];

    for(int i = 0; i < 256; i++)
    {
        H_LV[i] = cvCreateMat(3, 3, CV_64F);
        H_RV[i] = cvCreateMat(3, 3, CV_64F);
        H_VL[i] = cvCreateMat(3, 3, CV_64F);
        H_VR[i] = cvCreateMat(3, 3, CV_64F);
    }

    double depth_table_L[256];
    double depth_table_R[256];

    if(flag == 1){
        for(int i = 0; i < 256; i++)
            depth_table_L[i] = 1.0 / ((i / 255.0) * (1.0 / Z_near_L - 1.0 / Z_far_L) + 1.0 / Z_far_L);

        for(int i = 0; i < 256; i++)
            depth_table_R[i] = 1.0 / ((i / 255.0) * (1.0 / Z_near_R - 1.0 / Z_far_R) + 1.0 / Z_far_R);
    }

    if(flag == 0){
        for(int i = 0; i < 256; i++)
            depth_table_L[i] = exMat_L_cpy->data.db[11] - 1.0 / ((i / 255.0) * (1.0 / Z_near_L - 1.0 / Z_far_L) + 1.0 / Z_far_L);

        for(int i = 0; i < 256; i++)
            depth_table_R[i] = exMat_R_cpy->data.db[11] - 1.0 / ((i / 255.0) * (1.0 / Z_near_R - 1.0 / Z_far_R) + 1.0 / Z_far_R);
    }


#pragma omp parallel for
    for(int z = 0; z < 256; z++)
    {
        int u, v;
        CvMat* src_points = cvCreateMat(4, 2, CV_64F);
        CvMat* dst_points = cvCreateMat(4, 2, CV_64F);
        CvPoint2D32f pt;
        CvMat* m = cvCreateMat(3, 1, CV_64F);
        CvMat* M = cvCreateMat(4, 1, CV_64F);

        u = 0;
        v = 0;
        pt = projUVZtoXY(P_L, u, v, depth_table_L[z], height);
        cvmSet(M, 0, 0, pt.x);
        cvmSet(M, 1, 0, pt.y);
        cvmSet(M, 2, 0, depth_table_L[z]);
        cvmSet(M, 3, 0, 1);
        cvmMul(P_V, M, m);
        pt.x = cvmGet(m, 0, 0) / cvmGet(m, 2, 0);
        pt.y = height - cvmGet(m, 1, 0) / cvmGet(m, 2, 0) - 1;
        cvmSet(src_points, 0, 0, u);
        cvmSet(src_points, 0, 1, v);
        cvmSet(dst_points, 0, 0, pt.x);
        cvmSet(dst_points, 0, 1, pt.y);

        u = 0;
        v = height - 1;
        pt = projUVZtoXY(P_L, u, v, depth_table_L[z], height);
        cvmSet(M, 0, 0, pt.x);
        cvmSet(M, 1, 0, pt.y);
        cvmSet(M, 2, 0, depth_table_L[z]);
        cvmSet(M, 3, 0, 1);
        cvmMul(P_V, M, m);
        pt.x = cvmGet(m, 0, 0) / cvmGet(m, 2, 0);
        pt.y = height - cvmGet(m, 1, 0) / cvmGet(m, 2, 0);
        cvmSet(src_points, 1, 0, u);
        cvmSet(src_points, 1, 1, v);
        cvmSet(dst_points, 1, 0, pt.x);
        cvmSet(dst_points, 1, 1, pt.y);

        u = width - 1;
        v = height - 1;
        pt = projUVZtoXY(P_L, u, v, depth_table_L[z], height);
        cvmSet(M, 0, 0, pt.x);
        cvmSet(M, 1, 0, pt.y);
        cvmSet(M, 2, 0, depth_table_L[z]);
        cvmSet(M, 3, 0, 1);
        cvmMul(P_V, M, m);
        pt.x = cvmGet(m, 0, 0) / cvmGet(m, 2, 0);
        pt.y = height - cvmGet(m, 1, 0) / cvmGet(m, 2, 0);
        cvmSet(src_points, 2, 0, u);
        cvmSet(src_points, 2, 1, v);
        cvmSet(dst_points, 2, 0, pt.x);
        cvmSet(dst_points, 2, 1, pt.y);

        u = width - 1;
        v = 0;
        pt = projUVZtoXY(P_L, u, v, depth_table_L[z], height);
        cvmSet(M, 0, 0, pt.x);
        cvmSet(M, 1, 0, pt.y);
        cvmSet(M, 2, 0, depth_table_L[z]);
        cvmSet(M, 3, 0, 1);
        cvmMul(P_V, M, m);
        pt.x = cvmGet(m, 0, 0) / cvmGet(m, 2, 0);
        pt.y = height - cvmGet(m, 1, 0) / cvmGet(m, 2, 0);
        cvmSet(src_points, 3, 0, u);
        cvmSet(src_points, 3, 1, v);
        cvmSet(dst_points, 3, 0, pt.x);
        cvmSet(dst_points, 3, 1, pt.y);

        cvFindHomography(src_points, dst_points, H_LV[z]);

        u = 0;
        v = 0;
        pt = projUVZtoXY(P_R, u, v, depth_table_R[z], height);
        cvmSet(M, 0, 0, pt.x);
        cvmSet(M, 1, 0, pt.y);
        cvmSet(M, 2, 0, depth_table_R[z]);
        cvmSet(M, 3, 0, 1);
        cvmMul(P_V, M, m);
        pt.x = cvmGet(m, 0, 0) / cvmGet(m, 2, 0);
        pt.y = height - cvmGet(m, 1, 0) / cvmGet(m, 2, 0);
        cvmSet(dst_points, 0, 0, pt.x);
        cvmSet(dst_points, 0, 1, pt.y);

        u = 0;
        v = height - 1;
        pt = projUVZtoXY(P_R, u, v, depth_table_R[z], height);
        cvmSet(M, 0, 0, pt.x);
        cvmSet(M, 1, 0, pt.y);
        cvmSet(M, 2, 0, depth_table_R[z]);
        cvmSet(M, 3, 0, 1);
        cvmMul(P_V, M, m);
        pt.x = cvmGet(m, 0, 0) / cvmGet(m, 2, 0);
        pt.y = height - cvmGet(m, 1, 0) / cvmGet(m, 2, 0);
        cvmSet(dst_points, 1, 0, pt.x);
        cvmSet(dst_points, 1, 1, pt.y);

        u = width - 1;
        v = height - 1;
        pt = projUVZtoXY(P_R, u, v, depth_table_R[z], height);
        cvmSet(M, 0, 0, pt.x);
        cvmSet(M, 1, 0, pt.y);
        cvmSet(M, 2, 0, depth_table_R[z]);
        cvmSet(M, 3, 0, 1);
        cvmMul(P_V, M, m);
        pt.x = cvmGet(m, 0, 0) / cvmGet(m, 2, 0);
        pt.y = height - cvmGet(m, 1, 0) / cvmGet(m, 2, 0);
        cvmSet(dst_points, 2, 0, pt.x);
        cvmSet(dst_points, 2, 1, pt.y);

        u = width - 1;
        v = 0;
        pt = projUVZtoXY(P_R, u, v, depth_table_R[z], height);
        cvmSet(M, 0, 0, pt.x);
        cvmSet(M, 1, 0, pt.y);
        cvmSet(M, 2, 0, depth_table_R[z]);
        cvmSet(M, 3, 0, 1);
        cvmMul(P_V, M, m);
        pt.x = cvmGet(m, 0, 0) / cvmGet(m, 2, 0);
        pt.y = height - cvmGet(m, 1, 0) / cvmGet(m, 2, 0);
        cvmSet(dst_points, 3, 0, pt.x);
        cvmSet(dst_points, 3, 1, pt.y);

        cvFindHomography(src_points, dst_points, H_RV[z]);

        cvReleaseMat(&src_points);
        cvReleaseMat(&dst_points);
        cvReleaseMat(&m);
        cvReleaseMat(&M);
    }
    for(int z = 0; z < 256; z++)
    {
        cvInvert(H_LV[z], H_VL[z], CV_LU);
        cvInvert(H_RV[z], H_VR[z], CV_LU);
    }

    // copy H_LV and H_RV to global memory
    for (int i = 0; i < 256; i++) {
        homog_LV[i] = cvCloneMat(H_LV[i]);
        homog_RV[i] = cvCloneMat(H_RV[i]);
        homog_VL[i] = cvCloneMat(H_VL[i]);
        homog_VR[i] = cvCloneMat(H_VR[i]);
    }

    // udepth <-- virtual view depth computed from left view
    // udepth2 <-- virtual view depth computed from right view
    uchar* udepth = (uchar*)depth_V->imageData;
    uchar* udepth2 = (uchar*)depth_V2->imageData;
    // initialize with 0
    for(int ptv = 0; ptv < width * height; ptv++){
        udepth[ptv]  = 0;
        udepth2[ptv] = 0;
    }


    calcVirtualDepth(udepth, udepth2, depth_L, depth_R, H_LV, H_RV, width, height);
    int sigma_d = 20;
    int sigma_c = 50;

    //TODO:
    // Remove Mediand and Bilat filters and check if the result is identical to when
    // it is given left and right images as is.
    cvexMedian(depth_V);
    cvexBilateral(depth_V, sigma_d, sigma_c);
    cvexMedian(depth_V2);
    cvexBilateral(depth_V2, sigma_d, sigma_c);

    calcVirtualImage(dst, dst2, src_L, src_R, udepth, udepth2, H_VL, H_VR, width, height);
    dst = cvCloneImage(dst_left_g);
    dst2 = cvCloneImage(dst_right_g);

    // This loop updates the dst and dst2 that store RGB values of virtual view
    // It uses udepth to find H_vl and use projected location to update dst RGB value
    // It uses udepth2 to find H_vr and use projected location to update dst2 RGB value
//#pragma omp parallel for
    //for(int j = 0; j < height; j++)
    //{
        //CvMat* m = cvCreateMat(3, 1, CV_64F);
        //CvMat* mv = cvCreateMat(3, 1, CV_64F);
        //for(int i = 0; i < width; i++)
        //{
            //int ptv = i + j * width;
            //mv->data.db[0] = i;
            //mv->data.db[1] = j;
            //mv->data.db[2] = 1;
            //cvmMul(H_VL[udepth[ptv]], mv, m);
            //int u = m->data.db[0] / m->data.db[2];
            //int v = m->data.db[1] / m->data.db[2];
            //u = abs(u) % width;
            //v = abs(v) % height;
            //int pt = u + v * width;
            //for(int k = 0; k < 3; k++)
                //dst->imageData[ptv * 3 + k] = src_L->imageData[pt * 3 + k];

            //cvmMul(H_VR[udepth2[ptv]], mv, m);
            //u = m->data.db[0] / m->data.db[2];
            //v = m->data.db[1] / m->data.db[2];
            //u = abs(u) % width;
            //v = abs(v) % height;
            //pt = u + v * width;
            //for(int k = 0; k < 3; k++)
                //dst2->imageData[ptv * 3 + k] = src_R->imageData[pt * 3 + k];
        //}
        //cvReleaseMat(&m);
        //cvReleaseMat(&mv);
    //}

    IplImage* mask = cvCreateImage(cvSize(width, height), 8, 1);
    IplImage* mask2 = cvCreateImage(cvSize(width, height), 8, 1);
    IplImage* inpaint_mask = cvCreateImage(cvSize(width, height), 8, 1);
    cvThreshold( depth_V,  mask, 5, 255, CV_THRESH_BINARY_INV);
    cvThreshold(depth_V2, mask2, 5, 255, CV_THRESH_BINARY_INV);
    cvAnd(mask, mask2, inpaint_mask);
    IplImage* bound = cvCloneImage(mask);
    IplImage* bound2 = cvCloneImage(mask2);
    erodebound(bound, flag);
    erodebound(bound2, flag);
    cvDilate(bound, bound);
    cvDilate(bound2, bound2);
    cvDilate(bound, bound);
    cvDilate(bound2, bound2);
    cvOr(mask, bound, mask);
    cvOr(mask2, bound2, mask2);
    cvCopy(dst2, dst, mask);
    cvCopy(dst, dst2, mask2);

    // dist_LV is distance from camera Left to Virt computed from extrinsic matrix
    double dist_LV = (exMat_L_cpy->data.db[3] - exMat_V_cpy->data.db[3]) * (exMat_L_cpy->data.db[3] - exMat_V_cpy->data.db[3]) + (exMat_L_cpy->data.db[7] - exMat_V_cpy->data.db[7]) * (exMat_L_cpy->data.db[7] - exMat_V_cpy->data.db[7]) + (exMat_L_cpy->data.db[11] - exMat_V_cpy->data.db[11]) * (exMat_L_cpy->data.db[11] - exMat_V_cpy->data.db[11]);

    // dist_RV is distance from camera Right to Virt computed from extrinsic matrix
    double dist_RV = (exMat_R_cpy->data.db[3] - exMat_V_cpy->data.db[3]) * (exMat_R_cpy->data.db[3] - exMat_V_cpy->data.db[3]) + (exMat_R_cpy->data.db[7] - exMat_V_cpy->data.db[7]) * (exMat_R_cpy->data.db[7] - exMat_V_cpy->data.db[7]) + (exMat_R_cpy->data.db[11] - exMat_V_cpy->data.db[11]) * (exMat_R_cpy->data.db[11] - exMat_V_cpy->data.db[11]);

    dist_LV = sqrt(dist_LV);
    dist_RV = sqrt(dist_RV);

    double alpha = dist_LV / (dist_LV + dist_RV);
    // Update final dst based on dst and dst2 and average
    // them according to their distance from virtual camera
    cvAddWeighted(dst, 1-alpha, dst2, alpha, 0, dst2);
    cvInpaint(dst2, inpaint_mask, dst, 5, CV_INPAINT_NS);


    cvReleaseMat(&inMat_L_cpy);
    cvReleaseMat(&exMat_L_cpy);
    cvReleaseMat(&inMat_R_cpy);
    cvReleaseMat(&exMat_R_cpy);
    cvReleaseMat(&inMat_V_cpy);
    cvReleaseMat(&exMat_V_cpy);

    cvReleaseImage(&depth_V);
    cvReleaseImage(&depth_V2);
    cvReleaseImage(&dst2);
    cvReleaseImage(&mask);
    cvReleaseImage(&mask2);
    cvReleaseImage(&bound);
    cvReleaseImage(&bound2);
    cvReleaseImage(&inpaint_mask);
    cvReleaseMat(&P_L);
    cvReleaseMat(&P_R);
    cvReleaseMat(&P_V);

    for(int i = 0; i < 256; i++)
    {
        cvReleaseMat(&H_LV[i]);
        cvReleaseMat(&H_RV[i]);
        cvReleaseMat(&H_VL[i]);
        cvReleaseMat(&H_VR[i]);
    }
    total_t +=(float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0;

    timing_print("viewsynthesis took %f msec, ave %f over %d frames\n",
                (float)(cv::getTickCount() - time)/cv::getTickFrequency() *1000.0,
		total_t/count,
		count);

    count++;
    return dst;
}

