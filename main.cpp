#include "log.h"
#include "warping.h"
#include "yuv.h"

#include <string.h>
#include <stdio.h>

#ifndef LOGLEVEL
//#define LOGLEVEL (ZONE_ERROR | ZONE_WARNING | ZONE_INFO | ZONE_TIME)
#define LOGLEVEL (ZONE_ERROR | ZONE_TIME)
#endif

/* logs are also saved to a file if specified */
#ifndef LOG_NAME
#define LOG_NAME "logs.log"  // with double quotation mark
#endif

unsigned logLevel = LOGLEVEL;
FILE* logFile = NULL;


int main(int argc, char *argv[])
{
#ifdef LOG_NAME
    logFile = fopen(LOG_NAME, "w");
    info_print("Logs are outputted to %s.\n", LOG_NAME);
#endif

	int WIDTH ;                 //width of image
	int HEIGHT;                 //height of image

	double Z_near_L;              //z_near value of left view from camera or the origin of 3D space
	double Z_far_L;               //z_far value of left view from camera or the origin of 3D space

	double Z_near_R;              //z_near value of right view from camera or the origin of 3D space
	double Z_far_R;               //z_far value of right view from camera or the origin of 3D space

	char name_src_L[128];        //left image
	char name_src_R[128];        //right image

	char name_depth_L[128];      //left depth map
	char name_depth_R[128];      //right depth map

	char name_out[128];          //output file name

	char name_cam_L[128];        //left camera
	char name_cam_V[128];        //virtual camera
	char name_cam_R[128];        //right camera

	char name_cam_param[128];    //name of file which includes camera parameters
	int flag;                   //depth type
//	char name_view_setting[64]; //name of file which points virtual view points

	int num_frame;              //the total number of frames

	int n;                      //frame counter



	FILE *cam_param;
	FILE *view_setting;

	char cfg_path[80], *ppath = cfg_path;

	strcpy(cfg_path, argv[1]);
//	ppath = strrchr(cfg_path, '.');
//	ppath++;
//	strcpy(&*ppath, "cfg");

	CvMat* inMat_L = cvCreateMat(3, 3, CV_64F); //intrinsic parameter of left camera 3x3 matrix
	CvMat* exMat_L = cvCreateMat(3, 4, CV_64F); //extrinsic parameter of left camera 3x4 matrix
	CvMat* inMat_R = cvCreateMat(3, 3, CV_64F); //intrinsic parameter of right camera 3x3 matrix
	CvMat* exMat_R = cvCreateMat(3, 4, CV_64F); //extrinsic parameter of right camera 3x4 matrix
	CvMat* inMat_V = cvCreateMat(3, 3, CV_64F); //intrinsic parameter of virtual camera 3x3 matrix
	CvMat* exMat_V = cvCreateMat(3, 4, CV_64F); //extrinsic parameter of virtual camera 3x4 matrix

	if((view_setting=fopen(cfg_path, "r")) == NULL){
		error_print("can not open the view setting file\n");
		exit(1);
	} else {
		fscanf(view_setting, "%d", &flag);

		fscanf(view_setting, "%d", &WIDTH);
		fscanf(view_setting, "%d", &HEIGHT);

		info_print("Width of view image = %d\n", WIDTH);
		info_print("Height of view image = %d\n", HEIGHT);

		fscanf(view_setting, "%lf", &Z_near_L);
		fscanf(view_setting, "%lf", &Z_far_L);

		fscanf(view_setting, "%lf", &Z_near_R);
		fscanf(view_setting, "%lf", &Z_far_R);

		fscanf(view_setting, "%d", &num_frame);
		info_print("Total number of frames: %d\n", num_frame);

		fscanf(view_setting, "%s", name_cam_param);

		fscanf(view_setting, "%s", name_cam_L);
		fscanf(view_setting, "%s", name_cam_V);
		fscanf(view_setting, "%s", name_cam_R);

		info_print("View point %s \n", name_cam_V);

		if((cam_param = fopen(name_cam_param, "r")) == NULL)
			printf("can not open the param file\n");
		else {
			cvexSetCameraParam(name_cam_param, inMat_L, exMat_L, inMat_V, exMat_V, inMat_R, exMat_R, name_cam_L, name_cam_V, name_cam_R);  //右・仮想視点・左画像のカメラパラメータの読み込み
		}

		fscanf(view_setting, "%s", name_src_L);
		fscanf(view_setting, "%s", name_src_R);
		fscanf(view_setting, "%s", name_depth_L);
		fscanf(view_setting, "%s", name_depth_R);
		fscanf(view_setting, "%s", name_out);

		if(flag==1){
			info_print("Z_near of left view from origin of 3D space = %f \n", Z_near_L);
			info_print("Z_far of left view from origin of 3D space = %f \n", Z_far_L);

			info_print("Z_near of right view from origin of 3D space = %f \n", Z_near_R);
			info_print("Z_far of right view from origin of 3D space = %f \n", Z_far_R);
		}
		if(flag==0){
			info_print("Z_near of left view from camera = %f \n", Z_near_L);
			info_print("Z_near of left view from origin of 3D space = %f \n", exMat_L->data.db[11] - Z_near_L);
			info_print("Z_far of left view from camera = %f \n", Z_far_L);
			info_print("Z_far of left view from origin of 3D space = %f \n", exMat_L->data.db[11] - Z_far_L);

			info_print("Z_near of right view from camera = %f \n", Z_near_R);
			info_print("Z_near of right view from origin of 3D space = %f \n", exMat_R->data.db[11] - Z_near_R);
			info_print("Z_far of right view from camera = %f \n", Z_far_L);
			info_print("Z_far of right view from origin of 3D space = %f \n", exMat_R->data.db[11] - Z_far_R);
		}

		IplImage* src_R = cvCreateImage(cvSize(WIDTH, HEIGHT), 8, 3);
		IplImage* src_L = cvCreateImage(cvSize(WIDTH, HEIGHT), 8, 3);
		IplImage* depth_R = cvCreateImage(cvSize(WIDTH, HEIGHT), 8, 1);
		IplImage* depth_L = cvCreateImage(cvSize(WIDTH, HEIGHT), 8, 1);

		Image *im_src_R;
		Image *im_src_L;
		Image *im_depth_R;
		Image *im_depth_L;

		im_src_R = new_image(WIDTH, HEIGHT, 2);
		im_src_L = new_image(WIDTH, HEIGHT, 2);
		im_depth_R = new_image(WIDTH, HEIGHT, 2);
		im_depth_L = new_image(WIDTH, HEIGHT, 2);

		FILE *fpi_l, *fpi_r;
		FILE *fpd_l, *fpd_r;
		FILE *fpo;

		long position = 0L;

		unsigned char **IY, **IU, **IV;


		IY = ByteAlloc (WIDTH, HEIGHT);
		IU = ByteAlloc (WIDTH/2, HEIGHT/2);
		IV = ByteAlloc (WIDTH/2, HEIGHT/2);

		IplImage *dst;


		if((fpi_l=fopen(name_src_L, "rb"))==NULL){
			error_print("Can't open %s.\n", name_src_L);
			exit (1);
		}
		if((fpi_r=fopen(name_src_R, "rb"))==NULL){
			error_print("Can't open %s.\n", name_src_R);
			exit (1);
		}
		if((fpd_l=fopen(name_depth_L, "rb"))==NULL){
			error_print("Can't open %s.\n", name_depth_L);
			exit (1);
		}
		if((fpd_r=fopen(name_depth_R, "rb"))==NULL){
			error_print("Can't open %s.\n", name_depth_R);
			exit (1);
		}
		if((fpo=fopen(name_out, "wb"))==NULL){
			error_print("Can't open %s.\n", name_out);
			exit (1);
		}
			for (n = 0; n < num_frame; n++) {

				info_print("frame number = %d \n", n);

				if(fseek(fpi_l, position, SEEK_SET)){
					exit(1);
				} else {
					fread(im_src_L->data, sizeof(unsigned char), 3 * WIDTH * (HEIGHT/2), fpi_l);
				}
				if(fseek(fpi_r, position, SEEK_SET)){
					exit(1);
				} else {
					fread(im_src_R->data, sizeof(unsigned char), 3 * WIDTH * (HEIGHT/2), fpi_r);
				}

				if(fseek(fpd_l, position, SEEK_SET)){
					exit(1);
				} else {
					fread(im_depth_L->data, sizeof(unsigned char), 3 * WIDTH * (HEIGHT/2), fpd_l);
				}
				if(fseek(fpd_r, position, SEEK_SET)){
					exit(1);
				} else {
					fread(im_depth_R->data, sizeof(unsigned char), 3 * WIDTH * (HEIGHT/2), fpd_r);
				}

				position = ftell(fpi_l);

				putchar('\n');

/* ***************************************************

       Convert YUV(YCbCr) 4:2:0 to BGR

*************************************************** */

				YUV2BGR(im_src_R, src_R);
				YUV2BGR(im_src_L, src_L);

				for(int i = 0 ; i < HEIGHT; i++){
					for(int j = 0 ; j < WIDTH; j++){
						depth_L->imageData[i * WIDTH + j] = im_depth_L->data[i * WIDTH + j];
					}
				}

				for(int i = 0 ; i < HEIGHT; i++){
					for(int j = 0 ; j < WIDTH; j++){
						depth_R->imageData[i * WIDTH + j] = im_depth_R->data[i * WIDTH + j];
					}
				}


				CvMat* inMat_L_cpy = cvCreateMat(3, 3, CV_64F); //intrinsic parameter of left camera 3x3 matrix
				CvMat* exMat_L_cpy = cvCreateMat(3, 4, CV_64F); //extrinsic parameter of left camera 3x4 matrix
				CvMat* inMat_R_cpy = cvCreateMat(3, 3, CV_64F); //intrinsic parameter of right camera 3x3 matrix
				CvMat* exMat_R_cpy = cvCreateMat(3, 4, CV_64F); //extrinsic parameter of right camera 3x4 matrix
				CvMat* inMat_V_cpy = cvCreateMat(3, 3, CV_64F); //intrinsic parameter of virtual camera 3x3 matrix
				CvMat* exMat_V_cpy = cvCreateMat(3, 4, CV_64F); //extrinsic parameter of virtual camera 3x4 matrix

				dst = viewsynthesis(src_L, src_R, depth_L, depth_R, inMat_L, exMat_L, inMat_R, exMat_R, inMat_V, exMat_V, inMat_L_cpy, exMat_L_cpy, inMat_R_cpy, exMat_R_cpy, inMat_V_cpy, exMat_V_cpy, Z_near_L, Z_far_L, Z_near_R, Z_far_R, flag); //view generation

/* ***************************************************

       Convert BGR to YUV(YCbCr) 4:2:0

*************************************************** */

                cvShowImage("output", dst);
                cvWaitKey(0);

				BGR2YUV(dst, IY, IU, IV);

				for (int i = 0; i < HEIGHT; i++) {
					for (int j = 0; j < WIDTH; j++) {
						fputc(IY[i][j], fpo);
					}
				}

				for (int i = 0; i < HEIGHT/2; i++) {
					for (int j = 0; j < WIDTH/2; j++) {
						fputc(IU[i][j], fpo);
					}
				}

				for (int i = 0; i < HEIGHT/2; i++) {
					for (int j = 0; j < WIDTH/2; j++) {
						fputc(IV[i][j], fpo);
					}
				}

				info_print("Counter = %d \n", n);
				putchar('\n');

			} // for n

		cvReleaseImage(&dst);
		cvReleaseImage(&src_L);
		cvReleaseImage(&src_R);
		cvReleaseImage(&depth_L);
		cvReleaseImage(&depth_R);

		free_image(im_src_R);
		free_image(im_src_L);
		free_image(im_depth_R);
		free_image(im_depth_L);

		fclose(fpi_l);
		fclose(fpi_r);
		fclose(fpd_l);
		fclose(fpd_r);
		fclose(fpo);
}


	cvReleaseMat(&inMat_L);
	cvReleaseMat(&exMat_L);
	cvReleaseMat(&inMat_R);
	cvReleaseMat(&exMat_R);
	cvReleaseMat(&inMat_V);
	cvReleaseMat(&exMat_V);
	fclose(cam_param);
	fclose(view_setting);

    return 0;
}

