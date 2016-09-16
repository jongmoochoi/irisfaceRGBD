/*
  Copyright (c) 2010 Toru Tamaki

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without
  restriction, including without limitation the rights to use,
  copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <list>

//cude

#include "cudaMem.h"
#include <device_launch_parameters.h>


#include "3dregistration.h"
#include "engine.h"
#include "rply.h"

#include "stdafx.h"
#include "FaceBox.h"
#include "FaceDetector.h"
#include "Voxel.h"
#include "Modeling.h"
#include "openNI_general.h"
#include "CudaMem.h"

#include "poseEstimation.h"

#include "cublas.h"

#include <pthread.h>

#include <opencv2/gpu/gpu.hpp>

// dlib ,
#include "facePoseEstimation.h"
#include "FaceDetectLandmark.h"


std::vector<cv::Point2f> landmarkPts; //landmark
extern void initExpr(const string configFIle);
extern void process(cv::Mat &R, cv::Mat &T); // pose estimation
Voxel CENTROID;					// Current centroid of the points
float NORMALIZATION_FACTOR = 1.0f;// First factor used for the face normalization: used for alignement
cv::Mat R, T;								//rotation and translation matrix from pose estimation
void printPointClouds(float *h_x, int nb_X, char *filename); // print point cloud
float R_matrix[9];
float eyebrow_x;
float eyebrow_y;
float eyebrow_y_real;

bool oldversion;	// previous version
bool landmark;	    // version using landmark.
bool pose_estimation_R; //new version.
static int counts = 0;
using namespace std;


/**********************/
void init_RT(float *h_R, float *h_t){
	
  // set to Identity matrix
  h_R[0] = 1.0f;
  h_R[1] = 0.0f;
  h_R[2] = 0.0f;
  h_R[3] = 0.0f;
  h_R[4] = 1.0f;
  h_R[5] = 0.0f;
  h_R[6] = 0.0f;
  h_R[7] = 0.0f;
  h_R[8] = 1.0f;

  h_t[0] = 0.0f;
  h_t[1] = 0.0f;
  h_t[2] = 0.0f;
}
/**********************/


/**********************************************/
/* Create a rotation matrix from euler angles	*/
void eulerAngles2rotationMatrix(CvMat *out, float a, float b, float c) {
		
	CvMat *R1	= cvCreateMat(3,3,CV_64FC1);
	CvMat *R2	= cvCreateMat(3,3,CV_64FC1);
	CvMat *R3	= cvCreateMat(3,3,CV_64FC1);
	CvMat *R3R2 = cvCreateMat(3,3,CV_64FC1);

	float cosA = cos(a);
	float cosB = cos(b);
	float cosC = cos(c);
	float sinA = sin(a);
	float sinB = sin(b);
	float sinC = sin(c);

	cvSetIdentity(R1); 
	cvSetIdentity(R2); 
	cvSetIdentity(R3);
	
	if (	R1->rows==3 && R1->cols==3 
		&&	R2->rows==3 && R2->cols==3
		&&	R3->rows==3 && R3->cols==3) {
		cvmSet(R1,1,1,cosA);
		cvmSet(R1,2,2,cosA);
		cvmSet(R1,1,2,-sinA);
		cvmSet(R1,2,1,sinA);

		cvmSet(R2,0,0,cosB);
		cvmSet(R2,2,2,cosB);
		cvmSet(R2,0,2,sinB);
		cvmSet(R2,2,0,-sinB);

		cvmSet(R3,0,0,cosC);
		cvmSet(R3,1,1,cosC);
		cvmSet(R3,0,1,-sinC);
		cvmSet(R3,1,0,sinC);
	} else {
		fprintf(stderr, "Wrong size for the rotation matrix\n");
		exit(4);
	}

	if (	out->rows==3 && out->cols==3
		&&	R3R2->rows==3 && R3R2->cols==3) {
		cvMatMul(R3,R2,R3R2);
		cvMatMul(R3R2,R1,out);
	}
	else {
		fprintf(stderr, "Wrong size for the rotation matrix\n");
		exit(4);
	}

	cvReleaseMat(&R1);
	cvReleaseMat(&R2);
	cvReleaseMat(&R3);
	cvReleaseMat(&R3R2);
}
/**********************/


/**********************************/
/* Find if two matrices are equal	*/
void displayRendering(IplImage *img, float *h_R, CvPoint pt, float RScale) {
	double Xx, Xy, Yx, Yy, Zx, Zy;
	const double SCALE = 20;
	
	Xx = pt.x+SCALE*RScale*h_R[0];	Xy = pt.y-SCALE*RScale*h_R[3];
	Yx = pt.x+SCALE*RScale*h_R[1];	Yy = pt.y-SCALE*RScale*h_R[4];
	Zx = pt.x+SCALE*RScale*h_R[2];	Zy = pt.y-SCALE*RScale*h_R[5];

	cvLine(img, pt, cvPoint((int)Xx,(int)Xy),CV_RGB(255,255,0),2,8,0);
	cvLine(img, pt, cvPoint((int)Yx,(int)Yy),CV_RGB(0,255,0),2,8,0);
	cvLine(img, pt, cvPoint((int)Zx,(int)Zy),CV_RGB(255,0,0),2,8,0);
}
/**********************/


/**********************************/
/* Find if two matrices are equal	*/
void displayRendering(IplImage *img, float *euler, CvPoint pt) {
	float Xx, Xy, Yx, Yy, Zx, Zy;
	const float SCALE = 30.0f;

	Xx = pt.x+SCALE*euler[0];	Xy = pt.y-SCALE*euler[1];
	Yx = pt.x+SCALE*euler[3];	Yy = pt.y-SCALE*euler[4];
	Zx = pt.x+SCALE*euler[6];	Zy = pt.y-SCALE*euler[7];

	cvLine(img, pt, cvPoint((int)Xx,(int)Xy),CV_RGB(255,255,0),2,8,0);
	cvLine(img, pt, cvPoint((int)Yx,(int)Yy),CV_RGB(0,255,0),2,8,0);
	cvLine(img, pt, cvPoint((int)Zx,(int)Zy),CV_RGB(255,0,0),2,8,0);
}


/**********************/
void findFaceCenter(float *h_pReal, float *center_x, float *center_y, float NORMAL_FACOTR)
{
	float	X = 0.0f, Y = 0.0f, Z = 0.0f;
	int i = (int)(*center_y)*XN_VGA_X_RES + (*center_x);

	X = h_pReal[i];
	Y = h_pReal[i + MAX_I];
	Z = h_pReal[i + MAX_I2];

	(*center_x) = (X - CENTROID.getXr()) / NORMAL_FACOTR;
	(*center_y) = (Y - CENTROID.getYr()) / NORMAL_FACOTR;
}


void moveToFaceCenter(float *h_X, int nb_X, float center_x, float center_y);
void moveToFaceCenter(float *h_X, int nb_X, float center_x, float center_y)
{
	int	i_x = 0,
		i_y = nb_X,
		i_z = 2 * nb_X;
	float x = 0.0f,
		y = 0.0f,
		z = 0.0f;

	for (int i = 0; i<nb_X; i++) {
		x = h_X[i_x];
		y = h_X[i_y];
		z = h_X[i_z];

		x = x - center_x;
		y = y - center_y;

		h_X[i_x++] = x;
		h_X[i_y++] = y;
		h_X[i_z++] = z;
	}
}


/******************/
/* Apply inv rotation	*/
void applyInvRotation(float *h_X, int nb_X, float *h_R) {
	int	i_x = 0,
			i_y = nb_X,
			i_z = 2*nb_X;
	float x = 0.0f,
			y = 0.0f,
			z = 0.0f; 
	float x_avg = 0;
	float y_avg = 0;
	float z_avg = 0;

	float total = 0;

	i_x = 0; i_y = nb_X; i_z = 2 * nb_X;
	for (int i=0; i<nb_X; i++) {
		x = h_X[i_x];
		y = h_X[i_y];
		z = h_X[i_z];

		x = h_R[0]*x + h_R[3]*y + h_R[6]*z;
		y = h_R[1]*x + h_R[4]*y + h_R[7]*z;
		z = h_R[2]*x + h_R[5]*y + h_R[8]*z;

		h_X[i_x++] = x;
		h_X[i_y++] = y;
		h_X[i_z++] = z;
	}
}
/**********************/


/******************/
/* Apply rotation	*/
void applyRotation(float *h_X, int nb_X, float *h_R) {
	int	i_x = 0,
		i_y = nb_X,
		i_z = 2 * nb_X;
	float x = 0.0f,
		y = 0.0f,
		z = 0.0f;

	for (int i = 0; i<nb_X; i++) {
		x = h_X[i_x];
		y = h_X[i_y];
		z = h_X[i_z];

		h_X[i_x++] = h_R[0] * x + h_R[1] * y + h_R[2] * z;
		h_X[i_y++] = h_R[3] * x + h_R[4] * y + h_R[5] * z;
		h_X[i_z++] = h_R[6] * x + h_R[7] * y + h_R[8] * z;
	}
}
/**********************/


/* Apply translation	*/
void applyTranslation(float *h_X, int nb_X, float *h_T) {
	int	i_x = 0,
		i_y = nb_X,
		i_z = 2 * nb_X;
	float x = 0.0f,
		y = 0.0f,
		z = 0.0f;
	for (int i = 0; i<nb_X; i++) {
		x = h_X[i_x];
		y = h_X[i_y];
		z = h_X[i_z];

		h_X[i_x++] = x - h_T[0];
		h_X[i_y++] = y - h_T[1];
		h_X[i_z++] = z - h_T[2];
	}
}
/**********************/


void printPointClouds(float *h_x, int nb_X, char *filename)
{
	int	i_x = 0,
		i_y = nb_X,
		i_z = 2 * nb_X;

	FILE  *f = fopen(filename, "w");

	for (float i = 0; i<1; i = i + 0.001) {
		fprintf(f, "v %f 0 0 255 0 0\n", i);
		fprintf(f, "v 0 %f 0 0 255 0\n", i);
		fprintf(f, "v 0 0 %f 0 0 255\n", i);
	}
	
	for (float i = -0.001; i<0.01; i = i + 0.0001) {
		fprintf(f, "v %f %f 0 0 255 255\n", eyebrow_x + i, eyebrow_y + i);
	}

	for (float i = -0.001; i<0.01; i = i + 0.0001) {
		fprintf(f, "v %f %f 0 255 255 0\n",(CENTROID.getXr()/NORMALIZATION_FACTOR) + i, (CENTROID.getYr()/NORMALIZATION_FACTOR) + i);
	}

	/*for (int land = 3; land < 14; land++)
	{
		for (float i = -0.001; i<0.01; i = i + 0.0001) {
			fprintf(f, "v %f %f 0 200 32 125\n", (*convertedLand)[land].x + i, (*convertedLand)[land].y + i);
		}
	}
	*/

	for (int i = 0; i<nb_X; i++) {
		fprintf(f, "v %f %f %f\n",h_x[i_x+i], h_x[i_y+i], h_x[i_z+i]);
	}

	fclose(f);
}



/***************************************/
/*	Check if there is a transformation	*/
bool id(float *a, float *h_t)	{
	if (a[0]==0.0f &&	a[1]==0.0f	&&	a[2]==0.0f	&&	h_t[0]==0.0f &&	h_t[1]==0.0f	&&	h_t[2]==0.0f)
		return true;
	return false;
}
/**********************/


/*******************************/
/* Create the heat map palette */
void createPalette(int *palette){
	IplImage *pal = cvLoadImage(".\\forDisplay\\palette2.jpg");

	int	w=pal->width, 
			h=pal->height;
	uchar *data=(uchar *)pal->imageData;

	int	i_r=0, 
			i_g=w, 
			i_b=i_g+w;
	for (int i=0; i<w*3; i+=3) {
		palette[i_r++] = data[i+2];
		palette[i_g++] = data[i+1];
		palette[i_b++] = data[i];
	}
	
	cvReleaseImage(&pal);
}
/*********************/


/***********************/
/* Display information */
void displayInfo(IplImage *img, bool saveFrames, bool depth, float fps){
	double hScale=0.5;
	double vScale=0.5;
	int    lineWidth=1;	
	char s[100];
	
	// Create the font
	CvFont font, font_HR, font2, font3;		
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
	cvInitFont(&font_HR, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 2*hScale,2*vScale,0,2*lineWidth);
	cvInitFont(&font2, CV_FONT_HERSHEY_DUPLEX, 0.4,0.4,0,1);
	cvInitFont(&font3, CV_FONT_HERSHEY_DUPLEX, 0.8,0.8,0,2);



	// Display the speed
	//sprintf(s, "Speed: %3.2f  frames/s", fps);
	//((IS_HR && !depth) ?	cvPutText (img,s,cvPoint(860+ALIGN_X,40+ALIGN_Y), &font_HR, CV_RGB(0,255,0)) : 
	//						cvPutText (img,s,cvPoint(430,20), &font, CV_RGB(255,250,100)) );
	sprintf(s, "Reconstructed model", fps);
	((IS_HR && !depth) ?	cvPutText (img,s,cvPoint(860+ALIGN_X,40+ALIGN_Y), &font_HR, CV_RGB(0,255,0)) : 
							cvPutText (img,s,cvPoint(335,500), &font, CV_RGB(255,250,100)) );
	
	// Display the footer
	sprintf(s, "Institute for Robotics and Intelligent Systems");
	((IS_HR && !depth) ? 	cvPutText (img,s,cvPoint(670+ALIGN_X,930+ALIGN_Y), &font3, CV_RGB(153,0,0)) : 
							cvPutText (img,s,cvPoint(335,665), &font2, CV_RGB(153,0,0)) );
	
	sprintf(s, "University of Southern California");
	((IS_HR && !depth) ?	cvPutText (img,s,cvPoint(740+ALIGN_X,900+ALIGN_Y), &font3, CV_RGB(153,0,0)) : 
							cvPutText (img,s,cvPoint(370,650), &font2, CV_RGB(153,0,0)) );
		
	if (saveFrames) {
		for (int c=0; c<6; c++)
			cvCircle (img,cvPoint(620,40),c,CV_RGB(255,0,0),1,8,0);
		((IS_HR && !depth) ?	cvPutText (img, "Recording",cvPoint(525*2+ALIGN_X,42*2+ALIGN_Y), &font_HR, CV_RGB(255,0,0)) : 
								cvPutText (img, "Recording",cvPoint(525,43), &font, CV_RGB(255,100,0)) );
	}
}
/*********************/


float N(float t, float sigma) {
	return exp(-pow(t/sigma, 2.0f));
}

float norm1D(float x) {
	return fabs(x);
}

float norm2D(float x, float y) {
	return pow(x*x + y*y, 0.5f);
}

float norm3D(float x, float y, float z) {
	return pow(x*x + y*y + z*z, 0.5f);
}


/**************************/
/* Find unoccluded points */
int findUnoccludedPoints(float *MNp, int *unoccluded) {
	int nb=0;
	int ct=0;
	
	for (int i=MAX_IMG_INDEX_2; i<MAX_IMG_INDEX_3;i++) {
		if (MNp[i]>THRESH_NORMALS) {
			unoccluded[ct++] = i-MAX_IMG_INDEX_2;
		}
	}

	return ct;
}
/*****************/


/**************************************************************************************************************/
/**************************************************************************************************************/
/**************************************************************************************************************/
bool isRecord;
int numberOfPoseEstimation;

/*********************/
/*		main function	*/
int main(int argc, char **argv){
	int input_mode = MODE_ONLINE;

	if (argc > 1)
		input_mode = MODE_OFFLINE;

	////////////////////////////////////////////////////////////////////////
	// EM-ICP default parameters
	registrationParameters param ;
	param.sigma_p2			= ICP_P2;
	param.sigma_inf			= ICP_PREC;		// Precision for ICP algorithm, the lower, the more precise but the slower
	param.sigma_factor		= ICP_RED;		// Reduction factor for ICP algorithm, the higher, the more iterations and the slower
	param.d_02				= ICP_D02;
	param.noviewer			= NOVIEWER;		// False to display the alignment, true else
	param.nostop			= false;
	param.notimer			= true;				
	param.points1			= NULL;
	param.points2			= NULL;
	param.points3			= NULL;
	

	float	*h_Yleft=NULL,			// Left part of the frontal face
			*h_Yright=NULL,		// Right part of the frontal face
			*h_Yfull=NULL;			// Whole frontal face

	float *a				= new float[3];	// Euler angles
	float *pre_a			= new float[3];	// previous Euler Angles
	float *pre_t			= new float[3];	// Previous translation matrix
	int stuck=0;					// Counter incremented when the rigid 3D transformation is not updated
	float diff=0.0f;				// Difference between consecutive transformations

	// Initialize the tranformation
	for (int i=0; i<3; i++) {
		a[i] = pre_a[i] = pre_t[i] = 0.0f;
	}


	float *h_X, *d_X, *h_Y, *d_Y;				// Point clouds to be used for the EM-ICP
												// h_X stores points as the order of
												// [X_x0 X_x1 .... X_x(Xsize-1) X_y0 X_y1 .... X_y(Xsize-1)  X_z0 X_z1 .... X_z(Xsize-1) ],
												// where (X_xi X_yi X_zi) is the i-th point in X.
	float *d_Xx, *d_Xy, *d_Xz;
	float *d_Yx, *d_Yy, *d_Yz;

	float *d_A;
	float *d_Xprime, *d_XprimeX, *d_XprimeY, *d_XprimeZ;
	float *d_XprimeCenterd, *d_XprimeCenterdX, *d_XprimeCenterdY, *d_XprimeCenterdZ;
	float *d_YCenterd, *d_YCenterdX, *d_YCenterdY, *d_YCenterdZ;
	float *d_C, *d_lambda;
	
	int	Xsize = NB_PTS_X, Xsize2=NB_PTS_X2, Xsize3=NB_PTS_X3; // Number of points in the first (X) and second (Y) point cloud in ICP
	int Ysize = NB_PTS_X, Ysize2=NB_PTS_X2, Ysize3=NB_PTS_X3;		

	int	maxXY = max(Xsize,Ysize);
	int	rowsA = Ysize;
	int	colsA = Xsize;
	int	pitchA = (rowsA / 4 + 1) * 4;
	
	// Allocate memory for the point clouds	
	h_Yfull = new float[Ysize3];	
	h_Yleft = new float[Ysize3];
	h_Yright = new float[Ysize3]; 
	h_X = new float[Xsize3];

	mallocCUDA(X,Xsize3);
	float h_R[9], h_t[3], *d_R, *d_t; // rotation and translation matrices
	mallocCUDA(R, 9);
	mallocCUDA(t, 3);
	init_RT(h_R, h_t);
	float h_S[9], *d_S;	// S for finding R, t
	mallocCUDA(S,9);
	float h_Xc[3], *d_Xc, h_Yc[3], *d_Yc; // center of X, Y
	mallocCUDA(Xc, 3);	
	mallocCUDA(Yc, 3);
	float *h_one, *d_one;
	h_one = new float [maxXY];	// // a vector with all elements of 1.0f
	for(int t = 0; t < maxXY; t++) 
		h_one[t] = 1.0f;
	mallocCUDA(one, maxXY);
	copyHostToCUDA(one, maxXY);

	mallocCUDA(Y,Ysize3);
	mallocCUDA(Xprime,Ysize3);
	mallocCUDA(YCenterd, Ysize3);
	mallocCUDA(A, pitchA*colsA);
	mallocCUDA(C, Ysize);
	mallocCUDA(lambda, Ysize);

	d_Xx = &d_X[0];
	d_Xy = &d_X[Xsize];
	d_Xz = &d_X[Xsize2];

	d_Yx = &d_Y[0];
	d_Yy = &d_Y[Ysize];
	d_Yz = &d_Y[Ysize2];

	d_XprimeX = &d_Xprime[0];
	d_XprimeY = &d_Xprime[Ysize];
	d_XprimeZ = &d_Xprime[Ysize2];

	d_XprimeCenterd = d_Xprime;
	d_XprimeCenterdX = &d_XprimeCenterd[0];
	d_XprimeCenterdY = &d_XprimeCenterd[Ysize];
	d_XprimeCenterdZ = &d_XprimeCenterd[Ysize2];

	d_YCenterdX = &d_YCenterd[0];
	d_YCenterdY = &d_YCenterd[Ysize];
	d_YCenterdZ = &d_YCenterd[Ysize2];	

	////////////////////////////////////////////////////////////////////////
	//	OpenNI parameters
	int changedIndex=1;
	short firstC=0;
	Status rc = STATUS_OK;

	Device device;
	VideoStream sdepth, scolor;
	VideoFrameRef depthFrame, colorFrame;
	const OniDepthPixel *pDepthMap;
	const OniRGB888Pixel* pImageMap;
	VideoMode videoMode;
	
	rc = OpenNI::initialize();
	errorCheck(rc);

	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
	if (input_mode == MODE_ONLINE)
		rc = device.open(ANY_DEVICE);
	else
		rc = device.open(argv[1]);
	isRecord = true;
	openni::Recorder recorder;
	

	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	errorCheck(rc);

	//rc = device.CreateStream(ONI_SOURCE_DEPTH, sdepth);
	rc = sdepth.create(device, SENSOR_DEPTH);
	
	if (rc == STATUS_OK) 	{

		openni::VideoMode videoMode = sdepth.getVideoMode();
		videoMode.setResolution(640, 480);
		sdepth.setVideoMode(videoMode);

		if (openni::STATUS_OK != rc)
		{
			cout << "error: depth fromat not supprted..." << endl;
		}
		rc = sdepth.start();
		if (rc != STATUS_OK) 		{
			printf("Couldn't start depth stream:\n%s\n", oniGetExtendedError());
			sdepth.destroy();
		}
	} 	else 	{
		printf("Couldn't find depth stream:\n%s\n", oniGetExtendedError());		
	}

	rc = scolor.create(device, SENSOR_COLOR);
	if (rc == STATUS_OK) 	{ 

		openni::VideoMode videoMode = scolor.getVideoMode();
		videoMode.setResolution(640, 480);
		scolor.setVideoMode(videoMode);

		if (openni::STATUS_OK != rc)
		{
			cout << "error: depth fromat not supprted..." << endl;
		}
		rc = scolor.start(); 
		if (rc != STATUS_OK) 		{
			printf("Couldn't start color stream:\n%s\n", oniGetExtendedError());
			scolor.destroy();
		}
	} 	else 	{
		printf("Couldn't find color stream:\n%s\n", oniGetExtendedError());
	}

	if (!sdepth.isValid() && !scolor.isValid()) 	{
		printf("No valid streams. Exiting\n");
		return 2;
	}

	if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
		rc = device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		errorCheck(rc);
	}

	VideoStream** m_streams = new VideoStream*[2];
	m_streams[0] = &sdepth;
	m_streams[1] = &scolor;	
	
	////////////////////////////////////////////////////////////////////////
	//	OpenCV parameters
	IplImage *img	=	cvCreateImage( cvSize(XN_HR_X_RES ,XN_HR_Y_RES+Y_MAX), IPL_DEPTH_8U, CHANNELS );		// RGB image
	IplImage *depth	=	cvCreateImage( cvSize(XN_VGA_X_RES ,XN_VGA_Y_RES+Y_MAX), IPL_DEPTH_8U, CHANNELS );	// Depth image
	
	IplImage *currentImg;

	char *h_rgbData, *d_rgbData;
	h_rgbData = (char *)img->imageData;
	mallocCUDA_char(rgbData, MAX_I3);

	char *h_dData, *d_dData;
	h_dData = (char *)(depth->imageData);
	mallocCUDA_char(dData, MAX_I3);

	char *depthM = (char *) &h_dData[MAX_I3];
	char *imM = (char *) &h_rgbData[MAX_I3];

	cvNamedWindow("img", CV_WINDOW_AUTOSIZE);
	//cvResizeWindow("img", XN_VGA_X_RES, XN_VGA_Y_RES);
	cvMoveWindow("img", 20, 20);
	
	bool	displayFaceTracker	= true,				// Display the face bounding box if detected
			displayDepth		= false,				// Display the depth image instead of the RGB one
			isHeat				= false,			// Display depth as heat map
			updateModel			= true,				// Update the model
			simpleRendering		= false,			// Display the rendering
			saveFrames			= false,			// Save the frames until 's' is pressed or the program is exited
			saveForImprovement	= false;			// Save all the frames for post-processing refinement
	//
	bool printPointCloud = false;
	//
	currentImg = (displayDepth ? depth:img);
	
	////////////////////////////////////////////////////////////////////////
	// Needed for computation
	// Extracted points
	int nb_keyPoints	= 0;					// Number of points on the face
	//Voxel *keyPoints	= NULL;					// NB_PTS_X random points on the face
	//keyPoints	= new Voxel[NB_PTS_X];
	int *selectedPoints = new int[NB_PTS_X];

	bool *h_validityMask, *d_validityMask;
	h_validityMask	= new bool[MAX_I];
	mallocCUDA_bool(validityMask, MAX_I);

	////////////////////////////////////////////////////////////////////////	
	// FACE DETECTION
	FaceDetector faceDetector;					// Face detector
	FaceBox nullBox;							// Default null rectangle: width=height=0
	FaceBox maskBox;							// Current detected face
	bool first = true;							// If true, set the reference and turn false
	bool wrongRegistration=false;
	

	////////////////////////////////////////////////////////////////////////	
	// CUDA
	cublasInit();
	bool allocateMemory = true;	// Should be reset to true each time the size of h_Y changes
	int	mode=M_FRONT,			// Current mode: Use only one half of the face or the whole face
		previous_mode=M_FRONT;	// Previous mode

	////////////////////////////////////////////////////////////////////////	
	// Bilateral Filter on CUDA
	float euclidean_delta=EUCLIDEAN_DELTA;//0.03f;
	float gaussian_delta= GAUSSIAN_DELTA;//8.0f;
	int filter_radius = FILTER_RADIUS;//5;
	int iterations = 3;
	int nthreads=1;

	float *h_pDepth  = (float *)malloc(MAX_I * sizeof(float));
	float *h_pReal   = (float *)malloc(MAX_I3 * sizeof(float));
	float *h_face    = new float [MAX_I3];
	float *h_test    = new float [MAX_I3];
		
	////////////////////////////////////////////////////////////////////////
	//	Normal map
	float *h_pNormals    = (float *)malloc(MAX_I3 * sizeof(float));
	for (int i=0; i<MAX_I3; i++)
		h_pNormals[i]=0.0f;

	////////////////////////////////////////////////////////////////////////
	//	Time measurement
	clock_t start, end;
	float cpuTime;
	int frame_index=0;	// Index of the frame in the session
	int session=0;			// Session number
	int nb_frames=0;

	//////////////////////////////////////////////////////////////////////
	//	PALETTE FOR HEAT MAP
	int *h_palette, *d_palette;
	h_palette = new int[SIZE_PALETTE];
	createPalette(h_palette);
	mallocCUDA_int(palette, SIZE_PALETTE);
	copyHostToCUDA_int(palette,SIZE_PALETTE);
	delete[] h_palette;

	////////////////////////////////////////////////////////////////////////
	//	Rendering monkey
	float RScale=((!displayDepth && IS_HR) ? 3.0f:1.5f);

	float *h_monkey_1 = new float[NB_RENDERING3];
	float *d_monkey_1, *d_monkey_2;
	mallocCUDA(monkey_1, NB_RENDERING3);
	mallocCUDA(monkey_2, NB_RENDERING3);
	
	// Fill in matrices
	int i_X=0, i_Y=NB_RENDERING, i_Z=NB_RENDERING*2;
	FILE *f = fopen(".\\forDisplay\\MONKEY.off","r");
	fscanf(f, "OFF\n%d %d %d\n", &i_X, &i_X, &i_X);
	float xxx=0.0f,yyy=0.0f,zzz=0.0f;
	for (int i=0;i<NB_RENDERING;i++) {
		fscanf(f,"%f %f %f",&xxx,&yyy,&zzz);	
		h_monkey_1[i_X++] = -xxx - 0.000039f;
		h_monkey_1[i_Y++] = yyy  - 0.234032f;
		h_monkey_1[i_Z++] = -zzz + 0.671042f;
	}
	fclose(f);
	copyHostToCUDA(monkey_1, NB_RENDERING3);

	/////////////////////////////////////////////////////
	// OPENNI GENERAL CUDA
	float* d_pDepth;
	mallocCUDA(pDepth, MAX_I);

	float* d_pReal,/* *d_pNormals, */*d_face;
	mallocCUDA(pReal, MAX_I3);
	//mallocCUDA(pNormals, MAX_I3);
	mallocCUDA(face, MAX_I3);

	float *d_gaussian;
	int fr_g = filter_radius+1;
	mallocCUDA(gaussian, fr_g);
	initGaussian(d_gaussian, gaussian_delta, fr_g);

	int x=0, y=0;

	////////////////////////////////////////////////////////////////////////
	//	Options
	printf("------------------\n");
	printf(" Display options:\n");
	printf("------------------\n\n");
	printf("------------------\n\n");
	printf("ACQUISITION\n\n");
	printf(" - 'd': switch between depth map and RGB input\n");
	printf(" - 't': display the face detected by the face detector\n");
	printf(" - 'r': set the current frame as the reference frame\n");
	printf(" - 'q'/ESC: terminate acquisition\n\n");
	printf("----------------\n\n");
	printf("RENDERING\n\n");
	printf(" - 'q'/ESC: terminate the program\n\n");
	printf("----------------\n");


	////dlib
	numberOfPoseEstimation = 0;
	cv_image<bgr_pixel> cimg;
	frontal_face_detector detector = get_frontal_face_detector();
	std::vector<dlib::rectangle> faces;
	shape_predictor sp;
	deserialize("shape_predictor_68_face_landmarks.dat") >> sp;
	initExpr("../config/config.json");


{
	////////////////////////////////////////////////////////////////////////
	//	Modeling
	Modeling model;					// Model
	char *mdData = (char *)model.getDepthImg()->imageData;
	

	bool saveMesh=true;
	//float		y_left_max=0.0f, 
	//			y_right_max=0.0f;
	////////////////////////////////////////////////////////////////////////
	// Loop on frames
	////////////////////////////////////////////////////////////////////////
	if (isRecord && argc <= 1)
	{
		recorder.create("test.ini");
		recorder.attach(sdepth, false);
		recorder.attach(scolor, false);
		recorder.start();
	}

	oldversion = false;
	landmark = false;
	pose_estimation_R = true;

	while (true) { 
		start = clock();
	
		//////////////////////////////////////////////////////////////////////
		// Update the context
		rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed\n");
			return 1;
		}
		errorCheck(rc);
		rc = sdepth.readFrame(&depthFrame); 
		errorCheck(rc);
		rc = scolor.readFrame(&colorFrame); 
		errorCheck(rc);

		if (colorFrame.isValid())
		{ 
			pImageMap = (const OniRGB888Pixel*)colorFrame.getData();
		}
		

		if (depthFrame.isValid())
			pDepthMap = (const OniDepthPixel*)depthFrame.getData();

		if (firstC<5) {
			firstC++;
			continue;
		}
		//////////////////////////////////////////////////////////////////////
		// Bilateral filtering
		// Prepare the arrays for bilateral filter
		for (int i=0; i<MAX_I;i++) 
		{
			// Set h_pDepth
			h_pDepth[i] = (float) pDepthMap[i];
			// Set validityMask at the same time
			h_validityMask[i] = ((pDepthMap[i]>MIN_DEPTH && pDepthMap[i]<MAX_DEPTH) ? true:false);
		}

		// Set the validityMask in the CUDA device
		copyHostToCUDA_bool(validityMask, MAX_I);

		// Bilateral filter
		bilateralFilterRGBA(h_pDepth, d_pDepth, h_pReal, d_pReal, d_gaussian, d_validityMask, XN_VGA_X_RES, XN_VGA_Y_RES, gaussian_delta, euclidean_delta, filter_radius, iterations, nthreads);

		//////////////////////////////////////////////////////////////////////
		displayDepthImage(d_pReal, d_dData, h_dData, d_validityMask, d_palette, isHeat, XN_VGA_X_RES, XN_VGA_Y_RES);
		
		displayImageMap(	h_rgbData,			// TODO : ON GPU WHEN USING COLOR???
							pImageMap);

		faceDetector.setValidityMask(h_validityMask);

		// Detect the face
		bool faceDetected = faceDetector.detectFace(h_pReal);

		// Find the box containing the mask values
		maskBox = ( faceDetected ? faceDetector.getFaceBox() : nullBox);
		if (printPointCloud)
		{
			counts++;
		}
		 
		// Extract the points on the face
		if (faceDetected) {
			// Compute the normal map on the face		
			// Extract the face
			//dlib,,
			for (int i = 0; i < MAX_I3; i++)
			{
				h_face[i] = 0;
			}

			extractFace(h_face, h_pReal, maskBox, h_validityMask, &nb_keyPoints, &NORMALIZATION_FACTOR, &CENTROID, first);

			if (printPointCloud)
			{
				char temp[10];
				char temp2[20];

				memset(temp, 0, 10);
				memset(temp2, 0, 20);
				itoa(counts, temp, 10);
				strcat(temp2, ".\\origin\\");
				strcat(temp2, temp);
				strcat(temp2, ".obj");
				printPointClouds(h_face, MAX_I, temp2);
			}


			{
				//IplImage *ccimg = cvCreadeletePointUnderChinteImage(cvSize(440,280),IPL_DEPTH_8U,3);

				//cvSetImageROI(img, cvRect(100,100,440,280));
				//cvCopy(img,ccimg);
				cimg = img;

				start = clock();
				std::vector<full_object_detection> shapes;

				faces = detector(cimg);
				//dlib::rectangle faceRecct(faceDetector.getFaceBox().getLeftX() - 100, faceDetector.getFaceBox().getTopY() - 100, faceDetector.getFaceBox().getWidth() + 200, faceDetector.getFaceBox().getHeight() + 200);
				//	if (faces.size()>0)
				//	{
				if (faces.size() > 0){
					static int number = 0;

					full_object_detection shape = sp(cimg, faces[0]);
					//cout << "number of parts: "<< shape.num_parts() << endl;
					//cout << "pixel position of first part:  " << shape.part(0) << endl;
					//	cout << "pixel position of second part: " << shape.part(1) << endl;

					shapes.push_back(shape);

					if (shapes[0].num_parts() == 68)
					{
						for (unsigned long i = 0; i < shapes[0].num_parts(); i++)
						{
							point &a = shapes[0].part(i);
							cv::Point2f sPoint2fs(a.x(), a.y());
							landmarkPts.push_back(sPoint2fs);
							if (i == 22)
							{
								eyebrow_x = a.x();
								
							}

							if (i == 22)
							{
								eyebrow_y = a.y();
								eyebrow_y_real = a.y();
							}
						}

						process(R, T);
						FaceSegmentation sBorder(landmarkPts);


						findFaceCenter(h_pReal, &eyebrow_x, &eyebrow_y, NORMALIZATION_FACTOR);
						float vals2[9] = { 1, 0, 0, 0, -1, 0, 0, 0,  1 };
						float val_[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1 };

						// R,T evaluation
						std::vector<cv::Point3d> framePoints;
						std::vector<cv::Point2d> imgFramePoints;

						framePoints.push_back(cv::Point3d(0.0, 0.0, 0.0));
						framePoints.push_back(cv::Point3d(60.0, 0.0, 0.0));
						framePoints.push_back(cv::Point3d(0.0, 60.0, 0.0));
						framePoints.push_back(cv::Point3d(0.0, 0.0, 60.0));

						cv::Mat rvec = cv::Mat(cv::Size(3, 1), CV_64F);
						cv::Mat tvec = cv::Mat(cv::Size(3, 1), CV_64F);


						cv::Mat inv = R.inv();
						cv::Mat inv_T = -R.inv()*T;
						for (int i = 0; i < 3; i++)
						{
							for (int j = 0; j < 3; j++)
							{
								double temp = R.at<double>(i, j);
								double tmpe2 = inv.at<double>(i, j);
								R_matrix[i * 3 + j] = (float)temp;
							}
						}

						if ((R.rows == 3 && R.cols == 3))
						{
							cv::Rodrigues(R, rvec);
							cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
							cv::Mat out_K = cv::Mat(3, 3, CV_64F);
							cv::projectPoints(framePoints, rvec, T, out_K, distCoeffs, imgFramePoints);
						}
						if (imgFramePoints.size() > 0)
						{
							cvLine(currentImg, cvPoint(0, 0), cvPoint(30, 30), CV_RGB(255, 255, 0), 2, 8, 0);
							cvLine(currentImg, cvPoint((int)imgFramePoints[0].x, (int)imgFramePoints[0].y), cvPoint((int)imgFramePoints[1].x, (int)imgFramePoints[1].y), CV_RGB(255, 255, 0), 6, 8, 0);
							cvLine(currentImg, cvPoint((int)imgFramePoints[0].x, (int)imgFramePoints[0].y), cvPoint((int)imgFramePoints[2].x, (int)imgFramePoints[2].y), CV_RGB(0, 255, 0), 6, 8, 0);
							cvLine(currentImg, cvPoint((int)imgFramePoints[0].x, (int)imgFramePoints[0].y), cvPoint((int)imgFramePoints[3].x, (int)imgFramePoints[3].y), CV_RGB(0, 0, 255), 6, 8, 0);
						}
						/////

						numberOfPoseEstimation++;
						
						if (pose_estimation_R)
						{
							static int count = 0;
							char temp[10];
							char temp2[20];
							
							memset(temp, 0, 10);
							memset(temp2, 0, 20);
							if (printPointCloud  && 0)
							{
								memset(temp, 0, 10);
								memset(temp2, 0, 20);
								itoa(counts, temp, 10);
								strcat(temp2, ".\\before_cut\\");
								strcat(temp2, temp);
								strcat(temp2, ".obj");
								printPointClouds(h_face, MAX_I, temp2);
							}
							
							static int check = 0;
							
							// segment face

							sBorder.setEyebrow(eyebrow_x, eyebrow_y_real);
							sBorder.deletePointUnderChin(h_face, MAX_I, CENTROID.getXr(), CENTROID.getYr(), CENTROID.getZr(),NORMALIZATION_FACTOR, nb_keyPoints);
							sBorder.rearrangeArray(&h_face, MAX_I, nb_keyPoints);

							if (printPointCloud)
							{
								memset(temp, 0, 10);
								memset(temp2, 0, 20);
								itoa(counts, temp, 10);
								strcat(temp2, ".\\before\\");
								strcat(temp2, temp);
								strcat(temp2, ".obj");
								printPointClouds(h_face, MAX_I,temp2);
							}

							moveToFaceCenter(h_face, MAX_I, eyebrow_x, eyebrow_y);
							applyRotation(h_face, MAX_I, vals2);
							applyInvRotation(h_face, MAX_I, R_matrix);
							applyRotation(h_face, MAX_I, vals2);
							
							if (printPointCloud)
							{	
								memset(temp, 0, 10);
								itoa(counts, temp, 10);
								memset(temp2, 0, 20);
								strcat(temp2, ".\\after\\");
								strcat(temp2, temp);
								strcat(temp2, ".obj");
								printPointClouds(h_face, MAX_I, temp2);
							}

							cout << "pose estimation" << number++  << endl;
							//////////////////////////////////
							// display the face landmark
						}

						if (landmark || pose_estimation_R)
						{
							// Extract the points
							selectRandomPointOnFace(selectedPoints, nb_keyPoints, NB_PTS_X);
							//////////////////////////////////////////////////////////////////////////////////////////////
							// Demean/Normalize face for EM-ICP
							//// The centroid is the actual centroid, the normalization factor is the one of the first frame
							setH_X(h_X, NB_PTS_X, selectedPoints, h_face);
						}
						
					}
					else
					{
						if (landmark || pose_estimation_R)
						{
							nb_keyPoints = 0;
						}
					}
					
				}
				else
				{
					if (landmark || pose_estimation_R)
					{
						nb_keyPoints = 0;
					}
				
				}

	
				shapes.clear();
				faces.clear();
				landmarkPts.clear();
			}

			if (oldversion)
			{ 
				// printPointClouds(h_face, MAX_I, "hface.obj");
				// Extract the points
				selectRandomPointOnFace(selectedPoints, nb_keyPoints, NB_PTS_X);
				//////////////////////////////////////////////////////////////////////////////////////////////
				// Demean/Normalize face for EM-ICP
				//// The centroid is the actual centroid, the normalization factor is the one of the first frame
				setH_X(h_X, NB_PTS_X, selectedPoints, h_face);
			}

		} 
	
		else {
			nb_keyPoints = 0;
			//for (int i=0; i<MAX_I; i++)
			//	h_nData[i] = 0;
		}

		//////////////////////////////////////////////////////////////////////
		// Compute the pose
		// Find the mode
		previous_mode = mode;
		if (a[1]<-M_SWITCH) {
			mode = M_LEFT;
		}	else if (a[1]>M_SWITCH) {
			mode = M_RIGHT;
		}	else	{
			mode = M_FRONT;
		}
		
		// Update Ysize and h_Y depending on the mode
		if (previous_mode!=mode)
			allocateMemory=true;
		switch (mode) {
			case M_LEFT:
				h_Y	= h_Yleft;
				break;
			case M_RIGHT:
				h_Y	= h_Yright;
				break;
			default:
				h_Y = h_Yfull;
				break;	
		}
		
		///////////////////////////////////////////////////////////////

		// Apply EM-ICP
		if (Xsize>0 && Ysize>0 && !first)
			poseEstimation(	Xsize, Ysize, 
							h_X, d_X, d_Xx, d_Xy, d_Xz,
							h_Y, d_Y, d_Yx, d_Yy, d_Yz,
							h_R, d_R, h_t, d_t,
							h_S, d_S,
							h_Xc, d_Xc, h_Yc, d_Yc,
							h_one, d_one,
							d_A,
							d_Xprime, d_XprimeX, d_XprimeY, d_XprimeZ,
							d_XprimeCenterd, d_XprimeCenterdX, d_XprimeCenterdY, d_XprimeCenterdZ,
							d_YCenterd, d_YCenterdX, d_YCenterdY, d_YCenterdZ,
							d_C, d_lambda,

							maxXY, rowsA, colsA, pitchA,
							param,
							a, pre_a, pre_t,
							&allocateMemory,
							&diff);

		if (first && (nb_keyPoints>0 )) {
			//printPointClouds(h_face, MAX_I, ".\\before\\First.obj");
			first = false;

			swapPoints(h_X,h_Yfull,NB_PTS_X);

			int nb_l =0, nb_r=0;
			int *i_l = new int[nb_keyPoints];
			int *i_r = new int[nb_keyPoints];
			for (int i=0; i<nb_keyPoints; i++) {
				if (h_face[i] < 0.0f) {
					i_l[nb_l++] = i;
				} else {
					i_r[nb_r++] = i; 
				}
			}			

			selectRandomPointOnFace(selectedPoints, i_l, nb_l, NB_PTS_X);
			setH_X(h_Yleft, NB_PTS_X, selectedPoints, h_face);

			selectRandomPointOnFace(selectedPoints, i_r, nb_r, NB_PTS_X);
			setH_X(h_Yright, NB_PTS_X, selectedPoints, h_face);

			delete[] i_l;
			delete[] i_r;


			// Remember the first centroid
			model.setImFront(img);
			model.setCFront(CENTROID);
		} 

		//////////////////////////////////////////////////////////////////////
		//	Check that the transformation is possible
		diff  = tr_diff(h_t, pre_t, a, pre_a);
		stuck = ((diff ==0.0f) ? stuck+1: 0);

		/////// 
		if (stuck>STUCK_LIM || diff>90.0f) { 
			allocateMemory=true;
			h_t[0]=h_t[1]=h_t[2]=pre_t[0]=pre_t[1]=pre_t[2]=h_R[1]=h_R[2]=h_R[3]=h_R[5]=h_R[6]=h_R[7]=a[0]=a[1]=a[2]=pre_a[0]=pre_a[1]=pre_a[2]=0.0000001f;
			h_R[0]=h_R[4]=h_R[8]=1.0f;
			stuck=0;
		} 

		///////////////////////////////////////////////////////////////
		// Update the model
		if (updateModel && faceDetected)
		{ 
			wrongRegistration = false;
			model.model(nb_keyPoints, 
						h_face, 
						h_R, 
						h_t, 
						&wrongRegistration,
						nb_frames++);

			//cvShowImage("model depth", model.getDepthImg());
		}
		
		///////////////////////////////////////////////////////////////
		// Rendering the estimated pose		
		if (displayDepth) {
			//(simpleRendering	?	displayRendering(currentImg,h_R,cvPoint((int)(20*RScale),(int)(40*RScale)), RScale) 
			//					:	render_monkey(h_monkey_1, d_monkey_1, d_monkey_2, h_dData, d_dData, NB_RENDERING, d_R, d_t, RScale, true)); 
			
			if (simpleRendering)
			{
				displayRendering(currentImg, h_R, cvPoint((int)(20 * RScale), (int)(40 * RScale)), RScale);
			}
			else
			{
				render_monkey(h_monkey_1, d_monkey_1, d_monkey_2, h_dData, d_dData, NB_RENDERING, d_R, d_t, RScale, true);
			}
			copyCUDAToHost_char(dData, MAX_I3);
		} else {
			render_monkey(h_monkey_1, d_monkey_1, d_monkey_2, h_rgbData, d_rgbData, NB_RENDERING, d_R, d_t, RScale, false);
			copyCUDAToHost_char(rgbData, MAX_I3);
		}
			
		///////////////////////////////////////////////////////////////
		// Display the face tracker
		if (displayFaceTracker) {
			if (faceDetected) {
				if (IS_HR && !displayDepth) 
					cvRectangle(currentImg, cvPoint(faceDetector.getFaceBox().getLeftX()*2+ALIGN_X,faceDetector.getFaceBox().getTopY()*2+ALIGN_Y),
							cvPoint(faceDetector.getFaceBox().getRightX()*2+ALIGN_X,faceDetector.getFaceBox().getBottomY()*2+ALIGN_Y),CV_RGB(255,0,0),2,8,0);
				else
					cvRectangle(currentImg, cvPoint(faceDetector.getFaceBox().getLeftX(),faceDetector.getFaceBox().getTopY()),
							cvPoint(faceDetector.getFaceBox().getRightX(),faceDetector.getFaceBox().getBottomY()),CV_RGB(255,0,0),2,8,0);
			}
		}

		
		if (!wrongRegistration && nb_keyPoints>0 && saveForImprovement) 
		{
			// Save the frames 
			char p[100];
			FILE *f;
			sprintf(p, ".\\frames\\n3D_%d.obj", nb_frames++);

			// Save the depth map
			f = fopen(p, "w");

			fprintf(f, "# number of points\n# %d\n\n# rotation matrix\n# %f %f %f\n# %f %f %f\n# %f %f %f\n\n# translation matrix\n# %f %f %f\n\n# normalization factor\n# %f\n\n# centroid\n# %f %f %f\n\n", 
				nb_keyPoints,
				h_R[0], h_R[1], h_R[2], 
				h_R[3], h_R[4], h_R[5],
				h_R[6], h_R[7], h_R[8],
				h_t[0], h_t[1], h_t[2],
				NORMALIZATION_FACTOR,
				CENTROID.getXr(), CENTROID.getYr(), CENTROID.getZr());

			int	l = maskBox.getLeftX(),
				r = maskBox.getRightX(),
				t = maskBox.getTopY(),
				b = maskBox.getBottomY();

			int xx=0, yy=0;
			int begin	= t*XN_VGA_X_RES + l;
			int end		= (b+CHIN_ADD)*XN_VGA_X_RES + r;
			xx=l; yy=t;

			
			// Restrict the skin mask to the face area
			for (int i=begin; i<end; i++) 
			{
				if (xx>=l 	&&	xx<=r 	&&	yy>=t	&&	yy<=(b+CHIN_ADD) ) 
				{
					if (h_validityMask[i]) 
					{
						float Xr=(h_pReal[i]-CENTROID.getXr())/NORMALIZATION_FACTOR			-h_t[0];
						float Yr=(h_pReal[i+MAX_I]-CENTROID.getYr())/NORMALIZATION_FACTOR	-h_t[1];
						float Zr=(h_pReal[i+MAX_I2]-CENTROID.getZr())/NORMALIZATION_FACTOR	-h_t[2];

						float nXr=h_pNormals[i];
						float nYr=h_pNormals[i+MAX_I];
						float nZr=h_pNormals[i+MAX_I2];
						
						fprintf(f, "v %f %f %f\nvn %f %f %f\n", 
							-( h_R[0]*Xr + h_R[3]*Yr + h_R[6]*Zr ), 
							( h_R[1]*Xr + h_R[4]*Yr + h_R[7]*Zr ), 
							-( h_R[2]*Xr + h_R[5]*Yr + h_R[8]*Zr ) + CYLINDER_Z, 
							( h_R[0]*nXr + h_R[3]*nYr + h_R[6]*nZr ), 
							-( h_R[1]*nXr + h_R[4]*nYr + h_R[7]*nZr ), 
							( h_R[2]*nXr + h_R[5]*nYr + h_R[8]*nZr ));
					}	
				}

				// Update the line/column values
				if (++xx == XN_VGA_X_RES) 
				{
					xx=0;
					yy++;
				}
			}
		}

		fclose(f);

		/////////////////////////////////////////////
		// Save the raw data 
		if (saveFrames) 
		{
			char o[100], p[100],q[100],q1[100];
			FILE *f;
			sprintf(o, ".\\frames\\s%d_frame%d.jpg", session, frame_index);
			sprintf(p, ".\\frames\\s%d_data%d.txt", session, frame_index++);

			// Save the image
			cvSaveImage(o,img);
			// Save the depth map
			f = fopen(p, "w");

			for (int i=0; i<MAX_I; i++) {
				//fprintf(f, "%d %d %d\n", x, y, pDepthMap[i]);
				fprintf(f, "%d\n", pDepthMap[i]);
				// Update the line/column values
				if (++x == XN_VGA_X_RES) {
					x=0;
					y++;
				}
			}

			fclose(f);
		}

		// End of the timer
		end = clock();
		cpuTime = (float)( end-start )/(float)( CLOCKS_PER_SEC ) ;
		float fps = 1/cpuTime;
		
		// Update the model image
		for (int y=0; y<Y_MAX3; y+=3) {
			int ym = y*XN_VGA_X_RES;
			int yd = y*THETA_MAX;
			for (int t=0; t<THETA_MAX3; t+=3) {
				int indm=(ym+t);
				int ind =(yd+t);

				if (displayDepth) {
					for (int k=0; k<3; k++)
						depthM[indm+k] = mdData[ind+k];
				} else {
					for (int k=0; k<3; k++)
						imM[indm+k] = mdData[ind+k];
				}
			}
		}


		// Display the info
		displayInfo(currentImg, saveFrames, displayDepth, fps);

		// Show the image
		cvShowImage("img", currentImg);

		//////////////////////////////////////////////////////////////////////
		// Key options for display		
		char key = cvWaitKey(20);
		if (key == 'q' || key == 'Q' || key == 27)
			break;
		if (key=='t' || key=='T') {
			displayFaceTracker=!(displayFaceTracker);
		}
		if (key=='d' || key=='D') {
			displayDepth=!(displayDepth);
			currentImg = (displayDepth ? depth:img);
			RScale=((!displayDepth && IS_HR) ? 3.0f:1.5f);
		}
		/*if (key=='h' || key=='H') {
			isHeat=!(isHeat);
		}
		if (key=='m' || key=='M') {
			updateModel=!(updateModel);
		}*/
		if (key=='r' || key=='R') {
			first=true;
			allocateMemory=true;
			init_RT(h_R, h_t);
			for (int i=0; i<3; i++)
				pre_t[i]=a[i]=pre_a[i]=0.0f;
			faceDetector.setFaceBox(nullBox);
			faceDetector.setPreviousFaceBox(nullBox);	
			model.reset();
			//cvShowImage("model depth", model.getDepthImg());
			nb_frames=0;

			frame_index=0;
			session++;
			
		}
		/*if (key=='b' || key=='B') {
			simpleRendering = !(simpleRendering);
		}*/
		//if (key=='s' || key=='S') {
		//	saveFrames = !(saveFrames);
		//	if (saveFrames) {
		//		frame_index=0;
		//		session++;
		//	}
		//}
		//if (key == 'p' || key == 'P') {
		//	printPointCloud = !printPointCloud;
		//}
		//if (key=='a' || key=='A') {
		//	saveForImprovement = !(saveForImprovement);
		//	if (saveForImprovement) {
		//		nb_frames=0;

		//		// TODO ?????????????
		//		first=true;
		//		allocateMemory=true;
		//		init_RT(h_R, h_t);
		//		for (int i=0; i<3; i++)
		//			pre_t[i]=a[i]=pre_a[i]=0.0f;
		//		faceDetector.setFaceBox(nullBox);
		//		faceDetector.setPreviousFaceBox(nullBox);	
		//		model.reset();
		//		//cvShowImage("model depth", model.getDepthImg());
		//	}
		//}
		//if (key == ']')
		//{
		//	cvSaveImage(".\\model\\modeltexture.jpg", currentImg);
		//}

		//previous version
		if (key == '8')
		{
			oldversion = true;
			landmark = false;
			pose_estimation_R = false;

			first = true;
			allocateMemory = true;
			init_RT(h_R, h_t);
			for (int i = 0; i<3; i++)
				pre_t[i] = a[i] = pre_a[i] = 0.0f;
			faceDetector.setFaceBox(nullBox);
			faceDetector.setPreviousFaceBox(nullBox);
			model.reset();
			//cvShowImage("model depth", model.getDepthImg());
			nb_frames = 0;

			frame_index = 0;
			session++;
		}
		// filtering 
		if (key == '9')
		{
			oldversion = false;
			landmark = true;
			pose_estimation_R = false;

			first = true;
			allocateMemory = true;
			init_RT(h_R, h_t);
			for (int i = 0; i<3; i++)
				pre_t[i] = a[i] = pre_a[i] = 0.0f;
			faceDetector.setFaceBox(nullBox);
			faceDetector.setPreviousFaceBox(nullBox);
			model.reset();
			//cvShowImage("model depth", model.getDepthImg());
			nb_frames = 0;

			frame_index = 0;
			session++;
		}

		//proposed method
		if (key == '0')
		{
			oldversion = false;
			landmark = false;
			pose_estimation_R = true;

			first = true;
			allocateMemory = true;
			init_RT(h_R, h_t);
			for (int i = 0; i<3; i++)
				pre_t[i] = a[i] = pre_a[i] = 0.0f;
			faceDetector.setFaceBox(nullBox);
			faceDetector.setPreviousFaceBox(nullBox);
			model.reset();
			//cvShowImage("model depth", model.getDepthImg());
			nb_frames = 0;

			frame_index = 0;
			session++;
		}
	}	

	if (isRecord && argc <= 1)
	{
		recorder.stop();
	}
	cvDestroyWindow( "img" );
	//cvDestroyWindow( "model depth" );
	//cvDestroyWindow( "normal map" );
	cvReleaseImage(&img);
	cvReleaseImage(&depth);
	//cvReleaseImage(&normalImg);


	if (input_mode == MODE_ONLINE)
	{
		model.saveToOBJFile(".\\model\\mymodel.obj", saveMesh, NORMALIZATION_FACTOR);
	}
	else
	{
		string sModelName = string(argv[1]);
		
		sModelName = sModelName.substr(0, sModelName.length() - 3);
		string sModelPath = ".\\model\\" + sModelName + "obj";
		// Save the model
		model.saveToOBJFile((char *)sModelPath.c_str(), saveMesh, NORMALIZATION_FACTOR);
	}

	//// Normals computation
	//float *h_mReal, *d_mReal;
	float *h_mNormals;//, *d_mNormals;
	//float *d_md, *d_mM;

	//h_mReal = new float[MAX_IMG_INDEX_3];
	//mallocCUDA(mReal, MAX_IMG_INDEX_3);
	//mallocCUDA(mM, MAX_IMG_INDEX_9);
	//mallocCUDA(md, MAX_IMG_INDEX_3);
	h_mNormals = new float[MAX_IMG_INDEX_3];
	//mallocCUDA(mNormals, MAX_IMG_INDEX_3);



	//for (int i=0;i<MAX_IMG_INDEX_3; i++)
	//	h_mReal[i] = model.getHmodelXYZ(i);
	// Set d_mReal
	//copyHostToCUDA(mReal, MAX_IMG_INDEX_3);

	// Compute h_pNormals
	//PCA(d_mReal, d_mM, d_md, d_mNormals, h_mNormals, 0, THETA_MAX, 0, Y_MAX, RADIUS_PCA_MODEL, THETA_MAX, Y_MAX);

	int i_first = THETA_MAX+1;
	int i_last = (THETA_MAX-2)*Y_MAX-2;
	int i1=0, i2=0, i3=0, i4=0;
	float	x1=0.0f, y1=0.0f, z1=0.0f,
			x2=0.0f, y2=0.0f, z2=0.0f;
	float	nx=0.0f, ny=0.0f, nz=0.0f;
	float	nn=0.0f;

	
	for (int i1=i_first; i1<i_last; i1++) {

		i2 = i1+1;				// x+1, y
		i3 = i1+THETA_MAX;		// x, y+1
		i4 = i3+1;				// x+1, y+1

		if ((model.getHmodelXYZ(i1)!=0.0f || model.getHmodelXYZ(i1+MAX_IMG_INDEX_1)!=0.0f || model.getHmodelXYZ(i1+MAX_IMG_INDEX_2)!=0.0f)
		&&	(model.getHmodelXYZ(i4)!=0.0f || model.getHmodelXYZ(i4+MAX_IMG_INDEX_1)!=0.0f || model.getHmodelXYZ(i4+MAX_IMG_INDEX_2)!=0.0f)
		&&	(model.getHmodelXYZ(i2)!=0.0f || model.getHmodelXYZ(i2+MAX_IMG_INDEX_1)!=0.0f || model.getHmodelXYZ(i2+MAX_IMG_INDEX_2)!=0.0f)) {
			x1 = (model.getHmodelXYZ(i2)-model.getHmodelXYZ(i1));
			y1 = (model.getHmodelXYZ(i2+MAX_IMG_INDEX_1)-model.getHmodelXYZ(i1+MAX_IMG_INDEX_1));
			z1 = (model.getHmodelXYZ(i2+MAX_IMG_INDEX_2)-model.getHmodelXYZ(i1+MAX_IMG_INDEX_2));

			x2 = (model.getHmodelXYZ(i4)-model.getHmodelXYZ(i1));
			y2 = (model.getHmodelXYZ(i4+MAX_IMG_INDEX_1)-model.getHmodelXYZ(i1+MAX_IMG_INDEX_1));
			z2 = (model.getHmodelXYZ(i4+MAX_IMG_INDEX_2)-model.getHmodelXYZ(i1+MAX_IMG_INDEX_2));

			nx = y1*z2 - y2*z1;
			ny = z1*x2 - z2*x1; 
			nz = x1*y2 - x2*y1;

			nn = pow(nx*nx + ny*ny + nz*nz, 0.5f);

			if (nn>0.0f){
				h_mNormals[i1] = nx/nn;
				h_mNormals[i1+MAX_IMG_INDEX_1] = ny/nn; 
				h_mNormals[i1+MAX_IMG_INDEX_2] = nz/nn;
			}
		}

	}

	initEngineRGB();
	initEngineXYZ();
	initEngineNormals();
	initEngineMesh(model.getNbMesh());
	setNbEngineMesh(model.getNbMesh());

	for (int i=0; i<MAX_IMG_INDEX_3; i++) {
		
		if (model.getHmodelRGB(i) == 0)
		{
			setEngineRGB(model.getHmodelRGB(i), i);
		}
		else
		{
			setEngineRGB(0.7, i);
		}
		if (model.getHdefined(i % MAX_IMG_INDEX_1)) {
			setEngineXYZ(model.getHmodelXYZ(i), i );
			setEngineNormals(h_mNormals[i], i );
		}
	}
	for (int i=0; i<model.getNbMesh(); i++) {
		setEngineMesh(model.getMesh(0,i), model.getMesh(1,i), model.getMesh(2,i), i);
	}

	EngineInit();
	EngineCameraSetup(4.0f);
	

	InitPointCloud(model.getHmodelXYZ(), MAX_IMG_INDEX_1, model.getH_defined_C(), param.points1);
	
	while(EngineIteration(MAX_IMG_INDEX_1, param.points1, model.getHmodelXYZ(),  h_R, h_t)); // PointCloudViewer
	freeEngineRGB();
	freeEngineXYZ();
	freeEngineNormals();
	freeEngineMesh();
	
	cvSaveImage(".\\model\\modeltexture.jpg", model.getImFront());

	//////////////////////////////////////////////////////////////////////
	//	Free memory
	if( !param.noviewer ){
		delete[] param.points1;
		delete[] param.points2;
		delete[] param.points3;
		EngineShutDown();
	}
	cublasShutdown();
	
	//*******************************************************
	//OpenNI::shutdown();
	//*******************************************************

	delete[] m_streams;
	delete[] h_X;
	//delete[] h_Y;
	delete[] h_Yfull;
	delete[] h_Yleft;
	delete[] h_Yright;

	delete[] a;
	delete[] pre_a;
	delete[] pre_t;
	//delete[] keyPoints;
	delete[] selectedPoints;	
	
	releaseCUDAandHost(monkey_1);
	releaseCUDA(monkey_2);
	// OpenNI General CUDA
	releaseCUDAandHost(pDepth);
	releaseCUDAandHost(pReal);
	releaseCUDAandHost(face);
	releaseCUDA(gaussian);
	releaseCUDAandHost(validityMask);
	delete[] h_mNormals;
	
		
}// modelling

	releaseCUDA(rgbData);
	releaseCUDA(dData);
	releaseCUDA(palette);
	//releaseCUDA(nData);
	releaseCUDA(R);
	releaseCUDA(t);
	releaseCUDA(S);
	releaseCUDA(Xc);
	releaseCUDA(Yc);
	releaseCUDA(X);
	releaseCUDAandHost(one);
	releaseCUDA(C);
	releaseCUDA(lambda);
	releaseCUDA(Y);
	releaseCUDA(Xprime);
	releaseCUDA(YCenterd);
	releaseCUDA(A);
	killCUDAthread();
	
	return 0;
}
/**********************/
