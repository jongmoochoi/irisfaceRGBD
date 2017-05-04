//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Donghyun Kim, Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

#include "IRIS_utils.h"


void setEngines(Modeling &model, float* h_mNormals)
{
	for (int i = 0; i<MAX_IMG_INDEX_3; i++) {

		if (model.getHmodelRGB(i) == 0)
		{
			setEngineRGB(model.getHmodelRGB(i), i);
		}
		else
		{
			setEngineRGB(0.7, i);
		}
		if (model.getHdefined(i % MAX_IMG_INDEX_1)) {
			setEngineXYZ(model.getHmodelXYZ(i), i);
			setEngineNormals(h_mNormals[i], i);
		}
	}
}


void computerNormals(Modeling &model, float* h_mNormals)
{
	int i_first = THETA_MAX + 1;
	int i_last = (THETA_MAX - 2)*Y_MAX - 2;
	int i1 = 0, i2 = 0, i3 = 0, i4 = 0;
	float	x1 = 0.0f, y1 = 0.0f, z1 = 0.0f,
		x2 = 0.0f, y2 = 0.0f, z2 = 0.0f;
	float	nx = 0.0f, ny = 0.0f, nz = 0.0f;
	float	nn = 0.0f;


	for (int i1 = i_first; i1<i_last; i1++) {

		i2 = i1 + 1;				// x+1, y
		i3 = i1 + THETA_MAX;		// x, y+1
		i4 = i3 + 1;				// x+1, y+1

		if ((model.getHmodelXYZ(i1) != 0.0f || model.getHmodelXYZ(i1 + MAX_IMG_INDEX_1) != 0.0f || model.getHmodelXYZ(i1 + MAX_IMG_INDEX_2) != 0.0f)
			&& (model.getHmodelXYZ(i4) != 0.0f || model.getHmodelXYZ(i4 + MAX_IMG_INDEX_1) != 0.0f || model.getHmodelXYZ(i4 + MAX_IMG_INDEX_2) != 0.0f)
			&& (model.getHmodelXYZ(i2) != 0.0f || model.getHmodelXYZ(i2 + MAX_IMG_INDEX_1) != 0.0f || model.getHmodelXYZ(i2 + MAX_IMG_INDEX_2) != 0.0f)) {
			x1 = (model.getHmodelXYZ(i2) - model.getHmodelXYZ(i1));
			y1 = (model.getHmodelXYZ(i2 + MAX_IMG_INDEX_1) - model.getHmodelXYZ(i1 + MAX_IMG_INDEX_1));
			z1 = (model.getHmodelXYZ(i2 + MAX_IMG_INDEX_2) - model.getHmodelXYZ(i1 + MAX_IMG_INDEX_2));

			x2 = (model.getHmodelXYZ(i4) - model.getHmodelXYZ(i1));
			y2 = (model.getHmodelXYZ(i4 + MAX_IMG_INDEX_1) - model.getHmodelXYZ(i1 + MAX_IMG_INDEX_1));
			z2 = (model.getHmodelXYZ(i4 + MAX_IMG_INDEX_2) - model.getHmodelXYZ(i1 + MAX_IMG_INDEX_2));

			nx = y1*z2 - y2*z1;
			ny = z1*x2 - z2*x1;
			nz = x1*y2 - x2*y1;

			nn = pow(nx*nx + ny*ny + nz*nz, 0.5f);

			if (nn>0.0f) {
				h_mNormals[i1] = nx / nn;
				h_mNormals[i1 + MAX_IMG_INDEX_1] = ny / nn;

				h_mNormals[i1 + MAX_IMG_INDEX_2] = nz / nn;
			}
		}

	}
}


void updatingModel(bool displayDepth, char *depthM, char *imM,char *mdData)
{
	for (int y = 0; y<Y_MAX3; y += 3) {
		int ym = y*XN_VGA_X_RES;
		int yd = y*THETA_MAX;
		for (int t = 0; t<THETA_MAX3; t += 3) {
			int indm = (ym + t);
			int ind = (yd + t);

			if (displayDepth) {
				for (int k = 0; k<3; k++)
					depthM[indm + k] = mdData[ind + k];
			}
			else {
				for (int k = 0; k<3; k++)
					imM[indm + k] = mdData[ind + k];
			}
		}
	}
}

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
void findFaceCenter(float *h_pReal, float *center_x, float *center_y, float NORMAL_FACOTR, float centroidX, float centroidY)
{
	float	X = 0.0f, Y = 0.0f, Z = 0.0f;
	int i = (int)(*center_y)*XN_VGA_X_RES + (*center_x);

	X = h_pReal[i];
	Y = h_pReal[i + MAX_I];
	Z = h_pReal[i + MAX_I2];

	(*center_x) = (X - centroidX) / NORMAL_FACOTR;
	(*center_y) = (Y - centroidY) / NORMAL_FACOTR;
}


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


void printPointClouds(float *h_x, int nb_X, char *filename, float NORMALIZATION_FACTOR, float centroidXr,float eyebrow_x, float eyebrow_y)
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
		fprintf(f, "v %f %f 0 255 255 0\n", (centroidXr / NORMALIZATION_FACTOR) + i, (centroidXr / NORMALIZATION_FACTOR) + i);
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