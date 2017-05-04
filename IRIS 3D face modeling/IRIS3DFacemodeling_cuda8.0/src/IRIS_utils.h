
#include <opencv2/gpu/gpu.hpp>
#include "DATA.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Modeling.h"
#include "engine.h"

void init_RT(float *h_R, float *h_t);
void findFaceCenter(float *h_pReal, float *center_x, float *center_y, float NORMAL_FACOTR, float centroidX, float centroidY);
/**********************************************/
/* Create a rotation matrix from euler angles	*/
//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Donghyun Kim, Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

void eulerAngles2rotationMatrix(CvMat *out, float a, float b, float c);
/* Find if two matrices are equal	*/
void displayRendering(IplImage *img, float *h_R, CvPoint pt, float RScale);
/**********************************/
/* Find if two matrices are equal	*/
void displayRendering(IplImage *img, float *euler, CvPoint pt);
void findFaceCenter(float *h_pReal, float *center_x, float *center_y, float NORMAL_FACOTR, float centroidX, float centroidY);
void moveToFaceCenter(float *h_X, int nb_X, float center_x, float center_y);
void applyInvRotation(float *h_X, int nb_X, float *h_R);
/* Apply rotation	*/
void applyRotation(float *h_X, int nb_X, float *h_R);

/* Apply translation	*/
void applyTranslation(float *h_X, int nb_X, float *h_T);
void printPointClouds(float *h_x, int nb_X, char *filename, float NORMALIZATION_FACTOR, float centroidXr, float eyebrow_x, float eyebrow_y);
bool id(float *a, float *h_t);
void createPalette(int *palette);
void displayInfo(IplImage *img, bool saveFrames, bool depth, float fps);
float norm1D(float x);
float norm2D(float x, float y);
float norm3D(float x, float y, float z);
float N(float t, float sigma);
int findUnoccludedPoints(float *MNp, int *unoccluded);
void updatingModel(bool displayDepth, char *depthM, char *imM, char *mdData);
void computerNormals(Modeling &model, float* h_mNormals);
void setEngines(Modeling &model, float* h_mNormals);