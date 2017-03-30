//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.


#include "stdafx.h"
#include "3dregistration.h"
#include "FaceBox.h"
#include "FaceDetector.h"


#include "Voxel.h"


void errorCheck(Status nRetVal);



void displayImageMap(char* out, 
							const OniRGB888Pixel* pImageMap);


void extractFace(	float *h_face,
					float *h_pReal,
					FaceBox maskBox, 
					bool *validityMask,
					int *nb_pts, 
					float *normalization_factor,
					Voxel *centroid, 
					bool first);
FaceBox findBox(IplImage *mask);

bool isInside(int x, int y);


void set_normalizePoints(float *h_X, 
					int nb, 
					Voxel *pts, 
					bool first, 
					float *normalization_factor,
					Voxel *centroid);
void set_normalizePoints(float *h_X, int nb, int *pts, float *h_pReal, bool first, float *normalization_factor, Voxel *centroid);

void selectRandomPointOnFace(	int *selectedPoints, 
								int height, 
								int width, 
								int topY, 
								int leftX, 
								int nb, 
								bool *validityMask,
								int *nbb) ;
void selectRandomPointOnFace(	int *selectedPoints, 
								int nb_points_on_face,
								int nb_random_samples);
void selectRandomPointOnFace(	int *selectedPoints, 
								int *index_list,
								int nb_points_on_face,
								int nb_random_samples);
void setH_X(float *h_X, int nb, int *pts, float *h_face) ;


void setH_Yhalf(float *h_Y, int nbY, int *pts, int nb_pts, float *h_pts);
void setH_Y(float *h_Y, float *Mp, int *unoccluded, int nb_unoccluded) ;

void swapPoints(float *h_X, float *h_Y, int nb_X);
void findNbHalf(	float *h_Y, 
						int nb,  
						int *nb_left,
						int *nb_right	);
void splitHalf(	float *h_Y, 
						int nb, 
						float *h_Yleft, 
						int nb_left,
						float *h_Yright, 
						int nb_right	);

void poseEstimation(	int Xsize, int Ysize, 
						float *h_X, float *d_X, float *d_Xx, float *d_Xy, float *d_Xz,
						float* h_Y, float *d_Y, float *d_Yx, float *d_Yy, float *d_Yz,
						float* h_R, float *d_R, float* h_t, float *d_t,
						float* h_S, float *d_S,
						float *h_Xc, float *d_Xc, float *h_Yc, float *d_Yc,
						float *h_one, float *d_one,
						float *d_A,
						float *d_Xprime, float *d_XprimeX, float *d_XprimeY, float *d_XprimeZ,
						float *d_XprimeCenterd, float *d_XprimeCenterdX, float *d_XprimeCenterdY, float *d_XprimeCenterdZ,
						float *d_YCenterd, float *d_YCenterdX, float *d_YCenterdY, float *d_YCenterdZ,
						float *d_C, float *d_lambda,
						int	maxXY, int rowsA, int colsA, int pitchA,
						registrationParameters param,	// registration parameters
						float *a, float *pre_a, float *pre_t,
						bool *allocateMemory,
						float *diff );


void gradient(float *in, float *horizontal, float *vertical, int w, int h, int W, bool *validityMask);


void InitPointCloud(const float* h_X, const int Xsize,
		    const float* h_Y, const int Ysize,
		    float* &points1, float* &points2, float* &points3);
void InitPointCloud(float* h_X, const int Xsize, bool *h_validity,
		    float* &points1);