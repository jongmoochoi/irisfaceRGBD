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

#include "DATA.h"

#ifndef _3DREGISTRATION_H_
#define _3DREGISTRATION_H_


// threads in a 2D block is BLOCK_SIZE*BLOCK_SIZE


typedef struct {

  // for softassign
  int JMAX;      // Number of iterations for the annealing loop (1st loop). At first, the annealing temprature T is set to T_0, then T <- T*TFACOR at the end of each iteration. default: 100
  int I0;        // Number of iterations with the same temprature (2nd loop). default: 5
  int I1;        // Number of iterations for Shinkhorn’s row/column normalizations. default: 3
  float alpha;   // parameter for outliers (see the original Softassign paper). default: 3.0
  float T_0;     // initial temprature for the annealing process. default: 100.0
  float TFACTOR; // factor for reducing temprature. default: 0.95
  float moutlier;// values of elements in the extra row/column for outliers (see the original Softassign paper). default: 1/sqrtf(T_0)*expf(-1.0f)

  // for EM-ICP
  float sigma_p2;      // initial value for the main loop. sigma_p2 <- sigma_p2 * sigma_factor  at the end of each iteration while sigma_p2 > sigam_inf. default: 0.01
  float sigma_inf;     // minimum value of sigma_p2. default: 0.00001
  float sigma_factor;  // facfor for reducing sigma_p2. default: 0.9
  float d_02;          // values for outlier (see EM-ICP paper). default: 0.01

  // for ICP
  int maxIteration; // Number of ICP iterations. default: 30

  // misc
  int noviewer; // No viewer is shown. Just align point sets, and quit.
  int nostop;   // No interatction by the viewer is required.
  int notimer;  // No timer is shown.

  int argc;
  char **argv;

  float *points1;
  float *points2;
  float *points3;
  
} registrationParameters;


// uncomment if you do not use the viewer.
//#define NOVIEWER


void softassign(const int Xsize, const int Ysize,
		const float* h_X,
		const float* h_Y,
		float* h_R, float* h_t, 
		registrationParameters param
		);

void emicp(int Xsize, int Ysize,
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
		registrationParameters param,
		bool *error,
		bool *allocateMemory
	   );
void initCUDA(int Ysize);
void releaseCUDAmemory ();
void emicp_cpu(int Xsize, int Ysize,
               const float* h_X,
               const float* h_Y,
               float* h_R, float* h_t,
					registrationParameters param,
					bool *error
					);

void icp(int Xsize, int Ysize,
         const float* h_X,
         const float* h_Y,
	 float* h_R, float* h_t, 
	 registrationParameters param);




void findRTfromS(const float* h_Xc,
		 const float* h_Yc,
		 const float* h_S,
		 float* h_R, float* h_t,
		 bool *error);
void rotationMatrix2eulerAngles(float *R, 
										  float *a);
void eulerAngles2rotationMatrix(float *R, float *a)	;
void multiply(float *R1, float *R2, float *R3);

float tr_diff(float *t, 
				  float *pre_t, 
				  float *a, 
				  float *pre_a);

void UpdatePointCloud2(int Ysize, float* points2,
		       const float* h_Y, const float* h_R, const float* h_t);


void printRT(const float* R, const float* t, float *a);
void printR(float *a);


#endif
