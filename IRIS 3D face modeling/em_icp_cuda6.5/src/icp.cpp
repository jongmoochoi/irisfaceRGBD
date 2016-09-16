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
#ifndef WIN32
#include <unistd.h>
#endif

// uncomment if you do not use the viewer.
//#define NOVIEWER

#include "3dregistration.h"
#include "engine.h"


extern "C" {
  int sgemm_(const char *transa, const char *transb, const int *m, const int *n, const int *k,
             const float *alpha, const float *a, const int *lda, const float *b, const int *ldb,
             const float *beta, float *c, const int *ldc);
}

inline static float
distanceSquare(float x1, float y1, float z1,
	       float x2, float y2, float z2){

  float tmpx = (x1 - x2);
  float tmpy = (y1 - y2);
  float tmpz = (z1 - z2);

  return tmpx*tmpx + tmpy*tmpy + tmpz*tmpz;
}

inline static float
distanceSquareRT(float x1, float y1, float z1,
		 float x2, float y2, float z2, float* h_R, float* h_t){

  return distanceSquare(x1, y1, z1,
			(h_R[0]*x2 + h_R[1]*y2 + h_R[2]*z2) + h_t[0],
			(h_R[3]*x2 + h_R[4]*y2 + h_R[5]*z2) + h_t[1],
			(h_R[6]*x2 + h_R[7]*y2 + h_R[8]*z2) + h_t[2]);

}



#if 1
static void
findCenter(const float* h_X, const int Xsize,
	   float* h_Xc){

  const float* h_Xx = &h_X[Xsize*0];
  const float* h_Xy = &h_X[Xsize*1];
  const float* h_Xz = &h_X[Xsize*2];
  
  double Xcx = 0.0f, Xcy = 0.0f, Xcz = 0.0f;

#pragma omp parallel for reduction (+:Xcx,Xcy,Xcz)
  for(int i = 0; i < Xsize; i++){
    Xcx += h_Xx[i];
    Xcy += h_Xy[i];
    Xcz += h_Xz[i];
  }

  h_Xc[0] = (float) (Xcx / Xsize);
  h_Xc[1] = (float) (Xcy / Xsize);
  h_Xc[2] = (float) (Xcz / Xsize);

}

#else
static void
findCenter(const float* h_X, const int Xsize,
	   float* h_Xc){

  const float* h_Xx = &h_X[Xsize*0];
  const float* h_Xy = &h_X[Xsize*1];
  const float* h_Xz = &h_X[Xsize*2];
  
  h_Xc[0] = h_Xc[1] = h_Xc[2] = 0.0f;

  for(int i = 0; i < Xsize; i++){
    h_Xc[0] += h_Xx[i];
    h_Xc[1] += h_Xy[i];
    h_Xc[2] += h_Xz[i];
  }

  h_Xc[0] /= Xsize;
  h_Xc[1] /= Xsize;
  h_Xc[2] /= Xsize;

}
#endif


void icp(int Xsize, int Ysize,
         const float* h_X,
         const float* h_Y,
	 float* h_R, float* h_t, 
	 registrationParameters param
	 ){

  //
  // initialize paramters
  //
  int maxIteration = param.maxIteration;


  //
  // memory allocation
  //


  const float* h_Xx = &h_X[Xsize*0];
  const float* h_Xy = &h_X[Xsize*1];
  const float* h_Xz = &h_X[Xsize*2];

  const float* h_Yx = &h_Y[Ysize*0];
  const float* h_Yy = &h_Y[Ysize*1];
  const float* h_Yz = &h_Y[Ysize*2];

  float* h_Xcorr = new float[Ysize*3]; // points in X corresponding to Y
  float* h_Xcorrx = &h_Xcorr[Ysize*0];
  float* h_Xcorry = &h_Xcorr[Ysize*1];
  float* h_Xcorrz = &h_Xcorr[Ysize*2];

  float h_S[9];
  float h_Xc[3];
  float h_Yc[3];

  findCenter(h_Y, Ysize, h_Yc);


  // ICP main loop

  for(int iter=0; iter < maxIteration; iter++){


    // find closest points
#pragma omp parallel for
    for(int i = 0; i < Ysize; i++){
      float min_dist = 10e+10f;
      int min_index = 0;

      for(int j = 0; j < Xsize; j++){
	float dist = distanceSquareRT(h_Xx[j], h_Xy[j], h_Xz[j],
				      h_Yx[i], h_Yy[i], h_Yz[i],
				      h_R, h_t);
	if(dist < min_dist){
	  min_dist = dist;
	  min_index = j;	  
	}
      }

      // put the closest point to Ycorr
      h_Xcorrx[i] = h_Xx[min_index];
      h_Xcorry[i] = h_Xy[min_index];
      h_Xcorrz[i] = h_Xz[min_index];
    }



    // compute S

    {
      // SGEMM(TRANSA,TRANSB,M,N,K,ALPHA,A,LDA,B,LDB,BETA,C,LDC)
      //  C := alpha*op( A )*op( B ) + beta*C
      // 
      //  h_X^T * h_Y => S
      //  m*k     k*n    m*n
      int three = 3;
      float one = 1.0f, zero = 0.0f;
      sgemm_((char*)"t", (char*)"n", 
	     &three, &three, &Ysize, // m,n,k
	     &one, h_Xcorr, &Ysize, // alpha, op(A), lda
	     h_Y, &Ysize,  // op(B), ldb
	     &zero, h_S, &three);  // beta, C, ldc
    }
    
  
    findCenter(h_Xcorr, Ysize, h_Xc);



    // find RT from S

    findRTfromS(h_Xc, h_Yc, h_S, h_R, h_t, false);


#ifndef NOVIEWER
    if(!param.noviewer){
      UpdatePointCloud2(Ysize, param.points2, h_Y, h_R, h_t);
      if (!EngineIteration())
	break;
    }
#endif


  }


  delete [] h_Xcorr;


}


