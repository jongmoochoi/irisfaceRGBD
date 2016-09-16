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
#include <algorithm>



extern "C" {
  int sgemm_(const char *transa, const char *transb, const int *m, const int *n, const int *k,
             const float *alpha, const float *a, const int *lda, const float *b, const int *ldb,
             const float *beta, float *c, const int *ldc);
  int saxpy_(const int *n, const float *sa, const float *sx, const int *incx, float *sy, const int *incy);
  int sgemv_(const char *trans, const int *m, const int *n,
             const float *alpha, const float *a, const int *lda,
             const float *x, const int *incx, const float *beta, float *y, const int *incy);
  float sasum_(const int *n, const float *sx, const int *incx);
  int sscal_(const int *n, const float *sa, float *sx, const int *incx);
}

// uncomment if you do not use the viewer.
//#define NOVIEWER

#include "3dregistration.h"
#include "engine.h"

using namespace std;








static void
updateA(int rowsA, int colsA, int pitchA,
	const float* h_Xx, const float* h_Xy, const float* h_Xz, 
	const float* h_Yx, const float* h_Yy, const float* h_Yz,
	const float* h_R, const float* h_t,
	float* h_A,
	float sigma_p2){

	int j=0;
	const int gap=pitchA-rowsA; 

#pragma omp parallel for
  for(int c=0; c<colsA; c++){

    float Xx = h_Xx[c];
    float Xy = h_Xy[c];
    float Xz = h_Xz[c];

    for(int r=0; r<rowsA; r++){

      float Yx = h_Yx[r];
      float Yy = h_Yy[r];
      float Yz = h_Yz[r];

#define R(i) h_R[i]
#define t(i) h_t[i]

      // #define Euclid(a,b,c) ((a)*(a)+(b)*(b)+(c)*(c))
      //     float tmp =
      //       Euclid(Xx - (R(0)*Yx + R(1)*Yy + R(2)*Yz + t(0)),
      //              Xy - (R(3)*Yx + R(4)*Yy + R(5)*Yz + t(1)),
      //              Xz - (R(6)*Yx + R(7)*Yy + R(8)*Yz + t(2)) );
    
      //     tmp = expf(-tmp/sigma_p^2)

      float tmpX = Xx - (R(0)*Yx + R(1)*Yy + R(2)*Yz + t(0));
      float tmpY = Xy - (R(3)*Yx + R(4)*Yy + R(5)*Yz + t(1));
      float tmpZ = Xz - (R(6)*Yx + R(7)*Yy + R(8)*Yz + t(2));

      tmpX *= tmpX;
      tmpY *= tmpY;
      tmpZ *= tmpZ;

      tmpX += tmpY;
      tmpX += tmpZ;

      tmpX /= sigma_p2;
      tmpX = expf(-tmpX);

      h_A[j++/*c * pitchA + r*/] = tmpX;

    }
	 j+=gap;
  }
}


static void
normalizeRowsOfA(int rowsA, int colsA, int pitchA,
		 float *h_A,
		 const float *h_C){
  
#pragma omp parallel for
	//for(int c=0; c<colsA; c++)
	//  for(int r=0; r<rowsA; r++)
	//    if(h_C[r] > 10e-7f)
	//			// each element in A is normalized in C, then square-rooted
	//			h_A[c * pitchA + r] = sqrtf( h_A[c * pitchA + r] / h_C[r]);
	//    else
	//			h_A[c * pitchA + r] = 1.0f/colsA; // ad_hoc code to avoid 0 division
	
	//	MODIFIED		 
	const int maxA = colsA*pitchA, gap=pitchA-rowsA;
	int r=0;
	for (int i=0; i<maxA; i++) {
		if(h_C[r] > 10e-7f)
			// each element in A is normalized in C, then square-rooted
			h_A[i] = sqrtf( h_A[i] / h_C[r]);
	   else
			h_A[i] = 1.0f/colsA; // ad_hoc code to avoid 0 division
		if (++r == rowsA) {
			i+=gap;
			r=0;
		}
	}
}

static void
elementwiseDivision(int Xsize,
		    float* h_Xx, float* h_Xy, float* h_Xz,
		    const float* h_lambda){

#pragma omp parallel for
  for(int x=0; x<Xsize; x++){
    float l_lambda = h_lambda[x];
    h_Xx[x] /= l_lambda;
    h_Xy[x] /= l_lambda;
    h_Xz[x] /= l_lambda;
  }

}

static void
elementwiseMultiplication(int Xsize,
			  float* h_Xx, float* h_Xy, float* h_Xz,
			  const float* h_lambda){

#pragma omp parallel for
  for(int x=0; x<Xsize; x++){
    float l_lambda = h_lambda[x];
    h_Xx[x] *= l_lambda;
    h_Xy[x] *= l_lambda;
    h_Xz[x] *= l_lambda;
  }
}



static void
centeringXandY(int rowsA,
	       const float* h_Xc, const float* h_Yc,
	       const float* h_Xx, const float* h_Xy, const float* h_Xz,
	       const float* h_Yx, const float* h_Yy, const float* h_Yz,
	       float* h_XxCenterd, float* h_XyCenterd, float* h_XzCenterd,
	       float* h_YxCenterd, float* h_YyCenterd, float* h_YzCenterd){

  // do for both X and Y at the same time
  
#pragma omp parallel for
  for(int r=0; r<rowsA; r++){
    h_XxCenterd[r] = h_Xx[r] - h_Xc[0];
    h_XyCenterd[r] = h_Xy[r] - h_Xc[1];
    h_XzCenterd[r] = h_Xz[r] - h_Xc[2];
    
    h_YxCenterd[r] = h_Yx[r] - h_Yc[0];
    h_YyCenterd[r] = h_Yy[r] - h_Yc[1];
    h_YzCenterd[r] = h_Yz[r] - h_Yc[2];
  }

}







void emicp_cpu(int Xsize, int Ysize,
	       const float* h_X,
               const float* h_Y,
	       float* h_R, float* h_t, 
	       registrationParameters param,
			 bool *error
	       ){

  // const for blas functions
  float onef = 1.0f;
  float zerof = 0.0f;
  int onei = 1;
  int threei = 3;
  char transN = 'n';
  char transT = 't';


  //
  // initialize paramters
  //
  float sigma_p2 = param.sigma_p2;
  float sigma_inf = param.sigma_inf;
  float sigma_factor = param.sigma_factor;
  float d_02 = param.d_02;


  //
  // memory allocation
  //

  const float* h_Xx = &h_X[Xsize*0];
  const float* h_Xy = &h_X[Xsize*1];
  const float* h_Xz = &h_X[Xsize*2];

  const float* h_Yx = &h_Y[Ysize*0];
  const float* h_Yy = &h_Y[Ysize*1];
  const float* h_Yz = &h_Y[Ysize*2];

  float *h_Xprime = new float [Ysize*3];
  float *h_XprimeX = &h_Xprime[Ysize*0];
  float *h_XprimeY = &h_Xprime[Ysize*1];
  float *h_XprimeZ = &h_Xprime[Ysize*2];

  float *h_XprimeCenterd = h_Xprime;
  float *h_XprimeCenterdX = &h_XprimeCenterd[Ysize*0];
  float *h_XprimeCenterdY = &h_XprimeCenterd[Ysize*1];
  float *h_XprimeCenterdZ = &h_XprimeCenterd[Ysize*2];

  float *h_YCenterd = new float [Ysize*3];
  float *h_YCenterdX = &h_YCenterd[Ysize*0];
  float *h_YCenterdY = &h_YCenterd[Ysize*1];
  float *h_YCenterdZ = &h_YCenterd[Ysize*2];


  // center of X, Y
  float h_Xc[3], h_Yc[3];

  // S for finding R, t
  float h_S[9];




  // NOTE on matrix A
  // number of rows:     Ysize, or rowsA
  // number of columns : Xsize, or colsA
  // 
  //                    [0th in X] [1st]  ... [(Xsize-1)] 
  // [0th point in Y] [ A(0,0)     A(0,1) ... A(0,Xsize-1)      ] 
  // [1st           ] [ A(1,0)     A(1,1) ...                   ]
  // ...              [ ...                                     ]
  // [(Ysize-1)     ] [ A(Ysize-1, 0)     ... A(Ysize-1,Xsize-1)]
  //
  // 
  // CAUTION on matrix A
  // A is allcoated as a column-maijor format for the use of cublas.
  // This means that you must acces an element at row r and column c as:
  // A(r,c) = A[c * pitchA + r]


  int rowsA = Ysize;
  int colsA = Xsize;

  // pitchA: leading dimension of A, which is ideally equal to rowsA,
  //          but actually larger than that.
  int pitchA = (rowsA / 4 + 1) * 4;

  float *h_A = new float [pitchA*colsA];


  // a vector with all elements of 1.0f
  float* h_one = new float [max(Xsize,Ysize)];
  for(int t = 0; t < max(Xsize,Ysize); t++) h_one[t] = 1.0f;


  float *h_C = new float [rowsA]; // sum of a row in A
  float *h_lambda = new float [rowsA]; // weight of a row in A



  //
  // timer, notimer
  //

#define START_TIMER(timer)
#define STOP_TIMER(timer)




  // EM-ICP main loop
  int Titer = 1;
  while(sigma_p2 > sigma_inf){

    fprintf(stderr, "%d iter. sigma_p2 %f  \n", Titer++, sigma_p2);
    // fprintf(stderr, "time %.10f [s]\n", cutGetTimerValue(timerTotal) / 1000.0f);


#ifndef NOVIEWER
      if(!param.noviewer)
	if (!EngineIteration()) // PointCloudViewer
	  break;
#endif

      //
      // UpdateA
      //

      START_TIMER(timerUpdateA);

	  updateA
	    (rowsA, colsA, pitchA,
	     h_Xx, h_Xy, h_Xz, 
	     h_Yx, h_Yy, h_Yz,
	     h_R, h_t, 
	     h_A, sigma_p2);

      STOP_TIMER(timerUpdateA);



      //
      // Normalization of A
      //

      // cublasSgemv (char trans, int m, int n, float alpha, const float *A, int lda,
      //              const float *x, int incx, float beta, float *y, int incy)
      // int sgemv_(char *trans, int *m, int *n,
      // 	     float *alpha, float *a, int *lda,
      // 	     float *x, int *incx, float *beta, float *y, int *incy);
      //    y = alpha * op(A) * x + beta * y,

      // A * one vector = vector with elements of row-wise sum
      //     h_A      *    h_one    =>  h_C
      //(rowsA*colsA) *  (colsA*1)  =  (rowsA*1)
      sgemv_(&transN,          // char trans
	     &rowsA, &colsA, // int m (rows of A), n (cols of A) ; not op(A)
	     &onef,         // float alpha
	     h_A, &pitchA,  // const float *A, int lda
	     h_one, &onei,     // const float *x, int incx
	     &zerof,         // float beta
	     h_C, &onei);      // float *y, int incy


      // void cublasSaxpy (int n, float alpha, const float *x, int incx, float *y, int incy)
      // int saxpy_(int *n, float *sa, float *sx, int *incx, float *sy, int *incy);
      // alpha * x + y => y
      // exp(-d_0^2/sigma_p2) * h_one + h_C => h_C
      {
	float alpha = expf(-d_02/sigma_p2);
	saxpy_(&rowsA, &alpha, h_one, &onei, h_C, &onei);
      }


      normalizeRowsOfA
	(rowsA, colsA, pitchA, h_A, h_C);



      //
      // update R,T
      //

      ///////////////////////////////////////////////////////////////////////////////////// 

      // compute lambda

      // int sgemv_(char *trans, int *m, int *n,
      // 		 float *alpha, float *a, int *lda,
      // 		 float *x, int *incx, float *beta, float *y, int *incy);
      // A * one vector = vector with elements of row-wise sum
      //     h_A      *    h_one    =>  h_lambda
      //(rowsA*colsA) *  (colsA*1)  =  (rowsA*1)
      sgemv_(&transN,          // char trans
	     &rowsA, &colsA, // int m (rows of A), n (cols of A) ; not op(A)
	     &onef,         // float alpha
	     h_A, &pitchA,  // const float *A, int lda
	     h_one, &onei,     // const float *x, int incx
	     &zerof,         // float beta
	     h_lambda, &onei); // float *y, int incy


      // float cublasSasum (int n, const float *x, int incx) 
      //   float sasum_(int *n, float *sx, int *incx);
      float sumLambda = sasum_(&rowsA, h_lambda, &onei);



      ///////////////////////////////////////////////////////////////////////////////////// 

      // compute X'

      // cublasSgemm (char transa, char transb, int m, int n, int k, float alpha, 
      //              const float *A, int lda, const float *B, int ldb, float beta, 
      //              float *C, int ldc)
      //   C = alpha * op(A) * op(B) + beta * C,
      //
      // m      number of rows of matrix op(A) and rows of matrix C
      // n      number of columns of matrix op(B) and number of columns of C
      // k      number of columns of matrix op(A) and number of rows of op(B) 

      // A * X => X'
      //     h_A      *    h_X    =>  h_Xprime
      //(rowsA*colsA) *  (colsA*3)  =  (rowsA*3)
      //   m  * k           k * n        m * n   
      sgemm_(&transN, &transN, &rowsA, &threei, &colsA,
	     &onef, h_A, &pitchA,
	     h_X, &colsA,
	     &zerof, h_Xprime, &rowsA);


      // X' ./ lambda => X'
      elementwiseDivision
	(rowsA, h_XprimeX, h_XprimeY, h_XprimeZ, h_lambda);


      ///////////////////////////////////////////////////////////////////////////////////// 

      //
      // centering X' and Y
      //

      ///////////////////////////////////////////////////////////////////////////////////// 

      // find weighted center of X' and Y

      // h_Xprime^T *    h_lambda     =>   h_Xc
      //  (3 * rowsA)   (rowsA * 1)  =  (3 * 1)
      sgemv_(&transT,          // char trans
	     &rowsA, &threei,     // int m (rows of A), n (cols of A) ; not op(A)
	     &onef,         // float alpha
	     h_Xprime, &rowsA,  // const float *A, int lda
	     h_lambda, &onei,     // const float *x, int incx
	     &zerof,         // float beta
	     h_Xc, &onei);     // float *y, int incy

      // h_Y^T *    h_lambda     =>   h_Yc
      //  (3 * rowsA)   (rowsA * 1)  =  (3 * 1)
      sgemv_(&transT,          // char trans
	     &rowsA, &threei,     // int m (rows of A), n (cols of A) ; not op(A)
	     &onef,         // float alpha
	     h_Y, &rowsA,  // const float *A, int lda
	     h_lambda, &onei,     // const float *x, int incx
	     &zerof,         // float beta
	     h_Yc, &onei);     // float *y, int incy

      // void cublasSscal (int n, float alpha, float *x, int incx)
      // int sscal_(int *n, float *sa, float *sx, int *incx);
      // it replaces x[ix + i * incx] with alpha * x[ix + i * incx]
      {
	float alpha = 1.0f/sumLambda;
	sscal_(&threei, &alpha, h_Xc, &onei);
	sscal_(&threei, &alpha, h_Yc, &onei);
      }


      ///////////////////////////////////////////////////////////////////////////////////// 

      // centering X and Y

      // h_Xprime .- h_Xc => h_XprimeCenterd
      // h_Y      .- h_Yc => h_YCenterd
	centeringXandY
	  (rowsA, 
	   h_Xc, h_Yc,
	   h_XprimeX, h_XprimeY, h_XprimeZ,
	   h_Yx, h_Yy, h_Yz,
	   h_XprimeCenterdX, h_XprimeCenterdY, h_XprimeCenterdZ,
	   h_YCenterdX, h_YCenterdY, h_YCenterdZ);



      // XprimeCented .* h_lambda => XprimeCented
      elementwiseMultiplication
	(rowsA, h_XprimeCenterdX, h_XprimeCenterdY, h_XprimeCenterdZ, h_lambda);



      ///////////////////////////////////////////////////////////////////////////////////// 

      // compute S

      //  h_XprimeCented^T *   h_YCenterd     =>  h_S
      //    (3*rowsA)  *  (rowsA*3)  =  (3*3)
      //   m  * k           k * n        m * n
      sgemm_(&transT, &transN, &threei, &threei, &rowsA,
	     &onef, h_XprimeCenterd, &rowsA,
	     h_YCenterd, &rowsA,
	     &zerof, h_S, &threei);


      ///////////////////////////////////////////////////////////////////////////////////// 


      // find RT from S

      START_TIMER(timerAfterSVD);

      findRTfromS(h_Xc, h_Yc, h_S, h_R, h_t, error);
		if (*error)
			break;

      STOP_TIMER(timerAfterSVD);


      ///////////////////////////////////////////////////////////////////////////////////// 

#ifndef NOVIEWER
      if(!param.noviewer)
	UpdatePointCloud2(Ysize, param.points2, h_Y, h_R, h_t);
#endif


    sigma_p2 *= sigma_factor;
  }







  delete [] h_Xprime;
  delete [] h_YCenterd;

  delete [] h_A;

  delete [] h_one;

  delete [] h_C;
  delete [] h_lambda;


}
