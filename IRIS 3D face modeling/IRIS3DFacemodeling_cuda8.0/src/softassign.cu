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

#include <cutil.h>
#include <cublas.h>

// uncomment if you do not use the viewer.
//#define NOVIEWER

#include "3dregistration.h"
#include "engine.h"

using namespace std;




__global__ static void
updateM(int rowsM, int colsM, int pitchM,
	float* d_Xx, float* d_Xy, float* d_Xz, 
	float* d_Yx, float* d_Yy, float* d_Yz,
	float* d_R, float* d_t,
	float* d_M,
	float T_cur, float alpha){
  

  int r =  blockIdx.x * blockDim.x + threadIdx.x;
  int c =  blockIdx.y * blockDim.y + threadIdx.y;

  // Shared memory
  __shared__ float XxShare[BLOCK_SIZE];
  __shared__ float XyShare[BLOCK_SIZE];
  __shared__ float XzShare[BLOCK_SIZE];
  __shared__ float YxShare[BLOCK_SIZE];
  __shared__ float YyShare[BLOCK_SIZE];
  __shared__ float YzShare[BLOCK_SIZE];
  __shared__ float RShare[9]; // BLOCK_SIZE >= 9 is assumed
  __shared__ float tShare[3]; // BLOCK_SIZE >= 3 is assumed
  
  if(threadIdx.y == 0)
    if(threadIdx.x < 9){
      RShare[threadIdx.x] = d_R[threadIdx.x];
      if(threadIdx.x < 3)
	tShare[threadIdx.x] = d_t[threadIdx.x];
    }
  
  if(r < rowsM && c < colsM){ // check for only inside the matrix M
    
    if(threadIdx.y == 0){
      XxShare[threadIdx.x] = d_Xx[r];
      XyShare[threadIdx.x] = d_Xy[r];
      XzShare[threadIdx.x] = d_Xz[r];
    }
    if(threadIdx.x == 0){
      YxShare[threadIdx.y] = d_Yx[c];
      YyShare[threadIdx.y] = d_Yy[c];
      YzShare[threadIdx.y] = d_Yz[c];
    }

    __syncthreads();

#define Xx XxShare[threadIdx.x]
#define Xy XyShare[threadIdx.x]
#define Xz XzShare[threadIdx.x]
#define Yx YxShare[threadIdx.y]
#define Yy YyShare[threadIdx.y]
#define Yz YzShare[threadIdx.y]
#define R(i) RShare[i]
#define t(i) tShare[i]

// #define Euclid(a,b,c) ((a)*(a)+(b)*(b)+(c)*(c))
//     float tmp =
//       Euclid(Xx - (R(0)*Yx + R(1)*Yy + R(2)*Yz + t(0)),
//      	     Xy - (R(3)*Yx + R(4)*Yy + R(5)*Yz + t(1)),
//      	     Xz - (R(6)*Yx + R(7)*Yy + R(8)*Yz + t(2)) ) - alpha;
    
//     tmp = expf(-tmp/T_cur) / sqrtf(T_cur);

     float tmpX = Xx - (R(0)*Yx + R(1)*Yy + R(2)*Yz + t(0));
     float tmpY = Xy - (R(3)*Yx + R(4)*Yy + R(5)*Yz + t(1));
     float tmpZ = Xz - (R(6)*Yx + R(7)*Yy + R(8)*Yz + t(2));

#undef Xx
#undef Xy
#undef Xz
#undef Yx
#undef Yy
#undef Yz
#undef R
#undef t

    __syncthreads();

     tmpX *= tmpX;
     tmpY *= tmpY;
     tmpZ *= tmpZ;

     tmpX += tmpY;
     tmpX += tmpZ;
     tmpX -= alpha;

     tmpX /= T_cur;
     tmpX = expf(-tmpX);
     tmpX /= sqrtf(T_cur);



     d_M[c * pitchM + r] = tmpX;
  }


}

__global__ static void
normalizeMbySinkhorn_row(int rowsM, int colsM, int pitchM,
			 float *d_M,
			 const float *d_sumOfRow,
			 float *d_m_outliers_row
			 ){

  int r =  blockIdx.x * blockDim.x + threadIdx.x;
  int c =  blockIdx.y * blockDim.y + threadIdx.y;

  // Shared memory
  __shared__ float sumOfRowShare[BLOCK_SIZE];


  if(r < rowsM && c < colsM){ // check for only inside the matrix M

    if(threadIdx.y == 0)
      sumOfRowShare[threadIdx.x] = d_sumOfRow[r];

    __syncthreads();

    d_M[c * pitchM + r] /= sumOfRowShare[threadIdx.x];

    if(c == 0) d_m_outliers_row[r] /= sumOfRowShare[threadIdx.x];

    __syncthreads();

  }

}


__global__ static void
normalizeMbySinkhorn_col(int rowsM, int colsM, int pitchM,
			 float *d_M,
			 const float *d_sumOfCol,
			 float *d_m_outliers_col
			 ){

  int r =  blockIdx.x * blockDim.x + threadIdx.x;
  int c =  blockIdx.y * blockDim.y + threadIdx.y;

  // Shared memory
  __shared__ float sumOfColShare[BLOCK_SIZE];

  if(r < rowsM && c < colsM){ // check for only inside the matrix M

    if(threadIdx.x == 0)
      sumOfColShare[threadIdx.y] = d_sumOfCol[c];

    __syncthreads();

    d_M[c * pitchM + r] /= sumOfColShare[threadIdx.y];
    
    if(r == 0) d_m_outliers_col[c] /= sumOfColShare[threadIdx.y];

    __syncthreads();

  }

}



__global__ static void
elementwiseMultiplicationCopy(int rowsM,
			      const float* d_Xx, const float* d_Xy, const float* d_Xz,
			      const float* d_sumOfMRow,
			      float* d_Xx_result, float* d_Xy_result, float* d_Xz_result){

  int r =  blockIdx.x * blockDim.x + threadIdx.x;

  float l_sumOfRow = d_sumOfMRow[r];

  if(r < rowsM){ // check for only inside the matrix M
    d_Xx_result[r] = l_sumOfRow * d_Xx[r];
    d_Xy_result[r] = l_sumOfRow * d_Xy[r];
    d_Xz_result[r] = l_sumOfRow * d_Xz[r];
  }
}



__global__ static void
centeringXorY(int rowsM,
	      const float* d_Xc, float sum,
	      float* d_Xx_result, float* d_Xy_result, float* d_Xz_result){

  // can be work for both row and column
  
  int r =  blockIdx.x * blockDim.x + threadIdx.x;

  // Shared memory
  __shared__ float Xc[3];

  if(threadIdx.x < 3) Xc[threadIdx.x] = d_Xc[threadIdx.x];


  if(r < rowsM){ // check for only inside the matrix M

    __syncthreads();

    d_Xx_result[r] -= Xc[0];
    d_Xy_result[r] -= Xc[1];
    d_Xz_result[r] -= Xc[2];

    __syncthreads();

  }
}





void softassign(const int Xsize, const int Ysize,
		const float* h_X,
		const float* h_Y,
		float* h_R, float* h_t, 
		registrationParameters param
		){

  //
  // initialize paramters
  //

  int JMAX = param.JMAX;
  int I0 = param.I0;
  int I1 = param.I1;
  float T_cur = param.T_0; // current temprature
  float alpha = param.alpha;
  float TFACTR = param.TFACTOR;
  float moutlier = param.moutlier;



  //
  // initialize CUDA
  //
  CUT_DEVICE_INIT(param.argc, param.argv);



  //
  // memory allocation
  //


  // example: memCUDA(Xx, Xsize);   // declare d_Xx. no copy.
#define memCUDA(var,num)						\
  float* d_ ## var; CUDA_SAFE_CALL(cudaMalloc((void**) &(d_ ## var), sizeof(float)*num));


  // example:   memHostToCUDA(Xx, Xsize);   // declera d_Xx, then copy h_Xx to d_Xx.
#define memHostToCUDA(var,num)						\
  float* d_ ## var; CUDA_SAFE_CALL(cudaMalloc((void**) &(d_ ## var), sizeof(float)*num)); \
  CUDA_SAFE_CALL(cudaMemcpy(d_ ## var, h_ ## var, sizeof(float)*num, cudaMemcpyHostToDevice));



  memHostToCUDA(X, Xsize*3);
  float* d_Xx = &d_X[Xsize*0];
  float* d_Xy = &d_X[Xsize*1];
  float* d_Xz = &d_X[Xsize*2];

  memHostToCUDA(Y, Ysize*3);
  float* d_Yx = &d_Y[Ysize*0];
  float* d_Yy = &d_Y[Ysize*1];
  float* d_Yz = &d_Y[Ysize*2];

  memCUDA(X_result, Xsize*3);
  float *d_Xx_result = &d_X_result[Xsize*0];
  float *d_Xy_result = &d_X_result[Xsize*1];
  float *d_Xz_result = &d_X_result[Xsize*2];

  memCUDA(Y_result, Ysize*3);
  float *d_Yx_result = &d_Y_result[Ysize*0];
  float *d_Yy_result = &d_Y_result[Ysize*1];
  float *d_Yz_result = &d_Y_result[Ysize*2];

  // center of X, Y
  float h_Xc[3], h_Yc[3];
  memCUDA(Xc, 3);
  memCUDA(Yc, 3);

  // R, t
  memHostToCUDA(R, 3*3);
  memHostToCUDA(t, 3);

  // S for finding R, t
  float h_S[9];
  memCUDA(S, 9);


  // NOTE on matrix M
  // number of rows:     Xsize, or rowsM
  // number of columns : Ysize, or colsM
  // 
  //                    [0th in Y] [1st]  ... [(Ysize-1)] 
  // [0th point in X] [ M(0,0)     M(0,1) ... M(0,Ysize-1)      ] 
  // [1st           ] [ M(1,0)     M(1,1) ...                   ]
  // ...              [ ...                                     ]
  // [(Xsize-1)     ] [ M(Xsize-1, 0)     ... M(Xsize-1,Ysize-1)]
  //
  // 
  // CAUTION on matrix M
  // M is allcoated as a column-maijor format for the use of cublas.
  // This means that you must acces an element at row r and column c as:
  // M(r,c) = M[c * pitchM + r]

  int rowsM = Xsize;
  int colsM = Ysize;

  // pitchM: leading dimension of M, which is ideally equal to rowsM,
  //          but actually larger than that.
  int pitchM = (rowsM / 4 + 1) * 4;

  memCUDA(M, pitchM*colsM);

  // fprintf(stderr, "rowsM, rowsM*sizeof(float), colsM : %d %d %d\n",
  // 	  rowsM, rowsM * sizeof(float), colsM)


  memCUDA(D, 3*rowsM); // temporary vector


  // a vector with all elements of 1.0f
  float* h_one = new float [max(Xsize,Ysize)];
  for(int t = 0; t < max(Xsize,Ysize); t++) h_one[t] = 1.0f;
  memHostToCUDA(one, max(Xsize,Ysize)); // vector with all elements of 1


  memCUDA(sumOfMRow, rowsM);
  memCUDA(sumOfMCol, colsM);

  float* h_m_outliers_row = new float [rowsM]; 
  float* h_m_outliers_col = new float [colsM];
  for(int i = 0; i < rowsM; i++) h_m_outliers_row[i] = moutlier;
  for(int i = 0; i < colsM; i++) h_m_outliers_col[i] = moutlier;
  memHostToCUDA(m_outliers_row, rowsM);
  memHostToCUDA(m_outliers_col, colsM);


  

  //
  // threads
  //


  // for 2D block
  dim3 dimBlockForM(BLOCK_SIZE, BLOCK_SIZE); // a block is (BLOCK_SIZE*BLOCK_SIZE) threads
  dim3 dimGridForM( (pitchM + dimBlockForM.x - 1) / dimBlockForM.x,
		    (colsM  + dimBlockForM.y - 1) / dimBlockForM.y);

  // for 1D block
  int threadsPerBlockForYsize = 512; // a block is 512 threads
  int blocksPerGridForYsize
    = (Ysize + threadsPerBlockForYsize - 1 ) / threadsPerBlockForYsize;
  int threadsPerBlockForXsize = 512; // a block is 512 threads
  int blocksPerGridForXsize
    = (Xsize + threadsPerBlockForXsize - 1 ) / threadsPerBlockForYsize;







  //
  // timer
  //


#define START_TIMER(timer) \
  if(!param.notimer){ \
      CUDA_SAFE_CALL( cudaThreadSynchronize() );\
      CUT_SAFE_CALL(cutStartTimer(timer)); \
  }
#define STOP_TIMER(timer) \
  if(!param.notimer){ \
      CUDA_SAFE_CALL( cudaThreadSynchronize() );\
      CUT_SAFE_CALL(cutStopTimer(timer)); \
  }


  // timers
  unsigned int timerTotal, 
    timerUpdateM, timerShinkhorn, timerSumM,
    timerGetWeightedXY, timerGetXcYc, timerCenteringXY, timerFindS, timerAfterSVD, timerRT,
    timerShinkhorn1, timerShinkhorn2, timerShinkhorn3;

  if(!param.notimer){
    CUT_SAFE_CALL(cutCreateTimer(&timerUpdateM));
    CUT_SAFE_CALL(cutCreateTimer(&timerShinkhorn));
    CUT_SAFE_CALL(cutCreateTimer(&timerShinkhorn1));
    CUT_SAFE_CALL(cutCreateTimer(&timerShinkhorn2));
    CUT_SAFE_CALL(cutCreateTimer(&timerShinkhorn3));
    CUT_SAFE_CALL(cutCreateTimer(&timerSumM));
    CUT_SAFE_CALL(cutCreateTimer(&timerGetWeightedXY));
    CUT_SAFE_CALL(cutCreateTimer(&timerGetXcYc));
    CUT_SAFE_CALL(cutCreateTimer(&timerCenteringXY));
    CUT_SAFE_CALL(cutCreateTimer(&timerFindS));
    CUT_SAFE_CALL(cutCreateTimer(&timerAfterSVD));
    CUT_SAFE_CALL(cutCreateTimer(&timerRT));
  }


  CUT_SAFE_CALL(cutCreateTimer(&timerTotal));
  CUDA_SAFE_CALL( cudaThreadSynchronize() );
  CUT_SAFE_CALL(cutStartTimer(timerTotal));




  //
  // initializing cublas
  //
  cublasInit();



  //
  // softassign main loop
  //

  for(int Titer = 1; Titer <= JMAX; Titer++){

    fprintf(stderr, "%d iter. temp. %f  ", Titer, T_cur);
    fprintf(stderr, "time %.10f [s]\n", cutGetTimerValue(timerTotal) / 1000.0f);

#ifndef NOVIEWER
    if(!param.noviewer){
      UpdatePointCloud2(Ysize, param.points2, h_Y, h_R, h_t);
      if (!EngineIteration()) // PointCloudViewer
	break;
    }
#endif


    // inner loop with the same temperature

    for(int iter0 = 0; iter0 < I0; iter0++){

      //
      // UpdateM
      //

      START_TIMER(timerUpdateM);

      updateM
	<<< dimGridForM, dimBlockForM >>>
	(rowsM, colsM, pitchM,
	 d_Xx, d_Xy, d_Xz,
	 d_Yx, d_Yy, d_Yz,
	 d_R, d_t, d_M, T_cur, alpha);

      STOP_TIMER(timerUpdateM);




      //
      // Normalization of M by Shinkhorn
      //


      START_TIMER(timerShinkhorn);

      // shinkhorn loop until M converges
      for (int Sinkh_iter = 0; Sinkh_iter < I1; Sinkh_iter++){


	//
	// row normalization
	//

	START_TIMER(timerShinkhorn1);

	// cublasSgemv (char trans, int m, int n, float alpha, const float *A, int lda,
	//              const float *x, int incx, float beta, float *y, int incy)
	//    y = alpha * op(A) * x + beta * y,
	
	// M * one vector = vector with elements of row-wise sum
	//     d_M      *    d_one    =>  d_sumOfMRow
	//(rowsM*colsM) *  (colsM*1)  =  (rowsM*1)
	cublasSgemv('n',          // char trans
		    rowsM, colsM, // int m (rows of A), n (cols of A) ; not op(A)
		    1.0f,         // float alpha
		    d_M, pitchM,  // const float *A, int lda
		    d_one, 1,     // const float *x, int incx
		    0.0f,         // float beta
		    d_sumOfMRow, 1);   // float *y, int incy

	STOP_TIMER(timerShinkhorn1);
	START_TIMER(timerShinkhorn2);

	// void cublasSaxpy (int n, float alpha, const float *x, int incx, float *y, int incy)
	// alpha * x + y => y
	// m_outliers_row + d_sumOfMRow => d_sumOfMRow
	cublasSaxpy(rowsM, 1.0f, d_m_outliers_row, 1, d_sumOfMRow, 1);


	STOP_TIMER(timerShinkhorn2);
	START_TIMER(timerShinkhorn3);

	normalizeMbySinkhorn_row
	  <<< dimGridForM, dimBlockForM >>>
	  (rowsM, colsM, pitchM,
	   d_M, d_sumOfMRow, d_m_outliers_row);


	STOP_TIMER(timerShinkhorn3);



	//
	// column normalization
	//

	// cublasSgemv (char trans, int m, int n, float alpha, const float *A, int lda,
	//              const float *x, int incx, float beta, float *y, int incy)
	//    y = alpha * op(A) * x + beta * y,
	
	// M * one vector = vector with elements of column-wise sum
	//     d_M^T    *    d_one    =>  d_sumOfMCol
	//(coslM*rowsM) *  (rowsM*1)  =  (colsM*1)
	cublasSgemv('t',          // char trans
		    rowsM, colsM, // int m (rows of A), n (cols of A) ; not op(A)
		    1.0f,         // float alpha
		    d_M, pitchM,  // const float *A, int lda
		    d_one, 1,     // const float *x, int incx
		    0.0f,         // float beta
		    d_sumOfMCol, 1);   // float *y, int incy

	// void cublasSaxpy (int n, float alpha, const float *x, int incx, float *y, int incy)
	// alpha * x + y => y
	// m_outliers_col + d_sumOfMCol => d_sumOfMCol
	cublasSaxpy(colsM, 1.0f, d_m_outliers_col, 1, d_sumOfMCol, 1);

	

	normalizeMbySinkhorn_col
	  <<< dimGridForM, dimBlockForM >>> 
	  (rowsM, colsM, pitchM,
	   d_M, d_sumOfMCol, d_m_outliers_col);

      }

      STOP_TIMER(timerShinkhorn);




      //
      // update R,T
      //


      ///////////////////////////////////////////////////////////////////////////////////// 

      // compute sum of all elements in M


      START_TIMER(timerSumM);

      // cublasSgemv (char trans, int m, int n, float alpha, const float *A, int lda,
      //              const float *x, int incx, float beta, float *y, int incy)
      //    y = alpha * op(A) * x + beta * y,

      // M * one vector = vector with elements of row-wise sum
      //     d_M      *    d_one    =>  d_sumOfMRow
      //(rowsM*colsM) *  (colsM*1)  =  (rowsM*1)

      cublasSgemv('n',          // char trans
		  rowsM, colsM, // int m (rows of A), n (cols of A) ; not op(A)
		  1.0f,         // float alpha
		  d_M, pitchM,  // const float *A, int lda
		  d_one, 1,     // const float *x, int incx
		  0.0f,         // float beta
		  d_sumOfMRow, 1);   // float *y, int incy
      


      //sum of M
      // float cublasSasum (int n, const float *x, int incx) 
      // computes the sum of the absolute values of the elements
      float sumM = cublasSasum (rowsM, d_sumOfMRow, 1); 
      // sum of all elements in M, assuming that all are positive.

      STOP_TIMER(timerSumM);


      ///////////////////////////////////////////////////////////////////////////////////// 


      // compute weighted X and Y
	
      START_TIMER(timerGetWeightedXY);

      // X .* sumOfRow => X_result
      elementwiseMultiplicationCopy
	<<< blocksPerGridForXsize, threadsPerBlockForXsize>>>
	(rowsM,
	 d_Xx, d_Xy, d_Xz,
	 d_sumOfMRow,
	 d_Xx_result, d_Xy_result, d_Xz_result);

      // Y .* sumOfCol => Y_result
      elementwiseMultiplicationCopy
	<<< blocksPerGridForYsize, threadsPerBlockForYsize>>>
	(colsM,
	 d_Yx, d_Yy, d_Yz,
	 d_sumOfMCol,
	 d_Yx_result, d_Yy_result, d_Yz_result);
      
      STOP_TIMER(timerGetWeightedXY);


      ///////////////////////////////////////////////////////////////////////////////////// 


      // find weighted center of X' and Y

      START_TIMER(timerGetXcYc);

      // cublasSasum can not be used for summing up a vector
      //  because it is ABS sum, not just sum.

      // cublasSgemv (char trans, int m, int n, float alpha, const float *A, int lda,
      //              const float *x, int incx, float beta, float *y, int incy)
      //    y = alpha * op(A) * x + beta * y,

      // d_X_result^T *    d_one     =>   h_Xc
      //  (3 * rowsM)   (rowsM * 1)  =  (3 * 1)
      cublasSgemv('t',          // char trans
		  rowsM, 3,     // int m (rows of A), n (cols of A) ; not op(A)
		  1.0f,         // float alpha
		  d_X_result, rowsM,  // const float *A, int lda
		  d_one, 1,     // const float *x, int incx
		  0.0f,         // float beta
		  d_Xc, 1);     // float *y, int incy

      // d_Y_result^T *    d_one     =>   h_Yc
      //  (3 * colsM)   (colM * 1)  =  (3 * 1)
      cublasSgemv('t',          // char trans
		  colsM, 3,     // int m (rows of A), n (cols of A) ; not op(A)
		  1.0f,         // float alpha
		  d_Y_result, colsM,  // const float *A, int lda
		  d_one, 1,     // const float *x, int incx
		  0.0f,         // float beta
		  d_Yc, 1);     // float *y, int incy


      // void cublasSscal (int n, float alpha, float *x, int incx)
      // it replaces x[ix + i * incx] with alpha * x[ix + i * incx]
      cublasSscal (3, 1/sumM, d_Xc, 1);
      cublasSscal (3, 1/sumM, d_Yc, 1);


      CUDA_SAFE_CALL(cudaMemcpy(h_Xc, d_Xc, sizeof(float)*3, cudaMemcpyDeviceToHost));
      CUDA_SAFE_CALL(cudaMemcpy(h_Yc, d_Yc, sizeof(float)*3, cudaMemcpyDeviceToHost));


      STOP_TIMER(timerGetXcYc);


      ///////////////////////////////////////////////////////////////////////////////////// 

      // centering X and Y

      START_TIMER(timerCenteringXY);

      centeringXorY
	<<< blocksPerGridForXsize, threadsPerBlockForXsize>>>
	(rowsM,
	 d_Xc, sumM,
	 d_Xx_result, d_Xy_result, d_Xz_result);

      centeringXorY
	<<< blocksPerGridForYsize, threadsPerBlockForYsize>>>
	(colsM,
	 d_Yc, sumM,
	 d_Yx_result, d_Yy_result, d_Yz_result);
 

      STOP_TIMER(timerCenteringXY);


      ///////////////////////////////////////////////////////////////////////////////////// 

      // compute S

      START_TIMER(timerFindS)

      // S = d_X_result^T * d_M * d_Y_result

      // cublasSgemm (char transa, char transb, int m, int n, int k, float alpha, 
      //              const float *A, int lda, const float *B, int ldb, float beta, 
      //              float *C, int ldc)
      //   C = alpha * op(A) * op(B) + beta * C,
      //
      // m      number of rows of matrix op(A) and rows of matrix C
      // n      number of columns of matrix op(B) and number of columns of C
      // k      number of columns of matrix op(A) and number of rows of op(B) 

      //     d_M      * d_Y_result =>    d_D
      //(rowsM*colsM) *  (colsM*3)  =  (rowsM*3)
      //   m  * k           k * n        m * n   
      cublasSgemm('n', 'n', rowsM, 3, colsM, 
		  1.0f, d_M, pitchM,
		  d_Y_result, colsM,
		  0.0f, d_D, rowsM);

      //  d_X_result^T *     d_D     =>  d_S
      //    (3*rowsM)  *  (rowsM*3)  =  (3*3)
      //   m  * k           k * n        m * n
      cublasSgemm('t', 'n', 3, 3, rowsM,
		  1.0f, d_X_result, rowsM,
		  d_D, rowsM,
		  0.0f, d_S, 3);

      CUDA_SAFE_CALL(cudaMemcpy(h_S, d_S, sizeof(float)*9, cudaMemcpyDeviceToHost));


      STOP_TIMER(timerFindS);



      ///////////////////////////////////////////////////////////////////////////////////// 

      // find RT from S

      START_TIMER(timerAfterSVD);

      findRTfromS(h_Xc, h_Yc, h_S, h_R, h_t, false);

      STOP_TIMER(timerAfterSVD);

      ///////////////////////////////////////////////////////////////////////////////////// 

      // copy R,t to device

      START_TIMER(timerRT);

      CUDA_SAFE_CALL(cudaMemcpy(d_R, h_R, sizeof(float)*3*3, cudaMemcpyHostToDevice));
      CUDA_SAFE_CALL(cudaMemcpy(d_t, h_t, sizeof(float)*3,   cudaMemcpyHostToDevice));

      STOP_TIMER(timerRT);

      ///////////////////////////////////////////////////////////////////////////////////// 

#ifndef NOVIEWER
      if(!param.noviewer){
	UpdatePointCloud2(Ysize, param.points2, h_Y, h_R, h_t);
	if (!EngineIteration()) // PointCloudViewer
	  break;
      }
#endif

    }

    T_cur = T_cur*TFACTR;

  }


  CUDA_SAFE_CALL( cudaThreadSynchronize() );
  CUT_SAFE_CALL(cutStopTimer(timerTotal));

  fprintf(stderr, "comping time: %.10f [s]\n", cutGetTimerValue(timerTotal) / 1000.0f);

  if(!param.notimer){

    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerUpdateM)  / 1000.0f, "updateM");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerShinkhorn)/ 1000.0f, "shinkhorn");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerShinkhorn1)/ 1000.0f, "shinkhorn1");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerShinkhorn2)/ 1000.0f, "shinkhorn2");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerShinkhorn3)/ 1000.0f, "shinkhorn3");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerSumM)  / 1000.0f, "SumM");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerGetWeightedXY)   / 1000.0f, "getMXY");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerGetXcYc)  / 1000.0f, "getXcYc");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerCenteringXY) / 1000.0f, "getNewXY");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerFindS)    / 1000.0f, "findS");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerAfterSVD) / 1000.0f, "afterSVD");
    fprintf(stderr, "Average %.10f [s] for %s\n", cutGetAverageTimerValue(timerRT) / 1000.0f, "RT");

    CUT_SAFE_CALL(cutDeleteTimer(timerTotal));
    CUT_SAFE_CALL(cutDeleteTimer(timerUpdateM));
    CUT_SAFE_CALL(cutDeleteTimer(timerShinkhorn));
    CUT_SAFE_CALL(cutDeleteTimer(timerShinkhorn1));
    CUT_SAFE_CALL(cutDeleteTimer(timerShinkhorn2));
    CUT_SAFE_CALL(cutDeleteTimer(timerShinkhorn3));
    CUT_SAFE_CALL(cutDeleteTimer(timerSumM));
    CUT_SAFE_CALL(cutDeleteTimer(timerGetWeightedXY));
    CUT_SAFE_CALL(cutDeleteTimer(timerGetXcYc));
    CUT_SAFE_CALL(cutDeleteTimer(timerCenteringXY));
    CUT_SAFE_CALL(cutDeleteTimer(timerFindS));
    CUT_SAFE_CALL(cutDeleteTimer(timerAfterSVD));

  }

  cublasShutdown();




  CUDA_SAFE_CALL(cudaFree(d_Xx));
  CUDA_SAFE_CALL(cudaFree(d_Yx));
  CUDA_SAFE_CALL(cudaFree(d_R));
  CUDA_SAFE_CALL(cudaFree(d_t));
  CUDA_SAFE_CALL(cudaFree(d_M));
  CUDA_SAFE_CALL(cudaFree(d_D));
  CUDA_SAFE_CALL(cudaFree(d_S));
  CUDA_SAFE_CALL(cudaFree(d_one));
  CUDA_SAFE_CALL(cudaFree(d_sumOfMRow));
  CUDA_SAFE_CALL(cudaFree(d_sumOfMCol));
  CUDA_SAFE_CALL(cudaFree(d_X_result));
  CUDA_SAFE_CALL(cudaFree(d_Y_result));
  CUDA_SAFE_CALL(cudaFree(d_m_outliers_row));
  CUDA_SAFE_CALL(cudaFree(d_m_outliers_col));   

  delete [] h_m_outliers_row;
  delete [] h_m_outliers_col; 

  delete [] h_one;

  CUDA_SAFE_CALL( cudaThreadExit() );
}
