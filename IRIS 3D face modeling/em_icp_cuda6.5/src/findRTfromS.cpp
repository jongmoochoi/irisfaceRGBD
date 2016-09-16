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

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include "3dregistration.h"

void eigenvectorOfN(double* N, float* q, bool *error);

void findRTfromS(const float* h_Xc,
		 const float* h_Yc,
		 const float* h_S,
		 float* h_R, float* h_t,
		 bool *error){

	#define h_Sxx h_S[0]
	#define h_Sxy h_S[1]
	#define h_Sxz h_S[2]
	#define h_Syx h_S[3]
	#define h_Syy h_S[4]
	#define h_Syz h_S[5]
	#define h_Szx h_S[6]
	#define h_Szy h_S[7]
	#define h_Szz h_S[8]

	#define h_Xcx h_Xc[0]
	#define h_Xcy h_Xc[1]
	#define h_Xcz h_Xc[2]
	#define h_Ycx h_Yc[0]
	#define h_Ycy h_Yc[1]
	#define h_Ycz h_Yc[2]


  double N[16];	for(int n=0;n<16;n++) N[n] = 0.0;
  float q[4];		for(int a=0;a<4;a++)  q[a] = 0.0f;

  N[ 0] = h_Sxx + h_Syy + h_Szz;
  N[ 1] = h_Syz - h_Szy;
  N[ 2] = h_Szx - h_Sxz;
  N[ 3] = h_Sxy - h_Syx;
  N[ 4] = h_Syz - h_Szy;
  N[ 5] = h_Sxx - h_Syy - h_Szz;
  N[ 6] = h_Sxy + h_Syx;
  N[ 7] = h_Szx + h_Sxz;
  N[ 8] = h_Szx - h_Sxz;
  N[ 9] = h_Sxy + h_Syx;
  N[10] = h_Syy - h_Sxx - h_Szz;
  N[11] = h_Syz + h_Szy;
  N[12] = h_Sxy - h_Syx;
  N[13] = h_Szx + h_Sxz;
  N[14] = h_Syz + h_Szy;
  N[15] = h_Szz - h_Sxx - h_Syy;

  // computer the eigenvector corresponding the largest eivenvalue
  eigenvectorOfN(N, q, error);

  if ( !(*error) ) {
		float q0 = q[0], qx = q[1], qy = q[2], qz = q[3];
		float	q02=q0*q0, 
				qx2=qx*qx, 
				qy2=qy*qy, 
				qz2=qz*qz, 
				q0x=q0*qx,
				q0y=q0*qy,
				q0z=q0*qz,
				qxy=qx*qy,
				qxz=qx*qz,
				qyz=qy*qz;

	  // quaternion to rotation matrix
	  h_R[0] = q02 + qx2 - qy2 - qz2;
	  h_R[1] = 2 * (qxy - q0z);
	  h_R[2] = 2 * (qxz + q0y);
	  h_R[3] = 2 * (qxy + q0z);
	  h_R[4] = q02 - qx2 + qy2 - qz2;
	  h_R[5] = 2 * (qyz - q0x);
	  h_R[6] = 2 * (qxz - q0y);
	  h_R[7] = 2 * (qyz + q0x);
	  h_R[8] = q02 - qx2 - qy2 + qz2;

	  // translation vector
	  h_t[0] = h_Xcx - (h_R[0]*h_Ycx + h_R[1]*h_Ycy + h_R[2]*h_Ycz);
	  h_t[1] = h_Xcy - (h_R[3]*h_Ycx + h_R[4]*h_Ycy + h_R[5]*h_Ycz);
	  h_t[2] = h_Xcz - (h_R[6]*h_Ycx + h_R[7]*h_Ycy + h_R[8]*h_Ycz);
  }
}



/***************************************************************/
/* Convert the rotation matrix to Euler angles - in degrees-	*/
// Matthias Hernandez: 07/27/2011 University of Southern California
//	R: input: rotation matrix - float[9] should be allocated
//	   0 1 2
//    3 4 5
//    6 7 8
// a: output: Euler angles - float[3] should be allocated
//		0=pitch  1=yaw   2=roll
// Require <math.h>
void rotationMatrix2eulerAngles(float *R, float *a)	{
	float sinY, cosY;
			
	sinY = R[6];
	if (sinY>1.0f)	sinY=1.0f;
	if (sinY<-1.0f) sinY=-1.0f;

	if( sinY != 1.0f && sinY != -1.0f){
		a[1] = -asin( sinY );

		cosY=cos(a[1]);
		if (cosY>1.0f)	cosY=1.0f;
		if (cosY<-1.0f) cosY=-1.0f;

		a[0] = atan2( R[7]/cosY, R[8]/cosY );
		a[2] = atan2( R[3]/cosY, R[0]/cosY );
	} else {
		a[2] = 0.0f; // any
		float delta = atan2(R[1], R[2]);
		if(sinY == -1.0f){
			a[1] = (float) PI/2.0f;
			a[0] = a[2] + delta;
		} else {
			a[1] = -(float) PI/2.0f;
			a[0] = -a[2] + delta;
		}
	}
	a[0] *= (float)RAD2DEG ;
	a[1] *= (float)RAD2DEG ;
	a[2] *= (float)RAD2DEG ; 
}
/**********************/




/*************************/
/* Multiply two matrices */
void multiply(float *R1, float *R2, float *R3) {
	float	r10=R1[0], r11=R1[1], r12=R1[2],
			r13=R1[3], r14=R1[4], r15=R1[5],
			r16=R1[6], r17=R1[7], r18=R1[8];
	float	r20=R2[0], r21=R2[1], r22=R2[2],
			r23=R2[3], r24=R2[4], r25=R2[5],
			r26=R2[6], r27=R2[7], r28=R2[8];


	R3[0] = r10*r20 + r11*r23 + r12*r26;
	R3[1] = r10*r21 + r11*r24 + r12*r27;
	R3[2] = r10*r22 + r11*r25 + r12*r28;

	R3[3] = r13*r20 + r14*r23 + r15*r26;
	R3[4] = r13*r21 + r14*r24 + r15*r27;
	R3[5] = r13*r22 + r14*r25 + r15*r28;

	R3[6] = r16*r20 + r17*r23 + r18*r26;
	R3[7] = r16*r21 + r17*r24 + r18*r27;
	R3[8] = r16*r22 + r17*r25 + r18*r28;
}
/**********************/




/***************************************************************/
/* Convert the Euler angles to rotation Matrix - in degrees-	*/
// Matthias Hernandez: 07/27/2011 University of Southern California
void eulerAngles2rotationMatrix(float *R, float *a)	{
	float	psi = a[0]*DEG2RAD,
			theta = a[1]*DEG2RAD, 
			phi = a[2]*DEG2RAD;

	R[0] = cos(theta)*cos(phi);
	R[1] = sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
	R[2] = cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);

	R[3] = cos(theta)*sin(phi);
	R[4] = sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi);
	R[5] = cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi);

	R[6] = -sin(theta);
	R[7] = cos(theta)*sin(psi);
	R[8] = cos(theta)*cos(psi);
}
/**********************/

/********************************************************/
/* Difference between consecutive rigid transformations */
float tr_diff(float *t, float *pre_t, float *a, float *pre_a) {
	return (	/*abs(pre_t[0]-t[0]) + abs(pre_t[1]-t[1]) + abs(pre_t[2]-t[2]) 
			+	*/abs(pre_a[0]-a[0]) +	abs(pre_a[1]-a[1]) + abs(pre_a[2]-a[2]) );
} 
/**********************/

/**********************/
void printRT(const float* R, const float* t, float *a){
  printf("Euler angles\n%f %f %f deg\n", a[0],a[1],a[2]);
  printf("t\n");
  for(int r=0; r<3; r++)
    printf("%f ", t[r]);
  printf("\n");

}
/**********************/

/**********************/
void printR(float *a){
  fprintf(stderr, "[%3.2f\t%3.2f\t%3.2f]\tdeg\n", a[0],a[1],a[2]);
}
/**********************/


extern "C" {
int dsyev_(char *jobz, char *uplo, 
	   int *n, double *a, int *lda, 
	   double *w, double *work, int *lwork, 
	   int *info);
}

void eigenvectorOfN(double *N, float* q, bool *error){
  
  static float q_pre[4]; // previous result

  int dimN = 4;
  double w[4]; // eigenvalues
  double *work; // workspace
  double work2;
  int info;
  int lwork = -1;

  dsyev_((char*)"V", (char*)"U",
	 &dimN, N, &dimN,
	 w, &work2, &lwork, &info);
  if(info != 0){
    fprintf(stderr, "info = %d\n", info);
    exit(1);
  }
  lwork = (int)(work2+0.5f);

  work = new double [lwork];

  dsyev_((char*)"V", (char*)"U",
	 &dimN, N, &dimN,
	 w, work, &lwork, &info);

  delete[] work;


  if(info != 0){
    //fprintf(stderr, "computing eigenvector FAIL! info = %d\n", info);
    //exit(1);

    // if fail, put back the previous result
    for(int i=0; i<4; i++){
      q[i] = q_pre[i];
    }
	 *error = true;
	
  }else{

    // last column of N is the eigenvector of the largest eigenvalue 
    // and N is stored column-major
    for(int i=0; i<4; i++){
      q[i] = (float)(N[12 + i]);
      q_pre[i] = q[i];
    }
    
  }


}

/*
*  =========
*
*  JOBZ    (input) CHARACTER*1
*          = 'N':  Compute eigenvalues only;
*          = 'V':  Compute eigenvalues and eigenvectors.
*
*  UPLO    (input) CHARACTER*1
*          = 'U':  Upper triangle of A is stored;
*          = 'L':  Lower triangle of A is stored.
*
*  N       (input) INTEGER
*          The order of the matrix A.  N >= 0.
*
*  A       (input/output) DOUBLE PRECISION array, dimension (LDA, N)
*          On entry, the symmetric matrix A.  If UPLO = 'U', the
*          leading N-by-N upper triangular part of A contains the
*          upper triangular part of the matrix A.  If UPLO = 'L',
*          the leading N-by-N lower triangular part of A contains
*          the lower triangular part of the matrix A.
*          On exit, if JOBZ = 'V', then if INFO = 0, A contains the
*          orthonormal eigenvectors of the matrix A.
*          If JOBZ = 'N', then on exit the lower triangle (if UPLO='L')
*          or the upper triangle (if UPLO='U') of A, including the
*          diagonal, is destroyed.
*
*  LDA     (input) INTEGER
*          The leading dimension of the array A.  LDA >= max(1,N).
*
*  W       (output) DOUBLE PRECISION array, dimension (N)
*          If INFO = 0, the eigenvalues in ascending order.
*
*  WORK    (workspace/output) DOUBLE PRECISION array, dimension (MAX(1,LWORK))
*          On exit, if INFO = 0, WORK(1) returns the optimal LWORK.
*
*  LWORK   (input) INTEGER
*          The length of the array WORK.  LWORK >= max(1,3*N-1).
*          For optimal efficiency, LWORK >= (NB+2)*N,
*          where NB is the blocksize for DSYTRD returned by ILAENV.
*
*          If LWORK = -1, then a workspace query is assumed; the routine
*          only calculates the optimal size of the WORK array, returns
*          this value as the first entry of the WORK array, and no error
*          message related to LWORK is issued by XERBLA.
*
*  INFO    (output) INTEGER
*          = 0:  successful exit
*          < 0:  if INFO = -i, the i-th argument had an illegal value
*          > 0:  if INFO = i, the algorithm failed to converge; i
*                off-diagonal elements of an intermediate tridiagonal
*                form did not converge to zero.
*
*  =====================================================================
*/
