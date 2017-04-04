/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

//-----------------------------------------------------------------------------------------
//	Some of this file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

#include "stdafx.h"
#include "openNI_general.h"

#include "cuda_runtime.h"

#include <helper_cuda.h>
#include <helper_cuda_drvapi.h>
#include <helper_cuda_gl.h>
#include <helper_timer.h>
#include <helper_string.h>
#include <helper_math.h>
#include <helper_image.h>
#include <helper_functions.h>
#include <device_launch_parameters.h>

#include <device_functions.h>

//////////////////////////////////////////////
// TODO REMOVE THE WEIRD TEXTURE CRAP ????? //
//////////////////////////////////////////////






#ifndef _GENERAL_CUDA_H
#define _GENERAL_CUDA_H


/*
    Because a 2D gaussian mask is symmetry in row and column,
    here only generate a 1D mask, and use the product by row 
    and column index later.

    1D gaussian distribution :
        g(x, d) -- C * exp(-x^2/d^2), C is a constant amplifier

    parameters:
    og - output gaussian array in global memory
    delta - the 2nd parameter 'd' in the above function
    radius - half of the filter size
             (total filter size = 2 * radius + 1)
*/
//use only one block


/********************/
/********************/
/* BILATERAL FILTER */
/********************/
/********************/



__global__ void
d_generate_gaussian(float *og, float delta, int radius)  {
    int x = blockIdx.x * blockDim.x + threadIdx.x;//threadIdx.x;
    og[threadIdx.x] = __expf(-(x * x) /
        (2 * delta * delta));
}                                     

void initGaussian(float *d_gaussian, float gaussian_delta, int fr_g) {
	d_generate_gaussian <<< 1, fr_g>>>(d_gaussian, gaussian_delta, fr_g);
}


/*
    Euclidean Distance (x, y, d) = exp((|x - y| / d)^2 / 2)
*/
__device__ float euclideanLen(float ax, float ay, float bx, float by, float d) {
    float mod = (bx - ax) * (bx - ax) +
                (by - ay) * (by - ay);

    return __expf(-mod / (2 * d * d));
}


/*
    Depth Distance (x, y, d) = exp((|x - y| / d)^2 / 2)
*/
__device__ float euclideanLen(float az, float bz, float d) {
    float mod = (bz - az) * (bz - az);

    return __expf(-mod / (2 * d * d));
}


//extern "C"
void copyImage(uint *h_src, uint *d_dest,
               int width, int height) {
	cudaMemcpy(d_dest, h_src, width * height * sizeof(uint), cudaMemcpyHostToDevice);
   // cutilSafeCall(cudaMemcpy(d_dest, h_src, width * height * sizeof(uint), cudaMemcpyHostToDevice));
}

#endif




#ifndef _FILTER_KERNEL_H_
#define _FILTER_KERNEL_H_


/*
    Perform a simple bilateral filter.

    Bilateral filter is a nonlinear filter that is a mixture of range 
    filter and domain filter, the previous one preserves crisp edges and 
    the latter one filters noise. The intensity value at each pixel in 
    an image is replaced by a weighted average of intensity values from 
    nearby pixels.

    The weight factor is calculated by the product of domain filter
    component(using the gaussian distribution as a spatial distance) as 
    well as range filter component(Euclidean distance between center pixel
    and the current neighbor pixel). Because this process is nonlinear, 
    the sample just uses a simple pixel by pixel step. 

    Texture fetches automatically clamp to edge of image. 1D gaussian array
    is mapped to a 1D texture instead of using shared memory, which may 
    cause severe bank conflict.

    Threads are y-pass(column-pass), because the output is coalesced.

    Parameters
    od - pointer to output data in global memory
    d_f - pointer to the 1D gaussian array
    e_d - euclidean delta
    w  - image width
    h  - image height
    r  - filter radius
*/






//column pass using coalesced global memory reads
__global__ void
d_bilateral_filter_and_3D_conversion(float *pReal, float *pDepth, float *gaussian, bool *validity,
                   float s_s, float s_r, int w, int h, int r)
{
    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x < w && y < h) {
        float sum = 0.0f;
        float factor =0.0f;
	
		int max_i = w*h;
		int i1=y*w+x, 
			i2=i1+max_i, 
			i3=i2+max_i;

		float tz=0.0f;
		float centerz=pDepth[i1];
		float curPixz=0.0f;
		float Z=0.0f;

		//////////////////////////////////////////////////////////////
		// Apply the bilateral filter
		if (validity[i1]) {//(centerz<MAX_DEPTH && centerz>MIN_DEPTH){

			for(int i = -r; i <= r; i++)  {
				for(int j = -r; j <= r; j++)  {

					//curPixx = (x + j);
					//curPixy = (y + i);
					curPixz = pDepth[(y + i)*w + (x + j)];

					factor =  euclideanLen(curPixz, centerz, s_r)	//range factor
							* gaussian[abs(j)]*gaussian[abs(i)]; //euclideanLen(curPixx, curPixy, centerx, centery, s_s); 

					if (validity[i1]) {
						tz += factor * curPixz;
						sum += factor;
					}
				}
			}
			if (sum > 0.0f)
				Z = tz/sum;
		} 

		//////////////////////////////////////////////////////////////
		// Convert to 3D coordinate

		// Set the Z coordinate
		pReal[i3] = Z;

		// Find the X and Y coordinate
		if (Z > MIN_DEPTH && Z < MAX_DEPTH){
			float X_rw = ( (float)x /(float)w -0.5f)*Z*XtoZ;
			float Y_rw = (0.5f-(float)y / (float)h)*Z*YtoZ;

			pReal[i1] = X_rw;
			pReal[i2] = Y_rw;
		} else {
			pReal[i1] = 0.0f;
			pReal[i2] = 0.0f;
			pReal[i3] = 0.0f;
		}
    }
}


//column pass using coalesced global memory reads
__global__ void
d_bilateral_filter(float *pReal, float *pDepth, float *gaussian, bool *validity,
                   float s_s, float s_r, int w, int h, int r)
{
    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x < w && y < h) {
        float sum = 0.0f;
        float factor =0.0f;
	
		int i1=y*w+x;

		float tz=0.0f;
		float centerz=pDepth[i1];
		float curPixz=0.0f;
		float Z=0.0f;

		//////////////////////////////////////////////////////////////
		// Apply the bilateral filter
		if (validity[i1]) {//(centerz<MAX_DEPTH && centerz>MIN_DEPTH){

			for(int i = -r; i <= r; i++)  {
				for(int j = -r; j <= r; j++)  {

					//curPixx = (x + j);
					//curPixy = (y + i);
					curPixz = pDepth[(y + i)*w + (x + j)];

					factor =  euclideanLen(curPixz, centerz, s_r)	//range factor
							* gaussian[abs(j)]*gaussian[abs(i)]; //euclideanLen(curPixx, curPixy, centerx, centery, s_s); 

					if (validity[i1]) {
						tz += factor * curPixz;
						sum += factor;
					}
				}
			}
			if (sum > 0.0f)
				Z = tz/sum;
		} 

		//////////////////////////////////////////////////////////////
		// Convert to 3D coordinate

		// Set the Z coordinate
		pReal[i1] = Z;
	}
}














/**************************/
/* Compute the PCA matrix */
__global__ void
d_PCA(float *pReal, float *M, int rad, int l, int r, int t, int b, int w, int h, int max_i ) {
	uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;
	
	int i0=0;
	int k=0;

	float	xi=0.0f, yi=0.0f, zi=0.0f,
			x_=0.0f, y_=0.0f, z_=0.0f;

	if (x < w && y < h) {
		int max_i2 = 2*max_i;
		int i1=y*w+x; 
		//	i2=i1+max_i, 
		//	i3=i2+max_i;

		float *MM = &M[i1*9];

		if(x>=l && x<=r && y>=t && y<=b && pReal[i1+max_i2]!=0.0f) {
			for(int i = -rad; i <= rad; i++)  {
				for(int j = -rad; j <= rad; j++)  {
					i0 = (y-j)*w+(x-i);

					if(pReal[i0]!=0) {
						xi=pReal[i0];
						yi=pReal[i0+max_i];
						zi=pReal[i0+max_i2];

						x_ += xi;
						y_ += yi;
						z_ += zi;
						k++;

						MM[0] += xi*xi;
						MM[1] += xi*yi;
						MM[2] += xi*zi;
						MM[4] += yi*yi;
						MM[5] += yi*zi;
						MM[8] += zi*zi;
					}
				}
			}

			if (k>0) {
				x_ /= k;
				y_ /= k;
				z_ /= k;

				MM[0] = MM[0]/k - x_*x_;
				MM[1] = MM[1]/k - x_*y_;
				MM[2] = MM[2]/k - x_*z_;
				MM[4] = MM[4]/k - y_*y_;
				MM[5] = MM[5]/k - y_*z_;
				MM[8] = MM[8]/k - z_*z_;
				MM[3] = MM[1];
				MM[6] = MM[2];
				MM[7] = MM[5];
			} else {
				for (int i=0; i<9; i++)
					MM[i] = 0.0f;
			}
		} else
			for (int i=0; i<9; i++)
				MM[i] = 0.0f;
	}
}
/**************************************************/



//Perform a matrix decomposition of VV: 
// After running, VV will store the eigenvecotrs and dd the eigenvalues
// CODE FROM http://openslam.informatik.uni-freiburg.de/data/svn/gmapping/trunk/scanmatcher/eig3.cpp , copied from the public domain Java Matrix library JAMA.
__global__ void
d_eigendecomposition(float *VV, float *dd, int w, int h ) {
	//float	xi=0.0f, yi=0.0f, zi=0.0f,
	//		x_=0.0f, y_=0.0f, z_=0.0f;
	int n=3;
	
	//int L=n*(n-1);
	float e[3];

	uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x < w && y < h) {
		int i1=y*w+x; 

		float *d = &dd[i1*3];
		float *V = &VV[i1*9];

		if (	V[0]!=0.0f || V[1]!=0.0f || V[2]!=0.0f ||
				V[3]!=0.0f || V[4]!=0.0f || V[5]!=0.0f ||
				V[6]!=0.0f || V[7]!=0.0f || V[8]!=0.0f	)  {

			for (int j = 0; j < 3; j++) {
				d[j] = V[6+j];
			}

			// Householder reduction to tridiagonal form.

			for (int i = 2; i > 0; i--) {

			// Scale to avoid under/overflow.

				float scale = 0.0f;
				float h = 0.0f;
				for (int k = 0; k < i; k++) {
					scale = scale + fabs(d[k]);
				}
				if (scale == 0.0) {
					e[i] = d[i-1];
					for (int j = 0; j < i; j++) {
						d[j] = V[(i-1)*n+j];
						V[i*n+j] = 0.0;
						V[j*n+i] = 0.0;
					}
				} else {

			  // Generate Householder vector.

					for (int k = 0; k < i; k++) {
						d[k] /= scale;
						h += d[k] * d[k];
					}
					float f = d[i-1];
					float g = sqrt(h);
					if (f > 0) {
						g = -g;
					}
					e[i] = scale * g;
					h = h - f * g;
					d[i-1] = f - g;
					for (int j = 0; j < i; j++) {
						e[j] = 0.0;
					}

				  // Apply similarity transformation to remaining columns.

					for (int j = 0; j < i; j++) {
						f = d[j];
						V[j*n+i] = f;
						g = e[j] + V[j*n+j] * f;
						for (int k = j+1; k <= i-1; k++) {
							g += V[k*n+j] * d[k];
							e[k] += V[k*n+j] * f;
						}
						e[j] = g;
					}
					f = 0.0;
					for (int j = 0; j < i; j++) {
						e[j] /= h;
						f += e[j] * d[j];
					}
					float hh = f / (h + h);
					for (int j = 0; j < i; j++) {
						e[j] -= hh * d[j];
					}
					for (int j = 0; j < i; j++) {
						f = d[j];
						g = e[j];
						for (int k = j; k <= i-1; k++) {
							V[k*n+j] -= (f * e[k] + g * d[k]);
						}
						d[j] = V[(i-1)*n+j];
						V[i*n+j] = 0.0;
					}
				}
				d[i] = h;
			}

			// Accumulate transformations.

			for (int i = 0; i < n-1; i++) {
				V[(n-1)*n+i] = V[i*n+i];
				V[i*n+i] = 1.0;
				float h = d[i+1];
				if (h != 0.0) {
					for (int k = 0; k <= i; k++) {
						d[k] = V[k*n+i+1] / h;
					}
					for (int j = 0; j <= i; j++) {
						float g = 0.0;
						for (int k = 0; k <= i; k++) {
						  g += V[k*n+i+1] * V[k*n+j];
						}
						for (int k = 0; k <= i; k++) {
						  V[k*n+j] -= g * d[k];
						}
					}
				}
				for (int k = 0; k <= i; k++) {
					V[k*n+i+1] = 0.0;
				}
			}
			for (int j = 0; j < n; j++) {
				d[j] = V[(n-1)*n+j];
				V[(n-1)*n+j] = 0.0;
			}
			V[(n-1)*n+n-1] = 1.0;
			e[0] = 0.0;

			
			for (int i = 1; i < n; i++) {
				e[i-1] = e[i];
			}
			e[n-1] = 0.0;

			float f = 0.0;
			float tst1 = 0.0;
			float eps = pow(2.0,-52.0);
			for (int l = 0; l < 3; l++) {

				// Find small subdiagonal element

				tst1 = MAX(tst1,fabs(d[l]) + fabs(e[l]));
				int m = l;
				while (m < 3) {
					if (fabs(e[m]) <= eps*tst1) {
						break;
					}
					m++;
				}

				// If m == l, d[l] is an eigenvalue,
				// otherwise, iterate.

				if (m > l) {
					int iter = 0;
					do {
						iter = iter + 1;  // (Could check iteration count here.)

						// Compute implicit shift

						float g = d[l];
						float p = (d[l+1] - g) / (2.0 * e[l]);
						float r = sqrt(p*p+1.0f);
						if (p < 0) {
							r = -r;
						}
						d[l] = e[l] / (p + r);
						d[l+1] = e[l] * (p + r);
						float dl1 = d[l+1];
						float h = g - d[l];
						for (int i = l+2; i < n; i++) {
							d[i] -= h;
						}
						f = f + h;

						// Implicit QL transformation.

						p = d[m];
						float c = 1.0;
						float c2 = c;
						float c3 = c;
						float el1 = e[l+1];
						float s = 0.0;
						float s2 = 0.0;
						for (int i = m-1; i >= l; i--) {
							c3 = c2;
							c2 = c;
							s2 = s;
							g = c * e[i];
							h = c * p;
							r = sqrt(p*p+e[i]*e[i]);
							e[i+1] = s * r;
							s = e[i] / r;
							c = p / r;
							p = c * d[i] - s * g;
							d[i+1] = h + s * (c * g + s * d[i]);

							// Accumulate transformation.

							for (int k = 0; k < n; k++) {
								h = V[k*n+i+1];
								V[k*n+i+1] = s * V[k*n+i] + c * h;
								V[k*n+i] = c * V[k*n+i] - s * h;
							}
						}
						p = -s * s2 * c3 * el1 * e[l] / dl1;
						e[l] = s * p;
						d[l] = c * p;

							// Check for convergence.

					} while (fabs(e[l]) > eps*tst1);
				}
				d[l] = d[l] + f;
				e[l] = 0.0;
			}
				// Sort eigenvalues and corresponding vectors.

			for (int i = 0; i < n-1; i++) {
				int k = i;
				float p = d[i];
				for (int j = i+1; j < n; j++) {
					if (d[j] < p) {
						k = j;
						p = d[j];
					}
				}
				if (k != i) {
					d[k] = d[i];
					d[i] = p;
					for (int j = 0; j < n; j++) {
						p = V[j*n+i];
						V[j*n+i] = V[j*n+k];
						V[j*n+k] = p;
					}
				}
			}
		}
	}

}
/**************************************************/


//Save the normals
__global__ void
d_Convert2Normals(float *V, float *norm, int w, int h, int max_i ) {
	uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x < w && y < h) {
		int i = y*w+x;
		int i9 = 9*i;

		if ( V[i9+6]<0.0f) {
			norm[i]			= -V[i9];
			norm[i+max_i]	= -V[i9+3];
			norm[i+2*max_i]	= -V[i9+6];
		} else {
			norm[i]			= V[i9];
			norm[i+max_i]	= V[i9+3];
			norm[i+2*max_i]	= V[i9+6];
		}
	}
}
/**************************************************/



//Save the normals
__global__ void
d_Convert2Normals(float *V, float *norm, char *nData, int w, int h, int max_i ) {
	uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x < w && y < h) {
		int i = y*w+x;
		int i9 = 9*i;

		if ( V[i9+6]<0.0f) {
			norm[i]			= -V[i9];
			norm[i+max_i]	= -V[i9+3];
			norm[i+2*max_i]	= -V[i9+6];
		} else {
			norm[i]			= V[i9];
			norm[i+max_i]	= V[i9+3];
			norm[i+2*max_i]	= V[i9+6];
		}

		nData[i] = (char) (norm[MAX_I2+i]*255.0f+0.5f);
	}
}
/**************************************************/







//column pass using coalesced global memory reads
__global__ void
d_compute_normals(float *out, float *input,
				  int l, int r, int t, int b,
                  int w, int h, int max_i) {
    uint x = /*__umul24(blockIdx.x, blockDim.x)*/ blockIdx.x * blockDim.x + threadIdx.x;
    uint y = /*__umul24(blockIdx.y, blockDim.y)*/ blockIdx.y * blockDim.y + threadIdx.y;

	float	Ux=0.0f,  Uy=0.0f,  Uz=0.0f,
			Vx=0.0f,  Vy=0.0f,  Vz=0.0f,
			UVx=0.0f, UVy=0.0f, UVz=0.0f,
			n2=0.0f;
	//int MIN_DEPTH=450, MAX_DEPTH=800;

    if (x < w && y < h) {

		int i1x = y*w+x,	i1y = i1x+max_i,	i1z = i1y+max_i;
		out[i1x]= 0.0f;
		out[i1y]= 0.0f;
		out[i1z]= 0.0f;

		if ( x>=l && x<=r && y>=t && y<=b ) {

			int i2x = i1x+w,	i2y = i2x+max_i,	i2z = i2y+max_i;
			int i3x = i1x+1,	i3y = i1y+1,		i3z = i1z+1;
			

			if ( input[i1z]>=MIN_DEPTH && input[i1z]<=MAX_DEPTH) {

				if ( (x+1)<w && (y+1)<h) {	

					if (input[i1z]!=0.0f && input[i2z]!=0.0f && input[i3z]!=0.0f) {
						Ux = input[i2x]-input[i1x];
						Uy = input[i2y]-input[i1y];
						Uz = input[i2z]-input[i1z];

						Vx = input[i3x]-input[i1x];
						Vy = input[i3y]-input[i1y];
						Vz = input[i3z]-input[i1z];

						UVx	= Uy*Vz-Vy*Uz;
						UVy	= Vx*Uz-Ux*Vz;
						UVz	= Ux*Vy-Vx*Uy;

						n2 = UVx*UVx + UVy*UVy + UVz*UVz;

						if (n2>0.0f) {
							n2 = pow(n2, 0.5f);

							out[i1x]= UVx/n2;
							out[i1y]= UVy/n2;
							out[i1z]= UVz/n2;
						} 
					} 
				} 
			}  
		}
    }
}
/**************************************************/


__global__ void 
d_convertP2RW(float *pDepth, float *pReal, int w, int h, int max_i){

	uint x = /*__umul24(blockIdx.x, blockDim.x)*/ blockIdx.x * blockDim.x + threadIdx.x;
    uint y = /*__umul24(blockIdx.y, blockDim.y)*/ blockIdx.y * blockDim.y + threadIdx.y;

	//int MAX_DEPTH = 800;
	//int MIN_DEPTH = 400;

	//float FovH = 1.0144686707507438f;
    //float FovV =0.78980943449644714f;

	int i1 = (y * w + x),
		i2 = i1 + max_i,
		i3 = i2 + max_i;

	float Z = pDepth[i1];

    if (Z > MIN_DEPTH && Z < MAX_DEPTH){
        float X_rw = ( (float)x /(float)w -0.5f)*Z*XtoZ;
        float Y_rw = (0.5f-(float)y / (float)h)*Z*YtoZ;

		pReal[i1] = X_rw;
		pReal[i2] = Y_rw;
		pReal[i3] = Z;
	} else {
		pReal[i1] = 0.0f;
		pReal[i2] = 0.0f;
		pReal[i3] = 0.0f;
	}
}
/**************************************************/




//column pass using coalesced global memory reads
__global__ void d_display_images(	char *out_rgb, char *input_rgb,
									char *out_d, float *input_d,
									float max_d, float min_d, float normalize, int *palette,
									int w, int h) {
    uint x = __umul24(blockIdx.x, blockDim.x) + threadIdx.x;
    uint y = __umul24(blockIdx.y, blockDim.y) + threadIdx.y;

	int i_d = y*w+x,
		i_r = 3*i_d,
		i_g = i_r+1,
		i_b = i_g+1;

	out_rgb[i_r] = input_rgb[i_b];
	out_rgb[i_g] = input_rgb[i_g];
	out_rgb[i_b] = input_rgb[i_r];

	if (input_d[i_d] < min_d || input_d[i_d] > max_d) {
		int value = (int)(255.5f -(input_d[i_d]-(float)min_d)/(float)normalize);//palette

		out_d[i_r] = palette[value];
		out_d[i_g] = palette[value+256];
		out_d[i_b] = palette[value+512];
	} else {
		out_d[i_r] = 0;
		out_d[i_g] = 0;
		out_d[i_b] = 0;
	}
}
/**************************************************/



/***********************************************/
/* Calculate the norm of the vectorial product */
__global__ void d_cross_p(	float *mReal, float *fReal, float *sin_pf, int *index_f,
							int nbm, int nbf) {
    uint x = threadIdx.x;
	
	float Xp = fReal[x];
	float Yp = fReal[x+nbf];
	float Zp = fReal[x+2*nbf];

	float Xf = mReal[index_f[x]];
	float Yf = mReal[index_f[x]+nbm];
	float Zf = mReal[index_f[x]+2*nbm];

	float Xc = Yp*Zf - Yf*Zp;
	float Yc = Zp*Xf - Zf*Xp;
	float Zc = Xp*Yf - Xf*Yp;

	sin_pf[x] = Xc*Xc + Yc*Yc+ Zc*Zc;
	
}
/**************************************************/



//// Compute the normals
//void display_images(char *h_rgb_img, char *h_d_img,
//					char *h_rgb, float *h_d,
//					float max_d, float min_d, float normalize, int *palette,
//                    int width, int height) {
//	int max_i=width * height,
//		max_i3=max_i*3;
//
//	copyHostToCUDA_char(rgb, max_i3);
//	copyHostToCUDA(d, max_i);
//
//	dim3 gridSize((width + 16 - 1) / 16, (height + 16 - 1) / 16);
//    dim3 blockSize(16, 16);
//
//	
//	d_display_images<<< gridSize, blockSize>>>(d_rgb_img, d_rgb, d_d_img, d_d, max_d, min_d, normalize, palette, width, height);
//	
//	copyCUDAToHost_char(rgb_img, max_i3);
//	//copyCUDAToHost_char(d_img, max_i3);
//}

















/*
    Perform 2D bilateral filter on image using CUDA

    Parameters:
    d_dest - pointer to destination image in device memory
    width  - image width
    height - image height
    e_d    - euclidean delta
    radius - filter radius
    iterations - number of iterations
*/





/************************************/
/* Apply bilateral filter on h_orig */
/* Output is converted to 3D in pReal */
void bilateralFilterRGBA(float *h_pDepth, 
						 float *d_pDepth, 
						 float *h_pReal, 
						 float *d_pReal, 
						 float *d_gaussian,
						 bool *d_validity,
                         int width, int height,
                         float s_s, float s_r, int radius, int iterations,
                         int nthreads) {
	int max_i=width * height;
	
	dim3 gridSize((width + 16 - 1) / 16, (height + 16 - 1) / 16);
    dim3 blockSize(16, 16);

	copyHostToCUDA(pDepth, max_i);	

    for(int i=0; i<iterations; i++)  {
        // sync host and start kernel computation timer
        //synchronizeCUDA();
        d_bilateral_filter_and_3D_conversion<<< gridSize, blockSize>>>(d_pReal, d_pDepth, d_gaussian, d_validity, s_s, s_r, width, height, radius);
    }

	copyCUDAToHost(pReal,3*max_i);	
}
/*******************************************/



/************************************/
/* Apply bilateral filter on h_orig */
/* Output is converted to 3D in pReal */
void bilateralFilter(	 float *h_pDepth, 
						 float *d_pDepth, 
						 float *h_pReal, 
						 float *d_pReal, 
						 float *d_gaussian,
						 bool *d_validity,
                         int width, int height,
                         float s_s, float s_r, int radius, int iterations,
                         int nthreads) {
	int max_i=width * height;
	
	dim3 gridSize((width + 16 - 1) / 16, (height + 16 - 1) / 16);
    dim3 blockSize(16, 16);

	copyHostToCUDA(pDepth, max_i);	

    for(int i=0; i<(iterations/2); i++)  {
        // sync host and start kernel computation timer
        //synchronizeCUDA();
        d_bilateral_filter<<< gridSize, blockSize>>>(d_pReal, d_pDepth, d_gaussian, d_validity, s_s, s_r, width, height, radius);
		d_bilateral_filter<<< gridSize, blockSize>>>(d_pDepth, d_pReal, d_gaussian, d_validity, s_s, s_r, width, height, radius);
    }

	copyCUDAToHost(pDepth,max_i);	
}
/*******************************************/






/*******************************************/
/* Convert projective to Real World on GPU */
void convertP2RW(	float *d_pDepth, float *h_pDepth,
					float *d_pReal, float *h_pReal, 
					int width, int height) {
	int max_i=width * height;
	
	dim3 gridSize((width + 16 - 1) / 16, (height + 16 - 1) / 16);
    dim3 blockSize(16, 16);

	//copyHostToCUDA(pDepth, max_i);	

	//synchronizeCUDA();
    d_convertP2RW<<< gridSize, blockSize>>>(d_pDepth, d_pReal, width, height, max_i); // BUG UNTIL 159- WORKS AT 160

	copyCUDAToHost(pReal,3*max_i);	
	//int MIN_DEPTH=400,MAX_DEPTH=800;
	int x=0, y=0;

	int i2 = 0,
		i3 = 0;
	int i_max = 160*width;

	for (int i=0; i<i_max; i++) {
		x=i%width;
		y=(i-x)/width;
		float Z = h_pDepth[i];
		i2=i+max_i;
		i3=i2+max_i;
		if (Z > MIN_DEPTH && Z < MAX_DEPTH){
			float X_rw = ( (float)x /(float)width -0.5f)*Z*XtoZ;
			float Y_rw = (0.5f-(float)y / (float)height)*Z*YtoZ;

			h_pReal[i]  = X_rw;
			h_pReal[i2] = Y_rw;
			h_pReal[i3] = Z;
		} else {
			h_pReal[i] = 0.0f;
			h_pReal[i2] = 0.0f;
			h_pReal[i3] = 0.0f;
		}
	}
}
/*******************************************/


/***********************/
/* Compute the normals */
void normalComputation(float *h_pReal, float *h_normals, float* d_pReal, float *d_normals, FaceBox maskBox,
                           int width, int height) {
	int max_i=width * height,
		max_i3=max_i*3;
	dim3 gridSize((width + 16 - 1) / 16, (height + 16 - 1) / 16);
	dim3 blockSize(16, 16);


	//copyHostToCUDA(pReal, max_i3);	

	// sync host and start kernel computation timer
	//synchronizeCUDA();
	d_compute_normals<<< gridSize, blockSize>>>(d_normals, d_pReal, maskBox.getLeftX(), maskBox.getRightX(), maskBox.getTopY(), maskBox.getBottomY(), width, height, max_i); 

	copyCUDAToHost(normals,max_i3);	
}
/*******************************************/


/**************************************************/
/* Apply the PCA algorithm to compute the normals */
void PCA(float* d_pReal, 
		 float *d_M, float *d_d,
		 float *d_normals, float *h_normals,
		 int l, int r, int t, int b,
		 int radius,
		 int width, int height) {
	// TODO
	// MallocCUDA M[9*MAX_I]
	// mallocCUDA d[3*MAX_I]
	int max_i  = width * height,
		max_i3 = max_i*3;
	dim3 gridSize((width + 16 - 1) / 16, (height + 16 - 1) / 16);
	dim3 blockSize(16, 16);	

	// Compute the PCA matrices
	//synchronizeCUDA();
	d_PCA<<< gridSize, blockSize>>>(d_pReal, d_M, radius, l, r, t, b, width, height, max_i ); 
	// Perform the eigenvalue decomposition
	//synchronizeCUDA();
	d_eigendecomposition<<< gridSize, blockSize>>>(d_M, d_d, width, height );
	// Extract the normals
	//synchronizeCUDA();
	d_Convert2Normals<<< gridSize, blockSize>>>(d_M, d_normals, width, height, max_i );

	copyCUDAToHost(normals,max_i3);	
}
/*******************************************/








/**************************************************/
/* Apply the PCA algorithm to compute the normals */
void PCA(float* d_pReal, 
		 float *d_M, float *d_d,
		 float *d_normals, float *h_normals,
		 char *d_nData, char *h_nData,
		 int l, int r, int t, int b,
		 int radius,
		 int width, int height) {
	// TODO
	// MallocCUDA M[9*MAX_I]
	// mallocCUDA d[3*MAX_I]
	int max_i  = width * height,
		max_i3 = max_i*3;
	dim3 gridSize((width + 16 - 1) / 16, (height + 16 - 1) / 16);
	dim3 blockSize(16, 16);

	// Compute the PCA matrices
	//synchronizeCUDA();
	d_PCA<<< gridSize, blockSize>>>(d_pReal, d_M, radius, l, r, t, b, width, height, max_i ); 
	// Perform the eigenvalue decomposition
	//synchronizeCUDA();
	d_eigendecomposition<<< gridSize, blockSize>>>(d_M, d_d, width, height );
	// Extract the normals
	//synchronizeCUDA();
	d_Convert2Normals<<< gridSize, blockSize>>>(d_M, d_normals, d_nData, width, height, max_i );

	copyCUDAToHost(normals,max_i3);	
	copyCUDAToHost_char(nData,max_i);	
}
/*******************************************/





/***************************************************************/
/* Calculate the norm of the vectorial product for every point */
void cross_p(float *d_mNormals, float *h_mNormals, int nbm,
			 float *d_fNormals, float *h_fNormals, int nbf,
			 float *d_cross, float *h_cross,
			 int *d_index_f, int *h_index_f) {
	
	copyHostToCUDA(mNormals,nbm*3);
	copyHostToCUDA(fNormals,nbf*3);
	copyHostToCUDA_int(index_f,nbf);


	// Compute the Norm of the cross products
	//synchronizeCUDA();
	d_cross_p<<< nbf, 1 >>>(d_mNormals, d_fNormals, d_cross, d_index_f, nbm, nbf ); 

	copyCUDAToHost(cross,nbf);	
}
/*******************************************/







/*********************/
/* Display one pixel */
__global__ void  
d_displayDepthImage	(	float *pReal,
						char *data,
						bool *validity,
						int *palette, bool isHeat,
						int width, int height) {
	uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x < width && y < height) {
		int i=y*width+x,
			i3=3*i; 
		
		if (validity[i]) {
			int value = (int)(255-(float)(pReal[MAX_I2+i]-MIN_DEPTH)/NORMALIZE_DISPLAY);
			if (!isHeat) {
				data[i3]	= value;
				data[i3+1]	= data[i3];
				data[i3+2]	= data[i3];
			} else {
				data[i3]	= palette[value];
				data[i3+1]	= palette[value+SIZE_CHANNEL_PALETTE];
				data[i3+2]	= palette[value+SIZE_CHANNEL_PALETTE_2];
			}
		} else {
			data[i3]	= 0;
			data[i3+1]	= data[i3];
			data[i3+2]	= data[i3];
		}		
	}
}
/*******************************************/


/*************************/
/* Display the depth map */
void displayDepthImage	(	float *d_pReal,
							char *d_data, char *h_data, 
							bool  *d_validity,
							int *d_palette, bool isHeat,
							int width, int height) {
	dim3 gridSize((width + 16 - 1) / 16, (height + 16 - 1) / 16);
	dim3 blockSize(16, 16);
	
	d_displayDepthImage<<< gridSize, blockSize>>>(d_pReal, d_data, d_validity, d_palette, isHeat, width, height ); 			
}
/*******************************************/














__global__ 
void d_render_monkey(	float *h_monkey_1, float *h_monkey_2, char *data,
						int nb,
						float *h_R, float *h_t,
						float scale) {
	int i_X = blockIdx.x * blockDim.x + threadIdx.x;
	int i_Y = i_X + nb,
		i_Z = i_Y + nb;

	// Apply R,t
	h_monkey_2[i_X] = h_R[0]*h_monkey_1[i_X] + h_R[1]*h_monkey_1[i_Y] + h_R[2]*h_monkey_1[i_Z];
	h_monkey_2[i_Y] = h_R[3]*h_monkey_1[i_X] + h_R[4]*h_monkey_1[i_Y] + h_R[5]*h_monkey_1[i_Z];
	h_monkey_2[i_Z] = h_R[6]*h_monkey_1[i_X] + h_R[7]*h_monkey_1[i_Y] + h_R[8]*h_monkey_1[i_Z] + 5.0f;
	
	// Project
	h_monkey_2[i_X] = (h_monkey_2[i_X]/h_monkey_2[i_Z]/XtoZ + 0.5f)*(float)XN_HR_X_RES;
	h_monkey_2[i_Y] = (-h_monkey_2[i_Y]/h_monkey_2[i_Z]/YtoZ + 0.5f)*(float)XN_HR_Y_RES;

	// Move and scale
	h_monkey_2[i_X] = ( ((h_monkey_2[i_X]-290.0f)/4.0f+30.0f)*scale + 0.5f);
	h_monkey_2[i_Y] = ( ((h_monkey_2[i_Y]-245.0f)/4.0f+30.0f)*scale + 0.5f);
		
	int x = (int)h_monkey_2[i_X];
	int y = (int)h_monkey_2[i_Y];

	int ind = y*XN_VGA_X_RES*3+x*3;
	if (ind < 921597)
	{
	data[ind]	= (char)255;
	data[ind+1] = (char)255;
	data[ind+2] = (char)255;
	}
}

void render_monkey(		float *h_monkey_1, float *d_monkey_1, 
						float *d_monkey_2, 
						char *h_dData, char *d_dData,
						int nb,
						float *d_R, float *d_t,
						float scale, bool isDepth){
	int nb_g = (nb + BLOCK_SIZE-1) / BLOCK_SIZE;

	if (!isDepth)
		copyHostToCUDA_char(dData, MAX_I3);

	// sync host and start kernel computation timer
	//synchronizeCUDA();
	//d_render_monkey<<< nb_g, BLOCK_SIZE >>>(d_monkey_1, d_monkey_2, d_dData, nb, d_R, d_t, scale); 	
}


#endif
