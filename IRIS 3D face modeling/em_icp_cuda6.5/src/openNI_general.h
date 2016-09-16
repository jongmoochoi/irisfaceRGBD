//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

#include "DATA.h"
#include <helper_math.h>

#include "FaceBox.h"

void copyInCUDAarray(void *pImage, int size);
void copyImage(float *h_src, float *d_dest,
               int width, int height);

float euclideanLen(float4 a, float4 b, float d);
float euclideanLen(float ax, float ay, float bx, float by, float d);
float euclideanLen(float az, float bz, float d);


__global__ void d_generate_gaussian(float *og, float delta, int radius);
float4 rgbaIntToFloat(uint c);
uint rgbaFloatToInt(float4 rgba);
float rgbaFloat4ToFloat(float4 rgba);
float rgbaFloat4ToFloat2(float4 rgba);

__global__ void d_bilateral_filter_and_3D_conversion(float *pReal, float *pDepth, float *gaussian, bool *validity,
                   float s_s, float s_r, int w, int h, int r);

__global__ void d_bilateral_filter(float *pReal, float *pDepth, float *gaussian, bool *validity,
                   float s_s, float s_r, int w, int h, int r);



__global__ void d_compute_normals(float *out, float *pReal,
						int l, int r, int t, int b,
						int w, int h, int max_i) ;

__global__ void d_PCA(float *pReal, float *M, int rad, int l, int r, int t, int b, int w, int h, int max_i);
__global__ void d_Convert2Normals(float *V, float *norm, int w, int h, int max_i);

__global__ void d_convertP2RW(float *pDepth, float *pReal, int w, int h, int max_i);
__global__ void d_convertP2RW(float *pDepth, float *pReal, char *nData, int w, int h, int max_i);

__global__ void d_cross_p(float *pReal, float *fReal, float *sin_pf, int *index_f,
									int nbp, int nbf) ;

void convertP2RW(	float *d_pDepth,  float *h_pDepth,
					float *d_pReal, float *h_pReal, 
					int width, int height);

void freeTextures();
void updateGaussian(float delta, int radius);
void initTexture(int width, int height, void *pImage);

void initGaussian(float *d_gaussian, float gaussian_delta, int fr_g);

void bilateralFilterRGBA(float *h_pDepth, 
						 float *d_pDepth, 
						 float *h_pReal, 
						 float *d_pReal, float *d_gaussian,
						 bool *d_validity,
                         int width, int height,
                         float s_s, float s_r, int radius, int iterations,
                         int nthreads);
void bilateralFilter(	 float *h_pDepth, 
						 float *d_pDepth, 
						 float *h_pReal, 
						 float *d_pReal, 
						 float *d_gaussian,
						 bool *d_validity,
                         int width, int height,
                         float s_s, float s_r, int radius, int iterations,
                         int nthreads);

__global__ void d_display_images(char *out_rgb, char *input_rgb,
						char *out_d, float *input_d,
						float max_d, float min_d, float normalize, int *palette,
						int w, int h) ;
void display_images(	char *h_rgb_img, char *h_d_img,
						char *h_rgb, float *h_d,
						float max_d, float min_d, float normalize, int *palette,
						int width, int height);


void normalComputation(float *h_orig, float *h_dest, float* d_pReal, float *d_normals, FaceBox maskBox,
                           int width, int height);
void PCA(float* d_pReal, 
		 float *d_M, float *d_d, 
		 float *d_normals, float *h_normals,
		 int l, int r, int t, int b,
		 int radius,
		 int width, int height);


void PCA(float* d_pReal, 
		 float *d_M, float *d_d, 
		 float *d_normals, float *h_normals,
		 char *d_nData, char *h_nData,
		 int l, int r, int t, int b,
		 int radius,
		 int width, int height);

void cross_p(float *d_pNormals, float *h_pNormals, int nbp,
			 float *d_fNormals, float *h_fNormals, int nbf,
			 float *d_cross, float *h_cross,
			 int *d_index_f, int *h_index_f);




__global__ void d_displayDepthImage(float *pReal,
							char *data,
							bool *validity,
							int *palette, bool isHeat,
							int width, int height);
void displayDepthImage(	float *d_pReal,
						char *d_data, char *h_data, 
						bool  *d_validity,
						int *d_palette, bool isHeat,
						int width, int height);






__global__ void d_render_monkey(float *monkey_1, float *monkey_2, char *dData,
						int nb,
						float *h_R, float *h_t,
						float scale) ;
void render_monkey(		float *h_monkey_1, float *d_monkey_1, 
						float *d_monkey_2, 
						char *h_dData, char *d_dData,
						int nb,
						float *d_R, float *d_t,
						float scale, bool isDepth);