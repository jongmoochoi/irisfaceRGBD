//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

#include <helper_math.h>
#include <cuda.h>
#include <helper_functions.h>
#include <helper_cuda.h>
#include "cuda_runtime.h"
#include <cuda_runtime_api.h>
#include <device_functions.h>
#include <device_launch_parameters.h>
//#include "shrUtils.h"


//error



// For CUDA
#define START_TIMER(timer) \
	if(!param.notimer){ \
		 cudaThreadSynchronize();\
		CUT_SAFE_CALL(cutStartTimer(timer)); \
	}
#define STOP_TIMER(timer) \
	if(!param.notimer){ \
		cudaThreadSynchronize();\
		CUT_SAFE_CALL(cutStopTimer(timer)); \
	}


// allocate memory to CUDA
#define mallocCUDA(var,num)						\
	checkCudaErrors(cudaMalloc((void**) &(d_ ## var), sizeof(float)*num))
// copy h_var to d_var
#define copyHostToCUDA(var,num)						\
	checkCudaErrors(cudaMemcpy(d_ ## var, h_ ## var, sizeof(float)*num, cudaMemcpyHostToDevice))
// copy d_var to h_var
#define copyCUDAToHost(var,num)						\
	checkCudaErrors(cudaMemcpy(h_ ## var, d_ ## var, sizeof(float)*num, cudaMemcpyDeviceToHost))
// copy d_var to d_var2
#define copyCUDAToCUDA(var,var2,num)						\
	checkCudaErrors(cudaMemcpy(d_ ## var, d_ ## var2, sizeof(float)*num, cudaMemcpyDeviceToDevice))



#define copyHostToCUDA_XYZ(var,num)						\
	checkCudaErrors(cudaMemcpy(d_ ## var, h_ ## var, sizeof(float)*num, cudaMemcpyHostToDevice)); \
	checkCudaErrors(cudaMemcpy(&d_ ## var[MAX_I], &h_ ## var[MAX_I], sizeof(float)*num, cudaMemcpyHostToDevice)); \
	checkCudaErrors(cudaMemcpy(&d_ ## var[MAX_I2], &h_ ## var[MAX_I2], sizeof(float)*num, cudaMemcpyHostToDevice))



// release Memory from CUDA
#define releaseCUDA(var)					\
	checkCudaErrors(cudaFree(d_ ## var))
// release Memory from CUDA and free host memory
#define releaseCUDAandHost(var)					\
	checkCudaErrors(cudaFree(d_ ## var)); \
	free(h_ ## var)

#define mallocCUDAuint(var,num)						\
	checkCudaErrors(cudaMalloc((void**) &(d_ ## var), sizeof(unsigned int)*num))


// allocate memory to CUDA
#define mallocCUDA_int(var,num)						\
	checkCudaErrors(cudaMalloc((void**) &(d_ ## var), sizeof(int)*num))
// copy h_var to d_var
#define copyHostToCUDA_int(var,num)						\
	checkCudaErrors(cudaMemcpy(d_ ## var, h_ ## var, sizeof(int)*num, cudaMemcpyHostToDevice))
// copy d_var to h_var
#define copyCUDAToHost_int(var,num)						\
	checkCudaErrors(cudaMemcpy(h_ ## var, d_ ## var, sizeof(int)*num, cudaMemcpyDeviceToHost))

// allocate memory to CUDA
#define mallocCUDA_char(var,num)						\
	checkCudaErrors(cudaMalloc((void**) &(d_ ## var), sizeof(char)*num))
// copy h_var to d_var
#define copyHostToCUDA_char(var,num)						\
	checkCudaErrors(cudaMemcpy(d_ ## var, h_ ## var, sizeof(char)*num, cudaMemcpyHostToDevice))
// copy d_var to h_var
#define copyCUDAToHost_char(var,num)						\
	checkCudaErrors(cudaMemcpy(h_ ## var, d_ ## var, sizeof(char)*num, cudaMemcpyDeviceToHost))

// allocate memory to CUDA
#define mallocCUDA_bool(var,num)						\
	checkCudaErrors(cudaMalloc((void**) &(d_ ## var), sizeof(bool)*num))
// copy h_var to d_var
#define copyHostToCUDA_bool(var,num)						\
	checkCudaErrors(cudaMemcpy(d_ ## var, h_ ## var, sizeof(bool)*num, cudaMemcpyHostToDevice))
// copy d_var to h_var
#define copyCUDAToHost_bool(var,num)						\
	checkCudaErrors(cudaMemcpy(h_ ## var, d_ ## var, sizeof(bool)*num, cudaMemcpyDeviceToHost))


// allocate Array memory to CUDA
#define mallocCUDAarray(var, channel, w, h)						\
	 checkCudaErrors(cudaMallocArray(&(d_ ## var), &( ##channel), w, h))
// release Memory from CUDA array
#define releaseCUDAarray(var)					\
	checkCudaErrors(cudaFreeArray(d_ ## var))
#define copyToCUDAarray(var, pImage, size)						\
	checkCudaErrors(cudaMemcpyToArray(d_ ## var, 0, 0, ## pImage, size, cudaMemcpyHostToDevice))
#define copyToCUDAarrayD2D(var, pImage, size)						\
	checkCudaErrors(cudaMemcpyToArray(d_ ## var, 0, 0, ## pImage, size, cudaMemcpyDeviceToDevice))

// Bind texture to array
#define bindCUDA(var, v2, channelDesc)						\
	checkCudaErrors(cudaBindTextureToArray( var, d_ ## v2, channelDesc))
#define bindCUDA2(var, v2)						\
	checkCudaErrors(cudaBindTextureToArray( var, d_ ## v2))


// example: memCUDA(Xx, Xsize);   // declare d_Xx. no copy.
#define memCUDA(var,num)						\
	float* d_ ## var; checkCudaErrors(cudaMalloc((void**) &(d_ ## var), sizeof(float)*num))
// example:   memHostToCUDA(Xx, Xsize);   // declare d_Xx, then copy h_Xx to d_Xx.
#define memHostToCUDA(var,num)						\
	float* d_ ## var; checkCudaErrors(cudaMalloc((void**) &(d_ ## var), sizeof(float)*num)); \
	checkCudaErrors(cudaMemcpy(d_ ## var, h_ ## var, sizeof(float)*num, cudaMemcpyHostToDevice))
// example:   cpyHostToCUDA(Xx, Xsize);   // copy h_Xx to d_Xx

// Synchronize CUDA
#define synchronizeCUDA()				\
	checkCudaErrors( cutilDeviceSynchronize() )
#define killCUDAthread()					\
	 checkCudaErrors(cudaThreadExit() )

using namespace std;