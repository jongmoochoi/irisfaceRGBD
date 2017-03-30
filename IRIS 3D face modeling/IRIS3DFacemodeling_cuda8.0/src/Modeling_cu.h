//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

#include "DATA.h"
#include "cudaMem.h"


#include "Voxel.h"

#include <iostream>
#include <algorithm>
#include <cstdio>


#include "cublas.h"







#ifndef MODELING_CU_H_
#define MODELING_CU_H_


void d_applyInvTransformation(float *face, float *R, float *t, int nb);
void d_convert2Cylindrical(float *face, float *face_C, int *face_I, int nb);
void d_computeOverlap(float *face_C, int *face_I, float *dtdy, float *model, float *overlap, bool *defined, int nb);
void __global__ d_preparePoints(float *face, float *R, float *t, float *face_C, int *face_I, float *dtdy, float *model, float *overlap, bool *defined, int nb);
void __global__ d_updateModel(float *face_C, int *face_I, float *dtdy, float *overlap, float *model, float *meann, int nb);

void __global__ d_convertToXYZRGB(float *model, float *modelXYZ, float *modelRGB, char *image, bool *defined_C, float cx, float cy, float cz, float normalize, int w, int h);
void convertToXYZRGB(	float *h_model, float *d_model, 
						float *h_modelXYZ, float *d_modelXYZ, 
						float *h_modelRGB, float *d_modelRGB,
						char *h_iData, char *d_iData,
						bool *h_defined_C, bool *d_defined_C,
						Voxel c, float normalize);


void updateModel(	float *h_face, 
					float *d_face,
					int nb,
					float *h_R, 
					float *d_R, 
					float *h_t, 
					float *d_t,
					float *d_face_C,
					int *d_face_I,
					float *d_dtdy,
					float *h_model,
					float *d_model,
					float *h_overlap,
					float *d_overlap,
					bool *h_defined,
					bool *d_defined,
					char *h_dData,
					char *d_dData,
					float *d_meann,
					bool *wrongRegistration,
					int NB_FRAMES);


#endif
