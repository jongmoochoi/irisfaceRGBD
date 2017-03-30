//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.


//#include "stdafx.h"

#include "Modeling_cu.h"
#include <device_launch_parameters.h>

float overlap_history[5]={THRESHOLD_OVERLAP, THRESHOLD_OVERLAP, THRESHOLD_OVERLAP, THRESHOLD_OVERLAP, THRESHOLD_OVERLAP};
float threshold_overlap=0.0f;

/*************************************************************/
/* Apply the inverse of the transformation (R,t) to the face */
//__global__  
__device__  void d_applyInvTransformation(float *face, float *R, float *t, int nb) {
	//int i = threadIdx.x;
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if (x<nb) {
		float X = face[x]		- t[0];
		float Y = face[x+MAX_I] - t[1];
		float Z = face[x+MAX_I2] - t[2];

		face[x]			= (R[0]*X + R[3]*Y + R[6]*Z );
		face[x+MAX_I]	= (R[1]*X + R[4]*Y + R[7]*Z );
		face[x+MAX_I2]	= (R[2]*X + R[5]*Y + R[8]*Z ) - CYLINDER_Z;

		//out[x]			= (R[0]*X + R[3]*Y + R[6]*Z );
		//out[x+MAX_I]	= (R[1]*X + R[4]*Y + R[7]*Z );
		//out[x+MAX_I2]	= (R[2]*X + R[5]*Y + R[8]*Z );
	}
}
/*******************************/




/*******************************************/
/* Convert the face to a cylindrical model */
// face_C contains the ro,theta values
// face_I contains the indexes
//__global__  
__device__  void d_convert2Cylindrical(float *face, float *face_C, int *face_I, float *dtdy, int nb) {
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (ix<nb) {
		int iy = ix + MAX_I;
		int iz = iy + MAX_I;
		
		float x = face[ix];
		float y = face[iy];
		float z = face[iz];
		
		// Compute cylindrical Coordinate
		face_C[ix] = pow( (x*x+z*z), 0.5f);		
		if (x<0.0f)
			face_C[iy] = (atan(z/x)+PI)*RAD2DEG;
		if (x>0.0f) {
			if (z>=0.0f)
				face_C[iy] = atan(z/x)*RAD2DEG;
			else
				face_C[iy] = (atan(z/x)+_2PI)*RAD2DEG;
		}
		if (x==0.0f) {
			if (z>0.0f)
				face_C[iy] = 90.0f;
			if (z<0.0f)
				face_C[iy] = 270.0f;
			if (z==0.0f)
				face_C[iy] = 180.0f;
		}
		float	i_tf = ((float)THETA_EXPAND*(face_C[iy])-(float)SHIFT_T+0.5f),
				i_yf = (Y_EXPAND*(-y+1.0f)+0.5f);
		// Find the row/column on the image
		int i_tc = (int)i_tf;
		int i_yc = (int)i_yf;

		// Set the weights
		dtdy[ix]	= fabs(i_tf-(float)i_tc);
		dtdy[iy]	= fabs(i_yf-(float)i_yc);

		dtdy[ix] = (dtdy[ix]>=1.0f ? 1.0f : dtdy[ix]);
		dtdy[iy] = (dtdy[iy]>=1.0f ? 1.0f : dtdy[iy]);

		// Shift to put the nose in the middle
		if (face_C[iy]<=SHIFT_T) 
			i_tc += THETA_MAX;	

		// Find the index
		face_I[ix]	=	i_yc*THETA_MAX + i_tc;	

		//if (i_yc>150 && fabs(face_C[ix])>0.0f)
		//	face_C[ix] = 0.0f;
	}

}
/*******************************/






/**********************************************/
/* Compute the overlapping error for pixel ix */
// Overlap contains the error
// Defined is used to compute the number of defined points
//__global__  
__device__ void d_computeOverlap(float *face_C, int *face_I, float *dtdy, float *model, float *overlap, bool *defined, int nb) {
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (ix<nb) {
		int im = face_I[ix];

		if (model[im] != 0.0f) {
			overlap[ix] = pow( (face_C[ix]-model[im])*(face_C[ix]-model[im]), 0.5f);//fabs(face_C[ix]-model[im]);
			defined[ix] = true;
		} else {
			overlap[ix] = 0.0f;
			defined[ix] = false;
		}

		if (fabs(overlap[ix]) > 100.0f)
			overlap[ix]=0.0f;
	}
	
	
	/*int nb_d=0;

	if (ix<nb) {
		overlap[ix] = 0.0f;
		
		float	dt = fabs(dtdy[ix]),
				dy = fabs(dtdy[ix+MAX_I]);

		float	w0 = (1.0f-dt)*(1.0f-dy),
				w1 = (dt)*(1.0f-dy),
				w2 = (1.0f-dt)*(dy),
				w3 = (dt)*(dy);

		int im = face_I[ix];
		
		int im1=im+1,
			im2=im+THETA_MAX,
			im3=im2+1;

		if (im<MAX_I){
			if (model[im] != 0.0f && w0>MIN_WEIGHT) {
				overlap[ix] += w0*pow( (face_C[ix]-model[im])*(face_C[ix]-model[im]), 0.5f);
				nb_d++;
			} 
		}
		if (im1<MAX_I){
			if (model[im1] != 0.0f && w1>MIN_WEIGHT) {
				overlap[ix] += w1*pow( (face_C[ix]-model[im1])*(face_C[ix]-model[im1]), 0.5f);
				nb_d++;
			} 
		}
		if (im2<MAX_I){
			if (model[im2] != 0.0f && w2>MIN_WEIGHT) {
				overlap[ix] += w2*pow( (face_C[ix]-model[im2])*(face_C[ix]-model[im2]), 0.5f);
				nb_d++;
			} 
		}
		if (im3<MAX_I){
			if (model[im3] != 0.0f && w3>MIN_WEIGHT) {
				overlap[ix] += w3*pow( (face_C[ix]-model[im3])*(face_C[ix]-model[im3]), 0.5f);				
				nb_d++;
			} 
		}

		if (nb_d>0) {
			overlap[ix] /= (float)nb_d;
			defined[ix] = true;
		} else {
			overlap[ix] = 0.0f;
			defined[ix] = false;
		}

		if (fabs(overlap[ix]) > 100.0f)
			overlap[ix]=0.0f;
	}*/
}
/*******************************/





/********************/
/* Update the model */
__global__ void d_preparePoints(float *face, float *R, float *t, float *face_C, int *face_I, float *dtdy, float *model, float *overlap, bool *defined, int nb) {
	d_applyInvTransformation(face, R, t, nb);
	d_convert2Cylindrical(face, face_C, face_I, dtdy, nb);
	d_computeOverlap(face_C, face_I, dtdy, model, overlap, defined, nb);
}
/*******************************/




/********************/
/* Update the model */
__global__ void d_updateModel(float *face_C, int *face_I, float *dtdy, float *overlap, float *model, float *meann, int nb) {
	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (abs(ix) < nb) {
		int	im0 = face_I[ix],
			im1 = im0+1,
			im2 = im0+THETA_MAX,
			im3 = im2+1;

		float	dt = fabs(dtdy[ix]);
		float	dy = fabs(dtdy[ix+MAX_I]);
		float	w0 = fabs((1.0f-dt)*(1.0f-dy));
		float	w1 = fabs((dt)*(1.0f-dy));
		float	w2 = fabs((1.0f-dt)*(dy));
		float	w3 = fabs((dt)*(dy));

		float v=fabs(face_C[ix]);

		float value=0.0f;

		if (fabs(overlap[ix])<THRESHOLD_UPDATE) {
			if (v>0.0f) {
				if (im0 < MAX_IMG_INDEX_1 && w0>MIN_WEIGHT){
					value = (((meann[im0])*(model[im0]) + (w0)*(v)) / ((meann[im0])+(w0)));

					model[im0] = value;
					meann[im0]+=w0;
				}
				if (im1 < MAX_IMG_INDEX_1 && w1>MIN_WEIGHT) {
					value = (((meann[im1])*(model[im1]) + (w1)*(v)) / ((meann[im1])+(w1)));

					model[im1] = value;
					meann[im1]+=w1;
				}
				if (im2 < MAX_IMG_INDEX_1 && w2>MIN_WEIGHT) {	
					value = (((meann[im2])*(model[im2]) + (w2)*(v)) / ((meann[im2])+(w2)));

					model[im2] = value;
					meann[im2]+=w2;
				}
				if (im3 < MAX_IMG_INDEX_1 && w3>MIN_WEIGHT) { 
					value = (((meann[im3])*(model[im3]) + (w3)*(v)) / ((meann[im3])+(w3)));

					model[im3] = value;
					meann[im3]+=w3;
				}
			}
		}


/*
		if (model[im0]>RO_MAX) {
			model[im0] = 0.0f;//face_C[ix];
			meann[im0] = 0.0f;//w0;
		}
		if (model[im1]>RO_MAX) {
			model[im1] = 0.0f;//face_C[ix];
			meann[im1] = 0.0f;//w1;
		}
		if (model[im2]>RO_MAX) {
			model[im2] = 0.0f;//face_C[ix];
			meann[im2] = 0.0f;//w2;
		}
		if (model[im3]>RO_MAX) {
			model[im3] = 0.0f;//face_C[ix];
			meann[im3] = 0.0f;//w3;
		}*/

	}
	
}
/*******************************/

/******************************/
/* Update the unwrapped image */
__global__ void d_updateUnwrappedImage(float *model, char *dData, int w, int h) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < w && y < h) {
		int i1 = y*w+x,
			i3 = 3*i1;

		dData[i3] = ((model[i1]<1.0f) ? (char)(model[i1]*255.0f + 0.5f)  : 255);
		dData[i3+1] = dData[i3];
		dData[i3+2] = dData[i3];
	}
}
/*******************************/





/******************************/
/* Update the unwrapped image */
__global__ void d_convertToXYZRGB(float *model, float *modelXYZ, float *modelRGB, char *iData, bool *defined_C, float cx, float cy, float cz, float normalize, int w, int h) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

	float ro=0.0f, theta=0.0f;
	float xx=0.0f, yy=0.0f, zz=0.0f;
	float xR=0.0f, yR=0.0f, zR=0.0f;

	float ixf=0.0f, iyf=0.0f;
	int ixi=0, iyi=0;
	float dx=0.0f, dy=0.0f;
	float w00=0.0f, w01=0.0f, w10=0.0f, w11=0.0f;

	float R=0,G=0,B=0;

	int iii00=0, iii01=0, iii10=0, iii11=0;
	float b00=0, b01=0, b10=0, b11=0;
	float g00=0, g01=0, g10=0, g11=0;
	float r00=0, r01=0, r10=0, r11=0;
	int col=0;


    if (x < w && y < h) {
		int i = y*w+x,
			i1 = i+MAX_IMG_INDEX_1,
			i2 = i+MAX_IMG_INDEX_2,
			i3 = 3*i;

		if (defined_C[i]) {
			// Find the cylindric coordinate ro/theta
			ro = model[i];
			col = (i%THETA_MAX)/(THETA_MAX/360);
			
			if (theta<(THETA_MAX-SHIFT_T))
				theta = (float)(col+SHIFT_T) * DEG2RAD;
			else
				theta = (float)(col+SHIFT_T-THETA_MAX) * DEG2RAD;
			
			// Find the cartesian coordinate
			yy = -(((float)(i-col)/(float)THETA_MAX)/(Y_EXPAND)-1.0f);
			xx = ro * cos(theta);
			zz = ro * sin(theta);

			// Come back to real coordinate 
			xR = (xx/1.05f-0.02f)*normalize+cx;
			yR = (yy+0.02f)*normalize+cy;

			//xR = (xx)*normalize+cx;
			//yR = (yy)*normalize+cy;
			zR = (zz+CYLINDER_Z)*normalize+cz;	//TODO: CALIBRATE TO FIX THIS

			//#define CALIB_T0	0.0f//3.0f
			//#define CALIB_T1	0.0f//3.0f
			//#define CALIB_T2	0.0f
			//#define ANGLE_X		-0.00f//-0.005f
			//#define ANGLE_Y		0.00f

			/*#define ANGLE_X		0.021000f
			#define ANGLE_Y		-0.00280f//-0.001200f
			#define CALIB_T0	3.85000f//3.785000f
			#define CALIB_T1	-3.88f//-5.88f
			#define CALIB_T2	0.0f

			#define cY			cos(ANGLE_Y)
			#define cX			cos(ANGLE_X)
			#define sY			sin(ANGLE_Y)
			#define sX			sin(ANGLE_X)
			#define CALIB_R0	cY
			#define CALIB_R1	sX*sY
			#define CALIB_R2	cX*sY
			#define CALIB_R3	0.0f
			#define CALIB_R4	cX
			#define CALIB_R5	-sX
			#define CALIB_R6	-sY
			#define CALIB_R7	sX*cY
			#define CALIB_R8	cX*cY

			xR = CALIB_R0*xR+CALIB_R1*yR+CALIB_R2*zR+CALIB_T0;
			yR = CALIB_R3*xR+CALIB_R4*yR+CALIB_R5*zR+CALIB_T1;
			zR = CALIB_R6*xR+CALIB_R7*yR+CALIB_R8*zR+CALIB_T2;*/

			//if (IS_HR)
			yR+=2.0f;

			// Project onto the image plane
			ixf = (xR/zR/XtoZ + 0.5f)*(float)XN_HR_X_RES;
			iyf = (-yR/zR/YtoZ + 0.5f)*(float)XN_HR_Y_RES;

			// Take the integer part and decimal part
			ixi = (int)ixf; 
			iyi = (int)iyf;
			dx = ixf-(float)ixi;
			dy = iyf-(float)iyi;
			//ixi *= 3;
			//iyi *= 3;
			
			// Find the corresponding indexes onto the image
			iii00	= (iyi*XN_HR_X_RES	+ ixi)*3;
			iii10	= iii00 + 3;
			iii01	= iii00 + XN_HR_X_RES3;
			iii11	= iii01 + 3;

			if (iii11<MAX_I_HR3 && iii00>=0) {
				// Find the weights
				w00 = (1-dx) * (1-dy);
				w10 = dx * (1-dy);
				w01 = (1-dx) * dy;
				w11 = dx * dy;

				b00 = (iData[iii00]<0 ? 255.0f-fabs((float)iData[iii00]): (float)iData[iii00]);
				b10 = (iData[iii10]<0 ? 255.0f-fabs((float)iData[iii10]): (float)iData[iii10]);
				b01 = (iData[iii01]<0 ? 255.0f-fabs((float)iData[iii01]): (float)iData[iii01]);
				b11 = (iData[iii11]<0 ? 255.0f-fabs((float)iData[iii11]): (float)iData[iii11]);

				g00 = (iData[iii00+1]<0 ? 255.0f-fabs((float)iData[iii00+1]): (float)iData[iii00+1]);
				g10 = (iData[iii10+1]<0 ? 255.0f-fabs((float)iData[iii10+1]): (float)iData[iii10+1]);
				g01 = (iData[iii01+1]<0 ? 255.0f-fabs((float)iData[iii01+1]): (float)iData[iii01+1]);
				g11 = (iData[iii11+1]<0 ? 255.0f-fabs((float)iData[iii11+1]): (float)iData[iii11+1]);
				
				r00 = (iData[iii00+2]<0 ? 255.0f-fabs((float)iData[iii00+2]): (float)iData[iii00+2]);
				r10 = (iData[iii10+2]<0 ? 255.0f-fabs((float)iData[iii10+2]): (float)iData[iii10+2]);
				r01 = (iData[iii01+2]<0 ? 255.0f-fabs((float)iData[iii01+2]): (float)iData[iii01+2]);
				r11 = (iData[iii11+2]<0 ? 255.0f-fabs((float)iData[iii11+2]): (float)iData[iii11+2]);

				// Find R,G,B	-values should be between 0 and 1-		
				B = (w00*b00	+ w10*b10 
								+ w01*b01	
								+ w11*b11)	/ 255.0f;
				G = (w00*g00	+ w10*g10
								+ w01*g01 
								+ w11*g11)	/ 255.0f;
				R = (w00*r00	+ w10*r10
								+ w01*r01
								+ w11*r11)	/ 255.0f;
			}

			// Save into the file
			modelXYZ[i]  = -xx;
			modelXYZ[i1] =  yy;
			modelXYZ[i2] = -zz;

			modelRGB[i3]   = R;
			modelRGB[i3+1] = G;
			modelRGB[i3+2] = B;
		}
	}
}
/*******************************/



/*********************************/
/* Entire model updating process */
void convertToXYZRGB(	float *h_model, float *d_model, 
						float *h_modelXYZ, float *d_modelXYZ, 
						float *h_modelRGB, float *d_modelRGB,
						char *h_iData, char *d_iData,
						bool *h_defined_C, bool *d_defined_C,
						Voxel c, float normalize) {
	
	dim3 gridSize((THETA_MAX + 16 - 1) / 16, (Y_MAX + 16 - 1) / 16);
	dim3 blockSize(16, 16);

	copyHostToCUDA(model, MAX_IMG_INDEX_1);
	copyHostToCUDA_char(iData, MAX_I3);	// To modify if want HR
	copyHostToCUDA_bool(defined_C, MAX_IMG_INDEX_1);

	d_convertToXYZRGB<<< gridSize, blockSize >>>(d_model, d_modelXYZ, d_modelRGB, d_iData, d_defined_C, c.getXr(), c.getYr(), c.getZr(), normalize, THETA_MAX, Y_MAX);	
	
	copyCUDAToHost(modelXYZ, MAX_IMG_INDEX_3);
	copyCUDAToHost(modelRGB, MAX_IMG_INDEX_3);
}
/*******************************/










/*********************************/
/* Entire model updating process */
void updateModel(	float *h_face, float *d_face,
					int nb,
					float *h_R, float *d_R, 
					float *h_t, float *d_t,
					float *d_face_C,
					int *d_face_I,
					float *d_dtdy,
					float *h_model, float *d_model,
					float *h_overlap, float *d_overlap,
					bool *h_defined, bool *d_defined,
					char *h_dData, char *d_dData,
					float *d_meann,
					bool *wrongRegistration,
					int NB_FRAMES) {
	
	dim3 gridSize((THETA_MAX + BLOCK_SIZE - 1) / 16, (Y_MAX + BLOCK_SIZE - 1) / BLOCK_SIZE);
	dim3 blockSize(BLOCK_SIZE, BLOCK_SIZE);
	int nb_g = (nb + BLOCK_SIZE-1) / BLOCK_SIZE;

	copyHostToCUDA(t, 3);
	copyHostToCUDA(R, 9);
	//copyHostToCUDA(face, MAX_I3);
	copyHostToCUDA_XYZ(face, nb);

	//synchronizeCUDA();
	//d_preparePoints<<< nb, 1 >>>(d_face, d_R, d_t, d_face_C, d_face_I, d_dtdy, d_model, d_overlap, d_defined, nb);	
	d_preparePoints<<< nb_g, BLOCK_SIZE >>>(d_face, d_R, d_t, d_face_C, d_face_I, d_dtdy, d_model, d_overlap, d_defined, nb);	
	
	//copyCUDAToHost(overlap, MAX_I);
	//copyCUDAToHost_bool(defined, MAX_I);

	int		nb_overlap=0;
	float	overlap=0.0f,
			av_overlap=0.0f;
	//////////////////////////////////////////////////////////////////////////
	// VERSION 2
	copyCUDAToHost(overlap, MAX_I);
	copyCUDAToHost_bool(defined, MAX_I);
	for (int i=0; i<nb; i++){
		if (h_defined[i]) {
			nb_overlap++;
			av_overlap += h_overlap[i];
		}
	}
	
	
	
	
	
	
	//////////////////////////////////////////////////////////////////////////
	// VERSION 1
	//av_overlap = cublasSasum(nb, d_overlap, 1);
	//nb_overlap = nb-(int)cublasSasum(nb, (float *)d_defined, 1);

	if (nb_overlap > 0)
		av_overlap /= (float)nb_overlap;

	// Compute the average of the values greater than the average
	if (av_overlap > 0.0f) {
		//////////////////////////////////////////////////////////////////////////
		// VERSION 1
		// copyCUDAToHost(overlap, MAX_I);
		// copyCUDAToHost_bool(defined, MAX_I);
		nb_overlap=0;
		overlap=0.0f;
		for (int i=0; i<nb; i++){
			if (h_defined[i] && h_overlap[i]>av_overlap) {
				nb_overlap++;
				overlap += h_overlap[i];
			}
		}
		if (nb_overlap > 0)
			overlap /= (float)nb_overlap;
	}

	// Set the adaptive threshold
	if (NB_FRAMES > 0 && NB_FRAMES <= 6) 
		overlap_history[NB_FRAMES-1] = overlap;
	if (NB_FRAMES==0)
		threshold_overlap = THRESHOLD_OVERLAP;
	if (NB_FRAMES==6) 
		threshold_overlap = (overlap_history[0]+overlap_history[1]+overlap_history[2]+overlap_history[3]+overlap_history[4])/2.0f;

	//printf("%f // %f\n", overlap, threshold_overlap);



	// Detect wrong registrations
	if (overlap < threshold_overlap || overlap != overlap) {
		//synchronizeCUDA();
		d_updateModel<<< nb_g, BLOCK_SIZE >>>(d_face_C, d_face_I, d_dtdy, d_overlap, d_model, d_meann, nb);	

		//d_updateModel<<< nb, 1 >>>(d_face_C, d_face_I, d_dtdy, d_overlap, d_model, d_meann, nb);	
		(*wrongRegistration) = false;
	} else
		(*wrongRegistration) = true;

	
	////synchronizeCUDA();
	d_updateUnwrappedImage<<< gridSize, blockSize>>>(d_model, d_dData, THETA_MAX, Y_MAX ); 

	copyCUDAToHost_char(dData, MAX_IMG_INDEX_3);

	
	
	
	/*copyCUDAToHost(model, MAX_IMG_INDEX_1);
	float *h_meann = new float[MAX_IMG_INDEX_1];
	float *h_face_C = new float[MAX_IMG_INDEX_1];
	copyCUDAToHost(meann, MAX_IMG_INDEX_1);
	copyCUDAToHost(face_C, MAX_IMG_INDEX_1);
	for (int i=0;i<MAX_IMG_INDEX_1; i++) {
		if (h_face_C[i]>RO_MAX) {
			printf("face %f\n", h_face_C[i]);			
		}
		
		if (h_model[i]>RO_MAX) {
			printf("%f %f\n", h_model[i], h_meann[i]);			
		}
	}



	float *h_dtdy = new float[MAX_I2];
	copyCUDAToHost(dtdy, MAX_I2);
	//for (int i=0; i<nb; i++)
	//	printf("%f %f\n", h_dtdy[i], h_dtdy[i+MAX_I]);
	delete[] h_dtdy;
	delete[] h_meann;
	delete[] h_face_C;*/

}
/*******************************/




