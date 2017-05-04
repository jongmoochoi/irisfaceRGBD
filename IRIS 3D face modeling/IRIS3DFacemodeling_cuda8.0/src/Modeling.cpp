//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.


#include "stdafx.h"
#include "Modeling.h"
#include "Modeling_cu.h"
#include "CudaMem.h"

/***************/
/* Constructor	*/
Modeling::Modeling(){
	mallocCUDA(face, MAX_I3);
	mallocCUDA(R,9); 
	mallocCUDA(t,3);
	mallocCUDA(face_C, MAX_I2);
	mallocCUDA_int(face_I, MAX_I);
	mallocCUDA(dtdy, MAX_I2);

	h_overlap = new float[MAX_I];
	mallocCUDA(overlap, MAX_I);
	h_defined = new bool[MAX_I];
	mallocCUDA_bool(defined, MAX_I);

	h_model	= new float[MAX_IMG_INDEX_2];
	mallocCUDA(model, MAX_IMG_INDEX_2);

	mallocCUDA(meann, MAX_IMG_INDEX_1);
	h_meann = new float[MAX_IMG_INDEX_1];
	h_modelXYZ	= new float[MAX_IMG_INDEX_3];
	mallocCUDA(modelXYZ, MAX_IMG_INDEX_3);
	h_modelRGB	= new float[MAX_IMG_INDEX_3];
	mallocCUDA(modelRGB, MAX_IMG_INDEX_3);
	h_defined_C	= new bool[MAX_IMG_INDEX_1];
	mallocCUDA_bool(defined_C, MAX_IMG_INDEX_1);
	mallocCUDA_char(dData, MAX_IMG_INDEX_3);
	mallocCUDA_char(iData, MAX_I_HR3);

	for (int i=0; i<MAX_IMG_INDEX_1; i++) {
		h_model[i] = 0.0f;
		h_meann[i] = 0.0f;
		h_defined_C[i] = false;
	}
	copyHostToCUDA(meann, MAX_IMG_INDEX_1);
	copyHostToCUDA(model, MAX_IMG_INDEX_1);
	copyHostToCUDA_bool(defined_C, MAX_IMG_INDEX_1);

	

	// Create the images containing the model -depothImg is just for display-
	imFront		= cvCreateImage(cvSize(XN_HR_X_RES, XN_HR_Y_RES+Y_MAX), IPL_DEPTH_8U, CHANNELS );
	depthImg	= cvCreateImage(cvSize(THETA_MAX, Y_MAX), IPL_DEPTH_8U, CHANNELS );	// Depth model

	h_dData = (char *)depthImg->imageData;
	for (int i=0; i<MAX_IMG_INDEX_3; i++)
		h_dData[i]=0;
	h_iData = (char *)imFront->imageData;

	// Mesh, allocate the maximum number
	mesh_model = new int *[3];
	for (int i=0; i<3; i++) 
		mesh_model[i] = new int[2*(Y_MAX-1)*(THETA_MAX-1)];
	
}
/***************/

/***************/
/*	Destructor	*/
Modeling::~Modeling(){
	//	Release the images used for model
	cvReleaseImage(&imFront);
	cvReleaseImage(&depthImg);

	delete[] mesh_model[0];
	delete[] mesh_model[1];
	delete[] mesh_model[2];	
	delete[] mesh_model;

	// TODO SHOULD BE FREED
	releaseCUDA(R); 
	releaseCUDA(t);
	releaseCUDA(face_C);
	releaseCUDA(face_I);
	releaseCUDA(face);
	releaseCUDA(dtdy);

	releaseCUDAandHost(model);
	releaseCUDAandHost(overlap);
	releaseCUDAandHost(defined);
	releaseCUDAandHost(meann);
	
	releaseCUDA(dData);
	releaseCUDA(iData);

	releaseCUDAandHost(modelXYZ);
	releaseCUDAandHost(modelRGB);
	releaseCUDAandHost(defined_C);
	
}
/***************/




/***************/
/*	Destructor	*/
void Modeling::reset(){
	for (int i=0; i<MAX_IMG_INDEX_1; i++) {
		h_model[i] = 0.0f;
		h_meann[i] = 0.0f;
		h_defined_C[i] = false;
		h_dData[i]=0;
		h_dData[i+MAX_IMG_INDEX_1]=0;
		h_dData[i+MAX_IMG_INDEX_2]=0;
	}
	for (int i=MAX_IMG_INDEX_1; i<MAX_IMG_INDEX_3; i++)
		h_dData[i]=0;
	copyHostToCUDA(meann, MAX_IMG_INDEX_1);
	copyHostToCUDA(model, MAX_IMG_INDEX_1);
	copyHostToCUDA_bool(defined_C, MAX_IMG_INDEX_1);
}
/***************/





/***********************/
/* Getters and setters */
IplImage *Modeling::getDepthImg(){
	return depthImg;
}
float *Modeling::getDepthInfo(){
	return h_model;
}
float Modeling::getHmodelXYZ(int i){
	return h_modelXYZ[i];
}
float *Modeling::getHmodelXYZ(){
	return h_modelXYZ;
}
float Modeling::getHmodelRGB(int i){
	return h_modelRGB[i];
}
float *Modeling::getHmodelRGB(){
	return h_modelRGB;
}
bool *Modeling::getH_defined_C() {
	return h_defined_C;
}
bool Modeling::getHdefined(int i) {
	return h_defined_C[i];
}
int **Modeling::getMesh() {
	return mesh_model;
}
int Modeling::getMesh(int k, int i) {
	return mesh_model[k][i];
}
int Modeling::getNbMesh() {
	return nb_mesh;
}
void Modeling::getXYZ(int i, float *X, float *Y, float *Z) {
	float ro=0.0f, theta=0.0f;
	int col=0;
	
	// Find the cylindric coordinate ro/theta
	ro = h_model[i];
	col = (i%THETA_MAX)/(THETA_MAX/360);
	
	if (theta<(THETA_MAX-SHIFT_T))
		theta = (float)(col+SHIFT_T) * DEG2RAD;
	else
		theta = (float)(col+SHIFT_T-THETA_MAX) * DEG2RAD;
	
	// Find the cartesian coordinate
	(*Y) = -(((float)(i-(int)(col+0.5f))/(float)THETA_MAX)/(Y_MAX/2)-1.0f);
	(*X) = -ro * cos(theta);
	(*Z) = -ro * sin(theta);
}
IplImage *Modeling::getImFront(){
	return imFront;
}
void Modeling::setImFront(IplImage *imFront){
	cvCopy(imFront, this->imFront);
}
void Modeling::setCFront(Voxel cFront){
	this->cFront = cFront;
}
void Modeling::setDepthImg(IplImage *depthImg){
	this->depthImg = depthImg;
}
void Modeling::setDepthImg(int i, uchar c) {
	this->depthImg->imageData[i] = c;
}
void Modeling::setDepthInfo(float *h_model){
	this->h_model = h_model;
}
void Modeling::setDepthInfo(int i, float value) {
	this->h_model[i] = value;
}
void Modeling::setHModel(int i, float value) {
	this->h_model[i] = value;	
}
/***************/




/************************/
/* Transform the Voxels	*/
void Modeling::applyTransformation(Voxel *pt, float *h_R, float *h_t) {
	float x = 0.0f,
			y = 0.0f,
			z = 0.0f; 
	x = (*pt).getXr();
	y = (*pt).getYr();
	z = (*pt).getZr();

	(*pt).setXr(h_R[0]*x + h_R[1]*y + h_R[2]*z + h_t[0]);
	(*pt).setYr(h_R[3]*x + h_R[4]*y + h_R[5]*z + h_t[1]);
	(*pt).setZr(h_R[6]*x + h_R[7]*y + h_R[8]*z + h_t[2]);
}
/*********************/



/************************/
/* Transform the Voxels	*/
void Modeling::applyInvTransformation(Voxel *pt, float *h_R, float *h_t) {
	float	x = (*pt).getXr() - h_t[0],
			y = (*pt).getYr() - h_t[1],
			z = (*pt).getZr() - h_t[2];

	(*pt).setXr(h_R[0]*x + h_R[3]*y + h_R[6]*z );
	(*pt).setYr(h_R[1]*x + h_R[4]*y + h_R[7]*z );
	(*pt).setZr(h_R[2]*x + h_R[5]*y + h_R[8]*z );
	
}
/*********************/


/************************/
/* Transform the Voxels	*/
void Modeling::applyInvRotation(float *f1, float *f2, float *f3, float *h_R) {
	float	x = (*f1),
			y = (*f2),
			z = (*f3); 
	
	(*f1) = h_R[0]*x + h_R[3]*y + h_R[6]*z;
	(*f2) = h_R[1]*x + h_R[4]*y + h_R[7]*z;
	(*f3) = h_R[2]*x + h_R[5]*y + h_R[8]*z;
	
}
/*********************/




/************************************/
/*	Compute the values for the model	*/
void Modeling::model(		int nb, 
							float *h_face, 
							float *h_R, 
							float *h_t, 
							bool *wrongRegistration,
							int FRAME_NUM) {
	
	updateModel(h_face, d_face, nb,
		h_R, d_R, h_t, d_t,
		d_face_C, d_face_I, d_dtdy,
		h_model, d_model, 
		h_overlap, d_overlap,
		h_defined, d_defined,
		h_dData, d_dData,
		d_meann, wrongRegistration,
		FRAME_NUM);
}
/*********************/










/************************************/
/*	Save the model to the file name	*/
int Modeling::createRawMeshes( int *hash, bool *defined ) {
	int i1=0, i2=0, i3=0, i4=0;
	int i_first = THETA_MAX+1;
	int i_last = (THETA_MAX-2)*Y_MAX-2;

	int i5=0,i6=0,i7=0,i8=0,i9=0;

	nb_mesh=0;

	for (int i1=i_first; i1<i_last; i1++) {
		//x = i1%THETA_MAX;
		//y = (i1 - x)/THETA_MAX;
		if ( defined[i1] ) {
			i2 = i1+1;				// x+1, y
			i3 = i1+THETA_MAX;		// x, y+1
			i4 = i3+1;				// x+1, y+1

			i5 = i1-1;
			i6 = i1-THETA_MAX;
			i7 = i6-1;
			i8 = i6+1;
			i9 = i3-1;
			if (defined[i5] && defined[i6] && defined[i7] && defined[i8] && defined[i9]){
				if ( defined[i2]	&&	defined[i4] ) {
					mesh_model[0][nb_mesh]		= hash[i1];
					mesh_model[1][nb_mesh]		= hash[i2];//hash[i4];
					mesh_model[2][nb_mesh++]	= hash[i4];//hash[i2];
				}

				if ( defined[i3]	&&	defined[i4] ) {
					mesh_model[0][nb_mesh]		= hash[i1];
					mesh_model[1][nb_mesh]		= hash[i4];//hash[i3];
					mesh_model[2][nb_mesh++]	= hash[i3];//hash[i4];
				}
			}
		}
	}

	return nb_mesh;

}
/*********************/



/*****************/
//void Modeling::bilateralFiltering(int w, float sigma_d, float sigma_r) {
	//// Bilateral filter on the depth information
	//IplImage *d = cvCreateImage(cvSize(THETA_MAX, Y_MAX), IPL_DEPTH_32F, 1 );
	//IplImage *tmp = cvCreateImage(cvSize(THETA_MAX, Y_MAX), IPL_DEPTH_32F, 1 );
	//float *tmpData = (float *)tmp->imageData;
	//float *dData	= (float *)d->imageData;


	//for (int i=0; i<MAX_IMG_INDEX_1; i++) {
	//	tmpData[i] = h_model[i];
	//}


	//// Perform a bilateral filtering
	//cvSmooth( tmp, d, CV_BILATERAL, w, w, sigma_d, sigma_r );
	//cvSmooth( d, tmp, CV_BILATERAL, w, w, sigma_d, sigma_r );
	//cvSmooth( tmp, d, CV_BILATERAL, w, w, sigma_d, sigma_r );

	//for (int i=0; i<MAX_IMG_INDEX_1; i++) {
	//	if ( abs(h_model[i]-dData[i]) < SMOOTHING_T)
	//		h_model[i] = dData[i];
	//}
	//
	//cvReleaseImage(&tmp);
	//cvReleaseImage(&d);

//}
/*****************/

/*****************/
void Modeling::bilateralFiltering(int w, float sigma_d, float sigma_r) {
	
	int w2=w+4;
	
	// Bilateral filter on the depth information
	IplImage *d = cvCreateImage(cvSize(THETA_MAX, Y_MAX), IPL_DEPTH_32F, 1 );
	IplImage *tmp = cvCreateImage(cvSize(THETA_MAX, Y_MAX), IPL_DEPTH_32F, 1 );
	float *tmpData = (float *)tmp->imageData;
	float *dData	= (float *)d->imageData;

	IplImage *d2 = cvCreateImage(cvSize(THETA_MAX, Y_MAX), IPL_DEPTH_32F, 1 );
	float *d2Data	= (float *)d2->imageData;


	for (int i=0; i<MAX_IMG_INDEX_1; i++) {
		tmpData[i] = h_model[i];
	}


	// Perform a bilateral filtering
	cvSmooth( tmp, d, CV_BILATERAL, w, w, sigma_d, sigma_r );
	cvSmooth( d, tmp, CV_BILATERAL, w, w, sigma_d, sigma_r );
	cvSmooth( tmp, d, CV_BILATERAL, w, w, sigma_d, sigma_r );

	for (int i=0; i<MAX_IMG_INDEX_1; i++) {
		tmpData[i] = h_model[i];
	}

	// Perform a bilateral filtering
	cvSmooth( tmp, d2, CV_BILATERAL, w2, w2, sigma_d, sigma_r );
	cvSmooth( d2, tmp, CV_BILATERAL, w2, w2, sigma_d, sigma_r );
	cvSmooth( tmp, d2, CV_BILATERAL, w2, w2, sigma_d, sigma_r );



	for (int i=0; i<MAX_IMG_INDEX_1; i++) {
		int x=i%THETA_MAX;
		if ( abs(h_model[i]-dData[i]) < SMOOTHING_T) {
			if (x<THETA_L || x>THETA_R)
				h_model[i] = d2Data[i];
			else
				h_model[i] = dData[i];
		}
	}
	
	cvReleaseImage(&tmp);
	cvReleaseImage(&d);
	cvReleaseImage(&d2);
}
/*****************/




/*************************************/
/* Horizontal circular interpolation */
//TODO should be 2D interpolation
void Modeling::circularInterpolation( ) {
	int indx[THETA_MAX];	
	int nb_per_line=0;
	int i=0,i_d=0;
	int t_i=0, t_ip1=0;


	// Find the last Y on which interpolate...
	int Y_end = Y_MAX;	
	int beg = MAX_IMG_INDEX_1-THETA_MID;
	int end = THETA_MID;
	for (int j=beg; j>end; j-=THETA_MAX) {
		if (h_model[j] != 0.0f) {
			Y_end = j/THETA_MAX;
			break;
		}
	}

	for (int t=0; t<THETA_MAX; t++)
		indx[t]=0;

		
	for (int y=0; y<=Y_end; y++) {	
		
		// Initialize nb_per_line[y]
		nb_per_line=0;
		i=y*THETA_MAX;		


		//h_model[i] = terribleInterpolationOnFirst(y);


		// Compute nb_per_line[y]
		for (int t=0; t<THETA_MAX; t++) {			
			if(h_model[i++]!=0.0f) {
				indx[nb_per_line++]=(i-1);
			}
		}

		if (nb_per_line>2) {
			// Interpolate
			for (int n=0; n<(nb_per_line-1); n++) {
				t_i		= indx[n];
				t_ip1	= indx[n+1];
				// Reinitialize i
				i = t_i+1;
				i_d = 3*i;

				if ((t_ip1-t_i)>1) {
					for (int t=t_i+1; t<t_ip1; t++) {
						h_meann[i]++;
						h_model[i++] = ((float)(t-t_i)*h_model[t_ip1]+(float)(t_ip1-t)*h_model[t_i])/(float)(t_ip1-t_i);						
						//h_dData[i_d++] = (char)(255.0f*h_model[i-1]+0.5f);
						//h_dData[i_d++] = h_dData[i_d-1];
						//h_dData[i_d++] = h_dData[i_d-1];
						if (h_model[i-1] == 0.0f) printf("%f\n", h_model[i-1]);
					}
				}
			}
		}
	}

}
/*****************/





/*************************************/
/* Horizontal circular interpolation */
//TODO should be 2D interpolation
void Modeling::completeInterpolation( ) {
	int indx[2]={0,0};	
	int i=0, i2=0, i_d=0;
	int t_0=0, t_1=0; 
	bool found=false;
		
	for (int y=0; y<Y_MAX; y++) {	
		found=false;

		// Initialize nb_per_line[y]
		i=y*THETA_MAX;
		i2=i+THETA_MAX-1;		

		// Compute nb_per_line[y]
		for (int t=0; t<THETA_MAX; t++) {			
			if(h_model[i++]!=0.0f) {
				indx[0]=(i-1);
				found=true;
				break;
			}
		}

		// Compute nb_per_line[y]
		for (int t=THETA_MAX-1; t>=0; t--) {			
			if(h_model[i2--]!=0.0f) {
				indx[1]=(i2+1);
				found=true;
				break;
			}
		}

		if (found) {
			t_0		= indx[0];
			t_1		= indx[1];

			// Reinitialize index
			i		= y*THETA_MAX; 
			i_d		= 3*i;
			// Complete the left part
			for (int t=y*THETA_MAX; t<indx[0]; t++) {
				h_model[i++] = ((float)(t_0-t)*h_model[t_1]+(float)(t-t_1+THETA_MAX)*h_model[t_0])/(float)(t_0-t_1+THETA_MAX);						
				//dData[i_d++] = (uchar)(255.0f*h_model[i-1]+0.5f);
				//dData[i_d++] = dData[i_d-1];
				//dData[i_d++] = dData[i_d-1];
			}

			// Reinitialize index
			i		= (y+1)*THETA_MAX-1; 
			i_d		= 3*i+2;
			// Complete the right part
			for (int t=(y+1)*THETA_MAX-1; t>indx[1]; t--) {
				h_model[i--] = ((float)(t_0-t+THETA_MAX)*h_model[t_1]+(float)(t-t_1)*h_model[t_0])/(float)(t_0-t_1+THETA_MAX);						
				//dData[i_d--] = (uchar)(255.0f*h_model[i+1]+0.5f);
				//dData[i_d--] = dData[i_d+1];
				//dData[i_d--] = dData[i_d+1];
			}


			

		}
	}
}
/*****************/






/****************************/
/* Create a skin color mask */
bool Modeling::isSkinColor(IplImage *image, int i) {
	// mask should have one channel
	const int CBMIN=77; 	// Skin range
	const int CBMAX=127;
	const int CRMIN=133; 
	const int CRMAX=173;	
	const int MIXMIN=190;
	const int MIXMAX=215;
	int Y, Cr, Cb;	// YCrCb values
	int R, G, B; 	// RGB values -note that openCv's order is BGR
	int mix;

	uchar *data = (uchar *)image->imageData;
	int size = image->imageSize-2;
	
	if (i<size) {
		/* Get RGB values */
		B = data[ i ];
		G = data[ i + 1 ];
		R = data[ i + 2 ];				
		/* Convert image to YCrCb: formulas from wikipedia	*/		
		Y = (int)(0.299*(float)R + 0.587*(float)G + 0.114*(float)B + 0.5);
		Cb = (int)(-0.1687*(float)R - 0.3313*(float)G + 0.5*(float)B + 128 + 0.5);
		Cr = (int)(0.5*(float)R - 0.4187*(float)G - 0.0813*(float)B + 128 + 0.5);
		mix = (int)((float)Cb + 0.6*(float)Cr + 0.5);			

		if(Cr>CRMIN && Cr<CRMAX && Cb>CBMIN && Cb<CBMAX && mix<MIXMAX && mix>MIXMIN) 
			return true;
		else
			return false;
	} else
		return false;
}
/**********************/







// TODO: IN GPU ?????
void Modeling::transformIn3D(float *Mp, float *MNp, float *h_R, float *h_t) {
	int ct0=0,
		ct1=MAX_IMG_INDEX_1,
		ct2=2*MAX_IMG_INDEX_1;

	float	xx=0.0f, yy=0.0f, zz=0.0,
			xn=0.0f, yn=0.0f, zn=0.0f;
	float ro=0.0f, theta=0.0f;
	int col=0;

	for (int i=0; i<MAX_IMG_INDEX_1; i++)  {
		// Find ro/theta

		ro = h_model[i];
		if (ro!=0.0f){
			col = (i%THETA_MAX)/(THETA_MAX/360);
			
			if (theta<(THETA_MAX-SHIFT_T))
				theta = (float)(col+SHIFT_T) * DEG2RAD;
			else
				theta = (float)(col+SHIFT_T-THETA_MAX) * DEG2RAD;
			
			// Find the cartesian coordinate
			yy = -( ((float)(i-(int)(col+0.5f))/(float)THETA_MAX)/(Y_MAX/2)-1.0f );
			xx = ro * cos(theta);
			zz = ro * sin(theta) + CYLINDER_Z;

			//xn = normalInfo[i];
			//yn = normalInfo[i+MAX_IMG_INDEX_1];
			//zn = normalInfo[i+MAX_IMG_INDEX_2];

			Mp[ct0] = h_R[0]*xx + h_R[1]*yy + h_R[2]*zz + h_t[0];
			Mp[ct1] = h_R[3]*xx + h_R[4]*yy + h_R[5]*zz + h_t[1];
			Mp[ct2] = h_R[6]*xx + h_R[7]*yy + h_R[8]*zz + h_t[2];

			MNp[ct0] = h_R[0]*xn + h_R[1]*yn + h_R[2]*zn;
			MNp[ct1] = h_R[3]*xn + h_R[4]*yn + h_R[5]*zn;
			MNp[ct2] = h_R[6]*xn + h_R[7]*yn + h_R[8]*zn;
		}
		ct0++;
		ct1++;
		ct2++;
	}
}
/**********************/






void Modeling::FindCylindricalIndex(float x, float y, float z, float *i_tc, float *i_yc){

	//x = -x;
	//z = -z;

	float X=-x, Y=y, Z=-z;
	// GOOD : Y=y

	float theta=0.0f;

	// Polar coordinates, theta in [0,360[ deg
	if (X<0.0f)
		theta = (atan(Z/X)+PI)*RAD2DEG;
	if (X>0.0f) {
		if (Z>=0.0f)
			theta = atan(Z/X)*RAD2DEG;
		else
			theta = (atan(Z/X)+2*PI)*RAD2DEG;
	}
	if (X==0.0f) {
		if (Z>0.0f)
			theta = 90.0f;
		if (Z<0.0f)
			theta = 270.0f;
		if (Z==0.0f)
			theta = 180.0f;
	}

	if (theta>THETA_MAX)
		theta -= THETA_MAX;
	if (theta<0)
		theta += THETA_MAX;

	// Find the row/column on the image
	*i_tc = ((float)THETA_EXPAND*theta-(float)SHIFT_T);
	*i_yc = ((float)Y_EXPAND*(-Y+1.0f));

	// Shift to put the nose in the middle
	if (theta<=SHIFT_T)
		*i_tc += (float)THETA_MAX;		

	if (*i_yc >= (float)Y_MAX)
		*i_yc = (float)Y_MAX;
	if (*i_yc < 0.0f)
		*i_yc = 0.0f;
	if (*i_tc >= (float)THETA_MAX)
		*i_tc -= (float)THETA_MAX;
	if (*i_tc < 0.0f)
		*i_tc += (float)THETA_MAX;

	//// Find the index
	//(*index)	=	i_yc*THETA_MAX+ i_tc;

	//if (*index >= MAX_IMG_INDEX_1)
	//	*index = MAX_IMG_INDEX_1-1;
	//if (*index <0)
	//	*index = 0;
}




/********************************/
/*	Save the MTL file for OBJ	*/
void Modeling::saveMTL(	char *name ) {
	char mtlname[200];
	sprintf(mtlname, "%s.mtl", name);
	FILE *mtl = fopen(mtlname, "w");
	fprintf(mtl, "newmtl aaa\nKa 0.200000 0.200000 0.200000\nKd 0.80000 0.80000 0.80000\nKs 1.000000 1.000000 1.000000\nTr 1.0\nillum 2\nNs 0.000000\n\n");
	fclose(mtl);	
}
/**********************/







/************************************/
///*	Save the model to the file name	*/
void Modeling::saveToOBJFile(	char *name, 
								bool withMesh, 
								float normalize, 
								float *h_pReal,	
								float *d_gaussian,
								float *mConfidence) {
	const int	smooth_window=BILATERAL_WINDOW;
	const float smooth_sigma_s=30.0f,
					smooth_sigma_c=30.0f;

	// TODO: SMOOTH DIFFERENTLY IN FONCTION OF THE REGION	
	FILE  *f = fopen(name, "w");
	float xx=0.0f, yy=0.0f, zz=0.0f,
			xR=0.0f,	yR=0.0f, zR=0.0f;
	float R=0.0f, G=0.0f, B=0.0f,
			ro=0.0f, theta=0.0f;
	int col=0;
	//int nb_mesh=0, nb_vertices=0;
	uchar *data = NULL;		// Image used for the texture
	Voxel c;						// Voxel
	int	ixi=0, iyi=0;									// Integer part of the coordinates
	float dx=0.0f,	dy=0.0f;								// Decimal part of the coordinates
	int	iii00=0, iii10=0, iii01=0, iii11=0;		// Corresponding index
	float w00=0.0f, w01=0.0f, w10=0.0f, w11=0.0f;// Weight of the texture on each pixel
		
	//bool *defined = new bool[MAX_IMG_INDEX_1];// Pixels on the unwrapped cylindrical map that should be saved
	//int hash[MAX_IMG_INDEX_1];						// Index of the pixels that should be saved 
	//for (int i=0; i<MAX_IMG_INDEX_1; i++) {
	//	defined[i] = false;
	//	hash[i] = -1;
	//}

	

	// SimpleClosing...
	//simpleClosing( 5 );
	//circularInterpolation( );
	//verticalInterpolation( );
	//completeInterpolation( );

	/////////////////////////////////////////////////////////////////
	//// Find the number of vertices and set the hash map
	//for (int i=0; i<MAX_IMG_INDEX_1; i++)  {
	//	// Find ro/theta
	//	col = (i%THETA_MAX)/(THETA_MAX/360);
	//	ro = h_model[i];
	//	if (theta<(THETA_MAX-SHIFT_T))
	//		theta = (float)(col+SHIFT_T) * DEG2RAD;
	//	else
	//		theta = (float)(col+SHIFT_T-THETA_MAX) * DEG2RAD;
	//	
	//	// Find the cartesian coordinate
	//	yy = -( ((float)(i-(int)(col+0.5f))/(float)THETA_MAX)/(Y_MAX/2)-1.0f );
	//	xx = ro * cos(theta);
	//	zz = ro * sin(theta);

	//	// Use only the frontal image
	//	c = cFront;
	//	
	//	xR = xx*normalize+c.getXr();
	//	yR = yy*normalize+c.getYr();
	//	zR = (zz+CYLINDER_Z)*normalize+c.getZr();

	//	
	//	int row = (i-(int)(col+0.5f))/(int)THETA_MAX;
	//	
	//	// Save only the pixels that are non 0
	//	//if (h_model[i]!=0.0f && iyi <= (chin_first+CHIN_ADD)) {
	//		// Save a ROI around the middle to remove the noise around
	//		//if ( ((col-THETA_MID)*(col-THETA_MID) + (row-Y_MID)*(row-Y_MID))<=RADIUS_CUT_2 ) {			
	//		//	defined[i] = true;
	//		//	hash[i] = nb_vertices++;
	//		//}
	//	//}

	//	// Save only the pixels that are non 0
	//	if (h_model[i]!=0.0f) {
	//		defined[i] = true;
	//		hash[i] = nb_vertices++;			
	//	}
	//}
	//
	//// Find the mesh
	//if (withMesh)
	//	nb_mesh = createRawMeshes( hash, defined );

	//// Apply Bilateral filtering
	////bilateralFiltering(BILATERAL_WINDOW,smooth_sigma_s,smooth_sigma_c);
	//bilateralFilterRGBA(h_model, d_model, h_model, d_model, d_gaussian, d_defined_C, THETA_MAX, Y_MAX, smooth_sigma_s, smooth_sigma_c, BILATERAL_WINDOW,3,1);

	////////////////////////////////////////////////////////////////
	////	Create the mtl file
	//saveMTL(name);


	/////////////////////////////////////////////////////////////
	// Save in OBJ format
	//fprintf(f, "mtllib ./myModel.obj.mtl\nusemtl aaa\n\n");
	fprintf(f, "# normalization factor %f\n", normalize);
	fprintf(f, "# %d vertices\n\n", nb_vertices);	

	/////////////////////////////////////////////////////////////
	// Save in TXT format
	//FILE *txtfile = fopen("./model/myModel.txt", "w");

	//fprintf(txtfile, "%d\n", nb_vertices);


	// Save the vertices
	for (int i=0; i<MAX_IMG_INDEX_1; i++) {
		if (h_defined_C[i]) {
			// Find the cylindric coordinate ro/theta
			ro = h_model[i];
			col = (i%THETA_MAX)/(THETA_MAX/360);
			
			if (theta<(THETA_MAX-SHIFT_T))
				theta = (float)(col+SHIFT_T) * DEG2RAD;
			else
				theta = (float)(col+SHIFT_T-THETA_MAX) * DEG2RAD;
			
			// Find the cartesian coordinate
			yy = -(((float)(i-(int)(col+0.5f))/(float)THETA_MAX)/(Y_MAX/2)-1.0f);
			xx = ro * cos(theta);
			zz = ro * sin(theta);

			// Use only the frontal image
			c = cFront;
			data = (uchar *)h_iData;

			// Come back to real coordinate 
			xR = xx*normalize+c.getXr();
			yR = yy*normalize+c.getYr();
			zR = (zz+CYLINDER_Z)*normalize+c.getZr();

			R = (mConfidence[i]);
			G = 0.0f;
			B = 0.0f;

			// Save into the file
			fprintf(f, "v %f %f %f %f %f %f\n", -xx, yy, -zz, R, G, B);		
			//fprintf(txtfile, "%f %f %f\n", -xx*50.0f, yy*50.0f, -zz*50.0f);	
			
			h_pReal[i] = -xx;
			h_pReal[i+MAX_IMG_INDEX_1] = yy;
			h_pReal[i+MAX_IMG_INDEX_2] = -zz;
		}
	}

	// Save the meshes
	if (withMesh) {
		fprintf(f, "\n# %d meshes\n\n", nb_mesh);		
		for (int i=0; i<nb_mesh; i++) {
			fprintf(f, "f %d %d %d\n",	mesh_model[0][i], 
										mesh_model[1][i], 
										mesh_model[2][i]);
		}
	}

	// Release the memory
	//delete[] defined;
	fclose(f);
	//fclose(txtfile);

}
/*********************/




/************************************/
///*	Save the model to the file name	*/
void Modeling::saveToOBJFile(	char *name, 
								bool withMesh, 
								float normalize ) {
	const int	smooth_window=BILATERAL_WINDOW;
				//smooth_iterations=1,
				//smooth_nthreads=1;
	const float smooth_sigma_s=30.0f,
				smooth_sigma_c=30.0f;
	
	//int nb_mesh=0, nb_vertices=0;
	
	int hash[MAX_IMG_INDEX_1];	
	for (int i=0; i<MAX_IMG_INDEX_1; i++) 
		hash[i] = -1;

	FILE  *f = fopen(name, "w");
	string namePly(name);
	FILE *fPly = fopen((namePly.substr(0, namePly.length() - 3).append("ply")).c_str(), "w");
	char s[200];
	sprintf(s, "%s.s.obj", name); 
	FILE  *fs = fopen(s, "w");
	
	// Index of the pixels that should be saved 
	copyCUDAToHost(model, MAX_IMG_INDEX_1);	

	/////////////////////////////////////////////////////////////
	// Circular interpolation
	circularInterpolation( );

	/////////////////////////////////////////////////////////////////
	// Find the number of vertices and set the hash map
	nb_vertices=0;
	for (int i=0; i<MAX_IMG_INDEX_1; i++)  {
		// Save only the pixels that are non 0
		if (h_model[i]!=0.0f /*&& fabs(h_model[i])<RO_MAX &&i<68400*/) {
			h_defined_C[i] = true;
			hash[i] = nb_vertices++;			
		} else
			h_defined_C[i] = false;
	}

	completeInterpolation();	// Used to avoid edge blurring
	

	/////////////////////////////////////////////////////////////
	// Apply Bilateral filtering
	bilateralFiltering(smooth_window,smooth_sigma_s,smooth_sigma_c);
	//copyHostToCUDA_bool(defined_C,MAX_IMG_INDEX_1);
	//float *d_tmp, *h_tmp;
	//h_tmp = new float[MAX_IMG_INDEX_1];
	//mallocCUDA(tmp,MAX_IMG_INDEX_1);
	//initGaussian(d_gaussian, smooth_sigma_c, smooth_window+1);
	//bilateralFilter(h_model, d_model, h_tmp, d_tmp, d_gaussian, d_defined_C, THETA_MAX, Y_MAX, smooth_sigma_s, smooth_sigma_c, smooth_window,smooth_iterations,smooth_nthreads);
	//delete[] h_tmp;
	//releaseCUDA(tmp);

	//////////////////////////////////////////////////////////////
	//	Create the mtl file
	saveMTL(name);


	/////////////////////////////////////////////////////////////
	////// Save in OBJ format
	fprintf(f, "mtllib ./myModel.obj.mtl\nusemtl aaa\n\n");
	fprintf(f, "# normalization factor %f\n", normalize);
	fprintf(f, "# %d vertices\n\n", nb_vertices);

	fprintf(fs, "# normalization factor %f\n", normalize);
	fprintf(fs, "# %d vertices\n\n", nb_vertices);



	/////////////////////////////////////////////////////////////
	// Convert to XYZ and RGB
	convertToXYZRGB(	h_model, d_model, 
						h_modelXYZ, d_modelXYZ, 
						h_modelRGB, d_modelRGB,
						h_iData, d_iData,
						h_defined_C, d_defined_C,
						cFront, normalize);

	///////////////////////////////////////////////////////////////
	////// Save the points
	for (int i=0; i<MAX_IMG_INDEX_1; i++) {
		if (h_defined_C[i]/* && h_model[i]<RO_MAX*/) {
			fprintf(f, "v %f %f %f\n", h_modelXYZ[i] * normalize, h_modelXYZ[i + MAX_IMG_INDEX_1] * normalize, h_modelXYZ[i + MAX_IMG_INDEX_2] * normalize);

			fprintf(fs, "v %f %f %f\n", h_modelXYZ[i], h_modelXYZ[i + MAX_IMG_INDEX_1], h_modelXYZ[i + MAX_IMG_INDEX_2]);

		//	fprintf(f, "v %f %f %f %f %f %f\n", h_modelXYZ[i]*normalize, h_modelXYZ[i+MAX_IMG_INDEX_1]*normalize, h_modelXYZ[i+MAX_IMG_INDEX_2]*normalize, 
			//									h_modelRGB[i*3], h_modelRGB[i*3+1], h_modelRGB[i*3+2]);		

		//	fprintf(fs, "v %f %f %f %f %f %f\n", h_modelXYZ[i], h_modelXYZ[i+MAX_IMG_INDEX_1], h_modelXYZ[i+MAX_IMG_INDEX_2], 
			//									h_modelRGB[i*3], h_modelRGB[i*3+1], h_modelRGB[i*3+2]);		
												//h_modelRGB[i], h_modelRGB[i+MAX_IMG_INDEX_1], h_modelRGB[i+MAX_IMG_INDEX_2]);		
		}
	}

	/////////////////////////////////////////////////////////////
	// Save the mesh
	if (withMesh) {
		/////////////////////////////////////////////////////////////
		// Find the mesh
		nb_mesh = createRawMeshes( hash, h_defined_C );

		///////////////////////////////////////////////////////////
		//// Save the mesh
		fprintf(f, "\n# %d meshes\n\n", nb_mesh);		
		for (int i=0; i<nb_mesh; i++) {
			fprintf(f, "f %d %d %d\n",	mesh_model[0][i], 
										mesh_model[1][i], 
										mesh_model[2][i]);

			fprintf(fs, "f %d %d %d\n",	mesh_model[0][i], 
										mesh_model[1][i], 
										mesh_model[2][i]);
		}
	}

	//save in ply format
	fprintf(fPly, "ply\n");
	fprintf(fPly, "format ascii 1.0\n");
	fprintf(fPly, "comment VCGLIB generated\n");
	fprintf(fPly, "element vertex %d\n", nb_vertices);
	fprintf(fPly, "property float x\n");
	fprintf(fPly, "property float y\n");
	fprintf(fPly, "property float z\n");
	fprintf(fPly, "element face %d\n", nb_mesh);
	fprintf(fPly, "property list uchar int vertex_indices\n");
	fprintf(fPly, "end_header\n");

	for (int i = 0; i<MAX_IMG_INDEX_1; i++) {
		if (h_defined_C[i]/* && h_model[i]<RO_MAX*/) {
			fprintf(fPly, "%f %f %f\n", h_modelXYZ[i] * normalize, h_modelXYZ[i + MAX_IMG_INDEX_1] * normalize, h_modelXYZ[i + MAX_IMG_INDEX_2] * normalize);
		}
	}
	for (int i = 0; i<nb_mesh; i++) {
		fprintf(fPly, "3 %d %d %d\n", mesh_model[0][i],
			mesh_model[1][i],
			mesh_model[2][i]);
	}

	fclose(f);
	fclose(fs);
	fclose(fPly);
}
/*********************/