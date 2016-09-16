#include "stdafx.h"
#include "Voxel.h"

/****************/
/* Constructors */
/****************/

Voxel::Voxel() {
	this->Xr = 0.0f;
	this->Yr = 0.0f;
	this->Zr = 0.0f;
	this->ro = 0.0f;
	this->theta = 0.0f;
	this->i_c = 0;
	this->W  = 1.0f;
};

Voxel::Voxel(float X, float Y, float Z) {
	this->Xr = X;
	this->Yr = Y;
	this->Zr = (Z-CYLINDER_Z);
	this->W  = 1.0f;

	this->ro = pow( (X*X+Z*Z), 0.5f);		
	if (X<0.0f)
		this->theta = (atan(Z/X)+PI)*RAD2DEG;
	if (X>0.0f) {
		if (Z>=0.0f)
			this->theta = atan(Z/X)*RAD2DEG;
		else
			this->theta = (atan(Z/X)+2*PI)*RAD2DEG;
	}
	if (X==0.0f) {
		if (Z>0.0f)
			this->theta = 90.0f;
		if (Z<0.0f)
			this->theta = 270.0f;
		if (Z==0.0f)
			this->theta = 180.0f;
	}

	// Find the row/column on the image
	int i_tc = (int)((float)THETA_EXPAND*(this->theta)-(float)SHIFT_T+0.5f);
	int i_yc = (int)(Y_EXPAND*(-Y+1.0f)+0.5f);

	// Shift to put the nose in the middle
	if ((this->theta)<=SHIFT_T)
		i_tc += THETA_MAX;		

	// Find the index
	this->i_c	=	i_yc*THETA_MAX + i_tc;
}

/************************/
/* Getters and setters */
/***********************/

void Voxel::setXr(float Xr) {
	this->Xr = Xr;	
}
void Voxel::setYr(float Yr) {
	this->Yr = Yr;	
}
void Voxel::setZr(float Zr) {
	this->Zr = Zr;	
}
void Voxel::setXYZ(float Xr, float Yr, float Zr) {
	this->Xr = Xr;
	this->Yr = Yr;
	this->Zr = Zr;
}
void Voxel::setW(float W) {
	this->W = W;	
}
void Voxel::setRo(float ro) {
	this->ro = ro;	
}
void Voxel::setTheta(float theta) {
	this->theta = theta;	
}
void Voxel::setI_c(int i_c) {
	this->i_c = i_c;	
}

float Voxel::getXr() {
	return Xr;	
}
float Voxel::getYr() {
	return Yr;	
}
float Voxel::getZr() {
	return Zr;	
}
float Voxel::getW() {
	return W;	
}
float Voxel::getRo() {
	return ro;	
}
float Voxel::getTheta() {
	return theta;	
}
int Voxel::getI_c() {
	return i_c;	
}



/**************************/
/* Methods implementation */
/**************************/

void Voxel::calculateCylindricalCoordinate() {
	this->ro = pow( (Xr*Xr+Zr*Zr), 0.5f);		
	if (Xr<0.0f)
		this->theta = (atan(Zr/Xr)+PI)*RAD2DEG;
	if (Xr>0.0f) {
		if (Zr>=0.0f)
			this->theta = atan(Zr/Xr)*RAD2DEG;
		else
			this->theta = (atan(Zr/Xr)+2*PI)*RAD2DEG;
	}
	if (Xr==0.0f) {
		if (Zr>0.0f)
			this->theta = 90.0f;
		if (Zr<0.0f)
			this->theta = 270.0f;
		if (Zr==0.0f)
			this->theta = 180.0f;
	}

	// Find the row/column on the image
	int i_tc = (int)((float)THETA_EXPAND*(this->theta)-(float)SHIFT_T+0.5f);
	int i_yc = (int)(Y_EXPAND*(-Yr+1.0f)+0.5f);

	// Shift to put the nose in the middle
	if ((this->theta)<=SHIFT_T)
		i_tc += THETA_MAX;		

	// Find the index
	this->i_c	=	i_yc*THETA_MAX + i_tc;		
}

void Voxel::merge(Voxel v2){
	float	X1 = Xr,
			Y1 = Yr,
			Z1 = Zr,
			W1 = W,
			X2 = v2.getXr(),
			Y2 = v2.getYr(),
			Z2 = v2.getZr(),
			W2 = v2.getW();

	float	WW = W1 + W2;

	Xr = (X1*W1 + X2*W2)/WW,
	Yr = (Y1*W1 + Y2*W2)/WW,
	Zr = (Z1*W1 + Z2*W2)/WW,
	W  = W1+1.0f;
}

float Voxel::d2(Voxel v2){
	float	X1 = Xr,
			Y1 = Yr,
			Z1 = Zr,
			X2 = v2.getXr(),
			Y2 = v2.getYr(),
			Z2 = v2.getZr();

	return (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) + (Z2-Z1)*(Z2-Z1) ;
}

float Voxel::d2(Voxel v1, Voxel v2){
	float	X1 = v1.getXr(),
			Y1 = v1.getYr(),
			Z1 = v1.getZr(),
			X2 = v2.getXr(),
			Y2 = v2.getYr(),
			Z2 = v2.getZr();

	return (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) + (Z2-Z1)*(Z2-Z1) ;
}