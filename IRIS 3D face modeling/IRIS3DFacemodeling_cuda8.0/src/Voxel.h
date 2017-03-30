//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.


#ifndef VOXEL_H
#define VOXEL_H

class Voxel {
 	private: 	
		float Xr;	//	X value
		float Yr;	//	Y value 
		float Zr;	//	Z value
		float ro;	// Ro value in cylindrical
		float theta;// Theta value in cylindrical
		int i_c;	// Index in cylindrical
		float W;	// Mean weigth

 	public:
 		/* Constructor */
 		Voxel();
		Voxel(float X, float Y, float Z);

 		/* Getters and setters for all the members */
		void setXr(float Xr);
		void setYr(float Yr);
		void setZr(float Zr);
		void setRo(float ro);
		void setTheta(float theta);
		void setI_c(int i_c);
		void setW(float W);
		void setXYZ(float Xr, float Yr, float Zr);

		float getXr();
		float getYr();
		float getZr();
		float getRo();
		float getTheta();
		int getI_c();
		float getW();


		/* Methods */ 
		void calculateCylindricalCoordinate();
		void merge(Voxel v2);
		float d2(Voxel v2);
		static float d2(Voxel v1, Voxel v2);
};

#endif
