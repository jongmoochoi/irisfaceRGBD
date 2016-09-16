//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

#include "stdafx.h"

#ifndef FACEDETECTOR_H
#define FACEDETECTOR_H

#define THRESHOLD_YZ 20
#define THRESHOLD_XY 30
#define THRESHOLD_DIFF 15
#define THRESHOLD_DEPTH 5
#define THRESHOLD_DISCONTINUITY 5
#define SEARCHING_BOX_HEIGHT 20



class FaceDetector {
 	private:
		FaceBox faceBox;			// Current detected face
		FaceBox previousFaceBox;	// Previous detected face
		//IplImage * xOy;				// projection of the depth map on xOy

		//uchar *dataXY;	// variables to access the image's data easily
		//int stepXY; 
		//int channelsXY; 	
		int widthXY;
		int heightXY;
		bool *mask;
	
		// Private methods
		int findLeftMostPoint(int y0);
		int findRightMostPoint(int y0);
		int findSmallestDepth(int y0, float *h_pReal);
		int findMinDepth(float *h_pReal, int leftX, int rightX, int topY, int bottomY);
		void findMaxMinDepth(float *h_pReal, int leftX, int rightX, int topY, int bottomY);

		bool detectTop(float *h_pReal);
		bool detectChin(float *h_pReal, int y0, int height, int thresholdYZ, int thresholdXY, int thresholdDIFF, int thresholdDEPTH, int thresholdDISCONTINUITY,	int searching_box_height);
		bool detectLeftCheek(int topY, int bottomY, float *h_pReal);
		bool detectRightCheek(int topY, int bottomY, float *h_pReal);

		float tYr,bYr,lXr,rXr;
		

 	public:
 		/* Constructor */
 		FaceDetector();

 		/* Getters and setters for all the members */
 		void setFaceBox(FaceBox faceBox);
		void setPreviousFaceBox(FaceBox previousFaceBox);
		void setXOy(IplImage * xOy);
		void setZOy(IplImage * zOy);
		void setValidityMask(bool *mask);
		FaceBox getFaceBox();
		FaceBox getPreviousFaceBox();
		IplImage * getXOy();
		IplImage * getZOy();
		
		/* Methods */ 
		bool isInsideXY(int x, int y);
		bool detectFace(	float *h_pReal,
							int thresholdYZ = THRESHOLD_YZ, 
							int thresholdXY = THRESHOLD_XY, 
							int thresholdDIFF = THRESHOLD_DIFF,  
							int thresholdDEPTH = THRESHOLD_DEPTH, 
							int thresholdDISCONTINUITY = THRESHOLD_DISCONTINUITY,
							int searching_box_height = SEARCHING_BOX_HEIGHT);
		//void findNoseTip(float *h_pReal);
		bool IsFirst();
/*******************xxx**********************/
		//void findNoseTip(float *h_pReal);
/*******************xxx**********************/
};

#endif
