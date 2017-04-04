//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.


#include "stdafx.h"
#include "FaceBox.h"
#include "FaceDetector.h"


/****************/
/* Constructors */
/****************/

FaceDetector::FaceDetector() {
	this->faceBox.setBottomY(0);	// initialize face box
	this->faceBox.setTopY(0);
	this->faceBox.setLeftX(0);
	this->faceBox.setRightX(0);
	this->faceBox.setMinDepth(0);
	this->faceBox.setMaxDepth(0);

	this->previousFaceBox.setBottomY(0);	// initialize previous face box
	this->previousFaceBox.setTopY(0);
	this->previousFaceBox.setLeftX(0);
	this->previousFaceBox.setRightX(0);
	this->previousFaceBox.setMinDepth(0);
	this->previousFaceBox.setMaxDepth(0);

	this->mask=NULL;

	this->widthXY=XN_VGA_X_RES, 
	this->heightXY=XN_VGA_Y_RES;
};

/************************/
/* Getters and setters */
/***********************/

void FaceDetector::setFaceBox(FaceBox faceBox) {
	this->faceBox = faceBox;	
}
void FaceDetector::setPreviousFaceBox(FaceBox previousFaceBox) {
	this->previousFaceBox = previousFaceBox;	
}
void FaceDetector::setValidityMask(bool *mask) {
	this->mask = mask;	
}
FaceBox FaceDetector::getFaceBox() {
	return faceBox; 	
}
FaceBox FaceDetector::getPreviousFaceBox() {
	return previousFaceBox;	
}

/**************************/
/* Methods implementation */
/**************************/



/******************************************************/
/*	Says whether the point (x,y) is in the image xOy	*/
// Inputs: -(x,y) coordinate of the point
bool FaceDetector::isInsideXY(int x, int y){
	if (y>=0 && x>=0 && x<widthXY && y<heightXY)
		return true;
	return false;
}
/**********************/



/******************************************/
/*	Find the left-most point on xOy plan	*/
// Inputs: -y0 line on which we look for the left most point on xOy plan
int FaceDetector::findLeftMostPoint(int y0){
	int ind = y0*widthXY;
	for (int x=0; x<widthXY; x++) {
		if (isInsideXY(x,y0))
			if (mask[ind+x])
				return x;
	}
	return widthXY-1;
}
/**********************/



/******************************************/
/*	Find the right-most point on xOy plan	*/
// Inputs: -y0 line on which we look for the left most point on xOy plan
int FaceDetector::findRightMostPoint(int y0){
	int ind = y0*widthXY;
	for (int x=widthXY-1; x>=0; x--) {
		if (isInsideXY(x,y0))
			if (mask[ind+x])
				return x;
	}
	return 0;
}
/**********************/



/******************************************************************/
/* Find the left-most non-zero pixel for a height y0 on zOy plan	*/
// Inputs: -y0 line on which we look for the smallest depth in zOy
int FaceDetector::findSmallestDepth(int y0, float *h_pReal){	
	int sD = MAX_DEPTH;

	int begin = MAX(0,(int)lXr-LOOK_HEAD);
	int end = MIN(widthXY-1,(int)lXr+LOOK_HEAD);

	//for (int x=0; x<widthXY; x++) {
	int ind= MAX_I2+y0*XN_VGA_X_RES;

	for (int x=begin; x<end; x++) {
		int index= ind+x;
		if (index<MAX_I3){
			if (h_pReal[index]<sD && h_pReal[index]>=MIN_DEPTH) {
				sD = (int)(h_pReal[index]+0.5f);
			}
		}
	}

	return sD;
}
/**********************/




/**************************/
/* Find the minimum depth */
int FaceDetector::findMinDepth(	float *h_pReal, int leftX, int rightX, int topY, int bottomY) {
	const int MIN_ = MAX_I2+topY*XN_VGA_X_RES + leftX;
	const int MAX_ = MAX_I2+bottomY*XN_VGA_X_RES + rightX;
	const int W = rightX-leftX;
	const int STEP = XN_VGA_X_RES - W;
	
	int minDepth=MAX_DEPTH;
	int k=0;

	for (int i=MIN_; i<MAX_; i++){
		if (h_pReal[i] < minDepth && h_pReal[i] >= MIN_DEPTH)
				minDepth = (int) (h_pReal[i]+0.5f);
		if (++k == W) {
			k=0;
			i+=STEP;
		}
	}

	return minDepth;
}
/*************************/


/**************************/
/* Find the maximum depth */
// Inputs:	-pDepthMap contains the depth values for all the pixels
//					-(leftX, rightX, topY, bottomY)	window in which we look for the maximum depth
void FaceDetector::findMaxMinDepth(float *h_pReal, int leftX, int rightX, int topY, int bottomY) {
	const int MIN_ = MAX_I2+topY*XN_VGA_X_RES + leftX;
	const int MAX_ = MAX_I2+bottomY*XN_VGA_X_RES + rightX;
	const int W = rightX-leftX;
	const int STEP = XN_VGA_X_RES - W;
	
	int maxDepth=0, minDepth=MAX_DEPTH;
	int k=0;

	for (int i=MIN_; i<MAX_; i++){
		if (h_pReal[i] > maxDepth && h_pReal[i] <= MAX_DEPTH)
				maxDepth = (int)(h_pReal[i]+0.5f);
		if (h_pReal[i] < minDepth && h_pReal[i] >= MIN_DEPTH)
				minDepth = (int)(h_pReal[i]+0.5f);
		if (++k == W) {
			k=0;
			i+=STEP;
		}
	}

	faceBox.setMinDepth( minDepth );
	faceBox.setMaxDepth( maxDepth );
}
/*************************/



/**************************/
/*	Detect the top point	*/
bool FaceDetector::detectTop(float *h_pReal){
	int left, right, top, bottom;

	if (	previousFaceBox.getLeftX() == 0 && previousFaceBox.getRightX() == 0) {
		left = 0;
		right = widthXY;
		top = 0;
		bottom = heightXY-1;
	} else {
		left = previousFaceBox.getLeftX();
		right = previousFaceBox.getRightX();
		top = previousFaceBox.getTopY()-20;
		bottom = heightXY-1;
	}
	int index=0;

	for (int y=top;y<bottom;y++) {
		for (int x=left;x<right;x++) {
			if (isInsideXY(x,y)) {
				index = y*widthXY + x;
				if (mask[index])	{
					tYr = h_pReal[MAX_I+index];
					lXr = (float)x;
					faceBox.setTopY(y);
					return true;
				}
			}
		}
	}

	return false;
}
/**********************/



/********************/
/*	Detect the chin	*/
bool FaceDetector::detectChin(	float *h_pReal, 
											int y0, 
											int height, 
											int thresholdYZ, 
											int thresholdXY, 
											int thresholdDIFF, 
											int thresholdDEPTH, 
											int thresholdDISCONTINUITY,
											int searching_box_height) {
	

	// Generating a random number
	srand ( (int)time(NULL) );
	int p = rand() % 10 + 1;
	int threshold_p = 6;

	// First, try to find a discontinuity on yOz
	int min = findMinDepth(h_pReal, previousFaceBox.getLeftX(), previousFaceBox.getRightX(), previousFaceBox.getTopY()+previousFaceBox.getHeight()/2, previousFaceBox.getBottomY())-MIN_DEPTH+thresholdDEPTH;

	for (int y=y0;y<y0+height-3;y++) {
		if (findSmallestDepth(y+3, h_pReal) > findSmallestDepth(y, h_pReal)+thresholdYZ) 
			if ((previousFaceBox.getBottomY()==0	||	(y>previousFaceBox.getBottomY()-thresholdDIFF/*&& y<previous_chinY+thresholdDIFF*/)
																						||	(p<threshold_p)) )
				if 	(previousFaceBox.getBottomY()==0	|| (findSmallestDepth(y, h_pReal) > min) )	//Not a nose
					{
				faceBox.setBottomY(y);
				return true; 
			}

	}
	
	// Second, try to find a discontinuity on xOy
	for (int y=y0;y<y0+height;y++) { // looking left
		if (findLeftMostPoint(y+1) > findLeftMostPoint(y)+thresholdXY)	
			if (previousFaceBox.getBottomY()==0 ||	(y>previousFaceBox.getBottomY()-thresholdDIFF/*&& y<previous_chinY+thresholdDIFF*/)
														/*||	(p<threshold_p)*/){
				faceBox.setBottomY(y);
				return true; 
			}
	}	
	for (int y=y0;y<y0+height;y++) {	// looking right
		if (findRightMostPoint(y+1) < findRightMostPoint(y)-thresholdXY)
			if (previousFaceBox.getBottomY()==0 ||	(y>previousFaceBox.getBottomY()-thresholdDIFF/*&& y<previous_chinY+thresholdDIFF*/)
				/*||	(p<threshold_p)*/) {
				faceBox.setBottomY(y);
				return true; 
			}
	}

	// TRACKING
	int nb_discontinuities, nb_discontinuities_max=0, y_best=previousFaceBox.getBottomY();
	int ind=0;

	// Find the line with the highest number of discontinuities
	for(int y=previousFaceBox.getBottomY()-searching_box_height/2; y<previousFaceBox.getBottomY()+searching_box_height/2; y++) {
		nb_discontinuities = 0;
		ind = y*widthXY;
		for (int x=0; x<widthXY; x++) 
			if (isInsideXY(x, y) && isInsideXY(x, y+1))
				if (	mask[ind+x] 
				&&	mask[ind+widthXY+x]
				&&	h_pReal[MAX_I2+ind + x]<h_pReal[MAX_I2+ind+widthXY + x]+thresholdDISCONTINUITY)
					nb_discontinuities++;	
		if (nb_discontinuities > nb_discontinuities_max) {
			nb_discontinuities_max = nb_discontinuities;
			y_best = y;
		}
	}
	faceBox.setBottomY(y_best);
	return true;
}
/**********************/



/**************************/
/*	Detect the left cheek	*/
// Inputs:	-[topY, bottomY]	height range in which we look for the cheek
bool FaceDetector::detectLeftCheek(int topY, int bottomY, float *h_pReal) {
	int middleY = (bottomY+topY)/2;
	int boxY = (middleY-topY)/3; 
	int left, right;
	int searching_box_cheek = previousFaceBox.getWidth()/2;

	// Look in the neighborhood of the previous cheek
	if (previousFaceBox.getLeftX() == 0) {
		left = 0;
		right = widthXY-1;
	} else {
		left = previousFaceBox.getLeftX()-searching_box_cheek/2;
		right = previousFaceBox.getLeftX()+searching_box_cheek/4;
	}

	// Find the left cheek
	for (int x=left; x<right; x++) {
		for (int y=topY; y<middleY+boxY; y++) {
			if (isInsideXY(x, y))
				if (mask[y*widthXY+x]){
					faceBox.setLeftX(x);
					lXr = h_pReal[y*XN_VGA_X_RES+x];
					return true;
				}
		}
	}	

	// Switch to pure detection if tracking does not work
	left = 0;
	right = widthXY-1;
	for (int x=left; x<right; x++) {
		for (int y=topY; y<middleY+boxY; y++) {
			if (isInsideXY(x, y))
				if (mask[y*widthXY+x]) {
					faceBox.setLeftX(x);
					lXr = h_pReal[y*XN_VGA_X_RES+x];
					return true;
				}
		}
	}

	// If nothing is found anyway, return the previous position
	faceBox.setLeftX(previousFaceBox.getLeftX());
	return true;
}
/**********************/



/****************************/
/*	Detect the right cheek	*/
// Inputs:	-[topY, bottomY]	height range in which we look for the cheek
bool FaceDetector::detectRightCheek(int topY, int bottomY, float *h_pReal) {
	int middleY = (bottomY+topY)/2;
	int boxY = (middleY-topY)/3; 
	int left, right;
	int searching_box_cheek = previousFaceBox.getWidth()/2;

	// Look in the neighborhood of the previous cheek
	if (previousFaceBox.getRightX() == 0) {
		left = 0;
		right = widthXY-1;
	} else {
		left = previousFaceBox.getRightX()-searching_box_cheek/2;
		right = previousFaceBox.getRightX()+searching_box_cheek/4;
	}

	// Find the left cheek
	for (int x=right-1; x>=left; x--) {
		for (int y=topY; y<middleY+boxY; y++) {
			if (isInsideXY(x, y))
				if (mask[y*widthXY+x]){
					faceBox.setRightX(x);
					rXr = h_pReal[y*XN_VGA_X_RES+x];
					return true;
				}
		}
	}	

	// Switch to pure detection if tracking does not work
	left = 0;
	right = widthXY-1;
	for (int x=left; x<right; x++) {
		for (int y=topY; y<middleY+boxY; y++) {
			if (isInsideXY(x, y))
				if (mask[y*widthXY+x]) {
					faceBox.setRightX(x);
					rXr = h_pReal[y*XN_VGA_X_RES+x];
					return true;
				}
		}
	}

	// If nothing is found anyway, return the previous position
	faceBox.setRightX(previousFaceBox.getRightX());
	return true;
}
/**********************/



/********************/
/*	Detect the face	*/
bool FaceDetector::detectFace(			float *h_pReal,
										int thresholdYZ, 
										int thresholdXY, 
										int thresholdDIFF, 
										int thresholdDEPTH, 
										int thresholdDISCONTINUITY,
										int searching_box_height) {

	if (	detectTop(h_pReal) 
		&&	detectChin(	h_pReal,
							faceBox.getTopY()+HEAD_HEIGHT_MIN,
							HEAD_HEIGHT_MAX-HEAD_HEIGHT_MIN,
							thresholdYZ, 
							thresholdXY, 
							thresholdDIFF, 
							thresholdDEPTH, 
							thresholdDISCONTINUITY,
							searching_box_height)) {
		if ( !(faceBox.detected()) ) {
			faceBox.setBottomY(previousFaceBox.getBottomY());	
		}
	
		///////////////////////////////////
		// Find real face size
		int zMin=INT_MAX;
		int i=XN_VGA_X_RES*faceBox.getBottomY(), b_x=0;
		for (int x=0;x<XN_VGA_X_RES;x++) {
			if (h_pReal[MAX_I2+i]>=MIN_DEPTH && h_pReal[MAX_I2+i]<zMin) {
				b_x	= x;
				zMin	= (int)h_pReal[MAX_I2+i];
			}
			i++;
		}
		bYr = h_pReal[MAX_I+XN_VGA_X_RES*faceBox.getBottomY()+b_x];
		faceBox.setHr(tYr-bYr);
		//fprintf(stderr, "Wr:%3.2f Hr:%3.2f\n", faceBox.getWr(), faceBox.getHr());
		if ( previousFaceBox.getHeight() !=0 && abs(faceBox.getHr()-previousFaceBox.getHr())>T_RDIFF ) {
			this->faceBox.setBottomY(previousFaceBox.getBottomY());	
			faceBox.setHr(previousFaceBox.getHr());
		}
		///////////////////////////////////

		// Compute left and right cheeks
		detectLeftCheek(faceBox.getTopY(), faceBox.getBottomY(), h_pReal);
		detectRightCheek(faceBox.getTopY(), faceBox.getBottomY(), h_pReal);
		// Compute min and max depths
		findMaxMinDepth(	h_pReal, 
								faceBox.getLeftX(), 
								faceBox.getRightX(), 
								faceBox.getTopY(), 
								faceBox.getBottomY() );
		// Find width
		faceBox.setWr(rXr-lXr);
		

		if (faceBox.possible()) {
			previousFaceBox.setAll(faceBox);
		} else {
			faceBox.setAll(previousFaceBox);
			previousFaceBox.resetAll();
		}
	}

	//cout<<"\n*************************************\n>>the resolution:"<<XN_VGA_X_RES<<","<<XN_VGA_Y_RES<<endl;
	//cout<<"facebox resolution: "<<faceBox.getHeight()<<","<<faceBox.getWidth();
	//cout<<"\n*************************************\n";

	if (faceBox.getHeight()==0 || faceBox.getWidth()==0) {
		//faceBox.setNose(cvPoint(0,0));
		return false;
	} else {
		//findNoseTip(h_pReal);
		return true;
	}
}
/**********************/





///********************************************************/
///* Find the closest point -nose tip for a frontal face-	*/
//// Find the local minima
//void FaceDetector::findNoseTip(float *h_pReal) {
//	int widthX = faceBox.getWidth();
//	int heightY = faceBox.getHeight();
//	
//	int minDepth=MAX_DEPTH, global_minDepth=MAX_DEPTH;
//	int x_global=faceBox.getLeftX(), y_global=  faceBox.getTopY();
//
//	int top=0, bottom=0, right=0, left=0;
//
//	const int NEIGBORHOOD = 15;
//
//	if (	(previousFaceBox.getNose().x != 0 || previousFaceBox.getNose().y != 0) 
//		&&	(previousFaceBox.getNose().x>previousFaceBox.getLeftX()+NEIGBORHOOD && previousFaceBox.getNose().y>previousFaceBox.getTopY()+NEIGBORHOOD) ) {
//			top		= std::max(previousFaceBox.getNose().y-NEIGBORHOOD, faceBox.getTopY() + heightY/4);
//			bottom	= std::min(previousFaceBox.getNose().y+NEIGBORHOOD, faceBox.getBottomY());
//		left		= std::max(previousFaceBox.getNose().x-NEIGBORHOOD, faceBox.getLeftX()+1);
//		right		= std::min(previousFaceBox.getNose().x+NEIGBORHOOD, faceBox.getRightX());	
//	} else {
//		top		= faceBox.getTopY() + heightY/4;
//		bottom	= faceBox.getBottomY();
//		left		= faceBox.getLeftX()+1;
//		right		= faceBox.getRightX();
//	}
//	
//	// Find the global minimum
//	for (int y=top; y<bottom; y++) {
//		minDepth = MAX_DEPTH;
//		for (int x=left; x<right; x++) {
//			if (	isInsideXY(x,y) ) {
//				// Find the minimum depth on the row at height y
//				int index = XN_VGA_X_RES*y + x;
//				if (	h_pReal[MAX_I2+index]	> MIN_DEPTH
//					&&	h_pReal[MAX_I2+index]	<	minDepth) {
//					minDepth = 	(int)h_pReal[MAX_I2+index];
//					if (minDepth<global_minDepth){
//						global_minDepth = minDepth;
//						x_global = x;
//						y_global = y;
//					}
//				}
//			}
//		}
//	}
//
//	cout<<"Nose tip found at:"<<x_global<<","<<y_global<<endl;
//	//faceBox.setNose(cvPoint(x_global,y_global));
//}
///**********************/


/**************************************************************/
/* Say whether a face has been detected in the PREVIOUS frame	*/
bool FaceDetector::IsFirst() {
	return ((	getPreviousFaceBox().getMaxDepth()==0 && getPreviousFaceBox().getMinDepth()==0)	
			||		getPreviousFaceBox().getMaxDepth() < MIN_DEPTH
			||		getPreviousFaceBox().getMinDepth() > MAX_DEPTH);
}
/**********************/