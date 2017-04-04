//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

#include "stdafx.h"
#include "FaceBox.h"



/****************/
/* Constructors */
/****************/

FaceBox::FaceBox() {
	this->topY = 0;
	this->bottomY = 0;
	this->leftX = 0;
	this->rightX = 0;
	this->height = 0;
	this->width = 0;
	this->minDepth = 0;
	this->maxDepth = 0;
	this->Wr = 0.0f;
	this->Hr = 0.0f;
	
	
	//this->nose = cvPoint(0,0);
};

/************************/
/* Getters and setters */
/***********************/

void FaceBox::setTopY(int topY) {
	this->topY = topY;	
	this->height = bottomY - topY;
}
void FaceBox::setBottomY(int bottomY) {
	this->bottomY = bottomY;	
	this->height = bottomY - topY;
}
void FaceBox::setLeftX(int leftX) {
	this->leftX = leftX;	
	this->width = rightX - leftX;
}
void FaceBox::setRightX(int rightX) {
	this->rightX = rightX;	
	this->width = rightX - leftX;
}
void FaceBox::setHeight(int height) {
	this->height = height;
}
void FaceBox::setWidth(int width) {
	this->width = width;
}
void FaceBox::setHr(float Hr) {
	this->Hr = Hr;
}
void FaceBox::setWr(float Wr) {
	this->Wr = Wr;
}
void FaceBox::setMinDepth(int minDepth) {
	this->minDepth = minDepth;
}
void FaceBox::setMaxDepth(int maxDepth) {
	this->maxDepth = maxDepth;	
}



//void FaceBox::setNose(CvPoint nose) {
//	this->nose = nose;	
//}







int FaceBox::getTopY() {
	return topY; 	
}
int FaceBox::getBottomY() {
	return bottomY; 	
}
int FaceBox::getLeftX() {
	return leftX; 	
}
int FaceBox::getRightX() {
	return rightX; 	
}
int FaceBox::getHeight() {
	return height;
}
int FaceBox::getWidth() {
	return width;
}
float FaceBox::getHr() {
	return Hr;
}
float FaceBox::getWr() {
	return Wr;
}
int FaceBox::getMinDepth() {
	return minDepth;
}
int FaceBox::getMaxDepth() {
	return maxDepth;
}

/*******************xxx**************/
//CvPoint FaceBox::getNose() {
//	return nose;	
//}
/*******************xxx**************/





/**************************/
/* Methods implementation */
/**************************/

bool FaceBox::detected() {
	if (bottomY==0)
		return false;
	return true;
}

bool FaceBox::possible() {
	if (	height > HEAD_HEIGHT_MAX
		||	height < HEAD_HEIGHT_MIN && (topY != 0)
		||	width > HEAD_WIDTH_MAX
		||	width < HEAD_WIDTH_MIN && (leftX != 0))
		return false;
	return true;
}

void FaceBox::setAll(FaceBox faceBox) {
	this->bottomY = faceBox.getBottomY();
	this->topY = faceBox.getTopY();
	this->leftX = faceBox.getLeftX();
	this->rightX = faceBox.getRightX();
	this->height = faceBox.getHeight();
	this->width = faceBox.getWidth();
	this->minDepth = faceBox.getMinDepth();
	this->maxDepth = faceBox.getMaxDepth();
	
	
	//this->nose = faceBox.getNose();
	
	
	
	this->Hr = faceBox.getHr();
	this->Wr = faceBox.getWr();
}

void FaceBox::resetAll() {
	this->bottomY = 0;
	this->topY = 0;
	this->leftX = 0;
	this->rightX = 0;
	this->height = 0;
	this->width = 0;
	this->minDepth = 0;
	this->maxDepth = 0;
	this->Hr = 0.0f;
	this->Wr = 0.0f;
}

bool FaceBox::isInbox(int x, int y) {
	if (x>=(this->leftX) && x<=(this->rightX) && y>=(this->topY) && y<=(this->bottomY))
		return true;
	return false;
}