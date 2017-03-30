//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.


#ifndef FACEBOX_H
#define FACEBOX_H



class FaceBox {
 	private: 	
 		int topY;			// Top of the face box
		int bottomY;	// Bottom of the face box
		int leftX;		// Left of the face box
		int rightX;		// Right of the face box
		int height;		// Height of the face box
		int width;		// Width of the face box
		float Hr;		// Height of the face box in the real world
		float Wr;		// Width of the face box in the real world
		int minDepth;	// Minimum depth within the face box
		int maxDepth;	// Maximum depth within the face box
		//CvPoint nose;	// Position of the nose tip

 	public:
 		/* Constructor */
 		FaceBox();

 		/* Getters and setters for all the members */
 		void setTopY(int topY);
		void setBottomY(int bottomY);
		void setLeftX(int leftX);
		void setRightX(int rightX);
		void setHeight(int height);
		void setWidth(int width);
		void setHr(float Hr);
		void setWr(float Wr);
		void setMinDepth(int minDepth);
		void setMaxDepth(int maxDepth);
		
		
		//void setNose(CvPoint nose);


		int getTopY();
		int getBottomY();
		int getLeftX();
		int getRightX();
		int getHeight();
		int getWidth();
		float getHr();
		float getWr();
		int getMinDepth();
		int getMaxDepth();
		//CvPoint getNose();
		
		/* Methods */ 
		bool detected();	// True if a face is detected
		bool possible();	// True if the detected face features are possible
		void setAll(FaceBox faceBox);	// Set the features of face box to the features of the parameter
		void resetAll();	// Reset all the features
		bool isInbox(int x, int y);	// True if point (x,y) in the face box
};

#endif
