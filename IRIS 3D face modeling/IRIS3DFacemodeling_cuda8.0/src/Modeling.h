//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

#include "Voxel.h"
#include "FaceBox.h"
#include "CudaMem.h"
#include "openNI_general.h"

#ifndef MODELING_H
#define MODELING_H

class Modeling {
 	private: 	
		IplImage *imFront;
		IplImage *depthImg;		// Unwrapped depth image

		Voxel cFront;

		int nb_mesh;
		int nb_vertices;

		int **mesh_model;	// Mesh map used for the rendering





		float *d_face;
		float *d_R; 
		float *d_t;
		float *d_face_C;
		int	*d_face_I;
		float *d_dtdy;
		float *h_model, *d_model;		// Depth information full float value -to be used for modelling-
		float *h_modelXYZ, *d_modelXYZ; // Model converted in XYZ coordinate
		float *h_modelRGB, *d_modelRGB;	// RGB information for the model
		float *h_overlap, *d_overlap;
		bool *h_defined, *d_defined;
		bool *h_defined_C, *d_defined_C;
		float *h_meann, *d_meann;

		char *h_dData, *d_dData;
		char *h_iData, *d_iData;






		


		// Create the point cloud that will be added to the model
		void createPointCloud(	Voxel *pts,								// Output points that will be added to the model
										int *nb,									// Number of points
										FaceBox maskBox,						// box containing the face
										float *h_pReal,						// real 3D coordinate for the points	
										float *normalMap,
										float *h_R,								// rotation matrix
										float *h_t,								// translation matrix
										Voxel centroid,						// Current centroid used for the EM-ICP normalization step
										float normalization_factor,		// Normalization factor used for the EM-ICP normalization step
										bool first);							// Flag whether the point cloud is the first one
		void createPointCloud(	Voxel *pts, 
									int *nb, 
									FaceBox maskBox, 
									float *h_pReal, 
									float *h_R, 
									float *h_t, 
									Voxel centroid, 
									float normalization_factor,
									bool first) ;

		// Apply the inverse transformation of (h_R, h_t) to pt
		void applyInvTransformation(	Voxel *pt,		// Input/Output point
												float *h_R,		//	Rotation matrix, apply the inverse 
												float *h_t);	//	Translation matrix, apply the inverse 
		void applyInvTransformation(float *h_pts, int nb, Voxel *pts, float *h_R, float *h_t);
		void applyInvRotation(float *f1, float *f2, float *f3, float *h_R);

		// Apply the transformation (h_R, h_t) to pt
		void applyTransformation(	Voxel *pt,			// Input/Output point
											float *h_R,			//	Rotation matrix
											float *h_t);		//	Translation matrix
		// Add the points to the model
		void updateModelImages(	Voxel *pts,			// Points to add
								float *normalMap,	// Normal map to aggregate
								int nb);			// Number of points
		void updateModelImages(float *h_pts, int nb, Voxel *pts, bool *wrongRegistration);
		void updateModelImages(Voxel *pts, int nb, bool *wrongRegistration);
		// Close the model
		void simpleClosing( int nb_iterations );	// Number of iterations
		// Circular interpolation
		void circularInterpolation( );
		void circularInterpolation( float *mConfidence );
		// Circular interpolation on the back
		void completeInterpolation( );

		float terribleInterpolationOnFirst(int y);
		void verticalInterpolation( ) ;

		// Return true if the pixel i of image is a skin pixel
		bool isSkinColor( IplImage *img,			// Image
								int i );					// Index of the pixel
		// Create the meshes into hash for the pixels in defined
		int createRawMeshes( int *hash,			// Output: meshes
									bool *defined );	// Pixels that should be used for the meshing

		void saveMTL(char *name );

 	public:
		

 		/* Constructor */
 		Modeling();
		~Modeling();

		/* Reset */
		void reset();

 		/* Getters and setters for all the members */
		void setImFront(IplImage *imFront);
		void setImLeft(IplImage *imLeft);
 		void setImRight(IplImage *imRight);
		void setRRight(float *r_right);
		void setTRight(float *t_right);
		void setRLeft(float *r_left);
		void setTLeft(float *t_left);
		void setCFront(Voxel cFront);
		void setCRight(Voxel cRight);
		void setCLeft(Voxel cLeft);
		void setDepthImg(IplImage *depthImg);
		void setDepthImg(int i, uchar c);
		void setDepthInfo(float *depthInfo);
		void setDepthInfo(int i, float value);
		void setHModel(int i, float value);
		void setHMeann(int i, int value);

		//void setNormalInfo(int i, float value);
		void setMean(int i, int value);
		void setChinFirst(int chin_first);
		void setPts(float X, float Y, float Z, int i);

		static void Convert2Cylindrical(float x, float y, float z, float *ro, float *theta, float *yy, int *index);
		static void FindCylindricalIndex(float x, float y, float z, float *i_tc, float *i_yc);
		
		IplImage *getDepthImg();
		IplImage *getImFront();
		float *getDepthInfo();
		float getHmodelXYZ(int i);
		float *getHmodelXYZ();
		float getHmodelRGB(int i);
		float *getHmodelRGB();
		bool *getH_defined_C();
		bool getHdefined(int i);
		int **getMesh();
		int getMesh(int k, int i);
		int getNbMesh();
		int getIndexInfo(int i);
		void getXYZ(int i, float *X, float *Y, float *Z);
		float getNormalInfo(int i);
		int getStep();

		
		// Apply the whole modeling algorithm for the current image
		void model(	int nb,									// Number of points to add
					FaceBox maskBox,						// Detected face in the current image
					float *h_pReal,							// Current 3D points
					float *h_R,								// Current rotation matrix
					float *h_t,								// Current transformation matrix
					bool first,								// Same as first in main, 'true' if reference image
					Voxel centroid,							// Current centroid of the point cloud
					float normalization_factor,				// Normalization factor
					bool *wrongRegistration);			
		void model(	int nb, 
					FaceBox maskBox, 
					float *h_face, 
					float *h_R, 
					float *h_t, 
					bool first,
					bool *wrongRegistration);	
		void model(		int nb, 
							float *h_face, 
							float *h_R, 
							float *h_t, 
							bool *wrongRegistration,
							int FRAME_NUM);
		// Apply a bilateral filter to the model
		void bilateralFiltering(int w, 
								float sigma_d, 
								float sigma_r);
		// Apply 3D transformation to model and output in Mp
		void transformIn3D(float *Mp, float *MNp, float *h_R, float *h_t);
		// Save to an OFF file
		void saveToOFFFile( char *name, bool withMesh  );
		// Save to an OBJ file
		void saveToOBJFile(	char *name, 
								bool withMesh, 
								float normalize, 
								float *h_pReal,	
								float *d_gaussian,
								float *mConfidence);
		void saveToOBJFile(	char *name,						// Name of the file
							bool withMesh,					// 'true' if we want the meshes to be saved
							float normalize				// Current normalization factor
							);	
};



#endif
