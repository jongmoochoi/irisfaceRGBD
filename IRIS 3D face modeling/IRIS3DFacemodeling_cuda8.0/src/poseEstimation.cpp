
////////////////////////////////////////////////////////////////////////////////////
//	NI_eleventh.cpp : Estimate the face pose based on sparse sampling,						//
//	nose tip detection, particle filtering, score computed as NCC on intensity		//
////////////////////////////////////////////////////////////////////////////////////

#include "poseEstimation.h"
#include "3dregistration.h"
#include "engine.h"
#include "rply.h"



/********************************/
/* Check that there is no error */
void errorCheck(Status nRetVal) {
	if (nRetVal != STATUS_OK) {
		fprintf(stderr,"Failed: %s\n", oniGetExtendedError());
		exit(1);
	}
}
/*************************/







/*********************/
/* Display image map */
void displayImageMap(char *out, const OniRGB888Pixel* pImageMap) {
	int i3=0;
	for (int i=0; i<MAX_I_HR; i++) {
		out[i3]		= (char)(pImageMap[i]).b;
		out[i3+1]	= (char)(pImageMap[i]).g;
		out[i3+2]	= (char)(pImageMap[i]).r;
		i3+=3;
	}
}
/**********************/











/*****************************************/
/* Find the number of points on the face */
void extractFace(	float *h_face,
					float *h_pReal,
					FaceBox maskBox, 
					bool *validityMask,
					int *nb_pts, 
					float *normalization_factor,
					Voxel *centroid, 
					bool first) {

	int	l = maskBox.getLeftX(),
		r = maskBox.getRightX(),
		t = maskBox.getTopY(),
		b = maskBox.getBottomY();

	int x=l, y=t, index=0;
	*nb_pts=0;
	int begin	= t*XN_VGA_X_RES + l;
	int end		= (b+CHIN_ADD)*XN_VGA_X_RES + r;

	float	X=0.0f,		Y=0.0f,		Z=0.0f,
			X_=0.0f,	Y_=0.0f,	Z_=0.0f;
	int tpm=0, tpM=0;
	float maxx=0.0f, minn=FLT_MAX;
	


	// Restrict the skin mask to the face area
	for (int i=begin; i<end; i++) {
		// Remove those which are not on the face
		if (x>=l 	&&	x<=r 	&&	y>=t	&&	y<=(b+CHIN_ADD)) {
			if (validityMask[i] 
				// && (validityMask[i-1] || validityMask[i+1] || validityMask[i+XN_VGA_X_RES] || validityMask[i-XN_VGA_X_RES])
				) {
				X=h_pReal[i];
				Y=h_pReal[i+MAX_I];
				Z=h_pReal[i+MAX_I2];
				
				// Compute average
				X_ += X;
				Y_ += Y;
				Z_ += Z;
				
				h_face[index]		= X;
				h_face[index+MAX_I]	= Y;
				h_face[index+MAX_I2]= Z;
				index++;
				if (first) {
					// Compute max/min only for the first frame
					if ( X > maxx ) { maxx = X; tpM=0; }
					if ( Y > maxx ) { maxx = Y; tpM=1; }
					if ( X < minn ) { minn = X; tpm=0; }
					if ( Y < minn ) { minn = Y; tpm=1; }
				}
			}			
		}
				
		// Update the line/column values
		if (++x == XN_VGA_X_RES) {
			x=0;
			y++;
		}
	}

	if (index>0) {
		X_ /= (float)index;
		Y_ /= (float)index;
		Z_ /= (float)index;
	}
	// Set the centroid
	(*centroid).setXr(X_);
	(*centroid).setYr(Y_);
	(*centroid).setZr(Z_);

	// Update the nomalization_factor only for the first frame
	if (first) {
		if (tpM==0) maxx -= X_;
		if (tpM==1) maxx -= Y_;
		if (tpm==0) minn -= X_;
		if (tpm==1) minn -= Y_;
		maxx = std::max(fabs(maxx),fabs(minn));

		(*normalization_factor) = fabs(maxx);
	}



	// Set the points
	int ct0=0,
		ct1=MAX_I,
		ct2=MAX_I2;
	for (int i=0; i<index; i++) {
		h_face[ct0++] = (float)(h_face[i]-X_)/(*normalization_factor);
		h_face[ct1++] = (float)(h_face[i+MAX_I]-Y_)/(*normalization_factor);
		h_face[ct2++] = (float)(h_face[i+MAX_I2]-Z_)/(*normalization_factor);
	}

	(*nb_pts) = index;	

}
/**********************/



/***********************/
/* Horizontal gradient */
void gradient(float *in, float *horizontal, float *vertical, int w, int h, int W, bool *validityMask) {
	
	int	i=0,
		i1=0, 
		i2=0, 
		i3=0, 
		i4=0;
	float v=0.0f, v1=0.0f, v2=0.0f, v3=0.0f, v4=0.0f;

	for (int x=0; x<w; x++) {
		for (int y=0; y<h; y++) {
			i=y*W+x;

			if (validityMask[i]) {
				i1=i-W;
				i2=i-1;
				i3=i+W;
				i4=i+1;

				v = in[i];
				v1 = ( (y==0) ? 2*v-in[i3] : in[i1]);
				v2 = ( (x==0) ? 2*v-in[i4] : in[i2]);
				v3 = ( (y==h-1) ? 2*v-in[i1] : in[i3]);
				v4 = ( (x==w-1) ? 2*v-in[i2] : in[i4]);

				horizontal[i]	= (v4-v2)/2.0f;
				vertical[i]		= (v3-v1)/2.0f;
			} else {
				horizontal[i]=0.0f;
				vertical[i]  =0.0f;
			}
		}
	}
}
/**********************/




/************************************************************/
/* Find the smallest box containing all the non-zero pixels	*/
FaceBox findBox(IplImage *mask) {
	FaceBox faceBox;

	int width=mask->width;
	int height=mask->height;
	int step=mask->widthStep;
	uchar *data= (uchar *)mask->imageData;
	bool found=false;

	// Find top
	found=false;
	for (int y=0;y<height;y++) {
		for (int x=0;x<width;x++) {
			if (data[y*step+x]!=0) {
				faceBox.setTopY(y);
				found=true;
			}
			if (found)
				break;
		}
		if (found)
				break;
	}

	// Find bottom
	found=false;
	for (int y=height-1;y>=0;y--) {
		for (int x=0;x<width;x++) {
			if (data[y*step+x]!=0) {
				faceBox.setBottomY(y);
				found=true;
			}
			if (found)
				break;
		}
		if (found)
				break;
	}	

	// Find left
	found=false;
	for (int x=0;x<width;x++) {
		for (int y=0;y<height;y++) {
			if (data[y*step+x]!=0) {
				faceBox.setLeftX(x);
				found=true;
			}
			if (found)
				break;
		}
		if (found)
				break;
	}

	// Find right
	found=false;
	for (int x=width-1;x>=0;x--) {
		for (int y=0;y<height;y++) {
			if (data[y*step+x]!=0) {
				faceBox.setRightX(x);
				found=true;
			}
			if (found)
				break;
		}
		if (found)
				break;
	}

	return faceBox;
}
/***********************/



/********************************/
/*	Select points on the face	*/
void selectRandomPointOnFace(	int *selectedPoints, 
								int height, 
								int width, 
								int topY, 
								int leftX, 
								int nb, 
								bool *validityMask,
								int *nbb) {
	//if (normalized_height == 0 || normalized_width == 0) {
	//	printf("The normalized face should have a positive height and width\n");
	//	exit(2);
	//}
	bool broken=false;
	int index=0;
	int r_X=0, r_Y=0;
	int i=0;

	int it=0;
	const int max_it=10*NB_PTS_X;

	srand((int)time(NULL));
	if (width >0 && height>0) {
		for (i=0; i<nb; i++) {
			r_X = rand() % width + leftX;
			r_Y = rand() % height + topY;

			index=r_Y*XN_VGA_X_RES+r_X;
			selectedPoints[i]=index;

			if (!validityMask[index]) 
				i--;

			if ((++it)>max_it) {
				broken=true;
				break;
			}
		}
	}

	(*nbb) = (broken ? i : NB_PTS_X);
}
/***********************/



/********************************/
/*	Select points on the face	*/
void selectRandomPointOnFace(	int *selectedPoints, 
								int nb_points_on_face,
								int nb_random_samples) {
	int index=0;

	//int range = (int) ((float)nb_points_on_face / (float)nb_random_samples +0.5f);
	//int shift = 0;

	srand((int)time(NULL));
	if (nb_points_on_face>0) {
		for (int i=0; i<nb_random_samples; i++) {
			index = rand() % nb_points_on_face; //%range + shift;//
			selectedPoints[i]=index;
			//shift += range;
		}
	}

}
/***********************/




/********************************/
/*	Select points on the face	*/
void selectRandomPointOnFace(	int *selectedPoints, 
								int *index_list,
								int nb_points_on_face,
								int nb_random_samples) {
	int index=0;


	srand((int)time(NULL));
	if (nb_points_on_face>0) {
		for (int i=0; i<nb_random_samples; i++) {
			index = rand() % nb_points_on_face;//%range + shift;//
			selectedPoints[i] = index_list[index];
		}
	}

}
/***********************/




/********************************/
/*	Set the h_y with the model	*/
// TODO
void setH_Y(float *h_Y, float *Mp, int *unoccluded, int nb_unoccluded) {
	int r=0;
	srand((int)time(NULL));
	int index=0;

	for (int i=0; i<NB_PTS_X; i++) {
		r = rand() % nb_unoccluded;

		index = unoccluded[r];
		
		h_Y[i]				= Mp[index];
		h_Y[NB_PTS_X+i]		= Mp[MAX_IMG_INDEX_1+index];
		h_Y[NB_PTS_X*2+i]	= Mp[MAX_IMG_INDEX_2+index];
	}
}
/***********************/




/******************************************************/
/*	Says whether the point (x,y) is in the image xOy	*/
// Inputs: -(x,y) coordinate of the point
bool isInside(int x, int y){
	return (y>=0 && x>=0 && x<XN_VGA_X_RES && y<XN_VGA_Y_RES);
}
/**********************/


/*********************************************************/
/* Set the points in the correct format for EM-ICP input */
// Remember the first normalization_factor to be able to use it at any time
// Remember the centroid used here to be able to model correctly??
void set_normalizePoints(float *h_X, int nb, Voxel *pts, bool first, float *normalization_factor, Voxel *centroid) {	
	// Handle memory
	int		ct0=0,
			ct1=nb,
			ct2=2*nb;

	////////////////////////////////////////
	// Demean and normalize
	float	avgX=0.0, avgY=0.0, avgZ=0.0;
	float	minn=FLT_MAX, maxx=0.0;
	int		tpM=0, tpm=0;
	
	// Find the mean for the face values
	for (int i=0;i<nb;i++) {
		// Compute average
		avgX += (float)pts[i].getXr();
		avgY += (float)pts[i].getYr();
		avgZ += (float)pts[i].getZr();
		if (first) {
			// Compute max/min only for the first frame
			if ( pts[i].getXr() > maxx ) { maxx = (float)pts[i].getXr(); tpM=0; }
			if ( pts[i].getYr() > maxx ) { maxx = (float)pts[i].getYr(); tpM=1; }
			if ( pts[i].getXr() < minn ) { minn = (float)pts[i].getXr(); tpm=0; }
			if ( pts[i].getYr() < minn ) { minn = (float)pts[i].getYr(); tpm=1; }
		}
	}
	if (nb>0) {
		avgX /= (float)nb;
		avgY /= (float)nb;
		avgZ /= (float)nb;
	}
	// Set the centroid
	(*centroid).setXr(avgX);
	(*centroid).setYr(avgY);
	(*centroid).setZr(avgZ);

	// Update the nomalization_factor only for the first frame
	if (first) {
		if (tpM==0) maxx -= avgX;
		if (tpM==1) maxx -= avgY;
		if (tpm==0) minn -= avgX;
		if (tpm==1) minn -= avgY;
		maxx = std::max(maxx,abs(minn));

		(*normalization_factor) = maxx;
	}

	// Set the points
	for (int i=0; i<nb; i++) {
		h_X[ct0++] = (float)(pts[i].getXr()-avgX)/(*normalization_factor);
		h_X[ct1++] = (float)(pts[i].getYr()-avgY)/(*normalization_factor);
		h_X[ct2++] = (float)(pts[i].getZr()-avgZ)/(*normalization_factor);
	}
	
}
/*************************/








/*********************************************************/
/* Set the points in the correct format for EM-ICP input */
// Remember the first normalization_factor to be able to use it at any time
// Remember the centroid used here to be able to model correctly??
void set_normalizePoints(float *h_X, int nb, int *pts, float *h_pReal, bool first, float *normalization_factor, Voxel *centroid) {	
	// Handle memory	
	int	ct0=0,
		ct1=nb,
		ct2=2*nb;

	////////////////////////////////////////
	// Demean and normalize
	float	avgX=0.0, avgY=0.0, avgZ=0.0;
	float	X=0.0f, Y=0.0f, Z=0.0f;
	float	minn=FLT_MAX, maxx=0.0;
	int		tpM=0, tpm=0;
	int		index=0;
	
	// Find the mean for the face values
	for (int i=0;i<nb;i++) {
		index=pts[i];
		X=h_pReal[index];
		Y=h_pReal[index+MAX_I];
		Z=h_pReal[index+MAX_I2];
		// Compute average
		avgX += X;
		avgY += Y;
		avgZ += Z;
		if (first) {
			// Compute max/min only for the first frame
			if ( X > maxx ) { maxx = X; tpM=0; }
			if ( Y > maxx ) { maxx = Y; tpM=1; }
			if ( X < minn ) { minn = X; tpm=0; }
			if ( Y < minn ) { minn = Y; tpm=1; }
		}
	}
	if (nb>0) {
		avgX /= (float)nb;
		avgY /= (float)nb;
		avgZ /= (float)nb;
	}
	// Set the centroid
	(*centroid).setXr(avgX);
	(*centroid).setYr(avgY);
	(*centroid).setZr(avgZ);

	// Update the nomalization_factor only for the first frame
	if (first) {
		if (tpM==0) maxx -= avgX;
		if (tpM==1) maxx -= avgY;
		if (tpm==0) minn -= avgX;
		if (tpm==1) minn -= avgY;
		maxx = std::max(fabs(maxx),fabs(minn));

		(*normalization_factor) = fabs(maxx);
	}

	// Set the points
	for (int i=0; i<nb; i++) {
		index=pts[i];
		h_X[ct0++] = (float)(h_pReal[index]-avgX)/(*normalization_factor);
		h_X[ct1++] = (float)(h_pReal[index+MAX_I]-avgY)/(*normalization_factor);
		h_X[ct2++] = (float)(h_pReal[index+MAX_I2]-avgZ)/(*normalization_factor);
	}
	
}
/*************************/


/*********************************************************/
/* Set the points in the correct format for EM-ICP input */
// Remember the first normalization_factor to be able to use it at any time
// Remember the centroid used here to be able to model correctly??
void setH_X(float *h_X, int nb, int *pts, float *h_face) {	
	// Handle memory	
	int	ct0=0,
		ct1=nb,
		ct2=2*nb;
	int index=0;

	// Set the points
	for (int i=0; i<nb; i++) {
		index=pts[i];
		h_X[ct0++] = h_face[index];
		h_X[ct1++] = h_face[index+MAX_I];
		h_X[ct2++] = h_face[index+MAX_I2];
	}
	
}
/*************************/

/*********************************************************/
void setH_Yhalf(float *h_Y, int nbY, int *pts, int nb_pts, float *h_pts) {	
	// Handle memory	
	int	ct0=0,
		ct1=nbY,
		ct2=2*nbY;
	int index=0;

	// Set the points
	for (int i=0; i<nbY; i++) {
		index=pts[i];
		h_Y[ct0++] = h_pts[index];
		h_Y[ct1++] = h_pts[index+nb_pts];
		h_Y[ct2++] = h_pts[index+nb_pts*2];
	}
	
}
/*************************/












/*******************/
/* Swap the points */
void swapPoints(float *h_X, float *h_Y, int nb_X) {	
	int nb_X3=3*nb_X;

	// Set the points
	for (int i=0; i<nb_X3; i++) 
		h_Y[i] = h_X[i];	
}
/*************************/

/******************************************/
/* Find the number of points in each half	*/
void findNbHalf(	float *h_Y, 
						int nb,  
						int *nb_left,
						int *nb_right	) {
	*nb_left = *nb_right = 0;

	for (int i=0; i<nb; i++) {
		if (h_Y[i]<0) 
			(*nb_left)++;
	}
	*nb_right = nb-*nb_left;

}
/*************************/

/**********************/
/* Split in two halfs */
void splitHalf(	float *h_Y, 
						int nb, 
						float *h_Yleft, 
						int nb_left,
						float *h_Yright, 
						int nb_right	) {
	int	k_l0=0,
			k_l1=nb_left,
			k_l2=nb_left*2,
			k_r0=0,
			k_r1=nb_right,
			k_r2=nb_right*2,
			i1=nb,
			i2=2*nb;

	for (int i=0; i<nb; i++) {
		if (h_Y[i]<0)  {
			h_Yleft[k_l0++]	= h_Y[i];
			h_Yleft[k_l1++]	= h_Y[i1+i];
			h_Yleft[k_l2++]	= h_Y[i2+i];
		} else {
			h_Yright[k_r0++]	= h_Y[i];
			h_Yright[k_r1++]	= h_Y[i1+i];
			h_Yright[k_r2++]	= h_Y[i2+i];
		}
	}
}
/*************************/

/*********************/
/* Estimate the pose */
void poseEstimation(	int Xsize, int Ysize, 
						float *h_X, float *d_X, float *d_Xx, float *d_Xy, float *d_Xz,
						float *h_Y, float *d_Y, float *d_Yx, float *d_Yy, float *d_Yz,
						float *h_R, float *d_R, float *h_t, float *d_t,
						float *h_S, float *d_S,
						float *h_Xc, float *d_Xc, float *h_Yc, float *d_Yc,
						float *h_one, float *d_one,
						float *d_A,
						float *d_Xprime, float *d_XprimeX, float *d_XprimeY, float *d_XprimeZ,
						float *d_XprimeCenterd, float *d_XprimeCenterdX, float *d_XprimeCenterdY, float *d_XprimeCenterdZ,
						float *d_YCenterd, float *d_YCenterdX, float *d_YCenterdY, float *d_YCenterdZ,
						float *d_C, float *d_lambda,
						int	maxXY, int rowsA, int colsA, int pitchA,
						registrationParameters param,	// registration parameters
						float *a, float *pre_a, float *pre_t,
						bool *allocateMemory,
						float *diff ) {
	bool error=false;	
	bool first=true;

	if(!param.noviewer){
		// PointCloudViewer
		EngineInit();
		EngineCameraSetup(0.2f);
		InitPointCloud(h_X, Xsize, h_Y, Ysize,
						 param.points1, param.points2, param.points3);  // returns pointers points1,2,3 for visualization
	}

	if (*allocateMemory) {
		copyHostToCUDA(Y, 3*Ysize);
		*allocateMemory = false;
	}	
	// Copy X values in GPU
	copyHostToCUDA(X, 3*Xsize);






	do {
		//clock_t start, end;
		//start = clock();

		//// Swap the values for angles/tranlsation
		for (int i=0;i<3;i++) {
			pre_a[i] = a[i];
			pre_t[i] = h_t[i];
		}

		// Compute EM-ICP
		error=false;
		if (IS_GPU)
			emicp(	Xsize, Ysize, 
					h_X, d_X, d_Xx, d_Xy, d_Xz,
					h_Y, d_Y, d_Yx, d_Yy, d_Yz,
					h_R, d_R, h_t, d_t,
					h_S, d_S,
					h_Xc, d_Xc, h_Yc, d_Yc,
					h_one, d_one,
					d_A,
					d_Xprime, d_XprimeX, d_XprimeY, d_XprimeZ,
					d_XprimeCenterd, d_XprimeCenterdX, d_XprimeCenterdY, d_XprimeCenterdZ,
					d_YCenterd, d_YCenterdX, d_YCenterdY, d_YCenterdZ,
					d_C, d_lambda,
					maxXY, rowsA, colsA, pitchA,
					param,
					&error,
					allocateMemory);
		else
			emicp_cpu(	Xsize, Ysize, h_X, h_Y, 
							h_R, h_t, 
							param, 
							&error);

		//end = clock();
		//printf("elapsed %f\n", (double)(end - start) / CLOCKS_PER_SEC);

		// Display angles/translation
		//printRT(h_R, h_t, a);
		rotationMatrix2eulerAngles(h_R,a);

		// Go on if the program has been stop because of an error in the computation of the eigenvalues
		param.nostop=true;
		*diff = tr_diff(h_t, pre_t, a, pre_a);
		if ( error )
			if ( first || *diff>T_TR_DIFF ) 
				param.nostop = false;
		
		first = false;
	}  while(!param.nostop);

	//fprintf(stderr,"%3.2f\n", *diff);

	if (!param.noviewer)
		while(EngineIteration(Ysize, param.points2, h_Y,  h_R, h_t)); // PointCloudViewer
	setWinFlag();
}
/*********************/



/**********************/
void InitPointCloud(const float* h_X, const int Xsize,
		    const float* h_Y, const int Ysize,
		    float* &points1, float* &points2, float* &points3) {

	float	r1=1.0f, g1=0.2f, b1=0.0f, e1=2.5f,
			r2=0.2f, g2=0.8f, b2=0.2f, e2=2.5f,
			r3=0.1f, g3=0.4f, b3=1.0f, e3=2.5f;

  const float* h_Xx = &h_X[Xsize*0];
  const float* h_Xy = &h_X[Xsize*1];
  const float* h_Xz = &h_X[Xsize*2];

  const float* h_Yx = &h_Y[Ysize*0];
  const float* h_Yy = &h_Y[Ysize*1];
  const float* h_Yz = &h_Y[Ysize*2];

  // Generate a random point cloud:
  points1 = new float[Xsize*3];
  for (int i=0; i<Xsize; i++)
    {
      float* point = &points1[i*3];
      point[0] = h_Xx[i];
      point[1] = h_Xy[i];
      point[2] = h_Xz[i];
    }
  // Inform the engine about the first point cloud ("index" 0)...
  EnginePointCloudData(0, points1, Xsize);
  // ... and some fancy attributes like color and point size.
  EnginePointCloudDecoration(0, r1, g1, b1, e1);

  // Another point cloud:
  points2 = new float[Ysize*3];
  for (int i=0; i<Ysize; i++)
    {
      float* point = &points2[i*3];
      point[0] = h_Yx[i];
      point[1] = h_Yy[i];
      point[2] = h_Yz[i];
    }
  // The second point cloud has "index" 1.
  EnginePointCloudData(1, points2, Ysize);
  EnginePointCloudDecoration(1, r2, g2, b2, e2);


#if 1

  // third point cloud:
  points3 = new float[Ysize*3];
  for (int i=0; i<Ysize; i++)
    {
      float* point = &points3[i*3];
      point[0] = h_Yx[i];
      point[1] = h_Yy[i];
      point[2] = h_Yz[i];
    }

  EnginePointCloudData(2, points3, Ysize);
  EnginePointCloudDecoration(2, r3, g3, b3, e3);

#endif

}
/**********************/

/**********************/
void UpdatePointCloud2(int Ysize, float* points2,
		       const float* h_Y, const float* h_R, const float* h_t){
  const float* h_Yx = &h_Y[0];
  const float* h_Yy = &h_Y[Ysize];
  const float* h_Yz = &h_Y[Ysize*2];

  // Another point cloud:
  for (int i=0; i<Ysize; i++)  {
      float* point = &points2[i*3];
      point[0] = (h_R[0]*h_Yx[i] + h_R[1]*h_Yy[i] + h_R[2]*h_Yz[i]) + h_t[0];
      point[1] = (h_R[3]*h_Yx[i] + h_R[4]*h_Yy[i] + h_R[5]*h_Yz[i]) + h_t[1];
      point[2] = (h_R[6]*h_Yx[i] + h_R[7]*h_Yy[i] + h_R[8]*h_Yz[i]) + h_t[2];
    }
}
/**********************/


















/**********************/
void InitPointCloud(float* h_X, const int Xsize, bool *h_defined,
		    float* &points1) {

	float	r1=1.0f, g1=0.2f, b1=0.0f, e1=2.5f;

  float* h_Xx = &h_X[Xsize*0];
  float* h_Xy = &h_X[Xsize*1];
  float* h_Xz = &h_X[Xsize*2];
  float X=0.0f, Y=0.0f, Z=0.0f;

  // Generate a random point cloud:
  points1 = new float[Xsize*3];
  for (int i=0; i<Xsize; i++)
    {
		float* point = &points1[i*3];
		if (h_defined[i]) {
			X = h_Xx[i];
			Y = h_Xy[i];
			Z = h_Xz[i];
		}

		point[0] = X;
		point[1] = Y;
		point[2] = Z;
    }
  // Inform the engine about the first point cloud ("index" 0)...
  EnginePointCloudData(0, points1, Xsize);
  // ... and some fancy attributes like color and point size.
  EnginePointCloudDecoration(0, r1, g1, b1, e1);
}
/**********************/