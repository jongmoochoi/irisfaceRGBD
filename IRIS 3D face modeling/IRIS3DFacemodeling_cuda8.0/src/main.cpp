//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Donghyun Kim, Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.

#include <iostream>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <list>

#include "cudaMem.h"
#include <device_launch_parameters.h>


#include "3dregistration.h"
#include "engine.h"
#include "rply.h"

#include "stdafx.h"
#include "FaceBox.h"
#include "FaceDetector.h"
#include "Voxel.h"
#include "Modeling.h"
#include "openNI_general.h"
#include "CudaMem.h"

#include "poseEstimation.h"

#include "cublas.h"

#include <pthread.h>

#include <opencv2/gpu/gpu.hpp>
#include <windows.h>

#include "facePoseEstimation.h"
#include "FaceDetectLandmark.h"
#include "IRIS_utils.h"


std::vector<cv::Point2f> landmarkPts; //landmark
extern void initExpr(const string configFIle);
extern void process(cv::Mat &R, cv::Mat &T); // pose estimation
Voxel CENTROID;					// Current centroid of the points
float NORMALIZATION_FACTOR = 1.0f;// First factor used for the face normalization: used for alignement
cv::Mat R, T;								//rotation and translation matrix from pose estimation
float R_matrix[9];
float eyebrow_x;
float eyebrow_y;
float eyebrow_y_real;

static int counts = 0;
using namespace std;

bool isRecord;
int numberOfPoseEstimation;

/*********************/
/*		main function	*/
int main(int argc, char **argv){

	int input_mode = MODE_ONLINE;

	if (argc > 1)
		input_mode = MODE_OFFLINE;

	////////////////////////////////////////////////////////////////////////
	// EM-ICP default parameters
	registrationParameters param ;
	param.sigma_p2			= ICP_P2;
	param.sigma_inf			= ICP_PREC;		// Precision for ICP algorithm, the lower, the more precise but the slower
	param.sigma_factor		= ICP_RED;		// Reduction factor for ICP algorithm, the higher, the more iterations and the slower
	param.d_02				= ICP_D02;
	param.noviewer			= NOVIEWER;		// False to display the alignment, true else
	param.nostop			= false;
	param.notimer			= true;				
	param.points1			= NULL;
	param.points2			= NULL;
	param.points3			= NULL;
	

	float	*h_Yleft=NULL,			// Left part of the frontal face
			*h_Yright=NULL,		// Right part of the frontal face
			*h_Yfull=NULL;			// Whole frontal face

	float *a				= new float[3];	// Euler angles
	float *pre_a			= new float[3];	// previous Euler Angles
	float *pre_t			= new float[3];	// Previous translation matrix
	int stuck=0;					// Counter incremented when the rigid 3D transformation is not updated
	float diff=0.0f;				// Difference between consecutive transformations

	// Initialize the tranformation
	for (int i=0; i<3; i++) {
		a[i] = pre_a[i] = pre_t[i] = 0.0f;
	}

	float *h_X, *d_X, *h_Y, *d_Y;				// Point clouds to be used for the EM-ICP
												// h_X stores points as the order of
												// [X_x0 X_x1 .... X_x(Xsize-1) X_y0 X_y1 .... X_y(Xsize-1)  X_z0 X_z1 .... X_z(Xsize-1) ],
												// where (X_xi X_yi X_zi) is the i-th point in X.
	float *d_Xx, *d_Xy, *d_Xz;
	float *d_Yx, *d_Yy, *d_Yz;

	float *d_A;
	float *d_Xprime, *d_XprimeX, *d_XprimeY, *d_XprimeZ;
	float *d_XprimeCenterd, *d_XprimeCenterdX, *d_XprimeCenterdY, *d_XprimeCenterdZ;
	float *d_YCenterd, *d_YCenterdX, *d_YCenterdY, *d_YCenterdZ;
	float *d_C, *d_lambda;
	
	int	Xsize = NB_PTS_X, Xsize2=NB_PTS_X2, Xsize3=NB_PTS_X3; // Number of points in the first (X) and second (Y) point cloud in ICP
	int Ysize = NB_PTS_X, Ysize2=NB_PTS_X2, Ysize3=NB_PTS_X3;		

	int	maxXY = max(Xsize,Ysize);
	int	rowsA = Ysize;
	int	colsA = Xsize;
	int	pitchA = (rowsA / 4 + 1) * 4;
	
	// Allocate memory for the point clouds	
	h_Yfull = new float[Ysize3];	
	h_Yleft = new float[Ysize3];
	h_Yright = new float[Ysize3]; 
	h_X = new float[Xsize3];

	mallocCUDA(X,Xsize3);
	float h_R[9], h_t[3], *d_R, *d_t; // rotation and translation matrices
	mallocCUDA(R, 9);
	mallocCUDA(t, 3);
	init_RT(h_R, h_t);
	float h_S[9], *d_S;	// S for finding R, t
	mallocCUDA(S,9);
	float h_Xc[3], *d_Xc, h_Yc[3], *d_Yc; // center of X, Y
	mallocCUDA(Xc, 3);	
	mallocCUDA(Yc, 3);
	float *h_one, *d_one;
	h_one = new float [maxXY];	// // a vector with all elements of 1.0f
	for(int t = 0; t < maxXY; t++) 
		h_one[t] = 1.0f;
	mallocCUDA(one, maxXY);
	copyHostToCUDA(one, maxXY);

	mallocCUDA(Y,Ysize3);
	mallocCUDA(Xprime,Ysize3);
	mallocCUDA(YCenterd, Ysize3);
	mallocCUDA(A, pitchA*colsA);
	mallocCUDA(C, Ysize);
	mallocCUDA(lambda, Ysize);

	d_Xx = &d_X[0];
	d_Xy = &d_X[Xsize];
	d_Xz = &d_X[Xsize2];

	d_Yx = &d_Y[0];
	d_Yy = &d_Y[Ysize];
	d_Yz = &d_Y[Ysize2];

	d_XprimeX = &d_Xprime[0];
	d_XprimeY = &d_Xprime[Ysize];
	d_XprimeZ = &d_Xprime[Ysize2];

	d_XprimeCenterd = d_Xprime;
	d_XprimeCenterdX = &d_XprimeCenterd[0];
	d_XprimeCenterdY = &d_XprimeCenterd[Ysize];
	d_XprimeCenterdZ = &d_XprimeCenterd[Ysize2];

	d_YCenterdX = &d_YCenterd[0];
	d_YCenterdY = &d_YCenterd[Ysize];
	d_YCenterdZ = &d_YCenterd[Ysize2];	

	////////////////////////////////////////////////////////////////////////
	//	OpenNI parameters
	int changedIndex=1;
	short firstC=0;
	Status rc = STATUS_OK;

	Device device;
	VideoStream sdepth, scolor;
	VideoFrameRef depthFrame, colorFrame;
	const OniDepthPixel *pDepthMap;
	const OniRGB888Pixel* pImageMap;
	VideoMode videoMode;
	
	rc = OpenNI::initialize();
	errorCheck(rc);

	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	if (input_mode == MODE_ONLINE)
		rc = device.open(ANY_DEVICE);
	else
		rc = device.open(argv[1]);
	isRecord = true;
	openni::Recorder recorder;
	

	if (rc != openni::STATUS_OK)
	{
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	errorCheck(rc);

	//rc = device.CreateStream(ONI_SOURCE_DEPTH, sdepth);
	rc = sdepth.create(device, SENSOR_DEPTH);
	
	if (rc == STATUS_OK) 	{

		openni::VideoMode videoMode = sdepth.getVideoMode();
		videoMode.setResolution(640, 480);
		sdepth.setVideoMode(videoMode);

		if (openni::STATUS_OK != rc)
		{
			cout << "error: depth fromat not supprted..." << endl;
		}
		rc = sdepth.start();
		if (rc != STATUS_OK) 		{
			printf("Couldn't start depth stream:\n%s\n", oniGetExtendedError());
			sdepth.destroy();
		}
	} 	else 	{
		printf("Couldn't find depth stream:\n%s\n", oniGetExtendedError());		
	}

	rc = scolor.create(device, SENSOR_COLOR);
	if (rc == STATUS_OK) 	{ 

		openni::VideoMode videoMode = scolor.getVideoMode();
		videoMode.setResolution(640, 480);
		scolor.setVideoMode(videoMode);

		if (openni::STATUS_OK != rc)
		{
			cout << "error: depth fromat not supprted..." << endl;
		}
		rc = scolor.start(); 
		if (rc != STATUS_OK) 		{
			printf("Couldn't start color stream:\n%s\n", oniGetExtendedError());
			scolor.destroy();
		}
	} 	else 	{
		printf("Couldn't find color stream:\n%s\n", oniGetExtendedError());
	}

	if (!sdepth.isValid() && !scolor.isValid()) 	{
		printf("No valid streams. Exiting\n");
		return 2;
	}

	if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
		rc = device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		errorCheck(rc);
	}

	VideoStream** m_streams = new VideoStream*[2];
	m_streams[0] = &sdepth;
	m_streams[1] = &scolor;	
	
	////////////////////////////////////////////////////////////////////////
	//	OpenCV parameters
	IplImage *img	=	cvCreateImage( cvSize(XN_HR_X_RES ,XN_HR_Y_RES+Y_MAX), IPL_DEPTH_8U, CHANNELS );		// RGB image
	IplImage *depth	=	cvCreateImage( cvSize(XN_VGA_X_RES ,XN_VGA_Y_RES+Y_MAX), IPL_DEPTH_8U, CHANNELS );	// Depth image
	
	IplImage *currentImg;

	char *h_rgbData, *d_rgbData;
	h_rgbData = (char *)img->imageData;
	mallocCUDA_char(rgbData, MAX_I3);

	char *h_dData, *d_dData;
	h_dData = (char *)(depth->imageData);
	mallocCUDA_char(dData, MAX_I3);

	char *depthM = (char *) &h_dData[MAX_I3];
	char *imM = (char *) &h_rgbData[MAX_I3];

	cvNamedWindow("img", CV_WINDOW_AUTOSIZE);
	//cvResizeWindow("img", XN_VGA_X_RES, XN_VGA_Y_RES);
	cvMoveWindow("img", 20, 20);
	
	bool	displayFaceTracker	= true,				// Display the face bounding box if detected
			displayDepth		= false,				// Display the depth image instead of the RGB one
			isHeat				= false,			// Display depth as heat map
			updateModel			= true,				// Update the model
			simpleRendering		= false,			// Display the rendering
			saveFrames			= false,			// Save the frames until 's' is pressed or the program is exited
			saveForImprovement	= false;			// Save all the frames for post-processing refinement
	//
	bool printPointCloud = false;
	//
	currentImg = (displayDepth ? depth:img);
	
	////////////////////////////////////////////////////////////////////////
	// Needed for computation
	// Extracted points
	int nb_keyPoints	= 0;					// Number of points on the face
	//Voxel *keyPoints	= NULL;					// NB_PTS_X random points on the face
	//keyPoints	= new Voxel[NB_PTS_X];
	int *selectedPoints = new int[NB_PTS_X];

	bool *h_validityMask, *d_validityMask;
	h_validityMask	= new bool[MAX_I];
	mallocCUDA_bool(validityMask, MAX_I);

	////////////////////////////////////////////////////////////////////////	
	// FACE DETECTION
	FaceDetector faceDetector;					// Face detector
	FaceBox nullBox;							// Default null rectangle: width=height=0
	FaceBox maskBox;							// Current detected face
	bool first = true;							// If true, set the reference and turn false
	bool wrongRegistration=false;
	

	////////////////////////////////////////////////////////////////////////	
	// CUDA
	cublasInit();
	bool allocateMemory = true;	// Should be reset to true each time the size of h_Y changes
	int	mode=M_FRONT,			// Current mode: Use only one half of the face or the whole face
		previous_mode=M_FRONT;	// Previous mode

	////////////////////////////////////////////////////////////////////////	
	// Bilateral Filter on CUDA
	float euclidean_delta=EUCLIDEAN_DELTA;//0.03f;
	float gaussian_delta= GAUSSIAN_DELTA;//8.0f;
	int filter_radius = FILTER_RADIUS;//5;
	int iterations = 3;
	int nthreads=1;

	float *h_pDepth  = (float *)malloc(MAX_I * sizeof(float));
	float *h_pReal   = (float *)malloc(MAX_I3 * sizeof(float));
	float *h_face    = new float [MAX_I3];
	float *h_test    = new float [MAX_I3];
		
	////////////////////////////////////////////////////////////////////////
	//	Normal map
	float *h_pNormals    = (float *)malloc(MAX_I3 * sizeof(float));
	for (int i=0; i<MAX_I3; i++)
		h_pNormals[i]=0.0f;

	////////////////////////////////////////////////////////////////////////
	//	Time measurement
	clock_t start, end;
	float cpuTime;
	int frame_index=0;	// Index of the frame in the session
	int session=0;			// Session number
	int nb_frames=0;

	//////////////////////////////////////////////////////////////////////
	//	PALETTE FOR HEAT MAP
	int *h_palette, *d_palette;
	h_palette = new int[SIZE_PALETTE];
	createPalette(h_palette);
	mallocCUDA_int(palette, SIZE_PALETTE);
	copyHostToCUDA_int(palette,SIZE_PALETTE);
	delete[] h_palette;

	////////////////////////////////////////////////////////////////////////
	//	Rendering monkey
	float RScale=((!displayDepth && IS_HR) ? 3.0f:1.5f);

	float *h_monkey_1 = new float[NB_RENDERING3];
	float *d_monkey_1, *d_monkey_2;
	mallocCUDA(monkey_1, NB_RENDERING3);
	mallocCUDA(monkey_2, NB_RENDERING3);
	
	// Fill in matrices
	int i_X=0, i_Y=NB_RENDERING, i_Z=NB_RENDERING*2;
	FILE *f = fopen(".\\forDisplay\\MONKEY.off","r");
	fscanf(f, "OFF\n%d %d %d\n", &i_X, &i_X, &i_X);
	float xxx=0.0f,yyy=0.0f,zzz=0.0f;
	for (int i=0;i<NB_RENDERING;i++) {
		fscanf(f,"%f %f %f",&xxx,&yyy,&zzz);	
		h_monkey_1[i_X++] = -xxx - 0.000039f;
		h_monkey_1[i_Y++] = yyy  - 0.234032f;
		h_monkey_1[i_Z++] = -zzz + 0.671042f;
	}
	fclose(f);
	copyHostToCUDA(monkey_1, NB_RENDERING3);

	/////////////////////////////////////////////////////
	// OPENNI GENERAL CUDA
	float* d_pDepth;
	mallocCUDA(pDepth, MAX_I);

	float* d_pReal,/* *d_pNormals, */*d_face;
	mallocCUDA(pReal, MAX_I3);
	//mallocCUDA(pNormals, MAX_I3);
	mallocCUDA(face, MAX_I3);

	float *d_gaussian;
	int fr_g = filter_radius+1;
	mallocCUDA(gaussian, fr_g);
	initGaussian(d_gaussian, gaussian_delta, fr_g);

	int x=0, y=0;

	////////////////////////////////////////////////////////////////////////
	//	Options
	printf("------------------\n");
	printf(" Display options:\n");
	printf("------------------\n\n");
	printf("------------------\n\n");
	printf("ACQUISITION\n\n");
	printf(" - 'd': switch between depth map and RGB input\n");
	printf(" - 't': display the face detected by the face detector\n");
	printf(" - 'r': set the current frame as the reference frame\n");
	printf(" - 'q'/ESC: terminate acquisition\n\n");
	printf("----------------\n\n");
	printf("RENDERING\n\n");
	printf(" - 'q'/ESC: terminate the program\n\n");
	printf("----------------\n");


	////dlib

	cv_image<bgr_pixel> cimg;
	frontal_face_detector detector = get_frontal_face_detector();
	std::vector<dlib::rectangle> faces;
	shape_predictor sp;
	deserialize("shape_predictor_68_face_landmarks.dat") >> sp;
	initExpr("../config/config.json");


{
	////////////////////////////////////////////////////////////////////////
	//	Modeling
	Modeling model;					// Model
	char *mdData = (char *)model.getDepthImg()->imageData;
	

	bool saveMesh=true;

	// Loop on frames
	////////////////////////////////////////////////////////////////////////
	if (isRecord && argc <= 1)
	{
		recorder.create("test.ini");
		recorder.attach(sdepth, false);
		recorder.attach(scolor, false);
		recorder.start();
	}


	while (true) { 
		start = clock();
	
		//////////////////////////////////////////////////////////////////////
		// Update the context
		rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
		if (rc != openni::STATUS_OK)
		{
			printf("Wait failed\n");
			return 1;
		}
		errorCheck(rc);
		rc = sdepth.readFrame(&depthFrame); 
		errorCheck(rc);
		rc = scolor.readFrame(&colorFrame); 
		errorCheck(rc);

		if (colorFrame.isValid())
		{ 
			pImageMap = (const OniRGB888Pixel*)colorFrame.getData();
		}
		

		if (depthFrame.isValid())
			pDepthMap = (const OniDepthPixel*)depthFrame.getData();

		if (firstC<5) {
			firstC++;
			continue;
		}
		//////////////////////////////////////////////////////////////////////
		// Bilateral filtering
		// Prepare the arrays for bilateral filter
		for (int i=0; i<MAX_I;i++) 
		{
			// Set h_pDepth
			h_pDepth[i] = (float) pDepthMap[i];
			// Set validityMask at the same time
			h_validityMask[i] = ((pDepthMap[i]>MIN_DEPTH && pDepthMap[i]<MAX_DEPTH) ? true:false);
		}

		// Set the validityMask in the CUDA device
		copyHostToCUDA_bool(validityMask, MAX_I);

		// Bilateral filter
		bilateralFilterRGBA(h_pDepth, d_pDepth, h_pReal, d_pReal, d_gaussian, d_validityMask, XN_VGA_X_RES, XN_VGA_Y_RES, gaussian_delta, euclidean_delta, filter_radius, iterations, nthreads);

		//////////////////////////////////////////////////////////////////////
		displayDepthImage(d_pReal, d_dData, h_dData, d_validityMask, d_palette, isHeat, XN_VGA_X_RES, XN_VGA_Y_RES);
		
		displayImageMap(	h_rgbData,			// 
							pImageMap);

		faceDetector.setValidityMask(h_validityMask);

		// Detect the face
		bool faceDetected = faceDetector.detectFace(h_pReal);

		// Find the box containing the mask values
		maskBox = ( faceDetected ? faceDetector.getFaceBox() : nullBox);
		 
		// Extract the points on the face
		if (faceDetected) {
			// Compute the normal map on the face		
			// Extract the face
		
			for (int i = 0; i < MAX_I3; i++)
			{
				h_face[i] = 0;
			}

			extractFace(h_face, h_pReal, maskBox, h_validityMask, &nb_keyPoints, &NORMALIZATION_FACTOR, &CENTROID, first);

			{
				cimg = img;

				start = clock();
				std::vector<full_object_detection> shapes;

				faces = detector(cimg);
				
				if (faces.size() > 0){
					static int number = 0;
					full_object_detection shape = sp(cimg, faces[0]);
					shapes.push_back(shape);

					if (shapes[0].num_parts() == 68)
					{
						for (unsigned long i = 0; i < shapes[0].num_parts(); i++)
						{
							point &a = shapes[0].part(i);
							cv::Point2f sPoint2fs(a.x(), a.y());
							landmarkPts.push_back(sPoint2fs);
							if (i == 22)
							{
								eyebrow_x = a.x();
								eyebrow_y = a.y();
								eyebrow_y_real = a.y();
							}
						}

						process(R, T);
						FaceSegmentation sBorder(landmarkPts);

						findFaceCenter(h_pReal, &eyebrow_x, &eyebrow_y, NORMALIZATION_FACTOR, CENTROID.getXr(), CENTROID.getYr());
						float vals2[9] = { 1, 0, 0, 0, -1, 0, 0, 0,  1 };
						
						cv::Mat inv = R.inv();
						cv::Mat inv_T = -R.inv()*T;
						for (int i = 0; i < 3; i++)
						{
							for (int j = 0; j < 3; j++)
							{
								double temp = R.at<double>(i, j);
								double tmpe2 = inv.at<double>(i, j);
								R_matrix[i * 3 + j] = (float)temp;
							}
						}
						

						static int count = 0;
						static int check = 0;
							
						// segment face

						sBorder.setEyebrow(eyebrow_x, eyebrow_y_real);
						sBorder.deletePointUnderChin(h_face, MAX_I, CENTROID.getXr(), CENTROID.getYr(), CENTROID.getZr(),NORMALIZATION_FACTOR);
						sBorder.rearrangeArray(&h_face, MAX_I, nb_keyPoints);

						moveToFaceCenter(h_face, MAX_I, eyebrow_x, eyebrow_y);
						applyRotation(h_face, MAX_I, vals2);
						applyInvRotation(h_face, MAX_I, R_matrix);
						applyRotation(h_face, MAX_I, vals2);
							
						cout << "frame " << number++  << endl;

						// Extract the points
						selectRandomPointOnFace(selectedPoints, nb_keyPoints-1, NB_PTS_X);
						// Demean/Normalize face for EM-ICP
						//// The centroid is the actual centroid, the normalization factor is the one of the first frame
						setH_X(h_X, NB_PTS_X, selectedPoints, h_face);
						
					}
					else
						nb_keyPoints = 0;
				}
				else nb_keyPoints = 0;

				shapes.clear();
				faces.clear();
				landmarkPts.clear();
			}
		} 
	
		else nb_keyPoints = 0;

		//////////////////////////////////////////////////////////////////////
		// Compute the pose
		// Find the mode
		previous_mode = mode;
		if (a[1]<-M_SWITCH) {
			mode = M_LEFT;
		}	else if (a[1]>M_SWITCH) {
			mode = M_RIGHT;
		}	else	{
			mode = M_FRONT;
		}
		
		// Update Ysize and h_Y depending on the mode
		if (previous_mode != mode)
			allocateMemory=true;
		switch (mode) {
			case M_LEFT:
				h_Y	= h_Yleft;
				break;
			case M_RIGHT:
				h_Y	= h_Yright;
				break;
			default:
				h_Y = h_Yfull;
				break;	
		}
		
		///////////////////////////////////////////////////////////////

		// Apply EM-ICP
		if (Xsize>0 && Ysize>0 && !first)
			poseEstimation(	Xsize, Ysize, 
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
							a, pre_a, pre_t,
							&allocateMemory,
							&diff);

		if (first && (nb_keyPoints>0 )) {
			first = false;

			swapPoints(h_X,h_Yfull,NB_PTS_X);

			int nb_l =0, nb_r=0;
			int *i_l = new int[nb_keyPoints];
			int *i_r = new int[nb_keyPoints];
			for (int i=0; i<nb_keyPoints; i++) {
				if (h_face[i] < 0.0f) {
					i_l[nb_l++] = i;
				} else {
					i_r[nb_r++] = i; 
				}
			}			

			selectRandomPointOnFace(selectedPoints, i_l, nb_l, NB_PTS_X);
			setH_X(h_Yleft, NB_PTS_X, selectedPoints, h_face);

			selectRandomPointOnFace(selectedPoints, i_r, nb_r, NB_PTS_X);
			setH_X(h_Yright, NB_PTS_X, selectedPoints, h_face);

			delete[] i_l;
			delete[] i_r;


			// Remember the first centroid
			model.setImFront(img);
			model.setCFront(CENTROID);
		} 

		//////////////////////////////////////////////////////////////////////
		//	Check that the transformation is possible
		diff  = tr_diff(h_t, pre_t, a, pre_a);
		stuck = ((diff ==0.0f) ? stuck+1: 0);

		/////// 
		if (stuck>STUCK_LIM || diff>90.0f) { 
			allocateMemory=true;
			h_t[0]=h_t[1]=h_t[2]=pre_t[0]=pre_t[1]=pre_t[2]=h_R[1]=h_R[2]=h_R[3]=h_R[5]=h_R[6]=h_R[7]=a[0]=a[1]=a[2]=pre_a[0]=pre_a[1]=pre_a[2]=0.0000001f;
			h_R[0]=h_R[4]=h_R[8]=1.0f;
			stuck=0;
		} 

		///////////////////////////////////////////////////////////////
		// Update the model
		if (updateModel && faceDetected)
		{ 
			wrongRegistration = false;
			model.model(nb_keyPoints, 
						h_face, 
						h_R, 
						h_t, 
						&wrongRegistration,
						nb_frames++);
		}
		
			
		///////////////////////////////////////////////////////////////
		// Display the face tracker
		if (displayFaceTracker) {
			if (faceDetected) {
				if (IS_HR && !displayDepth) 
					cvRectangle(currentImg, cvPoint(faceDetector.getFaceBox().getLeftX()*2+ALIGN_X,faceDetector.getFaceBox().getTopY()*2+ALIGN_Y),
							cvPoint(faceDetector.getFaceBox().getRightX()*2+ALIGN_X,faceDetector.getFaceBox().getBottomY()*2+ALIGN_Y),CV_RGB(255,0,0),2,8,0);
				else
					cvRectangle(currentImg, cvPoint(faceDetector.getFaceBox().getLeftX(),faceDetector.getFaceBox().getTopY()),
							cvPoint(faceDetector.getFaceBox().getRightX(),faceDetector.getFaceBox().getBottomY()),CV_RGB(255,0,0),2,8,0);
			}
		}

		
		if (!wrongRegistration && nb_keyPoints>0 && saveForImprovement) 
		{
			// Save the frames 
			char p[100];
			FILE *f;
			sprintf(p, ".\\frames\\n3D_%d.obj", nb_frames++);

			// Save the depth map
			f = fopen(p, "w");

			fprintf(f, "# number of points\n# %d\n\n# rotation matrix\n# %f %f %f\n# %f %f %f\n# %f %f %f\n\n# translation matrix\n# %f %f %f\n\n# normalization factor\n# %f\n\n# centroid\n# %f %f %f\n\n", 
				nb_keyPoints,
				h_R[0], h_R[1], h_R[2], 
				h_R[3], h_R[4], h_R[5],
				h_R[6], h_R[7], h_R[8],
				h_t[0], h_t[1], h_t[2],
				NORMALIZATION_FACTOR,
				CENTROID.getXr(), CENTROID.getYr(), CENTROID.getZr());

			int	l = maskBox.getLeftX(),
				r = maskBox.getRightX(),
				t = maskBox.getTopY(),
				b = maskBox.getBottomY();

			int xx=0, yy=0;
			int begin	= t*XN_VGA_X_RES + l;
			int end		= (b+CHIN_ADD)*XN_VGA_X_RES + r;
			xx=l; yy=t;

			
			// Restrict the skin mask to the face area
			for (int i=begin; i<end; i++) 
			{
				if (xx>=l 	&&	xx<=r 	&&	yy>=t	&&	yy<=(b+CHIN_ADD) ) 
				{
					if (h_validityMask[i]) 
					{
						float Xr=(h_pReal[i]-CENTROID.getXr())/NORMALIZATION_FACTOR			-h_t[0];
						float Yr=(h_pReal[i+MAX_I]-CENTROID.getYr())/NORMALIZATION_FACTOR	-h_t[1];
						float Zr=(h_pReal[i+MAX_I2]-CENTROID.getZr())/NORMALIZATION_FACTOR	-h_t[2];

						float nXr=h_pNormals[i];
						float nYr=h_pNormals[i+MAX_I];
						float nZr=h_pNormals[i+MAX_I2];
						
						fprintf(f, "v %f %f %f\nvn %f %f %f\n", 
							-( h_R[0]*Xr + h_R[3]*Yr + h_R[6]*Zr ), 
							( h_R[1]*Xr + h_R[4]*Yr + h_R[7]*Zr ), 
							-( h_R[2]*Xr + h_R[5]*Yr + h_R[8]*Zr ) + CYLINDER_Z, 
							( h_R[0]*nXr + h_R[3]*nYr + h_R[6]*nZr ), 
							-( h_R[1]*nXr + h_R[4]*nYr + h_R[7]*nZr ), 
							( h_R[2]*nXr + h_R[5]*nYr + h_R[8]*nZr ));
					}	
				}

				// Update the line/column values
				if (++xx == XN_VGA_X_RES) 
				{
					xx=0;
					yy++;
				}
			}
		}

		fclose(f);

		// End of the timer
		end = clock();
		cpuTime = (float)( end-start )/(float)( CLOCKS_PER_SEC ) ;
		float fps = 1/cpuTime;
		
		// Update the model image
		updatingModel(displayDepth, depthM, imM, mdData);

		// Display the info
		displayInfo(currentImg, saveFrames, displayDepth, fps);

		// Show the image
		cvShowImage("img", currentImg);

		//////////////////////////////////////////////////////////////////////
		// Key options for display		

		char key = cvWaitKey(20);

		if (key == 'q' || key == 'Q' || key == 27)
			break;

		if (key=='t' || key=='T') {
			displayFaceTracker=!(displayFaceTracker);
		}

		if (key=='d' || key=='D') {
			displayDepth=!(displayDepth);
			currentImg = (displayDepth ? depth:img);
			RScale=((!displayDepth && IS_HR) ? 3.0f:1.5f);
		}

		if (key=='r' || key=='R') {
			first=true;
			allocateMemory=true;
			init_RT(h_R, h_t);
			for (int i=0; i<3; i++)
				pre_t[i]=a[i]=pre_a[i]=0.0f;
			faceDetector.setFaceBox(nullBox);
			faceDetector.setPreviousFaceBox(nullBox);	
			model.reset();
			nb_frames=0;

			frame_index=0;
			session++;
		}

	}	

	if (isRecord && argc <= 1)
	{
		recorder.stop();
	}
	cvDestroyWindow( "img" );
	//cvDestroyWindow( "model depth" );
	//cvDestroyWindow( "normal map" );
	cvReleaseImage(&img);
	cvReleaseImage(&depth);
	//cvReleaseImage(&normalImg);


	if (input_mode == MODE_ONLINE)
	{
		model.saveToOBJFile(".\\model\\mymodel.obj", saveMesh, NORMALIZATION_FACTOR);
	}
	else
	{
		string sModelName = string(argv[1]);
		
		sModelName = sModelName.substr(0, sModelName.length() - 3);
		string sModelPath = ".\\model\\" + sModelName + "obj";
		// Save the model
		model.saveToOBJFile((char *)sModelPath.c_str(), saveMesh, NORMALIZATION_FACTOR);
	}

	//// Normals computation

	float *h_mNormals = new float[MAX_IMG_INDEX_3];
	//mallocCUDA(mNormals, MAX_IMG_INDEX_3);
	// Compute h_pNormals

	//
	
	computerNormals(model, h_mNormals);

	initEngineRGB();
	initEngineXYZ();
	initEngineNormals();
	initEngineMesh(model.getNbMesh());
	setNbEngineMesh(model.getNbMesh());
	setEngines(model, h_mNormals);

	EngineInit();
	EngineCameraSetup(4.0f);
	
	InitPointCloud(model.getHmodelXYZ(), MAX_IMG_INDEX_1, model.getH_defined_C(), param.points1);
	
	while(EngineIteration(MAX_IMG_INDEX_1, param.points1, model.getHmodelXYZ(),  h_R, h_t)); // PointCloudViewer
	freeEngineRGB();
	freeEngineXYZ();
	freeEngineNormals();
	freeEngineMesh();
	
	cvSaveImage(".\\model\\modeltexture.jpg", model.getImFront());

	//////////////////////////////////////////////////////////////////////
	//	Free memory
	if( !param.noviewer ){
		delete[] param.points1;
		delete[] param.points2;
		delete[] param.points3;
		EngineShutDown();
	}
	cublasShutdown();
	
	//*******************************************************
	//OpenNI::shutdown();
	//*******************************************************

	delete[] m_streams;
	delete[] h_X;
	//delete[] h_Y;
	delete[] h_Yfull;
	delete[] h_Yleft;
	delete[] h_Yright;

	delete[] a;
	delete[] pre_a;
	delete[] pre_t;
	//delete[] keyPoints;
	delete[] selectedPoints;	
	
	releaseCUDAandHost(monkey_1);
	releaseCUDA(monkey_2);
	// OpenNI General CUDA
	releaseCUDAandHost(pDepth);
	releaseCUDAandHost(pReal);
	releaseCUDAandHost(face);
	releaseCUDA(gaussian);
	releaseCUDAandHost(validityMask);
	delete[] h_mNormals;
	
		
}// modelling

	releaseCUDA(rgbData);
	releaseCUDA(dData);
	releaseCUDA(palette);
	//releaseCUDA(nData);
	releaseCUDA(R);
	releaseCUDA(t);
	releaseCUDA(S);
	releaseCUDA(Xc);
	releaseCUDA(Yc);
	releaseCUDA(X);
	releaseCUDAandHost(one);
	releaseCUDA(C);
	releaseCUDA(lambda);
	releaseCUDA(Y);
	releaseCUDA(Xprime);
	releaseCUDA(YCenterd);
	releaseCUDA(A);
	killCUDAthread();
	
	return 0;
}
/**********************/
