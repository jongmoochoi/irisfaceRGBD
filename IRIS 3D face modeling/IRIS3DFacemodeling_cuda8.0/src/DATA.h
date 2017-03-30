//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.


#define MODE_ONLINE 0
#define MODE_OFFLINE 1

////////////////////////////////////////////
//	Options

#ifndef IS_HR
	#define IS_HR	false//true
#endif
#ifndef NOVIEWER
	#define NOVIEWER true
#endif
#ifndef IS_GPU
	#define IS_GPU true
#endif

#ifndef NB_PTS_X
	#define NB_PTS_X	1100
#endif
#ifndef NB_PTS_X2
	#define NB_PTS_X2	2200//2*NB_PTS_X
#endif
#ifndef NB_PTS_X3
	#define NB_PTS_X3	3300//3*NB_PTS_X
#endif

#ifndef ICP_P2
	#define ICP_P2		0.01f
#endif
#ifndef ICP_PREC
	#define ICP_PREC	0.0001f//0.000001f//0.00001f
#endif
#ifndef ICP_RED
	#define ICP_RED		0.85f//0.92f
#endif
#ifndef ICP_D02
	#define ICP_D02		0.001f
#endif
#ifndef ICP_CUDA_BLOCK
	#define ICP_CUDA_BLOCK 512
#endif
#ifndef BLOCK_SIZE
	#define BLOCK_SIZE	32//16
#endif

#ifndef XtoZ
	#define XtoZ 1.114880018171494f//1.111466646194458f//
#endif
#ifndef YtoZ
	#define YtoZ 0.836160013628620f//0.833599984645844f//
#endif
#ifndef RADIUS_CUT
	#define RADIUS_CUT 150
#endif
#ifndef RADIUS_CUT_2
	#define RADIUS_CUT_2 22500//RADIUS_CUT*RADIUS_CUT
#endif
#ifndef THRESH_NORMALS
	#define THRESH_NORMALS 0.55f
#endif


////////////////////////////////////////////
// General stuff

#ifndef CHANNELS
	#define CHANNELS 3
#endif
#ifndef XN_VGA_X_RES
	#define XN_VGA_X_RES 640
#endif
#ifndef XN_VGA_Y_RES
	#define XN_VGA_Y_RES 480
#endif





#ifndef XN_HR_X_RES
	#define XN_HR_X_RES	(IS_HR?1280:640)
#endif
#ifndef XN_HR_Y_RES
	#define XN_HR_Y_RES	(IS_HR?1024:480)
#endif


#ifndef XN_HR_X_RES3
	#define XN_HR_X_RES3	(IS_HR?3840:1920)
#endif
#ifndef XN_HR_Y_RES3
	#define XN_HR_Y_RES3	(IS_HR?3072:1440)
#endif




#ifndef MAX_I
	#define MAX_I 307200//XN_VGA_X_RES*XN_VGA_Y_RES
#endif
#ifndef MAX_I2
	#define MAX_I2 614400//2*MAX_I
#endif
#ifndef MAX_I3
	#define MAX_I3 921600//3*MAX_I
#endif
#ifndef MAX_I9
	#define MAX_I9 2764800//9*MAX_I
#endif

#ifndef MAX_I_HR
	#define MAX_I_HR (IS_HR?1310720:MAX_I)//XN_HR_X_RES*XN_HR_Y_RES
#endif
#ifndef MAX_I_HR3
	#define MAX_I_HR3 (IS_HR?3932160:MAX_I3)//3*MAX_I_HR
#endif

#ifndef SIZE_CHANNEL_PALETTE
	#define SIZE_CHANNEL_PALETTE 256
#endif
#ifndef SIZE_CHANNEL_PALETTE_2
	#define SIZE_CHANNEL_PALETTE_2 512
#endif
#ifndef SIZE_PALETTE
	#define SIZE_PALETTE 768
#endif

#ifndef MONKEY_MIN
	#define MONKEY_MIN 4.0f//0.369494000000000f
#endif
#ifndef MONKEY_MAX
	#define MONKEY_MAX 6.0f//0.877386000000000f
#endif
#ifndef MONKEY_NORMALIZE
	#define MONKEY_NORMALIZE 0.007843137254902f//0.001991733333333f
#endif
#ifndef NB_RENDERING
	#define NB_RENDERING  2359
#endif
#ifndef NB_RENDERING2
	#define NB_RENDERING2 4718 //2*NB_RENDERING
#endif
#ifndef NB_RENDERING3
	#define NB_RENDERING3 7077 //3*NB_RENDERING
#endif
////////////////////////////
// For registration
#ifndef ALIGN_X
	#define ALIGN_X	6
#endif
#ifndef ALIGN_Y
	#define ALIGN_Y	32
#endif
/*
#define ANGLE_X	0.3f//21000f
#define ANGLE_Y	-0.0f//0280f//-0.001200f

#define cY cos(ANGLE_Y)
#define cX cos(ANGLE_X)
#define sY sin(ANGLE_Y)
#define sX sin(ANGLE_X)


#define CALIB_R0	cY
#define CALIB_R1	sX*sY
#define CALIB_R2	cX*sY
#define CALIB_R3	0.0f
#define CALIB_R4	cX
#define CALIB_R5	-sX
#define CALIB_R6	-sY
#define CALIB_R7	sX*cY
#define CALIB_R8	cX*cY

#define CALIB_T0	0.0f//3.85000f//3.785000f
#define CALIB_T1	100.0f//-3.88f//-5.88f
#define CALIB_T2	0.0f*/

//#define CALIB_T0	-0.90f

//#define CALIB_T1	-3.88f
//#define CALIB_T2	0.0f
//
//#define CALIB_R0	0.999997f
//#define CALIB_R1	-0.000052f
//#define CALIB_R2	-0.002499f
//#define CALIB_R3	0.000000f
//#define CALIB_R4	0.999780f
//#define CALIB_R5	-0.020998f
//#define CALIB_R6	0.002500f
//#define CALIB_R7	0.020998f
//#define CALIB_R8	0.999776f

/////////////////////////////
// For head detection
#ifndef HEAD_HEIGHT_MIN
	#define HEAD_HEIGHT_MIN 140//120
#endif
#ifndef HEAD_HEIGHT_MAX
	#define HEAD_HEIGHT_MAX 250
#endif
#ifndef HEAD_WIDTH_MIN
	#define HEAD_WIDTH_MIN	85
#endif
#ifndef HEAD_WIDTH_MAX
	#define HEAD_WIDTH_MAX	250
#endif
#ifndef LOOK_HEAD
	#define LOOK_HEAD 20
#endif


#ifndef MIN_DEPTH
	#define MIN_DEPTH 450
#endif
#ifndef MAX_DEPTH
	#define MAX_DEPTH 800//1250//850
#endif
#ifndef	NORMALIZE_DISPLAY
	#define NORMALIZE_DISPLAY 1.372549019607843f//(MAX_DEPTH-MIN_DEPTH)/255.0f;
#endif


#ifndef PI
	#define PI		3.14159265358979323846f
#endif
#ifndef PI_2
	#define PI_2	1.570796326794897f//PI/2
#endif
#ifndef _2PI
	#define _2PI	6.283185307179586f//2*PI
#endif
#ifndef DEG2RAD
	#define DEG2RAD 0.0174532925199433f//PI/180
#endif
#ifndef RAD2DEG
	#define RAD2DEG 57.29577951308232f//180/PI
#endif

////////////////////////////////////////////
// For face detection
#define THRESHOLD_YZ 20
#define THRESHOLD_XY 30
#define THRESHOLD_DIFF 15
#define SEARCHING_BOX 40
#define THRESHOLD_DEPTH 5
#define THRESHOLD_DISCONTINUITY 5
#define THRESHOLD_P	6
#define THRESHOLD_HEIGHT 10
#define SEARCHING_BOX_HEIGHT 20
#define SEARCHING_BOX_IN_DEPTH 5
#define T_RDIFF 30.0f

////////////////////////////////////////////
// For sampling
#ifndef T_TR_DIFF
	#define T_TR_DIFF 1.50f
#endif
#define CHIN_ADD 4//1
//#define CHIN_CUT 10



//////////////////////////////////////////
// For switching between method
#define M_FRONT	0
#define M_LEFT	1
#define M_RIGHT	2

#define NB_REF	4
#define REF_STEP 20.0f
#define M_SWITCH 10.0f//REF_STEP/2
#define R_20	0
#define L_20	1
#define R_40	2
#define L_40	36

#define STUCK_LIM	4

////////////////////////////////////////////
// For modeling
#define	THETA_MAX		360
#define THETA_MAX3		1080
#define	Y_MAX			200
#define Y_MAX3			600

#define	THETA_MID		180//THETA_MAX/2
#define	Y_MID			110//Y_MAX/2+10

#define	THETA_EXPAND	1.0f//THETA_MAX/360.0f
#define	Y_EXPAND		100.0f//Y_MAX/2.0f
#define	SHIFT_T			90.0f//90.0f*THETA_EXPAND

#define MAX_IMG_INDEX_1 72000//THETA_MAX*Y_MAX
#define MAX_IMG_INDEX_2 144000//MAX_IMG_INDEX_1*2
#define MAX_IMG_INDEX_3 216000//MAX_IMG_INDEX_1*3
#define MAX_IMG_INDEX_9 648000//MAX_IMG_INDEX_1*9

#ifndef CYLINDER_Z
	#define CYLINDER_Z		0.6f//0.0f//0.15f
#endif
#ifndef BILATERAL_WINDOW
	#define BILATERAL_WINDOW 1//3
#endif

#define SMOOTHING_T		1.5f

#define THETA_SWITCH_IMG_MIN	90//THETA_MAX*0.25//0.27
#define THETA_SWITCH_IMG_MAX	270//THETA_MAX-THETA_SWITCH_IMG_MIN


#define THETA_CUT	30//THETA_MAX/12//THETA_MAX/10
#define RAD_CUT		130
#define RAD_CUT_SQ	3900//RAD_CUT*RAD_CUT

#define THRESHOLD_OVERLAP	0.025f//0.075f//0.15f//0.075f//0.075f

#define RADIUS_PCA 3

#define RADIUS_PCA_MODEL 1//5
#define THRESHOLD_NORMALS 0.60f
#define THRESHOLD_CONFIDENCE_NORMALS 0.8f

#define THRESHOLD_UPDATE 0.1f





#define EUCLIDEAN_DELTA 0.010f//0.15f
#define GAUSSIAN_DELTA 3.0f
#define FILTER_RADIUS 3

#ifndef MAX_ON_FACE
	#define MAX_ON_FACE 35000
#endif
#ifndef MAX_ON_FACE2
	#define MAX_ON_FACE2 70000//MAX_ON_FACE*2
#endif
#ifndef MAX_ON_FACE3
	#define MAX_ON_FACE3 105000//MAX_ON_FACE*3
#endif


#define MIN_WEIGHT 0.01f//0.4f
#define RO_MAX 2.0f





#define THETA_C 50
#define THETA_L THETA_MID-THETA_C
#define THETA_R THETA_MID+THETA_C
#define BRADDIFF 2

// TODO: reference additional headers your program requires here
