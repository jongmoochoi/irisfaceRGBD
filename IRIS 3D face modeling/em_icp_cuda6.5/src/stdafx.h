//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//      Published: Laser Scan Quality 3-D Face Modeling Using a Low-Cost Depth Camera, EUSIPCO 2012.
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.


#pragma once
#pragma warning (disable : 4996)



#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <tchar.h>

// OpenNI
//#include <XnCppWrapper.h>
//#include <XnTypes.h>
#include <OpenNI.h>
using namespace  openni;
// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

// Defines
#include "DATA.h"
#include "cudaMem.h"
#include "FaceSegmentation.h"

// Levenberg-Marquardt
//#include <levmar.>h