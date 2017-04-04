//-----------------------------------------------------------------------------------------
//	This file is part of the "USC IRIS 3D face modeler" 
//      developed at the University of Southern California by
//      Donghyun Kim, Matthias Hernandez, Jongmoo Choi, Gerard Medioni, 
//-----------------------------------------------------------------------------------------
//      Copyright (c) 2012 University of Southern California.  All Rights Reserved.
// face land mark detect using dlib.


#include <dlib/opencv.h>
#include <OpenNI.h>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <opencv2/core/types_c.h>


using namespace dlib;
using namespace std;


