/*
	main.h
	father of headers of all

	author: Jatuporn Toy Leksut
	version 2.2 04/27/2015
*/

#define FACE_MESH

#ifndef POSE_MAIN_H
#define POSE_MAIN_H
////////////////////////////////////////////////////////////////


// IO
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include "json.h" 

// containers
#include <vector>
#include <map>

// Boost
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

// math
#include <Eigen>

// OPENCV
#include <opencv2/opencv.hpp>


// OPENMESH
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>



using namespace std;
using namespace cv;
using namespace Eigen;
typedef OpenMesh::TriMesh_ArrayKernelT<>  Mesh;

#define LEFT_FACE 1
#define RIGHT_FACE 3
#define NEAR_FRONTAL_FACE 2

#define LEFT_LDMK 1
#define RIGHT_LDMK 3
#define MIDDLE_LDMK 2


//------------------------------------------------------------
//==============       GLOBAL VARIABLES       ================
//==============         (oh my glob)         ================
//------------------------------------------------------------

class Glob
{
public:

	// IO
	vector<string> img_inlist, ldmk_inlist;				// in list
	int counter;

	// config
	int numInputLdmks;
	string imgExt;	// image extension
	vector<int> inputOuterLdmkVertexIds, inputInnerLdmkVertexIds;
	vector<pair<int, int>> lrLdmkIdPairs;
	vector<int> rigidLdmkIds;		// indices of rigid landmarks (in the range of [0, numInputLdmks-1]
	vector<int> keyLdmkIds;			// indices of key landmarks used to compute blendshape weights (in the range of [0, numInputLdmks-1]
	size_t leftRigidLdmkId, midRigidLdmkId, rightRigidLdmkId; // ids based on the rigid ids [0, numRigidLdmks-1]
	map<int, int> ldmkIdPairLookupTable;
	vector<size_t> inputLdmksKeyTable, inputLdmksRigidTable;	// for looking up by ldmk_id
	vector<int> ldmkTypeTable; // look up landmark type (left, right, middle)
	vector<bool> sampleTable, segmentTable;
	double modelFaceWidth;
	
	// utils
	vector<string> fileList;	//only filename with no extension
	string inputDir, outputDir;

	vector<Point3d> inputOuterLdmk3DPts, inputInnerLdmk3DPts;
	MatrixXd b0, b0KeyLdmks;	// neutral shape
	int numVertices;	// number of vertices of a basis shape

	Mesh refMesh;

};


extern Glob omglob;
extern unsigned int fbo; 

//------------------------------------------------------------
//=================       MAIN FUNCs       ===================
//------------------------------------------------------------

// expressionModule.cpp
extern void dummydisplay();
extern void process();

//------------------------------------------------------------
//================       IO / SET UP       ===================
//------------------------------------------------------------

extern void initExpr(const string configFile);
extern void adjustObjPtsBasedOnPose(int faceType, vector<Point3f>& out_objPts);
extern int getFaceType(const vector<Point2f>& imgPts);


// utils.cpp
extern void scanDirForInputFiles(const std::string& input_dir, const vector<string>&  ext_list, vector<string>& out_filename_list);
extern bool loadImg(const string imgFile, Mat& out_img);
extern bool loadLdmks(const string ldmkFile, const Mat& img, vector<Point2f>& out_imgPts);
extern bool verifyDir(string pathstr);
extern bool verifyFile(string pathstr);
extern bool createDir(string pathstr);

//------------------------------------------------------------
//================    CAMERA CALIBRATION    ==================
//------------------------------------------------------------

// camera.cpp
extern void runCameraCalibrationModule(const vector<Point2f>& imgPts, const vector<Point3f>& objPts, 
								int imgWidth, int imgHeight, Mat& out_R, Mat& out_T);

////////////////////////////////////////////////////////////////
#endif

//------------------------------------------------------------
//==============      MESH PROCESSING       ===============
//------------------------------------------------------------

// mesh.cpp
extern void loadMesh(const string meshFile, Mesh& out_mesh);
