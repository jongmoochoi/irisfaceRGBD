/*
	cameraModule.cpp
	contains camera and pose estimation related functions

	author: Jatuporn Toy Leksut
	version 1.5 11/14/2014
*/

#include "facePoseEstimation.h"
#include "cameraModule.h"
#include <ctime>
// extern
void dummydisplay();


// intern
void runCameraCalibrationModule(const vector<Point2f>& imgPts, const vector<Point3f>& objPts, 
								int imgWidth, int imgHeight, Mat& out_R, Mat& out_T);
void calibrateCameraFromSingleView(const vector<Point2f>& imgPts, const vector<Point3f>& objPts, int imgWidth, int imgHeight, 
								 Mat& out_K, Mat& out_R, Mat& out_T);
void letsgo(Mat &R, Mat &T);

Mat img;
int imgWidth, imgHeight;
vector<Point2f> imgPts;
string filename;
extern std::vector<cv::Point2f> landmarkPts;
///////////////////////////////////////////////////////////////////

void dummydisplay()
{
}

void process(Mat &R, Mat &T)
{
	letsgo(R,T);
}

void letsgo(Mat &aR, Mat &aT)
{

	imgWidth = 640;
	imgHeight = 480;

	int faceType = getFaceType(landmarkPts);

	// ---------------------------------
	// Module 1: Camera Calibration
	// input: 2DPts, 3DPts, winWidth, winHeight
	// output: R, T
	Mat R, T;
	vector<Point2f> rigidImgPts;
	vector<Point3f> rigidObjPts;
	vector<Point3f> objPts;

	std::clock_t start;
	double duration;

	start = std::clock();

	adjustObjPtsBasedOnPose(faceType, objPts);
	for(int i=0; i<omglob.rigidLdmkIds.size(); i++) {
		rigidImgPts.push_back(landmarkPts[omglob.rigidLdmkIds[i]]);
		rigidObjPts.push_back(objPts[omglob.rigidLdmkIds[i]]);
	}

	runCameraCalibrationModule(rigidImgPts, rigidObjPts, imgWidth, imgHeight, R, T);

	//cout << "R: " << R << endl;
	//cout << "T: " << T << endl;
	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	aR = R;
	aT = T;
	//std::cout << "time: " << duration << '\n';
	// write output
	//writeOutputFiles(exprCoef, exprScore, v2pPts);
}

// ------------------------------------------
// Camera Calibration Module
// input: 2DPts, 3DPts, winWidth, winHeight
// output: R, T
// ------------------------------------------
void runCameraCalibrationModule(const vector<Point2f>& imgPts, const vector<Point3f>& objPts, 
								int imgWidth, int imgHeight, Mat& out_R, Mat& out_T)
{
	Mat K;
	calibrateCameraFromSingleView(imgPts, objPts, imgWidth, imgHeight, K, out_R, out_T);
}

/*
use PnP to solve for pose
*/
void calibrateCameraFromSingleView(const vector<Point2f>& imgPts, const vector<Point3f>& objPts, 
								   int imgWidth, int imgHeight, Mat& out_K, Mat& out_R, Mat& out_T)
{ 
	out_K = Mat(3, 3, CV_64F);
	out_R = Mat(3, 3, CV_64F);
	out_T = Mat(3, 1, CV_64F);

	// camera matrix K
	double flen = 0.5*min(imgWidth, imgHeight)/tan(M_PI/8.0); 
	double out_Karr[9] = {flen, 0, imgWidth/2.0, 
		0, flen, imgHeight/2.0,
		0, 0, 1.0};
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			out_K.at<double>(i, j) = out_Karr[i*3+j];
		}
	}

	Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
	Mat rvec, tvec;

	// PnP
	solvePnP(objPts, imgPts, out_K, distCoeffs, rvec, tvec, true, CV_EPNP);
	Rodrigues(rvec, out_R);
	tvec.copyTo(out_T);
}
