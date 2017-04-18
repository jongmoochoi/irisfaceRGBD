/*
	setup.cpp
	contains IO and setup related functions

	author: Jatuporn Toy Leksut
	version 2.0 12/10/2014
*/

#include "facePoseEstimation.h"


// extern
void initExpr(const string configFile, const string input_dir);
int getFaceType(const vector<Point2f>& imgPts);
void adjustObjPtsBasedOnPose(int faceType, vector<Point3f>& out_objPts);

// intern
void streamMat(ifstream& in, int numRows, int numCols, MatrixXd& out_mat);
void readConfigFile(string configFile, json::Object& out_config);
void openstreamConfigFile(const string fileloc, ifstream& out_in);
void setConfigAll(const json::Object& config);
void setConfig(const json::Value& configName, const json::Value& configObj);
Glob omglob;

///////////////////////////////////////////////////////////////////

void initExpr(const string configFile)
{

	if(!verifyFile(configFile)) {
		printf("Failed to locate %s\n", configFile.c_str());
		exit(-1);
	}

	// set config
	json::Object config;
	readConfigFile(configFile, config);
	setConfigAll(config);


	Point3f leftmostPt = omglob.inputOuterLdmk3DPts[omglob.rigidLdmkIds[omglob.leftRigidLdmkId]];
	Point3f rightmostPt = omglob.inputOuterLdmk3DPts[omglob.rigidLdmkIds[omglob.rightRigidLdmkId]];
	omglob.modelFaceWidth = norm(leftmostPt - rightmostPt);
	printf("face width = %f\n", omglob.modelFaceWidth); 
}

/*
read config.json and return a json object
*/
void readConfigFile(const string configFile, json::Object& out_config)
{
	ifstream in(configFile.c_str());
	if(!in) {
		printf("Failed to load config file %s\n", configFile.c_str());
		exit(-1);
	}

	string jsonstring((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	
	json::Value configValue = json::Deserialize(jsonstring);
	if (configValue.GetType() == json::NULLVal) {
		printf("Config file is not a valid JSON representation. See %s\n", configFile.c_str());
		exit(-1);
	}
	out_config = configValue.ToObject();
}

/*
open input stream to read from config file
quit program if failed
*/
void openstreamConfigFile(const string fileloc, ifstream& out_in)
{
	out_in.open(fileloc.c_str());
	if(!out_in) {
		printf("Failed to load config file %s\n", fileloc.c_str());
		exit(-1);
	}
}

/*
set config based on config's id (first character of config's name)
*/
void setConfig(const json::Value& configName, const json::Value& configObj)
{
	int configId = atoi(configName.ToString().substr(0, configName.ToString().find("_")).c_str());
	ifstream in;
	int id;
	string temp;
	char c;
	json::Array jarr;
	
	switch(configId) {
	case 1: // numInputLdmks
		omglob.numInputLdmks = configObj.ToInt(); 
		break; 

	case 2: // ldmk_metadata
		openstreamConfigFile(configObj.ToString(), in);
		getline(in, temp); // skip first line

		omglob.inputInnerLdmkVertexIds.clear();
		omglob.inputOuterLdmkVertexIds.clear();
		omglob.keyLdmkIds.clear(); omglob.inputLdmksKeyTable.clear();
		omglob.rigidLdmkIds.clear(); omglob.inputLdmksRigidTable.clear();
		omglob.lrLdmkIdPairs.clear();
		omglob.ldmkTypeTable.clear();
		omglob.segmentTable.clear();
		omglob.sampleTable.clear();

		for(int i=0; i<omglob.numInputLdmks; i++) {
			// col 1, ldmk_id
			int ldmk_id;
			in >> ldmk_id;

			// col 2, inner ldmk vertex id
			in >> id;
			omglob.inputInnerLdmkVertexIds.push_back(id);

			// col 3, outer ldmk vertex id
			in >> id;
			omglob.inputOuterLdmkVertexIds.push_back(id);

			// col 4, key ldmk id
			in >> c;
			if(c == 'K') {
				omglob.keyLdmkIds.push_back(ldmk_id);
				omglob.inputLdmksKeyTable.push_back(omglob.keyLdmkIds.size()-1);
			}
			else {
				omglob.inputLdmksKeyTable.push_back(-1);
			}

			// col 5, rigid ldmk id
			in >> c;
			if(c == 'R') {
				omglob.rigidLdmkIds.push_back(ldmk_id);
				omglob.inputLdmksRigidTable.push_back(omglob.rigidLdmkIds.size()-1);
			}
			else {
				omglob.inputLdmksRigidTable.push_back(-1);
			}

			// col 6, rigid pivot id
			in >> c;
			if(c == 'L') { 
				omglob.leftRigidLdmkId = omglob.rigidLdmkIds.size() - 1; 
			}
			if(c == 'M') { 
				omglob.midRigidLdmkId = omglob.rigidLdmkIds.size() - 1; 
			}
			if(c == 'R') { 
				omglob.rightRigidLdmkId = omglob.rigidLdmkIds.size() - 1; 
			}

			// col 7, 8 left/right pair
			in >> c >> id;
			if(c == 'L') { 
				omglob.ldmkTypeTable.push_back(LEFT_LDMK);
				omglob.lrLdmkIdPairs.push_back(std::pair<int, int>(ldmk_id, id));
			}
			if(c == 'M') { 
				omglob.ldmkTypeTable.push_back(MIDDLE_LDMK);
				omglob.lrLdmkIdPairs.push_back(std::pair<int, int>(ldmk_id, id));
			}
			if(c == 'R') { 
				omglob.ldmkTypeTable.push_back(RIGHT_LDMK);
			}
		}

		// construct a ldmk-pair lookup table
		omglob.ldmkIdPairLookupTable.clear();
		for(int i=0; i<omglob.lrLdmkIdPairs.size(); i++) {
			int leftId = omglob.lrLdmkIdPairs[i].first;
			int rightId = omglob.lrLdmkIdPairs[i].second;
			omglob.ldmkIdPairLookupTable[leftId] = rightId;
			omglob.ldmkIdPairLookupTable[rightId] = leftId;
		}

		break;

	case 3: // refMesh
		loadMesh(configObj.ToString(), omglob.refMesh);
		break;

	case 5: // b0
		openstreamConfigFile(configObj.ToString(), in);
		int numVerts, numCols;
		in >> numVerts;
		in >> numCols;
		omglob.numVertices = numVerts;
		streamMat(in, numVerts*3, numCols, omglob.b0);
		break;


	default: // do nothing
		;
	}
	in.close();
}

/*
read and set matrix from input stream
*/
void streamMat(ifstream& in, int numRows, int numCols, MatrixXd& out_mat)
{
	out_mat = MatrixXd(numRows, numCols);
	for(int i=0; i<numCols; i++) {
		for(int j=0; j<numRows; j++) {
			in >> out_mat(j, i);
		}
	}
}

void setConfigAll(const json::Object& config)
{

	for (json::Object::ValueMap::const_iterator it=config.begin(); it!=config.end(); ++it) {
		setConfig(it->first, it->second);
	}
			
	// inputOuterLdmk3DPts: reference 3d landmark points on mesh
	omglob.inputOuterLdmk3DPts.clear();
	for(int i=0; i<omglob.numInputLdmks; i++) {
		double x, y, z;
		x = omglob.b0(omglob.inputOuterLdmkVertexIds[i]*3 + 0, 0);
		y = omglob.b0(omglob.inputOuterLdmkVertexIds[i]*3 + 1, 0);
		z = omglob.b0(omglob.inputOuterLdmkVertexIds[i]*3 + 2, 0);
		omglob.inputOuterLdmk3DPts.push_back(Point3d(x, y, z));
	}

	// inputInnerLdmkPts: helps adjust ldmks in case of severe occlusion of contour
	omglob.inputInnerLdmk3DPts.clear();
	for(int i=0; i<omglob.numInputLdmks; i++) {
		double x, y, z;
		x = omglob.b0(omglob.inputInnerLdmkVertexIds[i]*3 + 0, 0);
		y = omglob.b0(omglob.inputInnerLdmkVertexIds[i]*3 + 1, 0);
		z = omglob.b0(omglob.inputInnerLdmkVertexIds[i]*3 + 2, 0);
		omglob.inputInnerLdmk3DPts.push_back(Point3d(x, y, z));
	}


}

int getFaceType(const vector<Point2f>& imgPts)
{
	int leftLdmkId = omglob.rigidLdmkIds[omglob.leftRigidLdmkId]; 
	int midLdmkId = omglob.rigidLdmkIds[omglob.midRigidLdmkId];
	int rightLdmkId = omglob.rigidLdmkIds[omglob.rightRigidLdmkId];
	double left2midDis = norm(imgPts[leftLdmkId] - imgPts[midLdmkId]);
	double right2midDis = norm(imgPts[rightLdmkId] - imgPts[midLdmkId]);
	double left2rightDis = norm(imgPts[leftLdmkId] - imgPts[rightLdmkId]);

	double epsilon = 0.75;	// empirical

	// if M is between L and R
	if(left2rightDis > left2midDis && left2rightDis > right2midDis) {
		// if ratio is near frontal
		double ratio = (left2midDis > right2midDis) ? (right2midDis / left2midDis) : (left2midDis / right2midDis);
		if(ratio > epsilon) {
			return NEAR_FRONTAL_FACE;
		}
	}

	return (left2midDis > right2midDis) ? LEFT_FACE : RIGHT_FACE;
}

void adjustObjPtsBasedOnPose(int faceType, vector<Point3f>& out_objPts)
{
	vector<Point3f> temp(omglob.numInputLdmks);
	// *   | *
	//     _
	if(faceType == LEFT_FACE) {
		for(int i=0; i<omglob.lrLdmkIdPairs.size(); i++) {
			// left: Outer
			// right: Inner
			pair<int, int> lfPair = omglob.lrLdmkIdPairs[i]; 
			Point3f lPt(omglob.inputOuterLdmk3DPts[lfPair.first]);
			Point3f rPt(omglob.inputInnerLdmk3DPts[lfPair.second]);
			temp[lfPair.first] = lPt;
			temp[lfPair.second] = rPt;
		}
	}
	// * |   *
	//   _
	else if(faceType == RIGHT_FACE){
		for(int i=0; i<omglob.lrLdmkIdPairs.size(); i++) {
			// left: Inner
			// right: Outer
			pair<int, int> lfPair = omglob.lrLdmkIdPairs[i]; 
			Point3f lPt(omglob.inputInnerLdmk3DPts[lfPair.first]);
			Point3f rPt(omglob.inputOuterLdmk3DPts[lfPair.second]);
			temp[lfPair.first] = lPt;
			temp[lfPair.second] = rPt;
		}
	}
	else {
		for(int i=0; i<omglob.numInputLdmks; i++) {
			// all: Outer
			Point3f pt(omglob.inputOuterLdmk3DPts[i]);
			temp[i] = pt;
		}
	}

	// set
	out_objPts.clear();
	for(int i=0; i<omglob.numInputLdmks; i++) {
		out_objPts.push_back(temp[i]);
	}
}
