/*
	utils.cpp
	contains helper functions

	author: Jatuporn Toy Leksut
	version 1.5 11/14/2014
*/

#include "facePoseEstimation.h"

// do not declare any Eigen variable in here
// will result in directive conflict with boost
using namespace boost::filesystem;	


// extern
void scanDirForInputFiles(const std::string& input_dir, const vector<string>&  ext_list, vector<string>& out_filename_list);

bool loadLdmks(vector<Point2f>& out_imgPts);
bool verifyDir(string pathstr) ;
bool verifyFile(string pathstr);
bool createDir(string pathstr);

///////////////////////////////////////////////////////////////////

/*
	scan input directory for input files with given extensions
*/
void scanDirForInputFiles(const string& input_dir, const vector<string>&  ext_list, vector<string>& out_filename_list)
{
	if(!is_directory(input_dir)) {
		cout << "input_dir not found!" << endl;
		exit(-1);
	}

	path root_dir = input_dir;
	string search_ext = ext_list.front();
	out_filename_list.clear();

	// scan for input files
	directory_iterator end_itr; // default construction yields past-the-end
	for(directory_iterator itr(root_dir); itr != end_itr; ++itr)
	{
	    path itr_path = itr->path();
	    if(itr_path.extension() == search_ext) 
	    {
			// add to input list if file exists in all requested extensions
			bool good = true;
			for(int i=0; i<(int)ext_list.size(); i++) {
				path f_path = itr_path.parent_path()/(itr_path.stem().string()+ext_list[i]);
				good &= is_regular_file(f_path);
			}
			
			if(good) {
				out_filename_list.push_back(itr_path.stem().string());
			}
		}
	}
}

/*
load image into cv::Mat
*/

/*
load landmark (.pts) file into a vector of cv::Point2f
*/
bool loadLdmks(const string ldmkFile, const Mat& img, vector<Point2f>& out_imgPts)
{
	int imgWidth = img.cols;
	int imgHeight = img.rows;

	// read 2D ldmks
	ifstream in(ldmkFile.c_str()); 
	if(!in) {
		printf("Failed to load ldmk file: %s\n", ldmkFile.c_str());
		return false;
	}
	string line;
	getline(in, line); 
	getline(in, line);
	getline(in, line);
	out_imgPts.clear();
	for(int i=0; i<omglob.numInputLdmks; i++) {
		if(!getline(in, line)) {
			in.close();
			printf(" skip %s: bad landmark format\n", ldmkFile.c_str());
			return false;
		}
		float x, y;
		stringstream(line) >> x >> y;
		Point2f pt(x, y);
		out_imgPts.push_back(pt);

		// skip if landmark is out-of-frame
		if(x < 0 || x >= imgWidth || y < 0 || y >= imgHeight) {
			in.close();	
			printf(" skip %s: landmark out-of-frame\n", ldmkFile.c_str());	
			return false;
		}
	} 
	in.close();
	return true;
}

/*
verify whether a directory exists
*/
bool verifyDir(string pathstr) 
{
	return is_directory(path(pathstr));
}

/* 
verify whether a file exists
*/
bool verifyFile(string pathstr)
{
	return is_regular_file(path(pathstr));
}

/* 
create directory
*/
bool createDir(string pathstr)
{
	return create_directories(pathstr);
}

