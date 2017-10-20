# irisfaceRGBD
This file is part of the "USC IRIS 3D face modeling and recognition software" <br />
developed at the University of Southern California. <br />
Copyright (c) 2017 University of Southern California.  All Rights Reserved. <br />

# 3D Face Modeling

## This is a 3D face modeling software.
Description <br />
This Window library supports 3D face modeling with the PrimeSense or Kinect 1.0 camera. It supports functions of face modeling. 
There are two supporting versions: (1) Visual Studio 2013 and (2) 2015 Solutions (x64 Release)

Assumptions <br />
The program is for a single user at a time. The person should not stand farther than 1 meter away from the camera or closer than 40 cm to the camera. This version requires "frontal faces" as first input for face modeling, and it can cover pose changes.

Input: RGB-D frame from a low-cost depth sensor (e.g. Kinnect or PrimeSense)
Output: 3D face model (~/model/mymodel.ply)

## Structure
This package includes 4 main folders:
1.	IRIS3DFacemodeling_cuda8.0\Include_vs2013 or Include_vs2015: 3rd party libraries in Include folder. (e.g) dlib, openCV, OpenMesh 3.2, OpenNI2, pose estimation, pthreads-w23-2-9-1, boost 1.55.0, CUDA 8.0
2. IRIS3DFacemodeling_cuda8.0\dll_vs2013 or dll_vs2015: 3rd party dll files
3. IRIS3DFacemodeling_cuda8.0\src: Face modeling source code.
4. IRIS3DFacemodeling_cuda8.0\model: Face modeling results of OBJ and PLY format.
5. IRIS3DFacemodeling_cuda8.0\Release: Executable file of the project.

## 3rd party libraries

Visual Studio 2013, CUDA8.0, OpenCV 2.4.9, dlib 18.17, pthreads 2.9.1, boost 1.55, OpenMesh 3.2 </br>
Visual Studio 2015, CUDA8.0, OpenCV 2.4.9, dlib 18.17, pthreads 2.9.1, boost 1.59, OpenMesh 4.1

## How to build

1. Download the ZIP file of the whole package and extract.
2. Download all the 3rd party libraries. <br />
Openmesh: https://www.openmesh.org/download/ <br />
Download Openmesh 4.1 if you are using VS2015, 3.2 is you are using VS2013. <br />
Directly using Openmesh can create bunch of errors when building the project. We need to Cmake it before using. <br />
OpenCV 2.4.9: https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.9/opencv-2.4.9.exe/download <br />
dlib 18.17: https://sourceforge.net/projects/dclib/files/dlib/v18.17/dlib-18.17.zip/download <br />
pthreads 2.9.1: https://www.sourceware.org/pthreads-win32/ <br />
Follow the instruction of 'Download'. <br />
boost: https://sourceforge.net/projects/boost/files/boost/1.59.0/boost_1_59_0.zip/download <br />


3. Open the solution file(.sln) of the project in irisfaceRGBD-master\IRIS 3D face modeling. There are two solution files3., you only need the one which fit in the version of your Visual Studio. <br />
You may get this error when loading the project: XXX\irisfaceRGBD-master\IRIS 3D face modeling\IRIS3DFacemodeling_cuda8.0\IRIS3DFacemodeling_cuda8.0_vs2015.vcxproj(55,5): The imported project "XXX\MSBuild\Microsoft.Cpp\v4.0\V140\BuildCustomizations\CUDA 8.0.props" was not found. Confirm that the path in the <Import> declaration is correct, and that the file exists on disk. <br />
In this case, firstly make sure you have installed CUDA 8.0 successfully, then find the CUDA 8.0.pros file. The path of this file depends on where you installed CUDA 8.0. It is generally in C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0\extras\visual_studio_integration\MSBuildExtensions.
Copy all the 4 files to the BuildCustomizations folder where the error told you the CODA 8.0.props was not found. Reload the project.

4. Set the solution configurations and platforms as Release x64. Go to Project → Properties (or right click on 'IRIS3DFacemodeling_cuda8.0_vs2015' in the solution explorer, then click on Properties), make sure the active configuration and platform is Release and X64.

5. To add 3rd party libraries into the project, there are three project properties needed to be configurated. <br />

Go to Configuration Properties → C/C++ → General → Additional Include Directories. Include additional include directories. They are usually the 'include' folders under the libraries main folders. When you get errors like 'XXX.h not found' when building the project, it basically means you did not include the directory of this header(.h) file.
The screen shot below shows how the Additional Include Directories should look like. However the path before the libraries' name can differ from each user depending on where the libraries are extracted/installed.
<p align="center">
  <img src="./includedir.PNG" width="700"/>
</p> 

Go to Configuration Properties → Linker → General → Additional Library Directories. Include additional library directories.
These directories include all the .lib files of all the libraries, as shown below. Same as the cinlude directories, the path before the libraries' name can differ from each user depending on where the libraries are extracted/installed.
<p align="center">
  <img src="./includelibdir.PNG" width="700"/>
</p> 

Go to Configuration Properties → Linker → Input → Additional Dependencies. You can just copy from the .lib files' names below:

cublas.lib <br />
OpenNI2.lib <br />
OpenMeshCore.lib <br />
OpenMeshTools.lib <br />
cudart.lib <br />
libblas.lib <br />
liblapack.lib <br />
libf2c.lib <br />
kernel32.lib <br />
user32.lib <br />
gdi32.lib <br />
winspool.lib <br />
comdlg32.lib <br />
advapi32.lib <br />
shell32.lib <br />
ole32.lib <br />
oleaut32.lib <br />
uuid.lib <br />
odbc32.lib <br />
odbccp32.lib <br />
opencv_calib3d249.lib <br />
opencv_contrib249.lib <br />
opencv_core249.lib <br />
opencv_features2d249.lib <br />
opencv_flann249.lib <br />
opencv_gpu249.lib <br />
opencv_highgui249.lib <br />
opencv_imgproc249.lib <br />
opencv_legacy249.lib <br />
opencv_ml249.lib <br />
opencv_nonfree249.lib <br />
opencv_objdetect249.lib <br />
opencv_ocl249.lib <br />
opencv_photo249.lib <br />
opencv_stitching249.lib <br />
opencv_superres249.lib <br />
opencv_ts249.lib <br />
opencv_video249.lib <br />
opencv_videostab249.lib <br />

6. Click on Build → Build Solution. If the solution is built successfully, the .exe file of this project will be in irisfaceRGBD-master\IRIS 3D face modeling\x64\Release.

## Common Error
1. If you get error in pthread, then make this change in pthread.h:

2. Some OpenMesh errors occur when you are using Visual Studio 2015 SP3. If you are suing VS2015 SP3 and see the error: OpenMesh/Core/Mesh/PolyConnectivity.hh(86): error C2440: 'specialization': cannot convert from 'overloaded-function' to 'bool (__cdecl OpenMesh::ArrayKernel::* )(void) const' (compiling source file DataMeshItemCurvedPipe.cpp), you need to modify the error lines following: https://mailman.rwth-aachen.de/pipermail/openmesh/2016-June/001234.html

3. In the file IRIS3DFacemodeling_cuda8.0_vs2015.vcxproj\IRIS3DFacemodeling_cuda8.0_vs2015.vcxproj, line 200, the path of source.cpp is set as where it is in our computer. If you extracted dlib library into different folders, you may get error. To correct it, find the sourse.cpp file in your dlib library, and replace the original path in IRIS3DFacemodeling_cuda8.0_vs2015.vcxproj line 200 with where you find your source.cpp.

4. You are likely to get dll issues when running the exe file. To fix them, download all the files and folders from https://www.dropbox.com/sh/70ak0adcb4yn3ca/AABvEdf_zikznxZ96hxAB3dPa?dl=0, then copy them to irisfaceRGBD-master\IRIS 3D face modeling\x64\Release (same path with the exe file).

## How to use

After starting, the program will display the RGB frames in real time & detected face bounding box. This program also will display a modeled face in real time. Here are your options for other processes: <br />
•	Press key 'r' to reset a face modeling and start to make a new face modeling. The assumption of the program is that the first frame should be a frontal face. You can adequately choose a first frame which is a frontal face by resetting. <br />
•	Press key 'd' to switch between the depth and the RGB display. <br />
•	Press key 'q/ESC' to exit the program and produce the modeled face as OBJ and PLY file. The default output file is ./model/mymodel.obj, ./model/mymodel.ply <br />
To visualize your output, open it with MeshLab http://meshlab.sourceforge.net/

## Demo

1. Download the sample RGB-D video (https://drive.google.com/file/d/0B47nI8lp4t_CTm0zUjg1S3RXeW8/view?usp=sharing), and place the file in ~/IRIS3DFacemodeling_cuda8.0
2. Set command arguments as the name of the video file. 
<p align="center">
  <img src="./modelingdemo.PNG" width="700"/>
</p>
3. Run the program.
 - modeling process
<p align="center">
  <img src="./demo1.PNG" width="700"/>
</p>
 - modeling result
<p align="center">
  <img src="./demo2.PNG" width="700"/>
</p>

# 3D Face Recogntion
Kim, D., Hernandez, M., Choi, J. and Medioni, G., 2017. Deep 3D Face Identification. arXiv preprint arXiv:1703.10714. <br />

## This is a 3D face recognition software. 

Dependencies: python 2, caffe (http://caffe.berkeleyvision.org/), numpy, scipy, sklearn <br />
Pre-trained Weight (https://drive.google.com/file/d/0B47nI8lp4t_CTUVxc3Y4b29VR2c/view?usp=sharing) <br />
Input: 3D face of point clouds (.ply format) in Probe and Gallery folder  <br />
Output: Similarities between probe and gallery 


### Step 1. Convert a 3D point cloud (.ply) into 2D depth map (.npy)

Run follwoing command to generate a 2D depth map in ./3DFace/Probe and ./3DFace/Gallery folder
```
python Preprocessing.py
```

### Step 2. Measure similarities between probe and gallery set

Run follwoing command to calculate similarities
```
python Recognition.py
```
