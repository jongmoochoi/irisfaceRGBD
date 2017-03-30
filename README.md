# irisfaceRGBD
fdf
This file is part of the "USC IRIS 3D face modeling and recognition software"
developed at the University of Southern California by USC IRIS.
Copyright (c) 2017 fasf of Southern California.  All Rights Reserved.
dasda

# 3D Face Recogntion

## This is a 3D face recognition software. 
Dependencies: python 2, caffe (http://caffe.berkeleyvision.org/), numpy, scipy, sklearn <br />
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


# 3D Face Modeling

## This is a 3D face modeling software. 
Dependencies: dlib, pthreads, boost, opencv, OpenNI2, OpenMesh, Eigen, EM-ICP, CUDA, lapack, OpenCV
### Visual Studio 2013 and 2015 Solutions (x64 Release)
Visual Studio 2013, CUDA8.0, OpenCV 2.4.9, dlib 18.17, pthreads 2.9.1, boost 1.55, OpenMesh 3.2
Visual Studio 2015, CUDA8.0, OpenCV 2.4.9, dlib 18.17, pthreads 2.9.1, boost 1.59, OpenMesh 4.1

##
