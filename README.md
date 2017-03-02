# irisfaceRGBD




# 3D Face Recogntion

## This is a 3D face recognition software. 
Dependencies: python 2, caffe (http://caffe.berkeleyvision.org/), numpy, scipy, sklearn
Input: 3D face of point clouds (.ply format) in Probe and Gallery folder
Output: Similarities between probe and gallery


### Step 1. Convert a 3D point cloud (.ply) into 2D depth map (.npy)

Run follwoing command to generate a 2D depth map in ./3DFace/Probe and ./3DFace/Gallery folder
```
python Preprocessing.py
```

### Step 2. Measure similarities between probe and gallery set

Run follwoing command to calculate similaritie
```
python Recognition.py
```


