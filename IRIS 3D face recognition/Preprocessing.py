from __future__ import print_function

import numpy as np

from plyfile import PlyData, PlyElement
from XYProjection import XYProjectedImage
from numpy import cross, eye, dot
from scipy.linalg import expm3, norm
from os import listdir
from os.path import isfile, join


def M(axis, theta):
    return expm3(cross(eye(3), axis/norm(axis)*theta))

def distance(v1,v2):
    return sum([(x-y)**2 for (x,y) in zip(v1,v2)])**(0.5)


## Define Path

Paths = ['./3DFace/Probe', './3DFace/Gallery']

for sDefaultPath in Paths:

    sSavePath = sDefaultPath

    x0 = 0;
    y0 = 0;
    sDefaultPath = sSavePath

    dir = [f for f in listdir(sDefaultPath) if isfile(join(sDefaultPath, f)) and (f.endswith('.ply'))]

    for sFilePath in dir:

        sPointCloud = PlyData.read(sDefaultPath + '/' + sFilePath)
        sPointCloud = sPointCloud['vertex'][:]
        sPointCloudVerts = [list(row)[:3] for row in sPointCloud]
        sPointCloudVerts = np.array(sPointCloudVerts)

        normalizationFactor = 100.0
        normalizedVerts_ = []

        meanX = np.mean(sPointCloudVerts[:, 0])
        meanY = np.mean(sPointCloudVerts[:, 1])
        meanZ = np.mean(sPointCloudVerts[:, 2])

        for v in sPointCloudVerts:
            v[0] = (v[0] - meanX)/normalizationFactor
            v[1] = (v[1] - meanY)/normalizationFactor
            v[2] = (v[2] - meanZ)/normalizationFactor
            normalizedVerts_.append(v)


        sFileName = sSavePath + '/' + sFilePath[:-4] + ".npy"
        normalizedVerts = np.array(normalizedVerts_)

        sConverted = XYProjectedImage(normalizedVerts)
        np.save(sFileName, sConverted)
