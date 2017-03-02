
import numpy as np


def XYProjectedImage(vertsArr):


    dx = []
    dy = []
    index = []
    MAX_SIZE = 200 * 200
    weight_mean = np.zeros(MAX_SIZE)
    model = np.zeros(MAX_SIZE)
    THRESHOLD_UPDATE = 0.1
    MIN_WEIGHT = 0.01

    Y_EXPAND = 100
    X_EXPAND = 100

    for v in vertsArr:
        x = v[0]
        y = v[1]
        z = v[2]

        i_tf = X_EXPAND*(x + 1.0)
        i_yf = Y_EXPAND*(-y + 1.0)

        i_tc = np.floor(i_tf)
        i_yc = np.floor(i_yf)

        #weights
        temp_dx = abs(i_tf - i_tc)
        temp_dy = abs(i_yf - i_yc)
        dx.append(temp_dx)
        dy.append(temp_dy)
        temp_dx = 1.0 if (temp_dx >= 1.0) else temp_dx
        temp_dy = 1.0 if (temp_dy >= 1.0) else temp_dy

        index.append(i_yc*200 + i_tc)

    for vert, adx, ady, aInd in zip(vertsArr, dx, dy, index) :

        im0 = aInd
        im1 = im0 + 1
        im2 = im0 + 200
        im3 = im2 + 1

        w0 = abs((1-adx)*(1-ady))
        w1 = abs(adx*(1-ady))
        w2 = abs((1-adx)*ady)
        w3 = abs(adx*ady)
        ##
        v = vert[2] + 0.15

        if v > 0 :
            if (im0 < MAX_SIZE and w0 > MIN_WEIGHT):
                value = (weight_mean[im0]*model[im0] + w0*v) / (weight_mean[im0] + w0)
                model[im0] = value
                weight_mean[im0] += w0

            if (im1 < MAX_SIZE and w1 > MIN_WEIGHT):
                value = (weight_mean[im1]*model[im1] + w1*v) / (weight_mean[im1] + w1)
                model[im1] = value
                weight_mean[im1] += w1
            if (im2 < MAX_SIZE and w2 > MIN_WEIGHT):
                value = (weight_mean[im2]*model[im2] + w2*v) / (weight_mean[im2] + w2)
                model[im2] = value
                weight_mean[im2] += w2
            if (im3 < MAX_SIZE and w3 > MIN_WEIGHT):
                value = (weight_mean[im3]*model[im3] + w3*v) / (weight_mean[im3] + w3)
                model[im3] = value
                weight_mean[im3] += w3

    modelArr = np.array(model)
    modelArr = np.reshape(modelArr, (200,200))


    minZ = np.min(modelArr)
    modelArr = modelArr - minZ
    maxZ = np.max(modelArr)

    normalizeZ = maxZ

    modelArr = modelArr/normalizeZ
    return modelArr
