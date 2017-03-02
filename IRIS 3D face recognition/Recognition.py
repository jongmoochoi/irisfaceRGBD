import caffe
import numpy as np
import scipy.misc
from os import listdir
from os.path import isfile, join
import time
from sklearn.metrics.pairwise import cosine_similarity
import sys


def getFeatures(model, adata):
    sFeatures = []
    for i in range(0,len(adata)):
        img = adata[i]
        net.blobs['data'].data[...] = img
        out = net.forward()

        caffe_fc7 = net.blobs['fc7'].data[0].copy()
        sTemp_layer_output = caffe_fc7
        sTemp_layer_output= sTemp_layer_output.reshape(1, 4096)

        sTemp_layer_output = np.sqrt(sTemp_layer_output)
        sFeatures.append(sTemp_layer_output)
    sFeatures = np.array(sFeatures)
    sFeatures = sFeatures.reshape(len(sFeatures), 4096)
    return sFeatures

def getIdentificationAccuracy(model, aGallery, aGalleryLabel,aProb, aProbLabel):
    i = 0
    sGalleryFeature = getFeatures(model, aGallery)

    sProbFeature = getFeatures(model, aProb)

    minindex = []
    similarities = []
    max_similarities = []
    similaritiesMatrix = []

    for i in range(0, sProbFeature.shape[0]):
        if i == 0:
          t0 = time.clock()
        temp_max = -10
        temp_index = 0;
        temp_max = -1000

        measure = []

        for j in range(0, sGalleryFeature.shape[0]):
            temp_similarity = cosine_similarity(sProbFeature[i,:].reshape(1,-1), sGalleryFeature[j,:].reshape(1,-1))
            if temp_max < temp_similarity:
                temp_max = temp_similarity
                temp_index = aGalleryLabel[j]

            if aGalleryLabel[j] == aProbLabel[i]:
                similarities.append(temp_similarity)

            measure.append(temp_similarity)

        max_similarities.append(temp_max)
        minindex.append(temp_index)
        similaritiesMatrix.append(np.squeeze(np.array(measure)))


    minindex = np.array(minindex)
    results = (minindex == aProbLabel)
    accuracy = np.count_nonzero(results)/float(results.shape[0])

    return accuracy, results, minindex, max_similarities, similarities, np.array(similaritiesMatrix)

def rankAccuracy(sProbLabel, similaritiesMatrix , k):

    RankLabel = []
    for i in range(0,similaritiesMatrix.shape[0]):
        ind = np.argpartition(-(similaritiesMatrix[i,:]), k)[:k]
        if sProbLabel[i] in ind:
            RankLabel.append(True)
        else:
            RankLabel.append(False)

    RankLabel = np.array(RankLabel)
    return  np.count_nonzero(RankLabel)/float(similaritiesMatrix.shape[0])


def to_rgb1a(im):
    w, h = im.shape
    ret = np.empty((w, h, 3), dtype=np.float32)
    ret[:, :, 2] =  ret[:, :, 1] =  ret[:, :, 0] =  im
    return ret




sTarget = '.npy'

caffe.set_mode_cpu()
net = caffe.Net("VGG_FACE_deploy.prototxt", "Weights.caffemodel",  caffe.TEST)

print 'done'

X_Gallery = []
X_Probe = []

Y_Gallery = []
Y_Probe = []
###
fileNames = []
avg = np.array([37,37,37])

## Gallery Path
sGalPath = './3DFace/gallery'


dirs = [f for f in listdir(sGalPath) if isfile(join(sGalPath, f)) and (f.endswith(sTarget))]
N_id = len(dirs)
for i, n in enumerate(dirs):

    Y_temp = np.zeros(N_id)
    Y_temp[i] = 1
    X_temp = np.load(sGalPath + '/'+ n)
    X_temp = to_rgb1a(X_temp)
    image = scipy.misc.imresize(X_temp.astype('float32'), [224, 224])
    image = image - avg
    image = image.transpose((2, 0, 1))

    X_Gallery.append(image)
    Y_Gallery.append(Y_temp)



## Probe Path
sProbPath = './3DFace/probe'
dirs = [f for f in listdir(sProbPath) if isfile(join(sProbPath, f)) and (f.endswith(sTarget))]

N_id = len(dirs)

for i, n in enumerate(dirs):
    Y_temp = np.zeros(N_id)
    Y_temp[i] = 1
    X_temp = np.load(sProbPath + '/' + n)
    X_temp = to_rgb1a(X_temp)
    image = scipy.misc.imresize(X_temp.astype('float32'), [224, 224])
    image = image - avg
    image = image.transpose((2, 0, 1))
    X_Probe.append(image)
    Y_Probe.append(Y_temp)
    fileNames.append(n)


X_Gallery = np.array(X_Gallery)
print('Gallery data shape: ', X_Gallery.shape)
X_Probe= np.array(X_Probe)
print('Probe data shape: ', X_Probe.shape)
Y_Gallery = np.array(Y_Gallery)
Y_Probe = np.array(Y_Probe)

sGallerylabel = np.where(Y_Gallery == 1)
sGallerylabel = sGallerylabel[1]

sProbLabel = np.where(Y_Probe == 1)
sProbLabel = sProbLabel[1]

[results, label, min, max_similarities, similarities, similaritiesMatrix] = getIdentificationAccuracy(net, X_Gallery, sGallerylabel, X_Probe, sProbLabel)


print("rank-1 acc: ", results)


for i in range(0, len(fileNames)):
    print(fileNames[i], 'Probe Label:', sGallerylabel[i], 'Matched Label:',
          min[i], ' max similarity: ', max_similarities[i], 'ref similarity: ', similarities[i])
    print ('\n')


