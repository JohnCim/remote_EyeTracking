

from txtUnpacker import txtUnpacker as txt


class vector_organizer_tool:
    def __init__(self, x,y,z ):
        self.x = x
        self.y = y
        self.z = z
    def vector_Driver(self):

        neck_vec = np.multiply([self.x[ 2], self.y[ 2], self.z[ 2]],100)
        eye_vec_R = np.multiply([self.x[1], self.y[ 1], self.z[ 1]],100)

        eye_vec_L = np.multiply([self.x[ 0], self.y[0], self.z[0]],100)


        return neck_vec,eye_vec_R,eye_vec_L

import numpy as np
def data_organizer(iter, n):
    x,y,z = txt(iter,n)
    if iter == None:
        iter = len(x)
    neck_vec = np.zeros([iter,3])
    leftEye_vec = np.zeros([iter,3])
    rightEye_vec = np.zeros([iter,3])

    for i in range(0,iter):
        #print(x[i, :], y[i, :], z[i, :]) #test
        v = vector_organizer_tool(x[i,:],y[i,:],z[i,:])
        neckVec, eyeL_vec,eyeR_vec = v.vector_Driver()
        # unpack into array
        #print(neckVec) #test
        neck_vec[i,:] = neckVec

        leftEye_vec[i,:] = eyeL_vec
        rightEye_vec[i, :] = eyeR_vec
        #print(neckVec)
        #print(eyeL_vec)
        #print(eyeR_vec)
    #print('done')
    return neck_vec,leftEye_vec,rightEye_vec

#x,y,z = data_organizer(None,0)