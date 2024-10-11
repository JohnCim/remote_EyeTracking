import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from txtUnpacker import txtUnpacker
from position_calculations import rotationM, transformR,angles,unitVector,frameTrans,angleVector,thetaAngles,alphaAngles,angleZ,angleY
from skspatial.objects import Sphere
from math import sin,cos,pi,sqrt
from time import perf_counter

from calculationDriver import calculation_Driver
def change_Angle(iter,n):
    dictionary = {'neck_theta': [], 'neck_alpha': [], 'eyeY': [], 'eyeZ': []}
    iters = 0
    for j in range(0,len(n)):

        v1,v2,angleDict = calculation_Driver(iter,n[j])
        if iter == None:
            iter = len(v1)
        #dictionary = {'neck_theta':[], 'neck_alpha':[], 'eyeY' :[], 'eyeZ': []}
        q = 0

        for i in range(0,iter):
            if i%5 == 0:
                #print(q) test

                q +=1

                if q == 1:
                    prev_it = i

                elif q==2:
                    dictionary['neck_alpha'].append(np.subtract(angleDict['alpha'][i],angleDict['alpha'][prev_it]))
                    dictionary['neck_theta'].append(np.subtract(angleDict['theta'][i], angleDict['theta'][prev_it]))
                    dictionary['eyeY'].append(np.subtract(angleDict['yAngles'][i], angleDict['yAngles'][prev_it]))
                    dictionary['eyeZ'].append(np.subtract(angleDict['zAngles'][i], angleDict['zAngles'][prev_it]))
                    q = 0
                    iters +=1
                    # test
                    #print(iters)
    return dictionary

#dicr = change_Angle(None,[0,1])



#print(len(dir['neck_theta']))