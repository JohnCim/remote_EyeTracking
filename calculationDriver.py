import numpy as np
import math
from txtUnpacker import txtUnpacker

from Extra.calculation_subfunctions import transform_driver,thetaAngles,alphaAngles,angleZ,angleY,dataException,Rz,Ry


def calculation_Driver(iter,n):

    x, y, z = txtUnpacker(iter, n)
    if iter == None:
        iter = len(x-1)

    eyeDict = {'neck_vec': [], 'theta': [],'alpha': [] , 'zAngles':[], 'yAngles': []}
    v1 = np.zeros([iter,3])
    v2 = np.zeros([iter,3])

    i_else = 0
    for i in range(0,iter):
        # initialize vectors at current iteration
        neck_vec_i = [x[i, 2], y[i, 2], z[i, 2]]
        eyeDict['neck_vec'].append(neck_vec_i)

            # calculate direction vectors of left, right eye in relation to head orientation frame
        for p in range(0,2):
            if p == 0:
                v = v1
            elif p == 1:
                v = v2

            eyeVec = [x[i, p], y[i, p], z[i, p]]

            try:
               # v[i, :] = dataException(transform_driver(neck_vec_i, eyeVec))
                v[i, :] = dataException(transform_driver(neck_vec_i, eyeVec))
                i_else = i

            except:
                ## TEST FOR V1/V0 when blinking or during error      #uncomment to test below
                #print('Error Raised: v%.f = '%(p))                  # test
                #print('%.f = iteration v1/v2 copies from'%i_else)   # test
                #print(v[i,:])                                       # test

                v[i, :] = v[i_else, :]  # transformR(neck_vec[i, :], leftEye_vec[i, :])


           # else:
                #print('working')
           #     v[i,:] =  (transform_driver(neck_vec_i, eyeVec))
           #     i_else = i


        eyeDict['theta'].append(thetaAngles(neck_vec_i))
        eyeDict['alpha'].append(alphaAngles(neck_vec_i))

        eyeDict['zAngles'].append(angleZ(v1[i,:]))
        eyeDict['yAngles'].append(angleY(v1[i,:]))
        # change in relation to delta

    return v1,v2,eyeDict
def orientation_driver(iter,n):

    v1, v2, eyeDict = calculation_Driver(iter, n)

    delta_theta = np.subtract(eyeDict['theta'], eyeDict['theta'][0])
    # print(delta_theta)
    delta_alpha =  np.subtract(eyeDict['alpha'], eyeDict['alpha'][0])
    # create dictionary of values for orientation of

    delta_dict = {'delta_y':[], 'delta_z':[] }
    orientation_dict = {'neck x-axis':[],'neck y-axis': [], 'neck z-axis': []}
    vL_new = np.zeros([len(v1)-1, 3])
    vR_new = np.zeros([len(v1)-1, 3])



    for i in range(1,len(delta_theta)):



        neck_vecjx = [1, 0, 0]  # unitVector(leftEye_vec[q])
        neck_vecjy = [0, 1, 0]
        neck_vecjz = [0, 0, 1]
        theta_Anglej = delta_theta[i - 1]
        alpha_Anglej = delta_alpha[i - 1]

        # change in orientation relative to initial frame
        Rzj = Rz(theta_Anglej)
        Ryj = Ry(alpha_Anglej)
        # change in orientation vectors
        neck_vecjx = (neck_vecjx * Rzj * Ryj)
        neck_vecjy = (neck_vecjy * Rzj * Ryj)
        neck_vecjz = (neck_vecjz * Rzj * Ryj)
        zAnglej = np.arcsin(neck_vecjx[0, 2])
        yAnglej = -np.arcsin(neck_vecjx[0, 1])

        # grab eye vectors at current iteration
        v1j = (v1[i])  # LEFT EYE
        # print(v1j)
        v2j = (v2[i])  # RIGHT EYE

        eyeLj = (v1j * Rzj * Ryj)  # CALCULATES LEFT AND RIGHT EYE NEW ORIENTATION AS EYE CHANGES
        eyeRj = (v2j * Rzj * Ryj)
        #print(eyeRj)
        # append into dictionaries
        orientation_dict['neck x-axis'].append(neck_vecjx)
        orientation_dict['neck y-axis'].append(neck_vecjy)
        orientation_dict['neck z-axis'].append(neck_vecjz)

        delta_dict['delta_y'].append(yAnglej*180/math.pi)
        delta_dict['delta_z'].append(zAnglej*180/math.pi)
        # modifies v1,v2 for new array
        vL_new[i-1,:] = eyeLj
        vR_new[i-1,:] = eyeRj
        #print(neck_vecjx)
    return vL_new,vR_new,delta_dict,orientation_dict








# test ###########################################################################

#vL_new, vR_new, delta_dict,orientation_dict= orientation_driver(None,0)
#print(vL_new)
#print(v1)
#print(v2)
#print(eyeDict['neck_vec'])

#plt.plot(eyeDict['neck_vec'])
#plt.plot(eyeDict['zAngles'])
#plt.plot(yAngles)
#plt.plot(zAngles)
#plt.show()