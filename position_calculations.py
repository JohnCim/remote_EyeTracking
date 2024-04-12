import matplotlib
import math
import numpy as np
from numpy import linalg, dot

def angleZ(v1):


    #u = [1,0,0]
    #v = [math.sqrt(1+v1[2]**2),0,v1[2]]

   # zangle  = (math.acos(v[0]/math.sqrt(v[0]**2 + v[2]**2)))
    try:
        zAngle = math.asin(v1[2])
    except:
        zAngle = math.asin(v1[0,2])
    else:

        zAngle = math.asin(v1[2])
    return zAngle

def angleY(v):
    try:
        yangle = math.asin(v[1])
    except:
        yangle = math.asin(v[0, 1])
    else:

        yangle = math.asin(v[1])


    #yangle =  np.arcsin(v[1])

    return yangle
def Rx(angle):
    R_x = np.matrix([[1, 0, 0], [0, math.cos(angle), -math.sin(angle)], [0, math.sin(angle), math.cos(angle)]])
    return R_x
def Ry(angle):

    R_y = np.matrix([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
    return R_y

def Rz(angle):
    R_z = np.matrix([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    return R_z
def thetaAngles(vecs):


    thetas = np.arccos(vecs[0]/math.sqrt(vecs[0]**2 + vecs[1]**2))
    #print('Theta = %.4f \nend'%(thetas*180/math.pi))
    return thetas
def alphaAngles(vecs):

    alpha = -np.arccos(vecs[0])  # * 180 / math.pi
    #print('alpha = %.4f \nend' % (alpha * 180 / math.pi))
    return alpha

def angles(vecs):
    theta1= np.arccos(vecs[0]/math.sqrt(vecs[0]**2 + vecs[1]**2)) #* 180 / math.pi


    alpha1 = -np.arccos(vecs[0]) #* 180 / math.pi
    beta1 = -np.arcsin(vecs[0])  #* 180 / math.pi
    #beta1 = alpha1-math.pi/2
    #print(alpha1,beta1)
    return theta1, alpha1, beta1


def frameTrans(vec, delta_theta,delta_alpha):
    # vecs = np.subtract(vec,center)

    # print(theta)
    # print(alpha)
    # print(beta)
    delta_beta = delta_alpha-math.pi/2

    Rx = np.matrix([[1, 0, 0], [0, math.cos(delta_beta), -math.sin(delta_beta)], [0, math.sin(delta_beta), math.cos(delta_beta)]])
    Ry = np.matrix([[np.cos(delta_alpha), 0, np.sin(delta_alpha)], [0, 1, 0], [-np.sin(delta_alpha), 0, np.cos(delta_alpha)]])
    Rz = np.matrix([[np.cos(delta_theta), -np.sin(delta_theta), 0], [np.sin(delta_theta), np.cos(delta_theta), 0], [0, 0, 1]])
    dvec = unitVector(vec*Rz*Ry*Rx)

    # print(Rx)
    # print(Ry)
    # print(Rz)
    # print(R)
    return dvec





def rotationM(vecs):
    #vecs = np.subtract(vec,center)
    theta,alpha,beta= angles(vecs)
    #print(theta)
    #print(alpha)
    #print(beta)
    R_x,R_y,R_z = Rx(beta), Ry(alpha), Rz(theta)
    #Ry = np.matrix([[np.cos(alpha), 0 , np.sin(alpha)],[0 , 1 , 0], [-np.sin(alpha), 0 , np.cos(alpha)]])
    #Rz = np.matrix([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta), 0] , [0,0,1]])


    #print(Rx)
    #print(Ry)
    #print(Rz)
    #print(R)
    return  R_z,R_y,R_x
def transformR(goalV,targetV):
    # error debugger
    goalV = dataException(goalV)
    targetV = dataException(targetV)
    #goalV = np.subtract(goalV)
    #if  sum(goalV) == 0 or np.isnan(sum(targetV)) == True:
    #    raise TypeError("shit fucked")
    #v_goal= dataException(unitVector(goalV))
    #v_target = dataException(unitVector(targetV))
    #vec_tg = dataException(   v_target)
    #print(unitVector(vec_tg))
    #theta, alpha, na = angles(goalV)
    theta = math.acos(targetV[3])#thetaAngles(goalV)

    alpha = alphaAngles(targetV)

    #Rz,Ry,Rx = rotationM(v_goal)
    R_y, R_z = Ry(alpha), Rz(theta)



    # throws error
    if theta == 0 or np.isnan(theta) == True:
        raise TypeError("shit fucked")
        #print(theta)
    #else:
        #b=1
        #print(theta*180/pi)
    #print('current head angles theta= %f, alpha = %f\n' % (theta*180/math.pi, alpha*180/3.14))
    targetV = [1,0,0]
    v_transform = unitVector(targetV) *R_z*R_y

    print(v_transform)
    return v_transform

def unitVector(v):


        uV = v/  np.linalg.norm(v)

        return uV
def angleVector(v,u):

    vectAng = math.acos(np.dot(v,u)/np.sqrt(np.sum(np.power(u,2) * np.power(v,2))))

    return vectAng

def dataException(v):
    if np.sum(v) == 0 or np.isnan(np.sum(v)) == True:

        #print(v)
        raise TypeError("shit fucked")

    else:

        return v



#u = np.zeros([5,3])
#u[1,:] = unitVector([1,1,1])
#leftEye = u[1,:]+ [5/

#u[2,:] = 2
#u[3,:] = 3
#u[4,:] = 4
#theta,zeta = angles(u[1,:])
#dv = [np.zeros([5,3])]
