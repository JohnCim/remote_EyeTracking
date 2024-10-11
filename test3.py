import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from txtUnpacker import txtUnpacker
from position_calculations import rotationM, transformR,angles,unitVector,frameTrans,angleVector,thetaAngles,alphaAngles,angleZ,angleY,dataException,Rz,Ry
from skspatial.objects import Sphere
from math import sin,cos,pi,sqrt
from skspatial.objects import Sphere
from time import perf_counter

from IPython.display import HTML

from IPython.display import HTML
iter = 9000
n = 0
x, y, z = txtUnpacker(iter, n)

# string1 = "Change in theta: % 02.2f | EyeAngle in Relation toX - axis: % .2f"
# string2 = "Change in alpha: % 01.2f | Eye Angle in Relation to X - axis: % .2f"

neck_vec = np.zeros([iter,3])
v1 = np.zeros([iter,3])
v2 = np.zeros([iter,3])
theta_vec = np.zeros([iter,1])
alpha_vec = np.zeros([iter,1])
leftEye_vec = np.zeros([iter, 3])
rightEye_vec = np.zeros([iter, 3])
zAngles = np.zeros([iter,1])
yAngles = np.zeros([iter,1])
#for i in range(0,iter):
  #  for j in range(0,3):
  #      if j == 0:
  #          # index in values into neck, left eye, and right eye direction vectors
  #         leftEye_vec[i,j] = x[i,0]
#
   #     elif j == 1:
   #         neck_vec[i, j] = y[i, 2]
   #         rightEye_vec[i, j] = y[i, 1]
   #         leftEye_vec[i, j] = y[i, 0]
   #     elif j == 2:
   #         neck_vec[i, j] = z[i, 2]
   #         rightEye_vec[i, j] = z[i, 1]
   #         leftEye_vec[i, j] = z[i, 0]
        # calculate direction vectors of left, right eye in relation to head orientation frame
   #     v1[i,:] = transformR(neck_vec[i,:], leftEye_vec[i,:])
   #     v2[i, :] = transformR(neck_vec[i, :], rightEye_vec[i, :])
   #     theta_vec[i] = thetaAngles(neck_vec[i,:])
   #     alpha_vec[i] = alphaAngles(neck_vec[i,:])
   #     zAngles[i] = angleZ(v1[i,:])
  #      yAngles[i] = angleY(v1[i,:])
    #print('yes')
    #print(theta_vec)
    # calculate change in head orientation, theta, and alpha
fig = plt.figure()
ax = plt.axes(projection='3d')

i_else = 0

for i in range(0,iter):
    for j in range(0,3):
        if j == 0:
            # index in values into neck, left eye, and right eye direction vectors
            neck_vec[i,j] = x[i, 2]
            rightEye_vec[i, j] = x[i, 1]
            leftEye_vec[i,j] = x[i,0]

        elif j == 1:
            neck_vec[i, j] = y[i, 2]
            rightEye_vec[i, j] = y[i, 1]
            leftEye_vec[i, j] = y[i, 0]
        elif j == 2:
            neck_vec[i, j] = z[i, 2]
            rightEye_vec[i, j] = z[i, 1]
            leftEye_vec[i, j] = z[i, 0]
        # calculate direction vectors of left, right eye in relation to head orientation frame
    for p in range(0,2):
        if p == 0:
            v = v1
            eyeVec = leftEye_vec
        elif p == 1:
            v = v2
            eyeVec = rightEye_vec
        else:
            raise IndexError
        try:
            v[i, :] = dataException(transformR(neck_vec[i, :], eyeVec[i, :]))
            #v2[i, :] = dataException(transformR(neck_vec[i, :], eyeVec[i, :]))
        except:
            ## TEST FOR V1/V0 when blinking or during error      #uncomment to test below
            #print('Error Raised: v%.f = '%(p))                  # test
            #print('%.f = iteration v1/v2 copies from'%i_else)   # test
            #print(v[i,:])                                       # test
            #print(thetaAngles(neck_vec[i, :]))

            v[i, :] =  v[i_else, :]#transformR(neck_vec[i, :], leftEye_vec[i, :])
            #v2[i, :] = v2[i_else, :] #transformR(neck_vec[i, :], rightEye_vec[i, :])
            #print(v[i,:])

            #theta_vec[i] = theta_vec[i-1]  #thetaAngles(neck_vec[i, :])
            #alpha_vec[i] = alpha_vec[i-1]  #alphaAngles(neck_vec[i, :])
            #zAngles[i] = zAngles[i-1] #angleZ(v1[i, :])
            #yAngles[i] = yAngles[i-1] #angleY(v1[i, :])
        else:
         #print(thetaAngles(neck_vec[i,:])*180/pi)

            v[i,:] = transformR(neck_vec[i,:], eyeVec[i,:])
            #v2[i, :] = transformR(neck_vec[i, :], rightEye_vec[i, :])

            i_else = i
            #print(i_else1)


    theta_vec[i] = thetaAngles((neck_vec[i,:]))

    #print(neck_vec[i,:])
    alpha_vec[i] = alphaAngles(neck_vec[i,:])
    zAngles[i] =  angleZ((leftEye_vec[i,:]))
    yAngles[i] = angleY(leftEye_vec[i,:])
delta_theta = np.subtract(theta_vec[:],theta_vec[0])
    #print(delta_theta)
delta_alpha= np.subtract(alpha_vec[:] ,alpha_vec[0])
print(delta_alpha)
new_vec = np.array([1,0,0])

sphere = Sphere([0, 0, 0], 0.4)
def init():
    ax.view_init(0, 90)

    ax.set_xlim3d([-1.2, 1.2])
    ax.set_xlabel('X')
    v1j = new_vec#leftEye_vec[0]

    # ax.set_ylim3d([output[1,1]*50, output[1,0]*2])
    ax.set_ylabel('Y')
    ax.set_ylim3d([-1.2, 1.2])
    # ax.set_zlim3d([-output[2,1]*2.5, output[2,0]*2])
    ax.set_zlim3d([-1.2, 1.2])
    ax.set_zlabel('Z')
    #eye1,= ax.plot3D([0, []], [(-0.2), ([]- 0.2)], [0, []], 'red')
    #eye2, ax.plot3D([0, []], [(0.2), ([] + 0.2)], [0, []], 'blue')
    ax.plot3D([0, v1j[0]], [(-0), (v1j[1] - 0)], [0, v1j[2]], 'red')
    sphere.plot_3d(ax, alpha=0.4)
    sphere.point.plot_3d(ax, s=100)
    ax.quiver(0, 0, 0, 0, 0, 0.75, length=0.75, normalize=True, color='green')
    ax.quiver(0, 0, 0, 0, 0.75, 0, length=0.75, normalize=True, color='green')
    ax.quiver(0, 0, 0, 0.75, 0, 0, length=0.75, normalize=True, color='green')
    #ax.quiver(0, 0, 0, 0, 0, 0.75, length=0.75, normalize=True, color='green')
    #ax.quiver(0, 0, 0, 0, 0.75, 0, length=0.75, normalize=True, color='green')

    ax.set_title('XZ Front View frame %d' % 0)
    #ax.plot_surface(X, Y, np.zeros_like(X), cmap='viridis')

def animate(q):




    ax.clear()
    ax.set_title('XZ Front View frame %d' % q)
    neck_vecjx = [1,0,0]#unitVector(leftEye_vec[q])
    neck_vecjy = [0,1,0]
    neck_vecjz = [0,0,1]
    zAnglej = delta_theta[q-1]
    yAnglej = delta_alpha[q-1]


    # TRANSFORMATION AXIS
    Rzj = Rz(zAnglej[0])
    Ryj = Ry(yAnglej[0])
   # Rzj2 = Rz(zAnglej2[0])

  #  Ryj2 = Ry(yAnglej2[0])
    neck_vecjx =  unitVector(neck_vecjx*Rzj*Ryj)
    neck_vecjy = unitVector(neck_vecjy * Rzj * Ryj)
    neck_vecjz = unitVector(neck_vecjz * Rzj * Ryj)

    #print((neck_vecjx))

    #print(neck_vecjy)
    #print(neck_vecjz)
    zAnglej = np.arcsin(neck_vecjx[0,2])


    #print(zAnglej)
    yAnglej =  -np.arcsin(neck_vecjx[0,1])

    # labels transformed x,y, z axis as y as head orientation changes from initial frame


    ax.text(neck_vecjx[0,0], neck_vecjx[0,1], neck_vecjx[0,2],s= "x' ", zdir=None, color='red')
    ax.text(neck_vecjy[0, 0], neck_vecjy[0, 1], neck_vecjy[0, 2],s= "y' ", zdir=None, color='red')
    ax.text(neck_vecjz[0, 0], neck_vecjz[0, 1], neck_vecjz[0, 2],s= "z'", zdir=None,color='red')

    # text for world frame orientation x,y,z
    ax.text(0.8, 0, 0, s="x ", zdir=None, color='green')
    ax.text(0,0.8,0, s="y ", zdir=None, color='green')
    ax.text(0, 0, 0.8, s="z ", zdir=None, color='green')
    neck_vecjx[:] = [1,0,0]#unitVector(leftEye_vec[q])
    neck_vecjy[:] = [0,1,0]
    neck_vecjz[:] = [0,0,1]
    #### Plot of transformed head frame
    ax.plot3D([0, 0.8*neck_vecjx[0, 0]], [0, 0.8*(neck_vecjx[0, 1])], [0, 0.8*neck_vecjx[0, 2]], 'red')
    ax.plot3D([0, 0.8*neck_vecjy[0, 0]], [0, 0.8*(neck_vecjy[0, 1])], [0, 0.8*neck_vecjy[0, 2]], 'red')
    ax.plot3D([0, 0.8*neck_vecjz[0, 0]], [0, 0.8*(neck_vecjz[0, 1])], [0, 0.8*neck_vecjz[0, 2]], 'red')
    ax.plot3D([0, 0.8 * x[q,0]], [0, 0.8 * (y[q,0])], [0, 0.8 * z[q,0]], 'purple')
    ax.plot3D([0, 0.8 * x[q, 1]], [0, 0.8 * (y[q, 1])], [0, 0.8 * z[q, 1]], 'purple')
    ax.plot3D([0, 0.8 * x[q, 2]], [0, 0.8 * (y[q, 2])], [0, 0.8 * z[q, 2]], 'black')
    # plots x and z angles of eyes ######################################
    ax.text2D(1, 1, "EyeAngle in Relation Z - axis: %2.2f | EyeAngle in Relation X - axis: %.2f" % (zAnglej * 180 / pi, yAnglej * 180 / pi), transform=ax.transAxes)

    # plot of world frame
    ax.quiver(0, 0, 0, 0, 0, 0.5, length=0.5, normalize=True, color='green')
    ax.quiver(0, 0, 0, 0, 0.5, 0, length=0.5, normalize=True, color='green')
    ax.quiver(0, 0, 0, 0.5, 0, 0, length=0.5, normalize=True, color='green')
    #####################################################################################
    ## for eye vectors ##################################################################

    # grab eye vectors at current iteration
    v1j = (v1[q]) # LEFT EYE
    #print(v1j)
    v2j = (v2[q])  # RIGHT EYE
    eyeLj = unitVector(v1j * Rzj * Ryj) # CALCULATES LEFT AND RIGHT EYE NEW ORIENTATION AS EYE CHANGES
    eyeRj = unitVector(v2j * Rzj * Ryj)
    eyeLj = (v1[q])  # LEFT EYE
    # print(v1j)
    eyeRj= (v2[q])  # RIGHT EYE
    #print(neck_vecjz*0.2)
    if q > 0:

        print(neck_vecjx)


    print(neck_vecjx[0,1])
    ax.plot3D([0, -neck_vecjy[0,0]*0.2], [0,-0.2*neck_vecjy[0,1]], [0, -neck_vecjy[0,2]*0.2], 'blue')
    ax.plot3D([0, neck_vecjy[0, 0] * 0.2], [0, 0.2 * neck_vecjy[0, 1]], [0, neck_vecjy[0, 2] * 0.2], 'blue')
    ax.quiver(-neck_vecjy[0,0]*0.2, -neck_vecjy[0,1]*0.2, -neck_vecjy[0,2]*0.2, eyeLj[0], eyeLj[1], (eyeLj[2]), length=0.9, normalize=True, color='red')
    ax.quiver(neck_vecjy[0, 0] * 0.2, neck_vecjy[0, 1] * 0.2, neck_vecjy[0, 2] * 0.2, eyeRj[0], eyeRj[1], (eyeRj[2]), length = 0.9, normalize = True, color = 'red')



    # sphere represents head ##########################################################################
    sphere.plot_3d(ax, alpha=0.4)
    sphere.point.plot_3d(ax, s=100)

    # EYES  #################################################################################################################################
    #sphereL = Sphere([eyeLj[0,0]-neck_vecjy[0,0],eyeLj[0,1],eyeLj[0,2]], 0.05)
    #sphereR = Sphere([eyeRj[0,0],eyeRj[0,1],eyeRj[0,2]], 0.05)

    #sphereL.plot_3d(ax, alpha=0.4)
    #sphereL.point.plot_3d(ax, s=100)
    #sphereR.plot_3d(ax, alpha=0.4)
    #sphereR.point.plot_3d(ax, s=100)






    # limits of x,y,z axis
    ax.set_xlim3d([-1.2, 1.2])
    ax.set_xlabel('X')

    #ax.set_ylim3d([output[1,1]*50, output[1,0]*2])
    ax.set_ylabel('Y')
    ax.set_ylim3d([-1.2, 1.2])
    #ax.set_zlim3d([-output[2,1]*2.5, output[2,0]*2])
    ax.set_zlim3d([-1.2, 1.2])
    ax.set_zlabel('Z')



ani = animation.FuncAnimation(fig, animate, init_func=init, frames=6000, interval=100)

plt.show()
