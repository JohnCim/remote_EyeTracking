
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from txtUnpacker import txtUnpacker
from position_calculations import rotationM, transformR,angles,unitVector,frameTrans,angleVector,thetaAngles,alphaAngles,angleZ,angleY,dataException,Rz,Ry
from skspatial.objects import Sphere
from math import sin,cos,pi,sqrt
from skspatial.objects import Sphere
from time import perf_counter
from calculationDriver import orientation_driver, calculation_Driver
from IPython.display import HTML
#def plotting_driver(iter, n):
iter = None
v1, v2, eyeDict = calculation_Driver(iter, 0)
vL_new, vR_new, delta_dict, orientation_dict = orientation_driver(iter, 0)
if iter == None:
    iter = len(vL_new)
fig = plt.figure()
ax = plt.axes(projection='3d')
sphere = Sphere([0, 0, 0], 0.4)

def init():
    ax.view_init(0, 90)

    ax.set_xlim3d([-1.2, 1.2])
    ax.set_xlabel('X')
    v1j = np.array([1,0,0])#leftEye_vec[0]

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


        yAnglej = delta_dict['delta_y'][q]
        zAnglej = delta_dict['delta_z'][q]


        ax.set_title('XZ Front View frame %d' % q)
        neck_vecjx = orientation_dict['neck x-axis'][q]
        neck_vecjy = orientation_dict['neck y-axis'][q]
        neck_vecjz = orientation_dict['neck z-axis'][q]

        #zAnglej = delta_theta[q-1]
        #yAnglej = delta_alpha[q-1]


        # TRANSFORMATION AXIS
    #Rzj = Rz(zAnglej[0])
    #Ryj = Ry(yAnglej[0])
   # Rzj2 = Rz(zAnglej2[0])

  #  Ryj2 = Ry(yAnglej2[0])
    #neck_vecjx =  unitVector(neck_vecjx*Rzj*Ryj)
    #neck_vecjy = unitVector(neck_vecjy * Rzj * Ryj)
    #neck_vecjz = unitVector(neck_vecjz * Rzj * Ryj)

    #print((neck_vecjx))

    #print(neck_vecjy)
    #print(neck_vecjz)
    #zAnglej = np.arcsin(neck_vecjx[0,2])
    #zAnglej =

    #print(zAnglej)
    #yAnglej =  -np.arcsin(neck_vecjx[0,1])

    # labels transformed x,y, z axis as y as head orientation changes from initial frame

        print(neck_vecjx[0,1])
        ax.text(neck_vecjx[0,0], neck_vecjx[0,1], neck_vecjx[0,2],s= "x' ", zdir=None, color='red')
        ax.text(neck_vecjy[0,0], neck_vecjy[0,1], neck_vecjy[ 0,2],s= "y' ", zdir=None, color='red')
        ax.text(neck_vecjz[0,0], neck_vecjz[0,1], neck_vecjz[0,2],s= "z'", zdir=None,color='red')

        # text for world frame orientation x,y,z
        ax.text(0.8, 0, 0, s="x ", zdir=None, color='green')
        ax.text(0,0.8,0, s="y ", zdir=None, color='green')
        ax.text(0, 0, 0.8, s="z ", zdir=None, color='green')

        #### Plot of transformed head frame
        ax.plot3D([0, 0.8*neck_vecjx[ 0,0]], [0, 0.8*(neck_vecjx[0, 1])], [0, 0.8*neck_vecjx[0,2]], 'red')
        ax.plot3D([0, 0.8*neck_vecjy[0,0]], [0, 0.8*(neck_vecjy[0,1])], [0, 0.8*neck_vecjy[0,2]], 'red')
        ax.plot3D([0, 0.8*neck_vecjz[ 0,0]], [0, 0.8*(neck_vecjz[0,1])], [0, 0.8*neck_vecjz[0, 2]], 'red')

        # plots x and z angles of eyes ######################################
        ax.text2D(1, 1, "EyeAngle in Relation Z - axis: %2.2f | EyeAngle in Relation X - axis: %.2f" % (zAnglej , yAnglej), transform=ax.transAxes)

        # plot of world frame
        ax.quiver(0, 0, 0, 0, 0, 0.5, length=0.5, normalize=True, color='green')
        ax.quiver(0, 0, 0, 0, 0.5, 0, length=0.5, normalize=True, color='green')
        ax.quiver(0, 0, 0, 0.5, 0, 0, length=0.5, normalize=True, color='green')
        #####################################################################################
        ## for eye vectors ##################################################################

        # grab eye vectors at current iteration
        #v1j = (v1[q]) # LEFT EYE
        #print(v1j)
        #v2j = (v2[q])  # RIGHT EYE

        eyeLj = vL_new[q]#unitVector(v1j * Rzj * Ryj) # CALCULATES LEFT AND RIGHT EYE NEW ORIENTATION AS EYE CHANGES
        eyeRj = vR_new[q]#unitVector(v2j * Rzj * Ryj)
        #print(neck_vecjz*0.2)
        #if q > 0:

        #print(neck_vecjx)



        ax.plot3D([0, -neck_vecjy[0,0]*0.2], [0,-0.2*neck_vecjy[0,1]], [0, -neck_vecjy[0,2]*0.2], 'blue')
        ax.plot3D([0, neck_vecjy[0, 0] * 0.2], [0, 0.2 * neck_vecjy[0, 1]], [0, neck_vecjy[0, 2] * 0.2], 'blue')


        ax.quiver(-neck_vecjy[0,0]*0.2, -neck_vecjy[0,1]*0.2, -neck_vecjy[0,2]*0.2, eyeLj[0], eyeLj[1], (eyeLj[2]), length=0.9, normalize=True, color='red')
        ax.quiver(neck_vecjy[0, 0] * 0.2, neck_vecjy[0, 1] * 0.2, neck_vecjy[ 0, 2] * 0.2, eyeRj[0], eyeRj[1], (eyeRj[2]), length = 0.9, normalize = True, color = 'red')



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



ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(v1), interval=100)

plt.show()
#plotting_driver(None,0)