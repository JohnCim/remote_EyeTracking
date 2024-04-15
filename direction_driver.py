
import numpy as np
def direction_driver(head,eye,n):
    classification = {'eye_x': [], 'eye_y': [], 'eye_z': [], 'eye_direction_x': [], 'eye_motion_y':[], 'head_motion_x' :[],'head_motion_y':[]}
    for i in range(0,n):
        if i% 10 == 0:
            headChange, headX,headY  =  changeDirection(head[i,:],head[i-10,:])
            eyeChange, eyeX, eyeY =  changeDirection(head[i, :], eye[i - 10, :])
            # append change in eye direction
            classification['eye_x'].append(eyeChange[0])
            classification['eye_y'].append(eyeChange[1])
            classification['eye_z'].append(eyeChange[2])

            # append 




def changeDirection(v_i,v_prev):
    v = np.subtract(v_i,v_prev)
    changeX = changeXY(v,0)
    changeY = changeXY(v,1)
    return v,changeX,changeY

def changeXY(v,i):
    if v[i] > 0:
        change = 0#'right'
    elif v[i]< 0:
        change = 1#'left'

    else:
        print('Error')
    return change