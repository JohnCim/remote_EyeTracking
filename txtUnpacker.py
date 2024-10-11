from mpl_toolkits import mplot3d
import numpy as np

import pandas as pd

pd.options.mode.chained_assignment = None



def txtUnpacker(i,number):
    dataframes = pd.read_csv("2_%s.txt" % number, delimiter=",").iloc[:, :-1]  # dataframe created


    if i == None:
        i = len(dataframes)

    x = np.zeros([i, 3])
    y = np.zeros([i, 3])
    z = np.zeros([i, 3])





    for j in range(0, i):




        df = dataframes.iloc[j, :]

        ## CURRENT STEPSIZE OF NECK POSITION


        x[j,:] = df["gaze_direct_L.x"], df["gaze_direct_R.x"], df["forward.x"]

        y[j,:] = df["gaze_direct_L.y"], df["gaze_direct_R.y"], df["forward.y"]

        z[j,:] = df["gaze_direct_L.z"], df["gaze_direct_R.z"], df["forward.z"]



    return x, y, z
#def txtCounter(number):
#    lines = 0
#    for i in range(0,len(number)):


#        dataframes = pd.read_csv("2_%s.txt"%number[i], delimiter=",").iloc[:, :-1]
#        lines += len(dataframes)
    #print('Total Number of lines:', lines)
#    return lines
#x,y,z = txtUnpacker(None,[0,1])

#from matplotlib import pyplot as plt
#plt.plot(x)
#plt.show()
