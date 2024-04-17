
import pandas as pd
import numpy as np

def txt_sorter(string,number,i):

    dataframes = pd.read_csv(string % number, delimiter=",").iloc[:, :-1]  # dataframe created

    if i == None:
        i = len(dataframes)

    x = []
    y = []
    z = []

    for j in range(0, i):
        df = dataframes.iloc[j, :]

        ## CURRENT STEPSIZE OF NECK POSITION

        x.append([df["gaze_direct_L.x"], df["gaze_direct_R.x"], df["forward.x"]])

        y.append([df["gaze_direct_L.y"], df["gaze_direct_R.y"], df["forward.y"]])

        z.append([df["gaze_direct_L.z"], df["gaze_direct_R.z"], df["forward.z"]])

    return x, y, z
def txt_adder(string,n):
    x = []
    y = []
    z = []
    for i in range(0,n+1):
        try:
            xi,yi,zi = txt_sorter(string,i,None)
            x += xi
            y += yi
            z += yi
            print(len(x))
        except:
            print('n out of bounds')


    return x,y,z

def txt_main():

    string =  ["2_%s.txt"]
    print(len(string))
    x,y,z = txt_adder(string[0],2)
    return x,y,z
#print(x)
x,y,z = txt_main()
print(len(x))
#FileName = "\Documents\User5"
#data=open(FileName)
#data.sort()
#for i in range(len(data)):
#    print(data[i])