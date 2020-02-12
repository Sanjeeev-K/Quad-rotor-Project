import xml.etree.ElementTree as ET
import pandas as pd
import numpy as np

'''
Parse relevant obstacle info from world file

input : path to the worl file
output : panda dataframe structured as follow

Reference | Name | Position | Size

Reference : Which obstacle it is
Name : Name of the part of the obstacle
Position : Position of the center of the part of the obstacle (x,y,z,yaw,pitch,roll)
Size : Size of the part of the obstacle (x,y,z)

Examples:

For first part of wall1

Reference is 1
Name wall1_1*1.5_1
Position 1.0 3.0 0.75 0 -0 0
Size 1 0.2 1.5

For second part of wall2

Reference is 2
Name  wall2_1.0*0.2_1	
Position 4.5 6.0 0.1 0 -0 0
Size 2.0 0.2 0.2

Pandas API : https://pandas.pydata.org/pandas-docs/stable/reference/index.html
'''

def parse_obstacles(filename):

    tree = ET.parse(filename)
    root = tree.getroot()

    obstacles = pd.DataFrame()

    #get obstacles names
    models = tree.findall('world/model')

    for m in models:
        if 'wall' in str(m.attrib):
            name = str(m.attrib).split("'")[3]
            ref = name[4]
            df = pd.concat([pd.DataFrame([ref]), pd.DataFrame([name])], axis=1, sort=False)
            obstacles = pd.concat([obstacles, df])


    #get obstacles positions
    poses = tree.findall('world/model/pose')

    df = pd.DataFrame()

    for p in poses:
        df = pd.concat([df, pd.DataFrame([p.text])], axis=0, sort=False)

    obstacles = pd.concat([obstacles, df], axis=1, sort=False)

    #get obstacles sizes
    sizes = tree.findall('world/model/link/visual/geometry/box/size')

    df = pd.DataFrame()

    for s in sizes:
        df = pd.concat([df, pd.DataFrame([s.text])], axis=0, sort=False)

    obstacles = pd.concat([obstacles, df], axis=1, sort=False)

    obstacles.columns = ['Reference', 'Name', 'Position', 'Size']

    obstacles.reset_index(drop=True, inplace=True)

    return(obstacles)

def transfertype(datalist):
    templist = []   #store the final list
    for i in range(len(datalist)):
        tempdata = datalist[i]    #The string date
        tempdatastr = tempdata.split()   #The string list data
        tempdataint = []     #Transfer str to int
        for j in range(len(tempdatastr)):
            tempdataint.append(float(tempdatastr[j]))
        templist.append(tempdataint)
    return templist

def createObs(center, size):
    pos = []
    obs = []
    for i in range(len(center)):
        temppos = (center[i][0], center[i][1], center[i][2])
        tempobs = (center[i][0] - size[i][0]/2, center[i][1] - size[i][1]/2, center[i][2] - size[i][2]/2,
                   center[i][0] + size[i][0]/2, center[i][1] + size[i][1]/2, center[i][2] + size[i][2]/2)
        pos.append(temppos)
        obs.append(tempobs)
    return pos, obs
if __name__ == "__main__":
    obstacles = parse_obstacles('world_test.world')
    print(obstacles)
    obscenter = obstacles['Position'].values.tolist()
    obssize = obstacles['Size'].values.tolist()

    obscenter_ = transfertype(obscenter)
    obssize_ = transfertype(obssize)
    pos, obs = createObs(obscenter_, obssize_)
    # print(obscenter_)
    # print(obssize_)
    print(pos)
    print(obs)
