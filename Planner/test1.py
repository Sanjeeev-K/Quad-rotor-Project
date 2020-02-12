# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np
import matplotlib.pyplot as plt

from src.rrt.rrt_star import RRTStar
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot
import test2
import pklexa

X_dimensions = np.array([(-20, 20), (0, 10), (0, 2)])  # dimensions of Search Space
x_init = (0, 0, 0)  # starting location
# x_goal= (10, 20, 2)  # goal location

x_goal = (0.1,0.1,0.1)


# x_goal= (4.5, 6.0, 1.575)  # goal location
# [2.5, 3.0, 0.75], [4.5, 6.0, 1.575], [3.0, 8.25, 0.75], [4.5, 12.0, 1.25]

# # obstacles
# Obstacles = np.array(
    # [(20, 20, 20, 40, 40, 40), (20, 20, 60, 40, 40, 80), (20, 60, 20, 40, 80, 40), (60, 60, 20, 80, 80, 40),
    #  (60, 20, 20, 80, 40, 40), (60, 20, 60, 80, 40, 80), (20, 60, 60, 40, 80, 80), (60, 60, 60, 80, 80, 80)])
# (20, 60, 20, 30, 65, 40) 
# 55, 55,
# (60, 20, 20, 55, 55, 25)

# Obstacles = np.array(
    # [(20, 20, 20, 25, 60, 25),(20, 55, 20, 60, 60, 25),(20, 20, 20,60, 25, 25),(55, 25, 20, 60, 60, 25)])

# goallist = [(10, 20, 40), (30, 40, 40)]
# Obstacles = test2.genSerObs(goallist)
goallist, Obstacles = pklexa.readfile()
print(goallist)
# print(Obstacles)
# input("111")
Obstacles = np.array(Obstacles)



############### Adding appropirate size factor here ######################################
size_factor = 0.15

for i in range(np.size(Obstacles,0)):
    Obstacles[i][0]=Obstacles[i][0]-size_factor
    Obstacles[i][1]=Obstacles[i][1]-size_factor
    Obstacles[i][2]=Obstacles[i][2]-size_factor
    Obstacles[i][3]=Obstacles[i][3]+size_factor
    Obstacles[i][4]=Obstacles[i][4]+size_factor
    Obstacles[i][5]=Obstacles[i][5]+size_factor

# Obstacles = np.array(
#     [(20, 20, 20, 25, 60, 25),(20, 55, 20, 60, 60, 25),(60, 25, 25,20, 20, 20),(55, 25, 20, 60, 60, 25)])


# goal_points = np.array([x_init,(10, 20, 40),(30, 40, 40),(100,100,100)])
goal_points = np.array([x_init ])


##################   Adding waypoints for Path to be planned  #####################################


goal_points = np.vstack((goal_points, goallist))
# goal_points2 = np.array([x_goal ])
# goal_points = np.vstack((goal_points, goal_points2))


# goal_points2 = goallist[1]
# goal_points = np.vstack((goal_points, goal_points2))
# x_goal = goallist[1]
print(goal_points)


# goal_points = np.array([x_init,(-5.5  , 3.0 ,  0.75),(-4.  ,  7.   , 0.75),(1. ,  10.5 ,  0.75), (4.5 ,  5. ,   0.75), (2.5   ,2.  ,  0.75), x_init])
# input("111")
# [ 0.    0.    0.  ]
#  [ 2.5   2.    0.75]
#  [ 4.5   5.    0.75]
#  [-5.5   3.    0.75]
#  [-4.    7.    0.75]
#  [ 1.   10.5   0.75]
#  [ 4.5   5.    0.75]]
# (60,60,60),(50,50,50),(75,75,75),

Q = np.array([(8, 4)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 1024  # max number of samples to take before timing out
rewire_count = 32  # optional, number of nearby branches to rewire
prc = 1  # probability of checking for a connection to goal

# create Search Space
X = SearchSpace(X_dimensions, Obstacles)

# create rrt_search
# rrt = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
# path = rrt.rrt_star()

# rrt = RRTStar(X, Q, x_goal, x_goal2, max_samples, r, prc, rewire_count)
# path2 = rrt.rrt_star()

# path = path + path2
path_final = []
for i in range(0,(np.size(goal_points,0)-1)):
    a = (goal_points[i][0],goal_points[i][1],goal_points[i][2]) #goal_points[i]
    b = (goal_points[i+1][0],goal_points[i+1][1],goal_points[i+1][2]) #goal_points[i+1]
    rrt = RRTStar(X, Q, a, b, max_samples, r, prc, rewire_count)
    path = rrt.rrt_star()
    path_final = path_final + path

path = path_final
# print(path)
# print(Obstacles)

# a1  = goal_points[1]
# print(a1)

# print(np.size(Obstacles,0))


# plot
plot = Plot("rrt_star_3d")
plot.plot_tree(X, rrt.trees)
# input("Press Enter to continue...")
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
# input("Press Enter to continue...")
plot.plot_start(X, x_init)
# plot.draw(auto_open=True)
# input("Press Enter to continue...")
# plt.plot3D(1,1,1)

for i in range(1,(np.size(goal_points,0))):
    a = (goal_points[i][0],goal_points[i][1],goal_points[i][2])
    plot.plot_goal(X, a)



# plot.plot_goal(X, x_goal)
# plot.plot_goal(X, x_goal2)
# for i in range(np.size(goal_points,0)-1)
np.savetxt("path3.csv", path, delimiter=",")

plot.draw(auto_open=True)
# input("Press Enter to continue...")
# plot.plot_start(X, x_init)
# plot.draw(auto_open=True)
