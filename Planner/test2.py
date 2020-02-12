import numpy as np

def generateObs(goal):
    """
    :param goal: the goal you want to reach, consist of (x, y, z)
    :return: a list of obstacles used to generate the window
    """
    obslist = []
    x = goal[0]
    y = goal[1]
    z = goal[2]
    A = [x - 5, y - 0.5, z - 5]
    B = [x - 2.5, y + 0.5, z + 5]
    C = [x - 2.5, y - 0.5, z - 5]
    D = [x + 2.5, y + 0.5, z - 2.5]
    E = [x - 2.5, y - 0.5, z + 2.5]
    F = [x + 2.5, y + 0.5, z + 5]
    G = [x + 2.5, y - 0.5, z - 5]
    H = [x + 5, y + 0.5, z + 5]
    obs1 = (A[0], A[1], A[2], B[0], B[1], B[2])
    obs2 = (C[0], C[1], C[2], D[0], D[1], D[2])
    obs3 = (E[0], E[1], E[2], F[0], F[1], F[2])
    obs4 = (G[0], G[1], G[2], H[0], H[1], H[2])
    obsseriallist = [obs1, obs2, obs3, obs4]
    return obsseriallist

def genSerObs(goallist):
    obs = []
    for goal in goallist:
        tempobs = generateObs(goal)
        for tempobs_ in tempobs:
            obs.append(tempobs_)
    return obs

if __name__ == "__main__":
    goallist = [(10, 20, 40), (30, 40, 40)]
    obslist = genSerObs(goallist)
    print (obslist)

