import pickle

def readfile():
    filename1 = "pos_xy.pkl"
    filename2 = "obs_xy.pkl"
    with open (filename1, "rb") as f:
        pos = pickle.load(f)

    with open (filename2, "rb") as f:
        obs = pickle.load(f)

    return pos, obs