import os
import numpy as np 
from pathlib import Path 
import matplotlib.pyplot as plt 

def loadCSV(filename):
    return np.loadtxt(filename, delimiter=",")

def mapPath(X, Xmin, Xmax, Ymin, Ymax):
    X_std = (X - Xmin) / (Xmax - Xmin)
    X_scaled = X_std * (Ymax - Ymin) + Ymin
    return X_scaled

def mapCSV(filename):
    data = loadCSV(filename)
    Xmax = np.array([300, 200]) 
    Xmin = np.array([0, 0])  
    Ymax = np.array([10, 10]) 
    Ymin = np.array([-10, -10])
    return mapPath(data, Xmax, Xmin, Ymax, Ymin )

def addExtraDim(data):
    extraDim = np.array([[0.93, 0.0, 0.0, 0.0, 1.0] for _ in range(len(data))])
    return np.hstack((data, extraDim))

if __name__ == "__main__":
    ROOT_DIR = Path("/home/airlab/CoppeliaSimProjects/vis_patrol/vrep_diff_controller/data")
    robots = {filename.name: addExtraDim(loadCSV(filename)) for filename in ROOT_DIR.glob("robot*")}
    targets = mapCSV(f"{ROOT_DIR}/targets.csv")
    print(targets.shape)

    for roboName, pathCoord in robots.items():
        print(roboName, pathCoord)
        plt.plot(pathCoord[:, 0], pathCoord[:, 1])
        break
        # pathHandle = sim.createPath(pathCoord.flatten().tolist())
    plt.show()
