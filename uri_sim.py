#python
import numpy as np 
from pathlib import Path 


class PathFollowerTarget:
    def __init__(self, path, pathData, targetHandle) -> None:
        self.path = path
        self.pathData = pathData
        self.pathPositions = pathData[:, :3].flatten().tolist()
        self.pathQuaternions = pathData[:, 3:].flatten().tolist()
        self.pathLengths, self.totalLength = sim.getPathLengths(self.pathPositions, 3)
        self.objectToFollowPath = sim.getObject(targetHandle)
        self.velocity = 0.1 # m/s
        self.posAlongPath = 0
        self.previousSimulationTime = sim.getSimulationTime()
    
    def __call__(self, *args, **kwds):
        t = args[0]
        self.posAlongPath += self.velocity * (t - self.previousSimulationTime)
        if self.posAlongPath >= self.totalLength:
            return
        #self.posAlongPath %= self.totalLength
        pos = sim.getPathInterpolatedConfig(self.pathPositions, self.pathLengths, self.posAlongPath)
        quat = sim.getPathInterpolatedConfig(self.pathQuaternions, self.pathLengths,
               self.posAlongPath, None, [2, 2, 2, 2])
        sim.setObjectPosition(self.objectToFollowPath, pos, self.path)
        sim.setObjectQuaternion(self.objectToFollowPath, quat, self.path)
        self.previousSimulationTime = t
        sim.step()

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
    

def sysCall_init():
    sim = require('sim')
    ROOT_DIR = Path("/home/airlab/PycharmProjects/VisibilityPatrol/results/folder_20240119_141840")
    
    robots = {filename.name: addExtraDim(mapCSV(filename)) for filename in ROOT_DIR.glob("Thread*")}
    # targets = mapCSV(f"{ROOT_DIR}/targets.csv")
    self.pathHandles = []
    for i, (roboName, pathCoord) in enumerate(robots.items()):
        print(pathCoord.shape)
        pathHandle = sim.createPath(pathCoord.flatten().tolist())
        follower = PathFollowerTarget(pathHandle, pathCoord, f'/Cuboid[{i}]')
        self.pathHandles.append(follower)
    # for t in targets:    
    #     objectHandle=sim.createPureShape(0,16,[0.1,0.1,0.1], 0.01)
    #     sim.setObjectPosition(objectHandle, [t[0], t[1], 10])    
    # do some initialization here

def sysCall_actuation():
    # put your actuation code here
    #t = sim.getSimulationTime()t
    #self.pathHandles[0](t)
    pass

def sysCall_thread():
    while True:
        t = sim.getSimulationTime()
        for p in self.pathHandles:
            p(t)
        

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
