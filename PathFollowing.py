#python
import numpy as np 
import matplotlib.pyplot as plt 
import yaml
import pathlib

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

def read_csv(filename):
    return np.loadtxt(filename, delimiter=",")

def read_csv_from_dir(dirname):
    path = pathlib.Path(dirname)
    for filename in path.glob("*.csv"):
        yield read_csv(filename)

def mapPath(X, Xmin, Xmax, Ymin, Ymax):
    X_std = (X - Xmin) / (Xmax - Xmin)
    X_scaled = X_std * (Ymax - Ymin) + Ymin
    return X_scaled
    

def sysCall_init():
    sim = require('sim')
    ROOT_DIR = '/home/redwan/CoppeliaSimProjects/visibility_patrol'
    
    result = f"{ROOT_DIR}/data/result.yaml"
    
    with open(result) as file:
        data = yaml.safe_load(file)
        xy_coord = np.loadtxt(f'{ROOT_DIR}/data/pvis.csv', delimiter=",")

    Xmax = np.array([300, 200]) 
    Xmin = np.array([0, 0])  
    Ymax = np.array([10, 10]) 
    Ymin = np.array([-10, -10])
    self.pathHandles = []

    for i in range(data['NUM_ROBOTS']):
        indexes = data[f'robot{i+1}']
        # xy_coord[indexes, :2] = mapPath(xy_coord[indexes, :2], Xmax, Xmin, Ymax, Ymin )
        pathCoord = mapPath(xy_coord[indexes, :2], Xmax, Xmin, Ymax, Ymin )
        # z qx qy qz qw
        extraDim = np.array([[0.93, 0.0, 0.0, 0.0, 1.0] for _ in range(len(pathCoord))])
        pathCoord = np.hstack((pathCoord, extraDim))
        #print(pathCoord.shape, extraDim.shape)
        pathHandle = sim.createPath(pathCoord.flatten().tolist())
        follower = PathFollowerTarget(pathHandle, pathCoord, f'/Cuboid[{i}]')
        self.pathHandles.append(follower)
        if i == 3:
            break
       
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
