import numpy as np 
import matplotlib.pyplot as plt 
import yaml
import pathlib

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

if __name__ == "__main__":
    result = "data/result.yaml"
    
    with open(result) as file:
        data = yaml.safe_load(file)
        xy_coord = np.loadtxt('data/pvis.csv', delimiter=",")

    Xmax = np.array([300, 200]) 
    Xmin = np.array([0, 0])  
    Ymax = np.array([10, 10]) 
    Ymin = np.array([-10, -10])

    for i in range(data['NUM_ROBOTS']):
        indexes = data[f'robot{i+1}']

        xy_coord[indexes, :2] = mapPath(xy_coord[indexes, :2], Xmax, Xmin, Ymax, Ymin )
        plt.plot(xy_coord[indexes, 0], xy_coord[indexes, 1])
        plt.scatter(xy_coord[indexes, 0], xy_coord[indexes, 1])
    
    # for obs in read_csv_from_dir('data/obstacles'):
    #     plt.fill(obs[:, 0], obs[:, 1], 'k')
    # plt.savefig('result.png')
    # print('result.png updated')
    plt.show()