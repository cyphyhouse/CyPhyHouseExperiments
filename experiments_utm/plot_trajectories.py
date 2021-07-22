import pickle
from typing import List, Tuple
import matplotlib.pyplot as plt
import copy
import mpl_toolkits.mplot3d as a3
import pypoman as ppm
import numpy as np

class Plotter:
    colors = ['b', 'g']
    # colors = ['#8dd3c7', '#ffffb3', '#bebada', '#fb8072', '#80b1d3', '#fdb462', '#b3de69', '#fccde5']
    # colors = ['#1b9e77', '#d95f02', '#7570b3', '#e7298a', '#66a61e', '#e6ab02', '#a6761d', '#666666']
    # colors = ['#b3de69',  '#bebada', '#fdb462', '#fccde5', '#bc80bd', '#ccebc5', '#ffed6f', '#ffffb3', '#c7eae5']
    # colors = ['#8dd3c7', '#8dd3c7', '#8dd3c7', '#8dd3c7', '#8dd3c7', '#8dd3c7', '#8dd3c7', '#8dd3c7', '#8dd3c7', '#8dd3c7', '#8dd3c7', '#8dd3c7']
    # TODO move away from static plotting and move towards OO plotting configurations

    @staticmethod
    def plot_line_3d(start, end, ax = None, color = 'blue'):
        x = [start[0], end[0]]
        y = [start[1], end[1]]
        z = [start[2], end[2]]
        line = a3.art3d.Line3D(x,y,z,color = color)
        ax.add_line(line)

    @staticmethod
    def plot2d(trajectory_list: List[List[Tuple[float, float, float]]]):

        # agents_tubes = [[segment.tube for segment in segment_list] for segment_list in tubes]
        # pdb.set_trace()
        plt.figure(0)
        plt.figure(1)
        for idx, trajectory in enumerate(trajectory_list):
            x = []
            y = []
            z = []
            x_normal = []
            y_normal = []
            z_normal = []
            x_acas = []
            y_acas = []
            z_acas = []
            trajectory = trajectory[:2000000]
            for point in trajectory:
                x.append(point[0][0])
                y.append(point[0][1])
                z.append(point[0][2])
                if point[1] == 1:
                    x_acas.append(point[0][0])
                    y_acas.append(point[0][1])
                    z_acas.append(point[0][2])
                else:
                    x_normal.append(point[0][0])
                    y_normal.append(point[0][1])
                    z_normal.append(point[0][2])  
                    z_acas.append(float('NAN'))
            plt.figure(0)
            plt.plot(x,y,Plotter.colors[idx%2])
            plt.plot(x_acas, y_acas, 'r.')
            plt.figure(1)
            plt.plot(z,Plotter.colors[idx%2])
            plt.plot(z_acas, 'r.')

        plt.show()

    @staticmethod
    def plot3d(trajectory_list: List[List[Tuple[float, float, float]]]):
        ax = a3.Axes3D(plt.figure())
        for idx, trajectory in enumerate(trajectory_list):
            for j in range(0,(len(trajectory)-100),100):
                xllim, xtlim = ax.get_xlim()
                yllim, ytlim = ax.get_ylim()
                zllim, ztlim = ax.get_zlim()
                x = [trajectory[j][0][0], trajectory[j+100][0][0], xllim+1, xtlim-1]
                y = [trajectory[j][0][1], trajectory[j+100][0][1], yllim+1, ytlim-1]
                z = [trajectory[j][0][2], trajectory[j+100][0][2], zllim+1, ztlim-1]

                ax.set_xlim(np.min(x)-1, np.max(x)+1)
                ax.set_ylim(np.min(y)-1, np.max(y)+1)
                ax.set_zlim(np.min(z)-1, np.max(z)+1)

                Plotter.plot_line_3d(trajectory[j][0], trajectory[j+100][0], ax, Plotter.colors[idx%2])
        plt.show()
        pass

def plot_trajectories(fn_list):
    trajectory_list = []
    for fn in fn_list:   
        with open(fn, 'rb') as f:
            tmp = pickle.load(f)
            trajectory_list.append(tmp)    
    plot = Plotter()
    plot.plot2d(trajectory_list)

if __name__ == "__main__":
    fn_list = [
        './trajectories/drone0',
        './trajectories/drone1',
    ]
    plot_trajectories(fn_list)