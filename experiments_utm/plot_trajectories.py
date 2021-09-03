import pickle
from typing import List, Tuple
import matplotlib.pyplot as plt
import copy
import mpl_toolkits.mplot3d as a3
#import pypoman as ppm
import numpy as np
import os

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
            point = trajectory[-1]
            for ele in point:
                if type(ele) == float:
                    if np.isnan(ele):
                        return
            t = []
            x = []
            y = []
            z = []
            yaw = []
            vx = []
            vy = []
            vz = []
            t_normal = []
            x_normal = []
            y_normal = []
            z_normal = []
            roll_normal = []
            pitch_normal = []
            yaw_normal = []
            vx_normal = []
            vy_normal = []
            vz_normal = []
            vangularx_normal = []
            vangulary_normal = []
            vangularz_normal = []
            t_acas = []
            x_acas = []
            y_acas = []
            z_acas = []
            roll_acas = []
            pitch_acas = []
            yaw_acas = []
            vx_acas = []
            vy_acas = []
            vz_acas = []
            vangularx_acas = []
            vangulary_acas = []
            vangularz_acas = []
            trajectory = trajectory[:2000000] # trajectory[:2000000]
            for idx2, point in enumerate(trajectory):
                if idx2%1000 == 0:
                    t.append(idx2)
                    x.append(point[1])
                    y.append(point[2])
                    z.append(point[3])
                    yaw.append(point[6])
                    vx.append(point[7])
                    vy.append(point[8])
                    vz.append(point[9])
                    # print(point[4])
                    if point[5] == 1:
                        t_acas.append(idx2)
                        x_acas.append(point[1])
                        y_acas.append(point[2])
                        z_acas.append(point[3])
                        # yaw_acas.append(point[6])
                        # yaw_acas.append(point[6])
                        roll_acas.append(point[6])
                        pitch_acas.append(point[7])
                        yaw_acas.append(point[8])
                        vx_acas.append(point[9])
                        vy_acas.append(point[10])
                        vz_acas.append(point[11])
                        vangularx_acas.append(point[12])
                        vangulary_acas.append(point[13])
                        vangularz_acas.append(point[14])
                        
                    else:
                        t_normal.append(idx2)
                        x_normal.append(point[1])
                        y_normal.append(point[2])
                        z_normal.append(point[3])  
                        #z_acas.append(float('NAN'))
                        roll_normal.append(point[6])
                        pitch_normal.append(point[7])
                        yaw_normal.append(point[8])
                        vx_normal.append(point[9])
                        vy_normal.append(point[10])
                        vz_normal.append(point[11])
                        vangularx_normal.append(point[12])
                        vangulary_normal.append(point[13])
                        vangularz_normal.append(point[14])
                        
            print("Last state of drone", idx, "is : ", point)
            plt.figure(0)
            plt.plot(x,y,Plotter.colors[idx%2])
            # plt.plot(x_acas, y_acas, 'r.')
            plt.figure(1)
            plt.plot(t, z,Plotter.colors[idx%2])
            # plt.plot(t_acas, z_acas, 'r.')
            plt.figure(2)
            plt.plot(t, yaw,Plotter.colors[idx%2])
            # plt.plot(t_acas, yaw_acas, 'r.')
            plt.figure(3)
            plt.plot(t, vx,Plotter.colors[idx%2])
            # plt.plot(t_acas, vx_acas, 'r.')
            plt.figure(4)
            plt.plot(t, vy,Plotter.colors[idx%2])
            # plt.plot(t_acas, vy_acas, 'r.')
            plt.figure(5)
            plt.plot(t, vz,Plotter.colors[idx%2])
            # plt.plot(t_acas, vz_acas, 'r.')
            plt.figure(6)
            plt.plot(t,x,Plotter.colors[idx%2])
            # plt.plot(t_acas, x_acas, 'r.')
            plt.figure(7)
            plt.plot(t, y,Plotter.colors[idx%2])
            # plt.plot(t_acas, y_acas, 'r.')
            # print("x_acas: ", x_acas)
            # print("y_acas: ", y_acas)
            # print("z_acas: ", z_acas)

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
    for i in range(9):
        for j in range(9):
            for k in range(4):
                scenario_idx = f"_{i}_{j}_{k}"
                trajectory_list = []
                print(scenario_idx)
                for fn in fn_list:   
                    fn_tmp = fn + scenario_idx
                    if os.path.isfile(fn_tmp):
                        with open(fn_tmp, 'rb') as f:
                            tmp = pickle.load(f)
                            trajectory_list.append(tmp)    
                plot = Plotter()
                plot.plot2d(trajectory_list)
 
    plt.show()


if __name__ == "__main__":
    fn_list = [
        './trajectories/drone0',
        './trajectories/drone1',
    ]
    plot_trajectories(fn_list)
