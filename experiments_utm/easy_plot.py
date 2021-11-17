import matplotlib.pyplot as plt
import pickle

fn = './trajectories/drone1_0_0_1_0'
f = open(fn, 'rb')
traj = pickle.load(f)
t = [elem[0]-traj[0][0] for elem in traj]
x = [elem[1] for elem in traj]
y = [elem[2] for elem in traj]
z = [elem[3] for elem in traj]

vx = [elem[9] for elem in traj]
vy = [elem[10] for elem in traj]
vz = [elem[11] for elem in traj]

plt.figure(0)
plt.plot(x[5:],y[5:])

plt.figure(1)
plt.plot(z[5:])

plt.figure(2)
plt.plot(vx[5:])
plt.ylabel('vx')

plt.figure(3)
plt.plot(vy[5:])
plt.ylabel('vy')

plt.figure(4)
plt.plot(vz[5:])
plt.ylabel('vz')
plt.show()