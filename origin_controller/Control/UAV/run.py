import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sys_params import SYS_PARAMS
from trajectory import TRAJECTORY
from utils import *


params = SYS_PARAMS()
sim_time = 20  # 총 시뮬레이션 시간 [s]
dt = params['Ts']
iter = int(sim_time / dt)

initial_state = np.array([0, 0, 0.1, 0, 0, 0, 0, 0, -np.pi/2, 0, 0, 0])
x = initial_state

trajName = 'circle'
isDesturbance = 0

c_rec = np.zeros((iter, 4))
x_rec = np.zeros((iter, 12))
u_rec = np.zeros((iter, 4))
F_rec = np.zeros((iter, 4))
t_rec = np.zeros(iter)
Fz_rec = np.zeros(iter)
Mr_rec = np.zeros((iter, 3))

for k in range(iter):
    t = k * dt
    r = TRAJECTORY(t, sim_time, params, 'circle')

    pose = np.concatenate((x[0:3], x[6:9]))
    pose_d = np.concatenate((r[0:3], [0, 0, r[3]]))
    Fz, Mr, ang_d = PID(pose, pose_d)

    u = np.linalg.inv(params['T2W']).dot([Fz] + Mr.tolist())
    F = params['T2W'].dot(u)

    x = x + dt * UAV(x, u, params)

    c_rec[k, :] = r
    x_rec[k, :] = x
    u_rec[k, :] = u
    F_rec[k, :] = F
    t_rec[k] = t
    Fz_rec[k] = Fz
    Mr_rec[k, :] = Mr

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([0, 30])

curve1, = ax.plot([], [], [], 'r', linewidth=1)
curve3, = ax.plot([], [], [], 'b', linestyle='-.', linewidth=1)
ax.grid(True)
ax.legend(['reference', 'LQ control'])
print("curve1")
print(c_rec.shape)
print("curve3")
print(x_rec.shape)


def update(num, c_rec, x_rec, curve1, curve3):
    curve1.set_data(c_rec[:num, 0], c_rec[:num, 1])
    curve1.set_3d_properties(c_rec[:num, 2])
    curve3.set_data(x_rec[:num, 0], x_rec[:num, 1])
    curve3.set_3d_properties(x_rec[:num, 2])
    return curve1, curve3


frame_step = 500
ani = animation.FuncAnimation(fig, update, frames=range(0, len(t_rec), frame_step), fargs=(
    c_rec, x_rec, curve1, curve3), interval=50, blit=False, repeat=False)
plt.show()


fig, axs = plt.subplots(2, 2)

axs[0, 0].plot(t_rec, F_rec[:, 0])
axs[0, 0].set_title('u(1)')
axs[0, 0].grid(True)

axs[0, 1].plot(t_rec, F_rec[:, 1])
axs[0, 1].set_title('u(2)')
axs[0, 1].grid(True)

axs[1, 0].plot(t_rec, F_rec[:, 2])
axs[1, 0].set_title('u(3)')
axs[1, 0].grid(True)

axs[1, 1].plot(t_rec, F_rec[:, 3])
axs[1, 1].set_title('u(4)')
axs[1, 1].grid(True)

plt.tight_layout()
plt.show()
