import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from utils import *
from sys_params import SYS_PARAMS

#to save numpy arry to csv
import pandas as pd

#load params
params = SYS_PARAMS()

#set simulation time
sim_time = 10
dt = params['Ts']
iter = sim_time/dt

#state x = [q1, q2, dq1, dq2]
x = np.array([0.0, 0.0, 0.0, 0.0])

trajName = 'circle'
isDesturbance = 0

#record for animation
rq_rec = np.zeros((int(iter)+1, 2))
rx_rec = np.zeros((int(iter)+1, 2))
ry_rec = np.zeros((int(iter)+1, 2))
x_rec = np.zeros((int(iter)+1, 2))
y_rec = np.zeros((int(iter)+1, 2))
q_rec = np.zeros((int(iter)+1, 2))
u_rec = np.zeros((int(iter)+1, 2))
t_rec = np.zeros(int(iter)+1)


for k in range(1, int(iter) + 1):
    t = k * dt

    Theta = t
    r, XE, YE = Inverse_Kinemetic(Theta)

    if t > 1:
        dr = np.array([0.01, 0.01])
        ddr = np.array([0.001, 0.001])
    else:
        dr = np.array([0, 0])
        ddr = np.array([0, 0])

    v = Controller(x, r, dr, ddr)
    u = Feedback_linearization(x, v)

    x[2:4] += dt * Arm_Dynamic(x, u)
    x[0:2] += dt * x[2:4]

    x1, y1, x2, y2 = Forward_Kinemetic(x[0:2])

    if k == 1:
        continue
    rq_rec[k, :] = r
    rx_rec[k, :] = XE
    ry_rec[k, :] = YE
    x_rec[k, :] = [x1, x2]
    y_rec[k, :] = [y1, y2]
    q_rec[k, :] = x[0:2]
    u_rec[k, :] = u
    t_rec[k] = t





############################
Joint_1 = [0, 0]
Joint_2 = [1, 0]
Joint_3 = [2, 0]
fig, ax = plt.subplots()
ax.axis('equal')
ax.set_xlim(-1, 2.5)
ax.set_ylim(-1, 2.5)
ax.grid(True)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Robot Movement')



Robot_arm_1, = ax.plot([Joint_1[0], Joint_2[0]], [
                       Joint_1[1], Joint_2[1]], 'k', linewidth=4)
Robot_arm_2, = ax.plot([Joint_2[0], Joint_3[0]], [
                       Joint_2[1], Joint_3[1]], 'k', linewidth=4)
Robot_path, = ax.plot([Joint_3[0]-0.01, Joint_3[0]],
                      [Joint_3[1]-0.01, Joint_3[1]], 'r.', linewidth=0.5)
Target_path, = ax.plot(rx_rec[3:, 0], ry_rec[3:, 0], '--b')

path_x, path_y = [], []

a1 = np.array(rx_rec[3:, 0]).reshape(-1, 1)
a2 = np.transpose(np.array(ry_rec[3:, 0])).reshape(-1, 1)
a3 = np.concatenate((a1, a2), axis = 1)
df = pd.DataFrame(a3)

df.to_csv('reference.csv')
def update(frame):
    Robot_X1 = x_rec[frame, 0]
    Robot_Y1 = y_rec[frame, 0]
    Robot_X2 = x_rec[frame, 1]
    Robot_Y2 = y_rec[frame, 1]

    Robot_arm_1.set_data([Joint_1[0], Robot_X1], [Joint_1[1], Robot_Y1])
    Robot_arm_2.set_data([Robot_X1, Robot_X2], [Robot_Y1, Robot_Y2])
    Robot_path.set_data(Robot_X2, Robot_Y2)

    path_x.append(Robot_X2)
    path_y.append(Robot_Y2)
    Robot_path.set_data(path_x, path_y)

    return Robot_arm_1, Robot_arm_2, Robot_path


ani = animation.FuncAnimation(fig, update, frames=range(
    0, int(iter)+1, 10), blit=True, interval=5, repeat=False)
plt.show()

############################
# plt.figure(1)

# plt.subplot(2, 2, 1)
# plt.plot(t_rec, 180/np.pi*q_rec[:, 0], 'k', t_rec,
#          180/np.pi*rq_rec[:, 0], '--b', linewidth=1.2)
# plt.title('Theta 1 Input & Output')
# plt.xlabel('Time(s)')
# plt.ylabel('Theta (Deg)')
# plt.axis([0, 10, -10, 160])
# plt.legend(['Theta 1 Output', 'Theta 1 Input'])
# plt.grid(True)

# plt.subplot(2, 2, 2)
# plt.plot(t_rec, 180/np.pi*q_rec[:, 1], 'k', t_rec,
#          180/np.pi*rq_rec[:, 1], '--b', linewidth=1.2)
# plt.title('Theta 2 Input & Output')
# plt.xlabel('Time(s)')
# plt.ylabel('Theta (Deg)')
# plt.axis([0, 10, -160, 10])
# plt.legend(['Theta 2 Output', 'Theta 2 Input'])
# plt.grid(True)

# plt.subplot(2, 2, 3)
# plt.plot(t_rec, x_rec[:, 1], 'k', t_rec, rx_rec[:, 0], '--b', linewidth=1.2)
# plt.title('X(end point) Input & Output')
# plt.xlabel('Time(s)')
# plt.ylabel('X (m)')
# plt.axis([0, 10, -1, 4])
# plt.legend(['X output', 'X input'])
# plt.grid(True)

# plt.subplot(2, 2, 4)
# plt.plot(t_rec, y_rec[:, 1], 'k', t_rec, ry_rec[:, 0], '--b', linewidth=1.2)
# plt.title('Y(end point) Input & Output')
# plt.xlabel('Time(s)')
# plt.ylabel('Y (m)')
# plt.axis([0, 10, -2, 4])
# plt.legend(['Y output', 'Y input'])
# plt.grid(True)

# ############################
# plt.figure(2)

# plt.subplot(2, 1, 1)
# plt.plot(t_rec, u_rec[:, 0], 'k', linewidth=1.2)
# plt.title('u(1)')
# plt.grid(True)

# plt.subplot(2, 1, 2)
# plt.plot(t_rec, u_rec[:, 1], 'k', linewidth=1.2)
# plt.title('u(2)')
# plt.grid(True)

# plt.show()
