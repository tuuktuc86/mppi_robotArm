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
sim_time = 20
dt = params['Ts']
iter = sim_time/dt


#x = np.array([1.03960371764376, 1.35008186526306, 1.46563474895373, -1.10207211330654,0, 0]) # x, y, q1, q2, dq1, dq2  #231번점
#원래 값은 ddq가 0,0
x = np.array([1.3999325014217017, 0.8089996490831619, 1.15330155041141, -1.25860400808722, 0.015, 0.015]) # x, y, q1, q2, dq1, dq2
#x = np.array([1.3999325014217017, 0.8089996490831619, 1.15330155041141, -1.25860400808722,-0.109067881729775, -0.740928212925687]) # x, y, q1, q2, dq1, dq2

trajName = 'circle'
isDesturbance = 0
ref_path = np.genfromtxt('reference_forest.csv', delimiter=',', skip_header=1)
#ref_path = np.genfromtxt('reference3_0.01.csv', delimiter=',', skip_header=1)
#print(ref_path.shape) #1214, 4 [x, y, q1, q2]

#to find optimal input
# origin_path= np.genfromtxt('origin_reference_10_0.005.csv', delimiter=',', skip_header=1)

#record for animation
rq_rec = np.zeros((int(iter)+1, 2))
rx_rec = np.zeros((int(iter)+1, 2))
ry_rec = np.zeros((int(iter)+1, 2))
x_rec = np.zeros((int(iter)+1, 2))
y_rec = np.zeros((int(iter)+1, 2))
q_rec = np.zeros((int(iter)+1, 2))
u_rec = np.zeros((int(iter)+1, 2))
t_rec = np.zeros(int(iter)+1)
best_rec = np.zeros((11, 2))
sample_rec = np.zeros((9, 2, 2))
ref_rec = ref_path
#time step t 조정하면 3군데 수정해야됨. bestrec, samplerec, moving avg


# initialize a mppi controller for the vehicle
mppi = MPPIControllerForRobotArm(
    ref_path=ref_path,
)

# iter_1 = 100
# for k in range(1, int(iter_1) + 1):
#     t = k * dt

#     Theta = t
#     r, XE, YE = Inverse_Kinemetic(Theta)
x_rec[0, :] = [0, x[0]]
y_rec[0, :] = [0, x[1]]
#print(f"k = 0, x = {x[0]}, y = {x[1]}")
for k in range(1, int(iter) + 1):

    t = k * dt

    Theta = t
    r, XE, YE = Inverse_Kinemetic(Theta)



    # best_rec = np.zeros((20, 2))
    # sample_rec = np.zeros((20, 20, 2))

    #r은 q1, q2가 따라야 하는 것. XE는 X가 따라야 하는 것, YE는 Y가 따라야 하는 것
    # print(r, "||", XE, "||", YE)
    # if t > 1:
    #     dr = np.array([0.01, 0.01])
    #     ddr = np.array([0.001, 0.001])
    # else:
    #     dr = np.array([0, 0])
    #     ddr = np.array([0, 0])
   
    position = x[0:2]
    q_state = x[2:4]
    dq_state = x[4:6]
    optimal_input, optimal_input_sequence, optimal_traj, sampled_traj_list, sampling_best_traj, sampling_best_input, flag = mppi.calc_control_input(
        observed_x = x
    )
    #print(f"k = {k}, control_input = {sampling_best_input[0]}")
    #to find optimal input
    # optimal_input = origin_path[k, 10:12]
    # q_state = origin_path[k, 4:6]
    # dq_state = origin_path[k, 6:8]

    # v = Controller(x, r, dr, ddr)
    # u = Feedback_linearization(x, v)
    dq_state += dt * Arm_Dynamic(q_state, dq_state, optimal_input)
    dq_state[0] = np.clip(dq_state[0], -0.9, 0.9)
    dq_state[1] = np.clip(dq_state[1], -0.5, 0.5)
    q_state += dt * dq_state
    #print(f"qstate = {q_state}")
    x1, y1, x2, y2 = Forward_Kinemetic(q_state)
    distance = ((position[0]-x2)**2 + (position[1] - y2)**2)**0.5
    position[0] = x2
    position[1] = y2
    # x[2] = q_state[0]
    # x[3] = q_state[1]
    # x[4] = dq_state[0]
    # x[5] = dq_state[1]
    
    #set next state
    next_state = np.concatenate((position, q_state, dq_state), axis = 0)
    x = next_state
    print(f"k = {k}, x2 = {x[0]:.10f}, y2 = {x[1]:.10f}, qstate = {x[2:4]}, dq_state = {x[4:6]} optINPUT = {sampling_best_input[0]}, dis = {distance}")
    
    # if k == 1:
    #     continue
    rq_rec[k, :] = r
    rx_rec[k, :] = XE
    ry_rec[k, :] = YE
    x_rec[k, :] = [x1, x2]
    y_rec[k, :] = [y1, y2]
    q_rec[k, :] = q_state
    u_rec[k, :] = optimal_input
    t_rec[k] = t


    # check 1 sample traj
         
    # trajectory 기록
    # for i in range(len(optimal_traj)):
    #     best_rec[i][0] = optimal_traj[i][0]
    #     best_rec[i][1] = optimal_traj[i][1]

    # for i in range(len(sampled_traj_list)):
    #     for j in range(len(sampled_traj_list[0])):
            
    #         sample_rec[i][j][0] = sampled_traj_list[i][j][0]
    #         sample_rec[i][j][1] = sampled_traj_list[i][j][1]


    # # optimal, sample trajectory 확인
    # print("best_rec")
    # for i in range(len(best_rec)):
    #     print(f"i = {i} //{best_rec[i][0]}, {best_rec[i][1]}")

    # for i in range(len(sample_rec)):
    #     for j in range(len(sample_rec[0])):
            
    #         print(f"i = {i} | j = {j} //{sample_rec[i][j][0]}, {sample_rec[i][j][1]}")

    # #여기부터 수정
    # Joint_1 = [0, 0]
    # Joint_2 = [1, 0]
    # Joint_3 = [2, 0]
    # fig, ax = plt.subplots()
    # ax.set_xlim(x[0]- 2*distance, x[0]+2*distance)
    # ax.set_ylim(x[1]-2*distance, x[1]+2*distance)
    # ax.grid(True)
    # ax.set_xlabel('X (m)')
    # ax.set_ylabel('Y (m)')
    # ax.set_title('Robot Movement')
    # Target_path, = ax.plot(ref_path[:, 0], ref_path[:, 1], 'o-b', linewidth=0.5)
    
    # colors = plt.cm.jet(np.linspace(0, 1, len(sample_rec))) #다른 색 사용하려고 구분

    # for i in range(len(sampled_traj_list)):
    #     #sample_path, = ax.plot(sample_rec[i,:,0], sample_rec[i,:,1], color = colors[i], linewidth=0.5)
    #     sample_path, = ax.plot(sampled_traj_list[i,:,0], sampled_traj_list[i,:,1], color = 'gray', linewidth=0.5)
    # best_path, = ax.plot(optimal_traj[:,0], optimal_traj[:,1], '--r',linewidth=3) #best rec는 optimal traj
    # #print(x_rec[:, 1])
    # position_path = ax.plot(x_rec[0:k,1], y_rec[0:k, 1], '--g', linewidth=3) #green은 traj 이동 경로
    # sampling_best = ax.plot(sampling_best_traj[:,0], sampling_best_traj[:, 1], color = 'orange', linewidth = 2, linestyle = '--') #sampling_best_traj는 traj 중 최고
    # #print(x_rec[2+k-2:2+k-1, 1], y_rec[2+k-2:2+k-1, 1])
    # #print(sampling_best_traj)
    # #print(best_rec)
    # plt.show()

    ##이동 이상함 더 움 직 이 는 듯         
    

    

    




############################
Joint_1 = [0, 0]
Joint_2 = [1, 0]
Joint_3 = [2, 0]
fig, ax = plt.subplots()

#set style
#plt.style.use('default')

ax.axis('equal')
#원래는 다 3
#비교용 1.39, 1.42, 0.770, 0.820
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
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
Target_path, = ax.plot(ref_path[:, 0], ref_path[:, 1], '--b')

# # check 1 sample traj
# #첫번째 경우만 사용해야됨. 
# best_path, = ax.plot(best_rec[:,0], best_rec[:,1], '--k',)
# colors = plt.cm.jet(np.linspace(0, 1, len(sample_rec))) #다른 색 사용하려고 구분

# for i in range(len(sample_rec)):
#     sample_path, = ax.plot(sample_rec[i,:,0], sample_rec[i,:,1], color = colors[i])


path_x, path_y = [], []

# # save reference trajectory
# a1 = np.array(rx_rec[3:, 0]).reshape(-1, 1)
# a2 = np.array(ry_rec[3:, 0]).reshape(-1, 1)
# a3 = np.array(rq_rec[3:, 0]).reshape(-1, 1)
# a4 = np.array(rq_rec[3:, 1]).reshape(-1, 1)
# # print(a1.shape)
# # print(a2.shape)
# # print(a3.shape)
# # print(a4.shape)
# a5 = np.concatenate((a1, a2), axis = 1)
# a6 = np.concatenate((a5, a3), axis = 1)
# a7 = np.concatenate((a6, a4), axis = 1) #1998, 1 and x, y, q1, q2의 추종값
# #print(a7.shape)
# df = pd.DataFrame(a7)

# df.to_csv('reference.csv')
# #중간에 1.4,0.8, 2.0.0 여러개 있어서 하나씩만 빼고 다 지움
# index가 안들어 있어서 x, y 따로 설정해서 저장하긴 함. 근데 없어도 무방. 그리고 columns 번호 삭제 해야됨
# '''

# #save u_rec
# b1 = np.array(u_rec[4:,0]).reshape(-1, 1)
# b2 = np.array(u_rec[4:,1]).reshape(-1, 1)
# b3 = np.concatenate((b1, b2), axis = 1)
# df1 = pd.DataFrame(b3)
# df1.to_csv("u_ref")

def update(frame):
    Robot_X1 = x_rec[frame, 0]
    Robot_Y1 = y_rec[frame, 0]
    Robot_X2 = x_rec[frame, 1]
    Robot_Y2 = y_rec[frame, 1]

    Robot_arm_1.set_data([Joint_1[0], Robot_X1], [Joint_1[1], Robot_Y1])
    Robot_arm_2.set_data([Robot_X1, Robot_X2], [Robot_Y1, Robot_Y2])
    Robot_path.set_data([Robot_X2], [Robot_Y2])

    path_x.append(Robot_X2)
    path_y.append(Robot_Y2)
    Robot_path.set_data(path_x, path_y)

    return Robot_arm_1, Robot_arm_2, Robot_path


ani = animation.FuncAnimation(fig, update, frames=range(
    0, int(iter)+1, 10), blit=True, interval=100, repeat=True)
#간격은 frame으로 시간은 interval
plt.show()

##########################
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
