import math

import numpy as np
from sys_params import SYS_PARAMS
from typing import Tuple
import copy

params = SYS_PARAMS()
m1 = params['m1']
m2 = params['m2']
l1 = params['l1']
l2 = params['l2']
lc1 = params['lc1']
lc2 = params['lc2']
g = params['g']

dt = params['Ts']

def Arm_Dynamic(q, dq, u):
    M11 = m1 * lc1 ** 2 + l1 + m2 * \
        (l1 ** 2 + lc2 ** 2 + 2 * l1 * lc2 * np.cos(q[1])) + l2
    M22 = m2 * lc2 ** 2 + l2
    M12 = m2 * l1 * lc2 * np.cos(q[1]) + m2 * lc2 ** 2 + l2
    M21 = m2 * l1 * lc2 * np.cos(q[1]) + m2 * lc2 ** 2 + l2
    M = np.array([[M11, M12], [M21, M22]])
    h = m2 * l1 * lc2 * np.sin(q[1])
    g1 = m1 * lc1 * g * np.cos(q[0]) + m2 * g * \
        (lc2 * np.cos(q[0] + q[1]) + l1 * np.cos(q[0]))
    g2 = m2 * lc2 * g * np.cos(q[0] + q[1])
    G = np.array([g1, g2])
    C = np.array([[-h * dq[1], -h * dq[0] - h * dq[1]], [h * dq[0], 0]])
    ddq = np.linalg.inv(M).dot(u - C.dot(dq) - G)

    return ddq


def Forward_Kinemetic(q):
    # print(f"q = {q}")
    if np.isnan(q[0]):
        print("nan")
        raise IndexError
    x1 = l1 * np.cos(q[0])
    y1 = l1 * np.sin(q[0])
    x2 = l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1])
    y2 = l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1])

    return x1, y1, x2, y2


def Inverse_Kinemetic(Theta):
    l1 = 1
    l2 = 1

    XE = 0.8 + 0.6 * np.cos(Theta)
    YE = 0.8 + 0.6 * np.sin(Theta)
    if 2 * np.pi - 0.2 <= Theta <= 2 * np.pi + 0.2:
        XE = 1.4
        YE = 0.8
    if Theta > 2 * np.pi + 0.2:
        XE = 2
        YE = 0

    term = np.sqrt(
        -XE ** 4 - 2 * XE ** 2 * YE ** 2 + 2 * XE ** 2 * l1 ** 2 + 2 * XE ** 2 * l2 ** 2 - YE ** 4 + 2 * YE ** 2 * l1 ** 2 + 2 * YE ** 2 * l2 ** 2 - l1 ** 4 + 2 * l1 ** 2 * l2 ** 2 - l2 ** 4)
    x1d = 2 * np.arctan((2 * YE * l1 + term) / (XE ** 2 +
                        2 * XE * l1 + YE ** 2 + l1 ** 2 - l2 ** 2))
    x2d = 2 * np.arctan((2 * YE * l1 - term) / (XE ** 2 +
                        2 * XE * l1 + YE ** 2 + l1 ** 2 - l2 ** 2))

    r = np.array([x1d, x2d - x1d])
    return r, XE, YE


def Feedback_linearization(x, v):

    M11 = m1 * lc1 ** 2 + l1 + m2 * \
        (l1 ** 2 + lc2 ** 2 + 2 * l1 * lc2 * np.cos(x[1])) + l2
    M22 = m2 * lc2 ** 2 + l2
    M12 = m2 * l1 * lc2 * np.cos(x[1]) + m2 * lc2 ** 2 + l2
    M21 = m2 * l1 * lc2 * np.cos(x[1]) + m2 * lc2 ** 2 + l2
    M = np.array([[M11, M12], [M21, M22]])

    h = m2 * l1 * lc2 * np.sin(x[1])
    g1 = m1 * lc1 * g * np.cos(x[0]) + m2 * g * \
        (lc2 * np.cos(x[0] + x[1]) + l1 * np.cos(x[0]))
    g2 = m2 * lc2 * g * np.cos(x[0] + x[1])
    G = np.array([g1, g2])

    C = np.array([[-h * x[3], -h * x[2] - h * x[3]], [h * x[2], 0]])

    u = np.dot(M, v) + np.dot(C, x[2:4]) + G
    #print(u)
    return u


def Controller(x, r, dr, ddr):
    KD = 20
    KP = 100
    TH = x[0:2] - r
    dTH = x[2:4] - dr
    v = ddr - (KD*(dTH)) - (KP*(TH))
    return v

class MPPIControllerForRobotArm():
    def __init__(
            self,
            max_u1 = 20, #값 보고 임의로 설정
            max_u2 = 10, #값 보고 임의로 설정
            ref_path: np.ndarray = np.array([[0.0, 0.0, 0.0, 1.0], [10.0, 0.0, 0.0, 1.0]]),
            horizon_step_T : int = 20,
            number_of_samples_K : int = 100,
            sigma: np.ndarray = np.array([[5, 3], [0.5, 0.5]]),

            #지금까지중에는 5, 3, 1, 1이 가장 좋은듯
            stage_cost_weight: np.ndarray = np.array([10.0, 10.0]), # weight for [x, y]
            terminal_cost_weight: np.ndarray = np.array([10.0, 10.0]), # weight for [x, y]
            param_exploration: float = 0.0,
            param_lambda: float = 50.0,
            param_alpha: float = 1.0,
            visualize_optimal_traj = True,  # if True, optimal trajectory is visualized
            visualze_sampled_trajs = True, # if True, sampled trajectories are visualized
    ) -> None:
        """initialize mppi controller for path-tracking"""
        # mppi parameters
        self.dim_x = 6 # dimension of system state vector
        self.dim_u = 2 # dimension of control input vector
        self.T = horizon_step_T # prediction horizon
        self.K = number_of_samples_K # number of sample trajectories
        self.ref_path = ref_path
        self.param_exploration = param_exploration  # constant parameter of mppi
        self.visualize_optimal_traj = visualize_optimal_traj
        self.visualze_sampled_trajs = visualze_sampled_trajs

        self.Sigma = sigma # deviation of noise
        self.stage_cost_weight = stage_cost_weight
        self.terminal_cost_weight = terminal_cost_weight
        self.param_lambda = param_lambda  # constant parameter of mppi
        self.param_alpha = param_alpha # constant parameter of mppi
        self.param_gamma = self.param_lambda * (1.0 - (self.param_alpha))  # constant parameter of mppi

        # mppi variables
        #self.u_prev = np.zeros((self.T, self.dim_u))

        self.u_prev = np.array([[11.0, 5.0] for i in range(self.T)])
        #중력때문에 시스템이 영향을 받기는 하는 것 같다. 그냥 느낌.
        #u는 진짜 많이 변해야 1정도 변함
        #원본 trajectory
        #k = 1 u = [10.35226742  4.51407217] x = 1.39993 y = 0.80897 q = [ 1.1532978  -1.25862889] dq = [-0.00037483 -0.00248918] ddq = [-0.03748314 -0.24891826]
        self.max_u1 = max_u1 # [rad]
        self.max_u2 = max_u2 # [m/s^2]
        # ref_path info
        self.prev_waypoints_idx = 0
    


    def calc_control_input(self, observed_x: np.ndarray):
            """calculate optimal control input"""
            # load privious control input sequence
            
            u = self.u_prev
            # print(u)
            # print("-----------------")
            # set initial x value from observation
            x0 = observed_x #[x, y, q1, q2, dq1, dq2]

            # get the waypoint closest to current vehicle position 
            self._get_nearest_waypoint(x0[0], x0[1], update_prev_idx=True)
            # print(f'x = {x0[0]}, y = {x0[1]}')
            # print(self.prev_waypoints_idx)

            #print(f"serch x = {x0[0]}, y = {x0[1]}, nearestPoint = {self.prev_waypoints_idx}")
            if self.prev_waypoints_idx >= self.ref_path.shape[0]-1:
                print("[ERROR] Reached the end of the reference path.")
                raise IndexError
            
            # prepare buffer
            S = np.zeros((self.K)) # state cost list

            # sample noise
            epsilon = self._calc_epsilon(self.Sigma, self.K, self.T, self.dim_u) # size is self.K x self.T

            # prepare buffer of sampled control input sequence
            v = np.zeros((self.K, self.T, self.dim_u)) # control input sequence with noise

            # loop for 0 ~ K-1 samples
            for k in range(self.K):         

                # set initial(t=0) state x i.e. observed state of the vehicle
                x = x0
                #print(x)

                #list가 mutable하기 때문에 deepcopy하지 않으면 state 변수들이 폭발한다. 그러니까 k가 변하면 상태가 초기화가 되어야 하는데 안될수 있다. 중요
                position = copy.deepcopy(x[0:2])
                q_state = copy.deepcopy(x[2:4])
                dq_state = copy.deepcopy(x[4:6])
                #print(position, q_state, dq_state)
                # loop for time step t = 1 ~ T
                # print(f"k = {k}, T = {0},ddq = {'-'}, dqstate = {dq_state}, q_state = {q_state}, state = {position}")
                # print(f"*** x = {x}")
                for t in range(1, self.T+1):

                    # get control input with noise
                    if k < (1.0-self.param_exploration)*self.K:
                        v[k, t-1] = u[t-1] + epsilon[k, t-1] # sampling for exploitation
                    else:
                        v[k, t-1] = epsilon[k, t-1] # sampling for exploration

                    # update x
                    # print("********")
                    # print(q_state.shape, dq_state.shape)
                    
                    
                    ddq = Arm_Dynamic(q_state, dq_state, self._g(v[k, t-1]))
                    dq_state += dt * ddq
                    q_state += dt * dq_state
                    
                    x1, y1, x2, y2 = Forward_Kinemetic(q_state)
                    
                    position = [x2, y2]
                    # print(f"k = {k}, T = {t},ddq = {ddq}, dqstate = {dq_state}, q_state = {q_state}, state = {position}")
                    # print(f"k = {k}, T = {t},v = {v[k, t-1]}, epsilon = {epsilon[k, t-1]}, u = {u[t-1]}")
                    # add stage cost
                    S[k] += self._c(position) + self.param_gamma * u[t-1].T @ np.linalg.inv(self.Sigma) @ v[k, t-1]

                # add terminal cost
                S[k] += self._phi(position)

            # compute information theoretic weights for each sample
            w = self._compute_weights(S)

            # calculate w_k * epsilon_k
            w_epsilon = np.zeros((self.T, self.dim_u))
            for t in range(self.T): # loop for time step t = 0 ~ T-1
                for k in range(self.K):
                    w_epsilon[t] += w[k] * epsilon[k, t]

            # apply moving average filter for smoothing input sequence
            w_epsilon = self._moving_average_filter(xx=w_epsilon, window_size=10)

            # update control input sequence
            u += w_epsilon
            #u에 w_epsilon 더해서 optimal input 생성. 그러나 optimal traj는 v를 이용하기 때문에 같다고는 볼 수 없을 것 같음

            # calculate optimal trajectory
            optimal_traj = np.zeros((self.T+1, self.dim_x))
            if self.visualize_optimal_traj:
                x = x0
                position= copy.deepcopy(x[0:2])
                q_state = copy.deepcopy(x[2:4])
                dq_state= copy.deepcopy(x[4:6])
                optimal_traj[0] = x
                for t in range(1, self.T+1):
                    ddq = Arm_Dynamic(q_state, dq_state, self._g(u[t-1]))
                    #ddq = Arm_Dynamic(q_state, dq_state, self._g(v[k, t-1]))
                    dq_state += dt * ddq
                    q_state += dt * dq_state

                    x1, y1, x2, y2 = Forward_Kinemetic(q_state)
                
                    position_opt = [x2, y2]
                    data_rec = [x2, y2, q_state[0], q_state[1], dq_state[0], dq_state[1]]
                    optimal_traj[t] = data_rec
                    #이거 가중치 계산한 샘플링이 아니라 지금 거 중에 가장 좋은거 경로 찍는거임 그래서 샘플 경로 하나랑 똑같음

            # calculate sampled trajectories
            sampled_traj_list = np.zeros((self.K, self.T+1, self.dim_x))
            sorted_idx = np.argsort(S) # sort samples by state cost, 0th is the best sample

            if self.visualze_sampled_trajs:
                for k in sorted_idx:
                    x = x0
                    position = copy.deepcopy(x[0:2])
                    q_state = copy.deepcopy(x[2:4])
                    dq_state = copy.deepcopy(x[4:6])
                    sampled_traj_list[k, 0] = x
                    for t in range(1, self.T+1):

                        ddq = Arm_Dynamic(q_state, dq_state, self._g(v[k, t-1]))
                        dq_state += dt * ddq
                        q_state += dt * dq_state

                        x1, y1, x2, y2 = Forward_Kinemetic(q_state)
                    
                        position = [x2, y2]
                        data_rec = [x2, y2, q_state[0], q_state[1], dq_state[0], dq_state[1]]

                        #x = self._F(x, self._g(v[k, t-1]))
                        sampled_traj_list[k, t] = data_rec

            # # update privious control input sequence (shift 1 step to the left)
            self.u_prev[:-1] = u[1:]
            self.u_prev[-1] = u[-1]

            # print("==============u=================")
            # print(self.u_prev)
            # raise ValueError
            
            # return optimal control input and input sequence
            return u[0], u, optimal_traj, sampled_traj_list
            

    def _get_nearest_waypoint(self, x: float, y: float, update_prev_idx: bool = False):
        """search the closest waypoint to the vehicle on the reference path"""

        SEARCH_IDX_LEN = 50 # [points] forward search range
        prev_idx = self.prev_waypoints_idx
        dx = [(x - ref_x)*1000 for ref_x in self.ref_path[prev_idx:(prev_idx + SEARCH_IDX_LEN), 0]]
        dy = [(y - ref_y)*1000 for ref_y in self.ref_path[prev_idx:(prev_idx + SEARCH_IDX_LEN), 1]]
        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
        min_d = min(d)
        nearest_idx = d.index(min_d) + prev_idx
        #print(nearest_idx)
        # get reference values of the nearest waypoint
        ref_x = self.ref_path[nearest_idx,0]
        ref_y = self.ref_path[nearest_idx,1]
        ref_q1 = self.ref_path[nearest_idx,2]
        ref_q2 = self.ref_path[nearest_idx,3]

        # update nearest waypoint index if necessary
        if update_prev_idx:
            self.prev_waypoints_idx = nearest_idx 
            
        return nearest_idx, ref_x, ref_y, ref_q1, ref_q2
    
    def _calc_epsilon(self, sigma: np.ndarray, size_sample: int, size_time_step: int, size_dim_u: int) -> np.ndarray:
        """sample epsilon"""
        # check if sigma row size == sigma col size == size_dim_u and size_dim_u > 0

        #sigma의 element가 다 같은 행렬이면 안됨
        if sigma.shape[0] != sigma.shape[1] or sigma.shape[0] != size_dim_u or size_dim_u < 1:
            print("[ERROR] sigma must be a square matrix with the size of size_dim_u.")
            raise ValueError

        # sample epsilon
        mu = np.zeros((size_dim_u)) # set average as a zero vector
        #mu = np.full((size_dim_u),100) 

        #어쩌면 mu가 답일까?
        epsilon = np.random.multivariate_normal(mu, sigma, (size_sample, size_time_step))
        return epsilon
    
    def _g(self, v: np.ndarray) -> float:
        """clamp input"""
        # limit control inputs
  
        v[0] = np.clip(v[0], -self.max_u1, self.max_u1) # limit u1 input
        v[1] = np.clip(v[1], -self.max_u2, self.max_u2) # limit u2 input
        
        return v
    
    def _c(self, x_t: np.ndarray) -> float:
        """calculate stage cost"""
        # parse x_t
        x, y= x_t
      

        # calculate stage cost
        _, ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
        stage_cost = self.stage_cost_weight[0]*(x-ref_x)**2 + self.stage_cost_weight[1]*(y-ref_y)**2
                     
        return stage_cost

    def _phi(self, x_T: np.ndarray) -> float:
        """calculate terminal cost"""
        # parse x_T
        x, y = x_T
       

        # calculate terminal cost
        _, ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
        terminal_cost = self.terminal_cost_weight[0]*(x-ref_x)**2 + self.terminal_cost_weight[1]*(y-ref_y)**2
        return terminal_cost
    
    def _compute_weights(self, S: np.ndarray) -> np.ndarray:
        """compute weights for each sample"""
        # prepare buffer
        w = np.zeros((self.K))

        # calculate rho
        rho = S.min()

        # calculate eta
        eta = 0.0
        for k in range(self.K):
            eta += np.exp( (-1.0/self.param_lambda) * (S[k]-rho) )

        # calculate weight
        for k in range(self.K):
            w[k] = (1.0 / eta) * np.exp( (-1.0/self.param_lambda) * (S[k]-rho) )
        return w
    
    def _moving_average_filter(self, xx: np.ndarray, window_size: int) -> np.ndarray:
        """apply moving average filter for smoothing input sequence
        Ref. https://zenn.dev/bluepost/articles/1b7b580ab54e95
        """
        #xx (T, dimu)
        b = np.ones(window_size)/window_size
        dim = xx.shape[1]
        xx_mean = np.zeros(xx.shape)

        for d in range(dim):
            xx_mean[:,d] = np.convolve(xx[:,d], b, mode="same")
            n_conv = math.ceil(window_size/2)
            xx_mean[0,d] *= window_size/n_conv
            for i in range(1, n_conv):
                xx_mean[i,d] *= window_size/(i+n_conv)
                xx_mean[-i,d] *= window_size/(i + n_conv - (window_size % 2)) 
        return xx_mean