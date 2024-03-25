import numpy as np

def TRAJECTORY(t, sim_time, params, trajName):
    if trajName == 'cycloid':
        r = np.array([np.sqrt(t) * np.sin(t), np.sqrt(t) * np.cos(t), 0.5 * t, 0])

    elif trajName == 'hover':
        T = sim_time
        if t <= T / 3:
            r = np.array([t, t, t, 0])
        else:
            r = np.array([T / 3, t, T / 3, 0])

    elif trajName == 'circle':
        r = np.array([np.sin(t), np.cos(t), t, 0])

    elif trajName == 'eight':
        T = sim_time
        radius = 2
        w = 6 * np.pi / T
        if t <= T / 3:
            rx = 0
            ry = 0
            rz = t
        elif t <= 2 * T / 3:
            rx = -radius * np.cos(w * (t - T / 3)) + radius
            ry = radius * np.sin(w * (t - T / 3))
            rz = T / 3
        else:
            rx = radius * np.cos(w * (t - T / 3)) - radius
            ry = radius * np.sin(w * (t - T / 3))
            rz = T / 3
        yawd = 0
        r = np.array([rx, ry, rz, yawd])

    elif trajName == 'rectangle':
        if t < 20:
            x_d = 0
            y_d = 0
            z_d = 10
        elif t < 80:
            x_d = (t - 20) / 5 * np.sin(2 * np.pi * t / 20)
            y_d = (t - 20) / 5 * np.cos(2 * np.pi * t / 20)
            z_d = 20
        else:
            x_d = 12 * np.sin(2 * np.pi * t / 20)
            y_d = 12 * np.cos(2 * np.pi * t / 20)
            z_d = 20

        psi_d = np.pi / 6
        r = np.array([x_d, y_d, z_d, psi_d])

    elif trajName == 'square':
        length = 6
        T = sim_time

        if t <= T / 4:
            vx = -length / (T / 4)
            vy = 0
            x_d = 3 + vx * t
            y_d = 3 + vy * t
        elif t <= T / 2:
            vx = 0
            vy = -length / (T / 4)
            x_d = -3 + vx * t
            y_d = 9 + vy * t
        elif t <= 3 * T / 4:
            vx = length / (T / 4)
            vy = 0
            x_d = -15 + vx * t
            y_d = -3 + vy * t
        else:
            vx = 0
            vy = length / (T / 4)
            x_d = 3 + vx * t
            y_d = -21 + vy * t

        z_d = 1
        psi_d = 0

        r = np.array([x_d, y_d, z_d, psi_d])

    elif trajName == 'boustrophedon':
        ver = 10
        hor = 2
        T = sim_time

        if t >= 0 and t <= T / 14:
            vx = 0
            vy = ver * (14 / T)
            rx = -3 * hor + vx * t
            ry = -ver / 2 + vy * t
        elif t > t/14 and t <= 2 * T / 14:
            vx = hor * (14 / T)
            vy = 0
            rx = -3 * hor + vx * (t - T / 14)
            ry = ver / 2 + vy * (t - T / 14)
        elif t > 2*T/14 and t <= 3 * T / 14:
            vx = 0
            vy = -ver * (14 / T)
            rx = -2 * hor + vx * (t - 2 * T / 14)
            ry = ver / 2 + vy * (t - 2 * T / 14)
        elif t > 3*T/14 and t <= 4 * T / 14:
            vx = hor * (14 / T)
            vy = 0
            rx = -2 * hor + vx * (t - 3 * T / 14)
            ry = -ver / 2 + vy * (t - 3 * T / 14)
        elif t > 4 * T / 14 and t <= 5 * T / 14:
            vx = 0
            vy = ver * (14 / T)
            rx = -hor + vx * (t - 4 * T / 14)
            ry = -ver / 2 + vy * (t - 4 * T / 14)
        elif t > 5 * T / 14 and t <= 6 * T / 14:
            vx = hor * (14 / T)
            vy = 0
            rx = -hor + vx * (t - 5 * T / 14)
            ry = ver / 2 + vy * (t - 5 * T / 14)
        elif t > 6 * T / 14 and t <= 7 * T / 14:
            vx = 0
            vy = -ver * (14 / T)
            rx = 0 + vx * (t - 6 * T / 14)
            ry = ver / 2 + vy * (t - 6 * T / 14)
        elif t > 7 * T / 14 and t <= 8 * T / 14:
            vx = hor * (14 / T)
            vy = 0
            rx = 0 + vx * (t - 7 * T / 14)
            ry = -ver / 2 + vy * (t - 7 * T / 14)
        elif t > 8 * T / 14 and t <= 9 * T / 14:
            vx = 0
            vy = ver * (14 / T)
            rx = hor + vx * (t - 8 * T / 14)
            ry = -ver / 2 + vy * (t - 8 * T / 14)
        elif t > 9 * T / 14 and t <= 10 * T / 14:
            vx = hor * (14 / T)
            vy = 0
            rx = hor + vx * (t - 9 * T / 14)
            ry = ver / 2 + vy * (t - 9 * T / 14)
        elif t > 10 * T / 14 and t <= 11 * T / 14:
            vx = 0
            vy = -ver * (14 / T)
            rx = 2 * hor + vx * (t - 10 * T / 14)
            ry = ver / 2 + vy * (t - 10 * T / 14)
        elif t > 11 * T / 14 and t <= 12 * T / 14:
            vx = hor * (14 / T)
            vy = 0
            rx = 2 * hor + vx * (t - 11 * T / 14)
            ry = -ver / 2 + vy * (t - 11 * T / 14)
        elif t > 12 * T / 14 and t <= 13 * T / 14:
            vx = 0
            vy = ver * (14 / T)
            rx = 3 * hor + vx * (t - 12 * T / 14)
            ry = -ver / 2 + vy * (t - 12 * T / 14)
        elif t > 13*T/14 and t <= T:
            vx = hor * (14 / T)
            vy = 0
            rx = 3 * hor + vx * (t - 13 * T / 14)
            ry = ver / 2 + vy * (t - 13 * T / 14)

        rz = 1
        yawd = 0

    else:
        print('wrong trajectory')
        r = np.zeros(4)

    return r