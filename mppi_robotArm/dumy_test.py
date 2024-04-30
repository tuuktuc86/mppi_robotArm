import numpy as np
l1 = 1
l2 = 1
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

_, _, x, y = Forward_Kinemetic([1.15330155041141, -1.25860400808722])
print(x, y)