import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple
from matplotlib import patches
from matplotlib.animation import ArtistAnimation

class robotArm():
    def __init__(
            self,
            
        ) -> None:
        pass

    def reset(
            self,
            init_state: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0]), # [x, y, yaw, v]
        )