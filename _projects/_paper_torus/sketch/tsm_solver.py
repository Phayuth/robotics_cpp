import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis
from scipy.spatial.transform import Rotation as R


def generate_random_task_transformation():
    translation = np.random.uniform(-1, 1, size=(3,))
    rotation = np.random.uniform(-np.pi, np.pi, size=(4,))
    transformation = np.eye(4)
    RR = R.from_quat(rotation)
    transformation[:3, :3] = RR.as_matrix()
    transformation[:3, 3] = translation
    return transformation


HH = [generate_random_task_transformation() for _ in range(10)]


def plot_tf(T: list, names: list[str] = None):
    ax = make_3d_axis(ax_s=1, unit="m")
    plot_transform(ax=ax, s=0.5, name="base_frame")  # basis
    if isinstance(T, list):
        for i, t in enumerate(T):
            if names is not None:
                name = names[i]
            else:
                name = f"frame_{i}"
            plot_transform(ax=ax, A2B=t, s=0.1, name=name)
    else:
        plot_transform(ax=ax, A2B=T, s=0.1, name="frame")
    plt.tight_layout()
    plt.show()


plot_tf(HH, names=["H"] * 10)
