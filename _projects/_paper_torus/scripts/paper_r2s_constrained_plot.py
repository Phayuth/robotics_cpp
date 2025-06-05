import os
import matplotlib.pyplot as plt
from simulator.sim_planar_rr import RobotArm2DSimulator
import numpy as np
from task_map import PaperTorusIFAC2025


def generate_cspace_point():
    """
    generate cspace figure with so2(minimal space), extended space.
    switch between torus=true/false for the corresponding
    """
    torus = True
    env = RobotArm2DSimulator(PaperTorusIFAC2025(), torusspace=torus)
    colp = env.generate_collision_points()
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
    if torus:
        np.save(rsrc + "collisionpoint_exts.npy", colp)
    else:
        np.save(rsrc + "collisionpoint_so2s.npy", colp)


def view_cspace_point():
    """
    view the generated cspace points.
    """
    torus = True
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"

    if torus:
        colp = np.load(rsrc + "collisionpoint_exts.npy")
    else:
        colp = np.load(rsrc + "collisionpoint_so2s.npy")

    fig, ax = plt.subplots(1, 1)
    ax.plot(colp[:, 0], colp[:, 1], "o", markersize=1, color="gray", alpha=0.5)
    if torus:
        ax.set_xlim([-2 * np.pi, 2 * np.pi])
        ax.set_ylim([-2 * np.pi, 2 * np.pi])
        ax.axhline(y=np.pi, color="gray", alpha=0.4)
        ax.axhline(y=-np.pi, color="gray", alpha=0.4)
        ax.axvline(x=np.pi, color="gray", alpha=0.4)
        ax.axvline(x=-np.pi, color="gray", alpha=0.4)
    else:
        ax.set_xlim([-np.pi, np.pi])
        ax.set_ylim([-np.pi, np.pi])
    plt.show()


def fig_constrained_task():
    """
    generate and visualize figure with minimal and extended space.
    """
    torus = True
    env = RobotArm2DSimulator(PaperTorusIFAC2025(), torusspace=torus)
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
    thetas = np.loadtxt(rsrc + "paper_r2s_constrained_path.csv", delimiter=",")
    print(thetas.shape)
    env.plot_view(thetas.T)

    fig, ax = plt.subplots(1, 1)
    colp = np.load(rsrc + "collisionpoint_exts.npy")
    ax.plot(colp[:, 0], colp[:, 1], "o", markersize=1, color="gray", alpha=0.5)
    ax.plot(thetas[:, 0], thetas[:, 1], "o-", markersize=2, color="red", alpha=0.5)
    if torus:
        ax.set_xlim([-2 * np.pi, 2 * np.pi])
        ax.set_ylim([-2 * np.pi, 2 * np.pi])
        ax.axhline(y=np.pi, color="gray", alpha=0.4)
        ax.axhline(y=-np.pi, color="gray", alpha=0.4)
        ax.axvline(x=np.pi, color="gray", alpha=0.4)
        ax.axvline(x=-np.pi, color="gray", alpha=0.4)
    else:
        ax.set_xlim([-np.pi, np.pi])
        ax.set_ylim([-np.pi, np.pi])
    plt.show()


if __name__ == "__main__":
    # fig_cspace_2d()
    # view_cspace_point()
    fig_constrained_task()
