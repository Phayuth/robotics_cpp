import os
import matplotlib.pyplot as plt
import numpy as np


def times_zero_to_one(numseg):
    time = np.linspace(0, 1, num=numseg)
    return time


def plot_joint_times(joint_path, times):
    numseg, numjoints = joint_path.shape
    fig, axs = plt.subplots(numjoints, 1, sharex=True)
    for i in range(numjoints):
        axs[i].plot(
            times,
            joint_path[:, i],
            color="blue",
            marker="o",
            linestyle="dashed",
            linewidth=2,
            markersize=12,
            label=f"Joint {i+1} Position",
        )
        axs[i].set_ylabel(f"Joint {i+1}")
        axs[i].set_xlim(times[0], times[-1])
        axs[i].set_ylim(-np.pi, np.pi)
        axs[i].legend(loc="upper right")
        axs[i].grid(True)
    axs[-1].set_xlabel("Time")
    fig.suptitle("Joint Position")
    plt.show()


def plot_joint_so2s():
    path = os.environ["RSRC_DIR"] + "/rnd_torus/"
    joint12 = np.loadtxt(path + "paper_so2s_path.csv", delimiter=",")
    times = times_zero_to_one(joint12.shape[0])
    plot_joint_times(joint12, times)


if __name__ == "__main__":
    plot_joint_so2s()
