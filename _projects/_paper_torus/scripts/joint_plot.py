import os
import matplotlib.pyplot as plt
import numpy as np

path = os.environ["RSRC_DIR"] + "/rnd_torus/"
joint12 = np.loadtxt(path + "paper_so2s_path.csv", delimiter=",")


numseg, numjoints = joint12.shape
time = np.linspace(0, 1, num=numseg)
fig, axs = plt.subplots(numjoints, 1, figsize=(10, 15), sharex=True)
for i in range(numjoints):
    axs[i].plot(
        time,
        joint12[:, i],
        color="blue",
        marker="o",
        linestyle="dashed",
        linewidth=2,
        markersize=12,
        label=f"Joint Pos {i+1}",
    )
    axs[i].set_ylabel(f"JPos {i+1}")
    axs[i].set_xlim(time[0], time[-1])
    axs[i].set_ylim(-np.pi, np.pi)
    axs[i].legend(loc="upper right")
    axs[i].grid(True)
axs[-1].set_xlabel("Time")
fig.suptitle("Joint Position")
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()
