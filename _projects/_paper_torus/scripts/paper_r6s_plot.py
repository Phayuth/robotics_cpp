import os
import matplotlib.pyplot as plt
from simulator.sim_planar_6r import RobotArm6RSimulator
import numpy as np
from task_map import PaperTorusIFAC2025


torus = True
env = RobotArm6RSimulator(PaperTorusIFAC2025(), torusspace=torus)
rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
thetas = np.loadtxt(rsrc + "paper_r6s_path.csv", delimiter=",")
print(thetas.shape)
env.plot_view(thetas)
