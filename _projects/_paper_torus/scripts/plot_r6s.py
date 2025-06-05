import os
import matplotlib.pyplot as plt
from simulator.sim_planar_6r import RobotArm6RSimulator
import numpy as np
from rnd_task_map import PaperTorusIFAC2025


def fig_r6s_snggoal():
    torus = True
    env = RobotArm6RSimulator(PaperTorusIFAC2025(), torusspace=torus)
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
    thetas = np.loadtxt(rsrc + "paper_r6s_snggoal_path.csv", delimiter=",")
    print(thetas.shape)
    env.plot_view(thetas)


def fig_r6s_cartgoal():
    pass


def fig_r6s_altgoal():
    pass


if __name__ == "__main__":
    while True:
        print("1: fig_r6s_snggoal")
        print("2: fig_r6s_cartgoal")
        print("3: fig_r6s_altgoal")
        print("0: exit")

        choice = input("Select an option: ")
        if choice == "1":
            fig_r6s_snggoal()
        elif choice == "2":
            fig_r6s_cartgoal()
        elif choice == "3":
            fig_r6s_altgoal()
        elif choice == "0":
            break
