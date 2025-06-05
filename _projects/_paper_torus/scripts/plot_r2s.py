import os
import matplotlib.pyplot as plt
from simulator.sim_planar_rr import RobotArm2DSimulator
import numpy as np
from rnd_task_map import PaperTorusIFAC2025
import networkx as nx
from rnd_task_map import PaperTorusIFAC2025


class PlotterConfig:
    globalLinewidth = 1

    obstColor = "darkcyan"
    obstFaceColor = "darkcyan"
    obstMarker = "o"
    obstMarkersize = 1.5

    treeColor = "darkgray"
    treeFaceColor = None
    treeMarker = None
    treeMarkersize = None

    stateStartColor = "blue"
    stateStartFaceColor = "yellow"
    stateAppColor = "blue"
    stateAppFaceColor = "green"
    stateGoalColor = "blue"
    stateGoalFaceColor = "red"
    stateMarkersize = 7
    stateMarker = "o"

    pathColor = "blue"
    pathFaceColor = "plum"
    pathMarker = "o"
    pathMarkersize = 7


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


def fig_r2s_cartgoal():
    """
    generate and visualize figure with minimal and extended space.
    """
    torus = True
    env = RobotArm2DSimulator(PaperTorusIFAC2025(), torusspace=torus)
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
    thetas = np.loadtxt(rsrc + "paper_r2s_cartgoal_path.csv", delimiter=",")
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


def fig_r2s_altgoal():
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
    graph = nx.read_graphml(rsrc + "paper_r2s_altgoal_planner_data.graphml")
    path = np.loadtxt(rsrc + "paper_r2s_altgoal_path.csv", delimiter=",")
    state = np.loadtxt(
        rsrc + "paper_r2s_altgoal_start_goal.csv",
        delimiter=",",
    )
    colp = np.load(rsrc + "collisionpoint_exts.npy")

    # task space
    torus = True
    env = RobotArm2DSimulator(PaperTorusIFAC2025(), torusspace=torus)
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
    env.plot_view(path.T)

    fig, ax = plt.subplots(1, 1)
    # tree
    for u, v in graph.edges:
        u = graph.nodes[u]["coords"].rsplit(",")
        v = graph.nodes[v]["coords"].rsplit(",")
        ax.plot(
            [float(u[0]), float(v[0])],
            [float(u[1]), float(v[1])],
            color=PlotterConfig.treeColor,
            linewidth=PlotterConfig.globalLinewidth,
            marker=PlotterConfig.treeMarker,
            markerfacecolor=PlotterConfig.treeFaceColor,
            markersize=PlotterConfig.treeMarkersize,
        )

    # path
    ax.plot(
        path[:, 0],
        path[:, 1],
        color=PlotterConfig.pathColor,
        linewidth=PlotterConfig.globalLinewidth,
        marker=PlotterConfig.pathMarker,
        markerfacecolor=PlotterConfig.pathFaceColor,
        markersize=PlotterConfig.pathMarkersize,
    )

    # state start
    ax.plot(
        state[0, 0],
        state[0, 1],
        color=PlotterConfig.stateStartColor,
        linewidth=0,
        marker=PlotterConfig.stateMarker,
        markerfacecolor=PlotterConfig.stateStartFaceColor,
        markersize=PlotterConfig.stateMarkersize,
    )
    # state goals
    ax.plot(
        state[1:, 0],
        state[1:, 1],
        color=PlotterConfig.stateGoalColor,
        linewidth=0,
        marker=PlotterConfig.stateMarker,
        markerfacecolor=PlotterConfig.stateGoalFaceColor,
        markersize=PlotterConfig.stateMarkersize,
    )

    ax.plot(
        colp[:, 0],
        colp[:, 1],
        color="darkcyan",
        linewidth=0,
        marker="o",
        markerfacecolor="darkcyan",
        markersize=1.5,
    )

    ax.set_xlim((-2 * np.pi, 2 * np.pi))
    ax.set_ylim((-2 * np.pi, 2 * np.pi))
    ax.axhline(y=np.pi, color="gray", alpha=0.4)
    ax.axhline(y=-np.pi, color="gray", alpha=0.4)
    ax.axvline(x=np.pi, color="gray", alpha=0.4)
    ax.axvline(x=-np.pi, color="gray", alpha=0.4)
    plt.show()


def fig_r2s_snggoal():
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
    graph = nx.read_graphml(rsrc + "paper_r2s_snggoal_planner_data.graphml")
    path = np.loadtxt(rsrc + "paper_r2s_snggoal_path.csv", delimiter=",")
    state = np.loadtxt(rsrc + "paper_r2s_snggoal_start_goal.csv", delimiter=",")
    colp = np.load(rsrc + "collisionpoint_exts.npy")

    # task space
    torus = True
    env = RobotArm2DSimulator(PaperTorusIFAC2025(), torusspace=torus)
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
    env.plot_view(path.T)

    fig, ax = plt.subplots(1, 1)
    # tree
    for u, v in graph.edges:
        u = graph.nodes[u]["coords"].rsplit(",")
        v = graph.nodes[v]["coords"].rsplit(",")
        ax.plot(
            [float(u[0]), float(v[0])],
            [float(u[1]), float(v[1])],
            color=PlotterConfig.treeColor,
            linewidth=PlotterConfig.globalLinewidth,
            marker=PlotterConfig.treeMarker,
            markerfacecolor=PlotterConfig.treeFaceColor,
            markersize=PlotterConfig.treeMarkersize,
        )

    # path
    ax.plot(
        path[:, 0],
        path[:, 1],
        color=PlotterConfig.pathColor,
        linewidth=PlotterConfig.globalLinewidth,
        marker=PlotterConfig.pathMarker,
        markerfacecolor=PlotterConfig.pathFaceColor,
        markersize=PlotterConfig.pathMarkersize,
    )

    # state start
    ax.plot(
        state[0, 0],
        state[0, 1],
        color=PlotterConfig.stateStartColor,
        linewidth=0,
        marker=PlotterConfig.stateMarker,
        markerfacecolor=PlotterConfig.stateStartFaceColor,
        markersize=PlotterConfig.stateMarkersize,
    )
    # state goals
    ax.plot(
        state[1:, 0],
        state[1:, 1],
        color=PlotterConfig.stateGoalColor,
        linewidth=0,
        marker=PlotterConfig.stateMarker,
        markerfacecolor=PlotterConfig.stateGoalFaceColor,
        markersize=PlotterConfig.stateMarkersize,
    )
    ax.plot(
        colp[:, 0],
        colp[:, 1],
        color="darkcyan",
        linewidth=0,
        marker="o",
        markerfacecolor="darkcyan",
        markersize=1.5,
    )

    plt.show()


def fig_r2s_prm():
    rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
    graph = nx.read_graphml(rsrc + "saved_planner.graphml")
    colp = np.load(rsrc + "collisionpoint_exts.npy")

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-2 * np.pi, 2 * np.pi)
    ax.set_ylim(-2 * np.pi, 2 * np.pi)
    # tree
    for u, v in graph.edges:
        u = graph.nodes[u]["coords"].rsplit(",")
        v = graph.nodes[v]["coords"].rsplit(",")
        ax.plot(
            [float(u[0]), float(v[0])],
            [float(u[1]), float(v[1])],
            color=PlotterConfig.treeColor,
            linewidth=PlotterConfig.globalLinewidth,
            marker=PlotterConfig.treeMarker,
            markerfacecolor=PlotterConfig.treeFaceColor,
            markersize=PlotterConfig.treeMarkersize,
        )

    ax.plot(
        colp[:, 0],
        colp[:, 1],
        color="darkcyan",
        linewidth=0,
        marker="o",
        markerfacecolor="darkcyan",
        markersize=1.5,
    )

    plt.show()


if __name__ == "__main__":
    while True:
        print("Select a function to execute:")
        print("1. Generate C-Space Point")
        print("2. View C-Space Point")
        print("3. Figure R2S Cart Goal")
        print("4. Figure R2S Alt Goal")
        print("5. Figure R2S Sng Goal")
        print("6. Figure R2S PRM")
        print("0. Exit")

        argid = input("Enter function id (1-6): ")

        if int(argid) == 1:
            generate_cspace_point()
        elif int(argid) == 2:
            view_cspace_point()
        elif int(argid) == 3:
            fig_r2s_cartgoal()
        elif int(argid) == 4:
            fig_r2s_altgoal()
        elif int(argid) == 5:
            fig_r2s_snggoal()
        elif int(argid) == 6:
            fig_r2s_prm()
        elif int(argid) == 0:
            print("Exiting...")
            break
