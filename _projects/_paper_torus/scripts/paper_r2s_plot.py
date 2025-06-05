import os
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np


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


rsrc = os.environ["RSRC_DIR"] + "/rnd_torus/"
graph = nx.read_graphml(rsrc + "paper_r2s_planner_data.graphml")
path = np.loadtxt(rsrc + "paper_r2s_path.csv", delimiter=",")
state = np.loadtxt(rsrc + "paper_r2s_start_goal.csv", delimiter=",")
colp = np.load(rsrc + "collisionpoint_so2s.npy")

# tree
for u, v in graph.edges:
    u = graph.nodes[u]["coords"].rsplit(",")
    v = graph.nodes[v]["coords"].rsplit(",")
    plt.plot(
        [float(u[0]), float(v[0])],
        [float(u[1]), float(v[1])],
        color=PlotterConfig.treeColor,
        linewidth=PlotterConfig.globalLinewidth,
        marker=PlotterConfig.treeMarker,
        markerfacecolor=PlotterConfig.treeFaceColor,
        markersize=PlotterConfig.treeMarkersize,
    )

# path
plt.plot(
    path[:, 0],
    path[:, 1],
    color=PlotterConfig.pathColor,
    linewidth=PlotterConfig.globalLinewidth,
    marker=PlotterConfig.pathMarker,
    markerfacecolor=PlotterConfig.pathFaceColor,
    markersize=PlotterConfig.pathMarkersize,
)

# state start
plt.plot(
    state[0, 0],
    state[0, 1],
    color=PlotterConfig.stateStartColor,
    linewidth=0,
    marker=PlotterConfig.stateMarker,
    markerfacecolor=PlotterConfig.stateStartFaceColor,
    markersize=PlotterConfig.stateMarkersize,
)
# state goals
plt.plot(
    state[1:, 0],
    state[1:, 1],
    color=PlotterConfig.stateGoalColor,
    linewidth=0,
    marker=PlotterConfig.stateMarker,
    markerfacecolor=PlotterConfig.stateGoalFaceColor,
    markersize=PlotterConfig.stateMarkersize,
)


plt.show()
