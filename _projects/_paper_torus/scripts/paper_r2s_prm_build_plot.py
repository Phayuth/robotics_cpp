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

plt.plot(
    colp[:, 0],
    colp[:, 1],
    color="darkcyan",
    linewidth=0,
    marker="o",
    markerfacecolor="darkcyan",
    markersize=1.5,
)

plt.show()
