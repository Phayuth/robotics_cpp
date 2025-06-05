import os
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from spatial_geometry.utils import Utils


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
graph = nx.read_graphml(rsrc + "paper_so2s_planner_data.graphml")
path = np.loadtxt(rsrc + "paper_so2s_path.csv", delimiter=",")
state = np.loadtxt(rsrc + "paper_so2s_start_goal.csv", delimiter=",")
colp = np.load(rsrc + "collisionpoint_so2s.npy")

# plotting
plt.figure(figsize=(8, 8))
plt.tight_layout()

limt2 = np.array([[-2 * np.pi, 2 * np.pi], [-2 * np.pi, 2 * np.pi]])
# tree
for u, v in graph.edges:
    u = graph.nodes[u]["coords"].rsplit(",")
    u = np.array(u).astype(np.float32).reshape(2, 1)
    v = graph.nodes[v]["coords"].rsplit(",")
    v = np.array(v).astype(np.float32).reshape(2, 1)
    quvw = Utils.nearest_qb_to_qa(u, v, limt2, ignoreOrginal=False)
    qvuw = Utils.nearest_qb_to_qa(v, u, limt2, ignoreOrginal=False)
    plt.plot(
        [u[0], quvw[0]],
        [u[1], quvw[1]],
        color=PlotterConfig.treeColor,
        linewidth=PlotterConfig.globalLinewidth,
        marker=PlotterConfig.treeMarker,
        markerfacecolor=PlotterConfig.treeFaceColor,
        markersize=PlotterConfig.treeMarkersize,
    )
    plt.plot(
        [v[0], qvuw[0]],
        [v[1], qvuw[1]],
        color=PlotterConfig.treeColor,
        linewidth=PlotterConfig.globalLinewidth,
        marker=PlotterConfig.treeMarker,
        markerfacecolor=PlotterConfig.treeFaceColor,
        markersize=PlotterConfig.treeMarkersize,
    )


# path
for i in range(path.shape[0] - 1):
    u = path[i].reshape(2, 1)
    v = path[i + 1].reshape(2, 1)
    quvw = Utils.nearest_qb_to_qa(u, v, limt2, ignoreOrginal=False)
    qvuw = Utils.nearest_qb_to_qa(v, u, limt2, ignoreOrginal=False)
    plt.plot(
        [u[0], quvw[0]],
        [u[1], quvw[1]],
        color=PlotterConfig.pathColor,
        linewidth=PlotterConfig.globalLinewidth,
        marker=PlotterConfig.pathMarker,
        markerfacecolor=PlotterConfig.pathFaceColor,
        markersize=PlotterConfig.pathMarkersize,
    )
    plt.plot(
        [v[0], qvuw[0]],
        [v[1], qvuw[1]],
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

plt.plot(
    colp[:, 0],
    colp[:, 1],
    color="darkcyan",
    linewidth=0,
    marker="o",
    markerfacecolor="darkcyan",
    markersize=1.5,
)

plt.xlim((-np.pi, np.pi))
plt.ylim((-np.pi, np.pi))
plt.show()
