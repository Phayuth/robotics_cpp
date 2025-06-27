import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from sklearn.manifold import TSNE

# Step 1: Generate n nodes in R^6
n = 100
np.random.seed(42)
X = np.random.rand(n, 6)  # Each row is a node in R^6

# Step 2: Create a fully connected graph
G = nx.complete_graph(n)

# Step 3: Reduce 6D positions to 2D using t-SNE
X_2d = TSNE(n_components=2, perplexity=3, random_state=42).fit_transform(X)

# Step 4: Assign positions to nodes
pos = {i: X_2d[i] for i in range(n)}

# Step 5: Draw graph
plt.figure(figsize=(8, 6))
nx.draw(G, pos, with_labels=True, node_color="skyblue", edge_color="gray")
plt.title("6D Fully Connected Graph Visualized in 2D via t-SNE")
plt.axis("equal")
plt.show()
