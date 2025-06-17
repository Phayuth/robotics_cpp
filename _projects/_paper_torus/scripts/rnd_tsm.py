import pyomo.environ as pyo
import math
import matplotlib.pyplot as plt

# ------------------------
# 1. Define city coordinates
# ------------------------
coords = {
    0: (0, 0),
    1: (1, 5),
    2: (5, 2),
    3: (6, 6),
    4: (8, 3)
}
cities = list(coords.keys())

# ------------------------
# 2. Compute Euclidean distances
# ------------------------
def euclidean(i, j):
    xi, yi = coords[i]
    xj, yj = coords[j]
    return math.hypot(xi - xj, yi - yj)

distances = {(i, j): euclidean(i, j) for i in cities for j in cities if i != j}

# ------------------------
# 3. Pyomo Model
# ------------------------
model = pyo.ConcreteModel()
model.N = pyo.Set(initialize=cities)
model.x = pyo.Var(model.N, model.N, domain=pyo.Binary)
model.u = pyo.Var(model.N, domain=pyo.NonNegativeReals, bounds=(0, len(cities) - 1))

# Objective: minimize total distance
def obj_rule(model):
    return sum(distances[i, j] * model.x[i, j] for i in model.N for j in model.N if i != j)
model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

# Constraints
def no_self_loop(model, i):
    return model.x[i, i] == 0
model.no_self = pyo.Constraint(model.N, rule=no_self_loop)

def leave_once(model, i):
    return sum(model.x[i, j] for j in model.N if i != j) == 1
model.leave = pyo.Constraint(model.N, rule=leave_once)

def enter_once(model, j):
    return sum(model.x[i, j] for i in model.N if i != j) == 1
model.enter = pyo.Constraint(model.N, rule=enter_once)

# MTZ subtour elimination
def mtz_constraint(model, i, j):
    if i == 0 or j == 0 or i == j:
        return pyo.Constraint.Skip
    return model.u[i] - model.u[j] + len(cities) * model.x[i, j] <= len(cities) - 1
model.mtz = pyo.Constraint(model.N, model.N, rule=mtz_constraint)

# ------------------------
# 4. Solve the model
# ------------------------
solver = pyo.SolverFactory('glpk')  # Use 'cbc' or 'gurobi' if installed
solver.solve(model, tee=False)

# ------------------------
# 5. Extract the tour
# ------------------------
tour = []
current = 0
visited = set([current])
while True:
    for j in cities:
        if j != current and pyo.value(model.x[current, j]) > 0.5:
            tour.append((current, j))
            current = j
            visited.add(j)
            break
    if len(visited) == len(cities):
        tour.append((current, 0))  # return to start
        break

print("Tour order:")
for i, j in tour:
    print(f"{i} -> {j}")

# ------------------------
# 6. Plot the tour
# ------------------------
plt.figure(figsize=(8,6))
for i, (x, y) in coords.items():
    plt.plot(x, y, 'bo')
    plt.text(x + 0.2, y + 0.2, str(i), fontsize=12)

for i, j in tour:
    xi, yi = coords[i]
    xj, yj = coords[j]
    plt.plot([xi, xj], [yi, yj], 'r-')

plt.title("TSP Solution")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.axis("equal")
plt.show()
