import pyomo.environ as pyo
import math
import matplotlib.pyplot as plt
import random


def travelling_sale_man_TSM():
    # ------------------------
    # 1. Define city coordinates
    # ------------------------
    coords = {0: (0, 0), 1: (1, 5), 2: (5, 2), 3: (6, 6), 4: (8, 3)}
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
    model.u = pyo.Var(
        model.N, domain=pyo.NonNegativeReals, bounds=(0, len(cities) - 1)
    )

    # Objective: minimize total distance
    def obj_rule(model):
        return sum(
            distances[i, j] * model.x[i, j]
            for i in model.N
            for j in model.N
            if i != j
        )

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
        return (
            model.u[i] - model.u[j] + len(cities) * model.x[i, j]
            <= len(cities) - 1
        )

    model.mtz = pyo.Constraint(model.N, model.N, rule=mtz_constraint)

    # ------------------------
    # 4. Solve the model
    # ------------------------
    solver = pyo.SolverFactory("glpk")  # Use 'cbc' or 'gurobi' if installed
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
    plt.figure(figsize=(8, 6))
    for i, (x, y) in coords.items():
        plt.plot(x, y, "bo")
        plt.text(x + 0.2, y + 0.2, str(i), fontsize=12)

    for i, j in tour:
        xi, yi = coords[i]
        xj, yj = coords[j]
        plt.plot([xi, xj], [yi, yj], "r-")

    plt.title("TSP Solution")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis("equal")
    plt.show()


def generalized_travelling_salesman_GTSM():
    # --------------------------
    # 1. Generate cities & clusters
    # --------------------------
    random.seed(49)
    num_clusters = 5
    cities_per_cluster = 3
    clusters = {}
    coords = {}
    city_id = 0

    for c in range(num_clusters):
        base_x = random.uniform(0, 100)
        base_y = random.uniform(0, 100)
        clusters[c] = []
        for _ in range(cities_per_cluster):
            x = base_x + random.uniform(-5, 5)
            y = base_y + random.uniform(-5, 5)
            coords[city_id] = (x, y)
            clusters[c].append(city_id)
            city_id += 1

    all_cities = list(coords.keys())

    # --------------------------
    # 2. Euclidean distances
    # --------------------------
    def euclidean(i, j):
        xi, yi = coords[i]
        xj, yj = coords[j]
        return math.hypot(xi - xj, yi - yj)

    distances = {
        (i, j): euclidean(i, j) for i in all_cities for j in all_cities if i != j
    }

    # --------------------------
    # 3. Pyomo Model
    # --------------------------
    model = pyo.ConcreteModel()
    model.N = pyo.Set(initialize=all_cities)
    model.x = pyo.Var(model.N, model.N, domain=pyo.Binary)  # tour edges
    model.y = pyo.Var(model.N, domain=pyo.Binary)  # selected cities
    model.u = pyo.Var(
        model.N, domain=pyo.NonNegativeReals, bounds=(0, len(clusters))
    )  # MTZ

    # --------------------------
    # 4. Objective: minimize tour length
    # --------------------------
    def obj_rule(model):
        return sum(
            distances[i, j] * model.x[i, j]
            for i in model.N
            for j in model.N
            if i != j
        )

    model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)

    # --------------------------
    # 5. Constraints
    # --------------------------
    # Visit exactly one city from each cluster
    def one_city_per_cluster(model, c):
        return sum(model.y[i] for i in clusters[c]) == 1

    model.one_per_cluster = pyo.Constraint(
        range(num_clusters), rule=one_city_per_cluster
    )

    # Only allow edges between selected cities
    def only_selected_edges(model, i, j):
        if i == j:
            return model.x[i, j] == 0
        return model.x[i, j] <= model.y[i]

    model.valid_edge = pyo.Constraint(model.N, model.N, rule=only_selected_edges)

    # Enter and leave selected cities exactly once
    def in_out_flow(model, i):
        return sum(model.x[j, i] for j in model.N if j != i) == model.y[i]

    model.inflow = pyo.Constraint(model.N, rule=in_out_flow)

    def out_in_flow(model, i):
        return sum(model.x[i, j] for j in model.N if j != i) == model.y[i]

    model.outflow = pyo.Constraint(model.N, rule=out_in_flow)

    # MTZ constraints to eliminate subtours among selected cities
    def mtz_rule(model, i, j):
        if i == j or i == list(model.N)[0] or j == list(model.N)[0]:
            return pyo.Constraint.Skip
        return (
            model.u[i] - model.u[j] + len(clusters) * model.x[i, j]
            <= len(clusters) - 1
        )

    model.mtz = pyo.Constraint(model.N, model.N, rule=mtz_rule)

    # --------------------------
    # 6. Solve
    # --------------------------
    solver = pyo.SolverFactory("glpk")
    solver.solve(model, tee=False)

    # --------------------------
    # 7. Extract and Plot Tour
    # --------------------------
    # Extract selected cities and edges
    selected_cities = [i for i in all_cities if pyo.value(model.y[i]) > 0.5]
    edges = [
        (i, j)
        for i in selected_cities
        for j in selected_cities
        if i != j and pyo.value(model.x[i, j]) > 0.5
    ]
    print("Selected cities:", selected_cities)
    print("Tour edges:", edges)

    # Reconstruct ordered tour
    tour = []
    current = selected_cities[0]
    for i, j in edges:
        if i == current:
            tour.append((i, j))
            current = j
    tour.append((tour[-1][1], tour[0][0]))
    print("Tour order:", tour)

    # Plot
    plt.figure(figsize=(8, 6))
    colors = ["r", "g", "b", "c", "m", "y"]

    # Plot all cities by cluster
    for c, city_ids in clusters.items():
        for i in city_ids:
            x, y = coords[i]
            plt.plot(x, y, "o", color=colors[c % len(colors)])
            plt.text(x + 0.5, y + 0.5, str(i), fontsize=9)

    # Highlight selected cities
    for i in selected_cities:
        x, y = coords[i]
        plt.plot(x, y, "ko", markersize=10)

    # Draw tour
    for i, j in tour:
        xi, yi = coords[i]
        xj, yj = coords[j]
        plt.plot([xi, xj], [yi, yj], "k-")

    plt.title("GTSP Tour (One City per Cluster)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    while True:
        func = [
            travelling_sale_man_TSM,
            generalized_travelling_salesman_GTSM,
        ]

        for i, f in enumerate(func, start=1):
            print(f"{i}: {f.__name__}")

        arg = input("Enter argument number (` to exit): ")

        if arg == "`":
            print("Exiting...")
            break
        elif arg.isdigit() and 1 <= int(arg) <= len(func):
            func[int(arg) - 1]()
