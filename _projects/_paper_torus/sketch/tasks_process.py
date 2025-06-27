import numpy as np
from spatial_geometry.utils import Utils
from eaik.IK_DH import DhRobot


def ur5e_dh():
    # Ur5e
    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996])
    alpha = np.array([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])
    a = np.array([0, -0.425, -0.3922, 0, 0, 0])
    bot = DhRobot(alpha, a, d)
    return bot


def solve_fk(bot, angles):
    return bot.fwdKin(angles)


def solve_ik(bot, h):
    sols = bot.IK(h)
    return sols.num_solutions(), sols.Q


def generate_random_tasks(bot, num_tasks=10, seed=42):
    angle = np.random.uniform(-np.pi, np.pi, size=(num_tasks, 6))
    T = []
    for i in range(num_tasks):
        t = solve_fk(bot, angle[i])
        T.append(t)
    return T


def solve_ik_for_tasks(bot, tasksH):
    solutions = []
    for h in tasksH:
        num_sols, q_sols = solve_ik(bot, h)
        solutions.append(q_sols)
    return solutions


def find_alt_configs(Q):
    limt6 = np.array(
        [
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
        ]
    )
    QQQQ = []
    for q in Q:
        alt = Utils.find_alt_config(q, limt6)
        QQQQ.append(alt)
    return QQQQ


bot = ur5e_dh()
n = 3
tasksH = generate_random_tasks(bot, n)
for i in range(n):
    print(f"task {i}: \n{tasksH[i]}")

iksols = solve_ik_for_tasks(bot, tasksH)
print(iksols)
