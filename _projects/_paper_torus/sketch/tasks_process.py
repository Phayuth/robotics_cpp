import numpy as np
from spatial_geometry.utils import Utils
from eaik.IK_DH import DhRobot

np.set_printoptions(precision=3, suppress=True, linewidth=2000)


def ur5e_dh():
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
    np.random.seed(seed)
    angle = np.random.uniform(-np.pi, np.pi, size=(num_tasks, 6))
    T = []
    for i in range(num_tasks):
        t = solve_fk(bot, angle[i])
        T.append(t)
    return T


def find_alt_configs(Q):
    limt6 = np.array(
        [
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-np.pi, np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
        ]
    )
    Qalts = []
    for q in Q:
        print(q)
        alt = Utils.find_alt_config(q.reshape(6, 1), limt6)
        Qalts.append(alt)
    Qalts = np.hstack(Qalts)
    return Qalts


bot = ur5e_dh()

taskH = generate_random_tasks(bot, 1)
task1 = taskH[0]
print("Task H:\n", task1)

num_sol, ik_sol = solve_ik(bot, task1)
Qalts = find_alt_configs(ik_sol)
print(Qalts.shape)
