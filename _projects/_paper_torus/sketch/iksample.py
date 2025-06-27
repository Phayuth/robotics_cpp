import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis


def explore_nullspace():
    from robot.nonmobile.planar_rrr import PlanarRRR

    def nullspace_qdot(Jacobian):
        U, S, Vt = np.linalg.svd(Jacobian)
        null_vec = Vt[-1]
        check = Jacobian @ null_vec
        print("Check Jacobian @ null_vec:\n", check)
        return null_vec / np.linalg.norm(null_vec)

    robot = PlanarRRR()
    q0 = np.array([1.0, -1.0, -1.0]).reshape(-1, 1)
    dt = 0.01
    n = 50
    Q = np.zeros((3, n))
    for i in range(n):
        Jac = robot.jacobian(q0)
        Jacxy = Jac[:2, :]
        qdot = nullspace_qdot(Jacxy)
        q0 = q0 - dt * qdot.reshape(-1, 1)
        Q[:, i, np.newaxis] = q0

    fig, ax = plt.subplots()
    robot.plot_arm(Q, ax)
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    ax.grid()
    ax.set_aspect("equal", adjustable="box")
    plt.show()


def eaik_analytical_solution_dh_ur5():
    from eaik.IK_DH import DhRobot

    # Ur5
    d = np.array([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])
    alpha = np.array([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])
    a = np.array([0, -0.612, -0.573, 0, 0, 0])
    bot = DhRobot(alpha, a, d)
    print(bot.kinematicFamily())
    print(bot.hasKnownDecomposition())

    # angles = np.array([1, 1, 1, 1, 1, 1])
    angles = np.array([0, 0, 0, 0, 0, 0])
    print("Forward kinematics for angles:", angles)

    t = bot.fwdKin(angles)
    print("Forward kinematics result:", t)
    ik_solutions = bot.IK(t)

    print(ik_solutions.num_solutions())
    print(ik_solutions.Q)

    plot_tf(t)


def eaik_analytical_solution_dh_ur5e():
    from eaik.IK_DH import DhRobot

    # Ur5e
    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996])
    alpha = np.array([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])
    a = np.array([0, -0.425, -0.3922, 0, 0, 0])
    bot = DhRobot(alpha, a, d)
    print(bot.kinematicFamily())
    print(bot.hasKnownDecomposition())

    angles = np.array([1, 1, 1, 1, 1, 1])
    # angles = np.array([0, 0, 0, 0, 0, 0])
    print("Forward kinematics for angles:", angles)

    t = bot.fwdKin(angles)
    print("Forward kinematics result:", t)

    ik_solutions = bot.IK(t)
    print("Inverse kinematics solutions:", ik_solutions)

    print(ik_solutions.num_solutions())
    print(ik_solutions.Q)

    plot_tf(t)

    for i in range(ik_solutions.num_solutions()):
        q = ik_solutions.Q[i]
        t = bot.fwdKin(q)
        print(t)


def eaik_analytical_solution_urdf():
    import os
    from eaik.IK_URDF import UrdfRobot

    rsrcpath = os.environ["RSRC_DIR"] + "/rnd_torus/"
    urdfname = rsrcpath + "ur5e/ur5e_extract_calibrated.urdf"
    bot = UrdfRobot(urdfname, [])
    bot.H_remodeled()
    bot.P_remodeled()
    print(bot.kinematicFamily())
    print(bot.hasKnownDecomposition())

    angles = np.array([1, 1, 1, 1, 1, 1])
    print("Forward kinematics for angles: ", angles)

    t = bot.fwdKin(angles)
    print("Forward kinematics result: ", t)

    ik_solutions = bot.IK(t)
    print("Inverse kinematics solutions: ", ik_solutions)


def plot_tf(T: list, names: list[str] = None):
    ax = make_3d_axis(ax_s=1, unit="m")
    plot_transform(ax=ax, s=0.5, name="base_frame")  # basis
    if isinstance(T, list):
        for i, t in enumerate(T):
            if names is not None:
                name = names[i]
            else:
                name = f"frame_{i}"
            plot_transform(ax=ax, A2B=t, s=0.5, name=name)
    else:
        plot_transform(ax=ax, A2B=T, s=0.5, name="frame")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    while True:
        func = [
            explore_nullspace,
            eaik_analytical_solution_dh_ur5,
            eaik_analytical_solution_urdf,
            eaik_analytical_solution_dh_ur5e,
        ]

        for i, f in enumerate(func, start=1):
            print(f"{i}: {f.__name__}")

        arg = input("Enter argument number (` to exit): ")

        if arg == "`":
            print("Exiting...")
            break
        elif arg.isdigit() and 1 <= int(arg) <= len(func):
            func[int(arg) - 1]()
