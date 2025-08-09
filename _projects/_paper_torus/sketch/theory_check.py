import numpy as np
import matplotlib.pyplot as plt
from spatial_geometry.utils import Utils
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


def sampling_test():
    """
    does sample in pi and use find_alt_config fill the rest or not ?
    """
    limt2 = np.array([[-2 * np.pi, 2 * np.pi], [-2 * np.pi, 2 * np.pi]])
    npts = 100
    sample_limit = np.array([[-np.pi, np.pi], [-np.pi, np.pi]])
    sample = np.random.uniform(
        low=sample_limit[:, 0], high=sample_limit[:, 1], size=(npts, 2)
    )

    alt_sample = []
    for samp in sample:
        sampi = Utils.find_alt_config(
            samp.reshape(2, 1), limt2, filterOriginalq=True
        ).T
        alt_sample.append(sampi)
    alt_sample = np.vstack(alt_sample)

    fig, ax = plt.subplots()
    ax.set_xlim(limt2[0, 0], limt2[0, 1])
    ax.set_ylim(limt2[1, 0], limt2[1, 1])
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Sample points in the toroidal space")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid()
    ax.plot(
        alt_sample[:, 0],
        alt_sample[:, 1],
        "bo",
        markersize=2,
        label="Alt sampled points",
    )
    ax.plot(sample[:, 0], sample[:, 1], "ro", markersize=2, label="Sampled points")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axhline(0, color="k", linewidth=5, linestyle="--")
    ax.axvline(0, color="k", linewidth=5, linestyle="--")
    ax.axvline(-np.pi, color="m", linewidth=5, linestyle="--")
    ax.axhline(np.pi, color="c", linewidth=5, linestyle="--")
    ax.axvline(np.pi, color="m", linewidth=5, linestyle="--")
    ax.axhline(-np.pi, color="c", linewidth=5, linestyle="--")
    ax.axvline(-2 * np.pi, color="m", linewidth=5, linestyle="-")
    ax.axhline(2 * np.pi, color="c", linewidth=5, linestyle="-")
    ax.axvline(2 * np.pi, color="m", linewidth=5, linestyle="-")
    ax.axhline(-2 * np.pi, color="c", linewidth=5, linestyle="-")
    ax.set_xlim(-2 * np.pi - 0.1, 2 * np.pi + 0.1)
    ax.set_ylim(-2 * np.pi - 0.1, 2 * np.pi + 0.1)

    ax.legend()
    plt.show()


def determine_quadrant():
    """
    Determine the quadrant of a point in the toroidal space.
    """
    quad1 = np.array([[-np.pi, 0], [0, np.pi]])
    quad2 = np.array([[0, np.pi], [0, np.pi]])
    quad3 = np.array([[-np.pi, 0], [-np.pi, 0]])
    quad4 = np.array([[0, np.pi], [-np.pi, 0]])

    xp = np.array([-2 * np.pi, -np.pi, 0, np.pi, 2 * np.pi])
    yp = np.array([-2 * np.pi, -np.pi, 0, np.pi, 2 * np.pi])
    X, Y = np.meshgrid(xp, yp)
    fig, ax = plt.subplots()
    ax.set_xlim(-2 * np.pi - 0.1, 2 * np.pi + 0.1)
    ax.set_ylim(-2 * np.pi - 0.1, 2 * np.pi + 0.1)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Quadrants in the toroidal space")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.plot(X, Y, "ro", markersize=2, label="Sampled points")
    plt.show()

    box1 = np.array([[-np.pi, 0], [0, np.pi]])
    p = np.array([-1, 1]).reshape(2, 1)
    print(box1)
    print("Point:", p)

    min = box1[:, 0].reshape(2, 1)
    print("min:", min)
    max = box1[:, 1].reshape(2, 1)
    print("max:", max)
    a = p > min
    print(a)
    b = p < max
    print(b)


def find_number_of_feasible_alternatives():
    """
    Find the number of alternative configurations for a given point in the toroidal space.
    """
    lmt2 = np.array(
        [
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
        ]
    )
    lmt3 = np.array(
        [
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
        ]
    )
    lmt4 = np.array(
        [
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
        ]
    )
    lmt5 = np.array(
        [
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
        ]
    )
    lmt6 = np.array(
        [
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
            [-2 * np.pi, 2 * np.pi],
        ]
    )

    q2 = np.array([1, 1]).reshape(2, 1)
    q3 = np.array([1, 1, 1]).reshape(3, 1)
    q4 = np.array([1, 1, 1, 1]).reshape(4, 1)
    q5 = np.array([1, 1, 1, 1, 1]).reshape(5, 1)
    q6 = np.array([1, 1, 1, 1, 1, 1]).reshape(6, 1)

    print(
        "2D fesible #q is :",
        Utils.find_alt_config(q2, lmt2).shape[1],
    )
    print(
        "3D fesible #q is :",
        Utils.find_alt_config(q3, lmt3).shape[1],
    )
    print(
        "4D fesible #q is :",
        Utils.find_alt_config(q4, lmt4).shape[1],
    )
    print(
        "5D fesible #q is :",
        Utils.find_alt_config(q5, lmt5).shape[1],
    )
    print(
        "6D fesible #q is :",
        Utils.find_alt_config(q6, lmt6).shape[1],
    )


if __name__ == "__main__":
    while True:
        func = [
            explore_nullspace,
            eaik_analytical_solution_dh_ur5,
            eaik_analytical_solution_urdf,
            eaik_analytical_solution_dh_ur5e,
            sampling_test,
            determine_quadrant,
            find_number_of_feasible_alternatives,
        ]

        for i, f in enumerate(func, start=1):
            print(f"{i}: {f.__name__}")

        arg = input("Enter argument number (` to exit): ")

        if arg == "`":
            print("Exiting...")
            break
        elif arg.isdigit() and 1 <= int(arg) <= len(func):
            func[int(arg) - 1]()
