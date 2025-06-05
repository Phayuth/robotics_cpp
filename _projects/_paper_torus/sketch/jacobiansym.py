import sympy as sp
import numpy as np


def jacobian_two_joints():
    """
    Computes the Jacobian for a two-joint planar manipulator.
    """
    theta1, theta2 = sp.symbols("theta1 theta2")
    l1, l2 = sp.symbols("l1 l2")

    # Forward kinematics
    x = l1 * sp.cos(theta1) + l2 * sp.cos(theta1 + theta2)
    y = l1 * sp.sin(theta1) + l2 * sp.sin(theta1 + theta2)

    pos = sp.Matrix([x, y])
    joint_angles = sp.Matrix([theta1, theta2])

    J = pos.jacobian(joint_angles)
    J.simplify()

    print("Jacobian J-Two:")
    sp.pprint(J, use_unicode=True)
    return J


def jacobian_three_joints():
    """
    Computes the Jacobian for a three-joint planar manipulator.
    """
    theta1, theta2, theta3 = sp.symbols("theta1 theta2 theta3")
    l1, l2, l3 = sp.symbols("l1 l2 l3")

    # Forward kinematics
    x = (
        l1 * sp.cos(theta1)
        + l2 * sp.cos(theta1 + theta2)
        + l3 * sp.cos(theta1 + theta2 + theta3)
    )
    y = (
        l1 * sp.sin(theta1)
        + l2 * sp.sin(theta1 + theta2)
        + l3 * sp.sin(theta1 + theta2 + theta3)
    )
    phi = theta1 + theta2 + theta3

    pos = sp.Matrix([x, y, phi])
    joint_angles = sp.Matrix([theta1, theta2, theta3])

    J = pos.jacobian(joint_angles)
    J.simplify()

    print("Jacobian J-Three:")
    sp.pprint(J, use_unicode=True)
    return J


def jacobian_six_joints():
    """
    Computes the Jacobian for a six-joint manipulator.
    """
    theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols(
        "theta1 theta2 theta3 theta4 theta5 theta6"
    )
    l1, l2, l3, l4, l5, l6 = sp.symbols("l1 l2 l3 l4 l5 l6")

    # Forward kinematics (example for a 6-DOF manipulator)
    x = (
        l1 * sp.cos(theta1)
        + l2 * sp.cos(theta1 + theta2)
        + l3 * sp.cos(theta1 + theta2 + theta3)
        + l4 * sp.cos(theta1 + theta2 + theta3 + theta4)
        + l5 * sp.cos(theta1 + theta2 + theta3 + theta4 + theta5)
        + l6 * sp.cos(theta1 + theta2 + theta3 + theta4 + theta5 + theta6)
    )
    y = (
        l1 * sp.sin(theta1)
        + l2 * sp.sin(theta1 + theta2)
        + l3 * sp.sin(theta1 + theta2 + theta3)
        + l4 * sp.sin(theta1 + theta2 + theta3 + theta4)
        + l5 * sp.sin(theta1 + theta2 + theta3 + theta4 + theta5)
        + l6 * sp.sin(theta1 + theta2 + theta3 + theta4 + theta5 + theta6)
    )

    phi = theta1 + theta2 + theta3 + theta4 + theta5 + theta6

    pos = sp.Matrix([x, y, phi])
    joint_angles = sp.Matrix([theta1, theta2, theta3, theta4, theta5, theta6])

    J = pos.jacobian(joint_angles)
    J.simplify()

    print("Jacobian J-Six:")
    sp.pprint(J, use_unicode=True)
    return J


def jacobian_six_joints_numpy(thetas, link_lengths):
    thetas = np.asarray(thetas)
    link_lengths = np.asarray(link_lengths)

    cum_thetas = np.cumsum(thetas)
    sin_t = np.sin(cum_thetas)
    cos_t = np.cos(cum_thetas)

    J = np.zeros((3, 6))
    J[2, :] = 1

    for i in range(6):
        J[0, i] = -np.sum(link_lengths[i:] * sin_t[i:])
        J[1, i] = np.sum(link_lengths[i:] * cos_t[i:])

    return J


def jacobian_six_joints_unit_test():
    """
    Unit test for the Jacobian of a six-joint manipulator.
    """
    thetas = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    link_lengths = [1, 1, 1, 1, 1, 1]

    Jnumpy = jacobian_six_joints_numpy(thetas, link_lengths)
    print("Jacobian J-Six (NumPy):")
    print(Jnumpy)

    Jsym = jacobian_six_joints()
    Jsym_np = sp.lambdify(
        (
            sp.symbols("theta1 theta2 theta3 theta4 theta5 theta6"),
            sp.symbols("l1 l2 l3 l4 l5 l6"),
        ),
        Jsym,
        "numpy",
    )
    Jsym_eval = Jsym_np(thetas, link_lengths)
    print("Jacobian J-Six (SymPy):")
    print(Jsym_eval)

    assert np.allclose(Jnumpy, Jsym_eval), "Jacobian matrices do not match!"
    print("Unit test passed: Jacobian matrices match!")


if __name__ == "__main__":
    # jacobian_two_joints()
    # jacobian_three_joints()
    # jacobian_six_joints()
    jacobian_six_joints_unit_test()
