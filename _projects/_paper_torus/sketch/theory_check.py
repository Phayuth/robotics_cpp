import numpy as np
import matplotlib.pyplot as plt
from spatial_geometry.utils import Utils


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


if __name__ == "__main__":
    # sampling_test()
    determine_quadrant()
