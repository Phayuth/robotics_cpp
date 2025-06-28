import os
import numpy as np
import pinocchio
import time
from pinocchio.visualize import MeshcatVisualizer

np.set_printoptions(precision=4, suppress=True, linewidth=120)
# unset PYTHONPATH


class UR5ePinocchio:

    def __init__(self):
        self.rsrcpath = os.environ["RSRC_DIR"] + "/rnd_torus"
        self.urdf_filename = self.rsrcpath + "/ur5e/ur5e_extract_calibrated.urdf"

        # self.model = pinocchio.buildModelFromUrdf(self.urdf_filename)
        (
            self.urmodel,
            self.urcollision_model,
            self.urvisual_model,
        ) = pinocchio.buildModelsFromUrdf(
            self.urdf_filename,
            self.rsrcpath,
            pinocchio.JointModelFreeFlyer(),
        )

        self.modelname = self.urmodel.name
        self.tip = self.urmodel.getFrameId("gripper")
        self.data = self.urmodel.createData()
        self.jntid = [7, 8, 9, 10, 11, 12]  # q1=7, q2=8, q3=9, q4=10, q5=11, q6=12

    def print_debug(self):
        print("model name: " + self.modelname)
        print("tip name: " + self.urmodel.frames[self.tip].name)
        print("tip id: " + str(self.tip))
        for i, frame in enumerate(self.urmodel.frames):
            print(f"Frame {i}: {frame.name}")

        print("-----------URMODEL-------------")
        print(self.urmodel)
        print("-----------COLLISION-----------")
        print(self.urcollision_model)
        print("-----------VISUAL--------------")
        print(self.urvisual_model)
        print("-----------END-----------------")

    def random_configuration(self):
        q = pinocchio.randomConfiguration(self.urmodel)
        return q

    def make_joint_configuration(self, q):
        q0 = pinocchio.neutral(self.urmodel)
        q0[self.jntid] = q
        return q0

    def make_joint_configuration_neutral(self):
        q0 = pinocchio.neutral(self.urmodel)
        return q0

    def forward_kinematics(self, q):
        pinocchio.forwardKinematics(self.urmodel, self.data, q)
        pinocchio.updateFramePlacements(self.urmodel, self.data)
        return (
            self.data.oMf[self.tip].translation,
            self.data.oMf[self.tip].rotation,
        )

    def inverse_kinematics(self, q_init, target_position, target_rotation):
        pinocchio.forwardKinematics(self.urmodel, self.data, q_init)
        pinocchio.updateFramePlacements(self.urmodel, self.data)
        target_frame = pinocchio.SE3(target_rotation, target_position)
        q_sol = pinocchio.ik(self.urmodel, self.data, q_init, target_frame)
        return q_sol

    def visualize(self):
        self.viz = MeshcatVisualizer(
            self.urmodel, self.urcollision_model, self.urvisual_model
        )
        self.viz.initViewer(open=True)
        self.viz.loadViewerModel(rootNodeName="ur5e")
        self.viz.displayVisuals(True)
        self.viz.displayCollisions(True)
        self.viz.displayFrames(True)

        q0 = pinocchio.neutral(self.urmodel)
        self.viz.display(q0)

        time.sleep(5)  # no sleep cause the viewer to not show up WTF!

    def simulate_motion(self, path):
        pass

    def demo_visualize_motion(self, neutral=None):
        if neutral is not None:
            q0 = neutral
        else:
            q0 = pinocchio.neutral(self.urmodel)
        self.viz.display(q0)

        n_steps = 1000
        amplitude = 3.14  # radians
        center = q0[3]
        for i in range(n_steps):
            q = q0.copy()
            q[7] = center + amplitude * np.sin(2 * np.pi * i / n_steps)
            self.viz.display(q)
            time.sleep(0.01)


def eaik_analytical_solution_dh_ur5e():
    from eaik.IK_DH import DhRobot

    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996])
    alpha = np.array([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])
    a = np.array([0, -0.425, -0.3922, 0, 0, 0])
    bot = DhRobot(alpha, a, d)
    angles = np.array([1, 1, 1, 1, 1, 1])
    t = bot.fwdKin(angles)
    ik_solutions = bot.IK(t)

    print(ik_solutions.num_solutions())
    print(ik_solutions.Q)

    ur5e = UR5ePinocchio()
    q00 = ur5e.make_joint_configuration_neutral()
    q = np.tile(q00, (8, 1))
    print("Neutral configuration: ", q00)
    print(q)

    q[:, 7:] = ik_solutions.Q
    print(q)


def prnt_debug():
    ur5e = UR5ePinocchio()
    ur5e.print_debug()


def rnd_config():
    ur5e = UR5ePinocchio()
    q = ur5e.random_configuration()
    print("Random configuration: ", q)


def forward_kin():
    ur5e = UR5ePinocchio()
    q0 = ur5e.make_joint_configuration([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    tran, rot = ur5e.forward_kinematics(q0)
    print("Translation: ", tran)
    print("Rotation: ", rot)


def vizualize():
    ur5e = UR5ePinocchio()
    ur5e.visualize()
    ur5e.demo_visualize_motion()


if __name__ == "__main__":
    while True:
        func = [
            eaik_analytical_solution_dh_ur5e,
            prnt_debug,
            rnd_config,
            forward_kin,
            vizualize,
        ]

        for i, f in enumerate(func, start=1):
            print(f"{i}: {f.__name__}")

        arg = input("Enter argument number (` to exit): ")

        if arg == "`":
            print("Exiting...")
            break
        elif arg.isdigit() and 1 <= int(arg) <= len(func):
            func[int(arg) - 1]()
