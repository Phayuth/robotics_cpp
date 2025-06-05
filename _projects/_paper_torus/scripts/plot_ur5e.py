import os
import numpy as np
import pinocchio
import time
from pinocchio.visualize import MeshcatVisualizer

# unset PYTHONPATH


class UR5ePinocchio:

    def __init__(self):
        self.rsrcpath = os.environ["RSRC_DIR"] + "/rnd_torus/"
        self.urdf_filename = self.rsrcpath + "/ur5e/ur5e_extract_calibrated.urdf"

        self.model = pinocchio.buildModelFromUrdf(self.urdf_filename)
        self.urmodel, self.urcollision_model, self.urvisual_model = (
            pinocchio.buildModelsFromUrdf(
                self.urdf_filename, self.rsrcpath, pinocchio.JointModelFreeFlyer()
            )
        )

        self.modelname = self.urmodel.name
        self.tip = self.urmodel.getFrameId("gripper")
        self.data = self.urmodel.createData()
        self.jointid = [7, 8, 9, 10, 11, 12]
        # q1=7, q2=8, q3=9, q4=10, q5=11, q6=12

    def print_debug(self):
        print("model name: " + self.modelname)
        print("tip name: " + self.urmodel.frames[self.tip].name)
        print("tip id: " + str(self.tip))

        print("-----------------------------")
        print(self.urmodel)
        print("-----------------------------")
        print(self.urcollision_model)
        print("-----------------------------")
        print(self.urvisual_model)
        print("-----------------------------")

    def random_configuration(self):
        q = pinocchio.randomConfiguration(self.urmodel)
        return q

    def forward_kinematics(self, q):
        pinocchio.forwardKinematics(self.urmodel, self.data, q)
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
        time.sleep(2)  # no sleep cause the viewer to not show up WTF!

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


if __name__ == "__main__":
    ur5e = UR5ePinocchio()

    q = ur5e.random_configuration()
    print("Random configuration: ", q)

    q = np.array([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
    tran, rot = ur5e.forward_kinematics(q)
    print("Forward kinematics: ", tran, rot)

    ur5e.visualize()
    ur5e.demo_visualize_motion()
