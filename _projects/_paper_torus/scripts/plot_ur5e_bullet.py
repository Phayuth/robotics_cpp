import numpy as np
import pybullet as p
import pybullet_data
import os


class UR5eBullet:

    def __init__(self, mode="gui", loadobs=False) -> None:
        # connect
        if mode == "gui":
            p.connect(p.GUI)
            # p.connect(p.SHARED_MEMORY_GUI)
        if mode == "no_gui":
            p.connect(p.DIRECT)
            # p.connect(p.SHARED_MEMORY)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # load model and properties
        self.load_model(loadobs)
        self.ghost_model = []
        self.numJoints = self.get_num_joints()
        self.jointNames = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.jointIDs = [1, 2, 3, 4, 5, 6]
        self.gripperlinkid = 9

        # inverse kinematic
        self.lower_limits = [-np.pi] * 6
        self.upper_limits = [np.pi] * 6
        self.joint_ranges = [2 * np.pi] * 6
        self.rest_poses = [0, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, 0]
        self.joint_damp = [0.01] * 6

    def load_model(self, loadobs):
        rsrcpath = os.environ["RSRC_DIR"] + "/rnd_torus/"
        plane_urdf = "plane.urdf"
        ur5e_urdf = rsrcpath + "ur5e/ur5e_extract_calibrated.urdf"
        table_urdf = rsrcpath + "table/table.urdf"
        pole_urdf = rsrcpath + "ur5e/simple_box.urdf"

        self.planeID = p.loadURDF(plane_urdf, [0, 0, 0])
        self.ur5eID = p.loadURDF(
            ur5e_urdf,
            [0, 0, 0],
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION,
        )
        if loadobs:
            # self.tableID = p.loadURDF(table_urdf, [0, 0, 0])
            self.pole1 = p.loadURDF(pole_urdf, [0.3, 0.3, 0], useFixedBase=True)
            self.pole2 = p.loadURDF(pole_urdf, [-0.3, 0.3, 0], useFixedBase=True)
            self.pole3 = p.loadURDF(pole_urdf, [-0.3, -0.3, 0], useFixedBase=True)
            self.pole4 = p.loadURDF(pole_urdf, [0.3, -0.3, 0], useFixedBase=True)

    def load_models_ghost(self, color=None):
        rsrcpath = os.environ["RSRC_DIR"] + "/rnd_torus/"
        ur5e_urdf = rsrcpath + "ur5e/ur5e_extract_calibrated.urdf"
        ur5e_ghost_id = p.loadURDF(
            ur5e_urdf,
            [0, 0, 0],
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION,
        )

        # Disable all collisions
        for link_index in range(-1, p.getNumJoints(ur5e_ghost_id)):
            p.setCollisionFilterGroupMask(ur5e_ghost_id, link_index, 0, 0)

        # Set the color of the ghost model
        if color:
            self.change_color(
                ur5e_ghost_id,
                red=color[0],
                green=color[1],
                blue=color[2],
                alpha=color[3],
            )

        self.ghost_model.append(ur5e_ghost_id)

    def load_slider(self):
        self.redSlider = p.addUserDebugParameter("red", 0, 1, 1)
        self.greenSlider = p.addUserDebugParameter("green", 0, 1, 0)
        self.blueSlider = p.addUserDebugParameter("blue", 0, 1, 0)
        self.alphaSlider = p.addUserDebugParameter("alpha", 0, 1, 0.5)

    def read_slider(self):
        red = p.readUserDebugParameter(self.redSlider)
        green = p.readUserDebugParameter(self.greenSlider)
        blue = p.readUserDebugParameter(self.blueSlider)
        alpha = p.readUserDebugParameter(self.alphaSlider)
        return red, green, blue, alpha

    def change_color(self, urdf_id, red=1, green=0, blue=0, alpha=1):
        num_joints = p.getNumJoints(urdf_id)
        for link_index in range(-1, num_joints):  # -1 includes the base link
            visuals = p.getVisualShapeData(urdf_id)
            for visual in visuals:
                if visual[1] == link_index:
                    p.changeVisualShape(
                        objectUniqueId=urdf_id,
                        linkIndex=link_index,
                        rgbaColor=[red, green, blue, alpha],
                    )

    def get_visualizer_camera(self):
        (
            width,
            height,
            viewMatrix,
            projectionMatrix,
            cameraUp,
            cameraForward,
            horizontal,
            vertical,
            yaw,
            pitch,
            dist,
            target,
        ) = p.getDebugVisualizerCamera()

        print(f"> width: {width}")
        print(f"> height: {height}")
        print(f"> viewMatrix: {viewMatrix}")
        print(f"> projectionMatrix: {projectionMatrix}")
        print(f"> cameraUp: {cameraUp}")
        print(f"> cameraForward: {cameraForward}")
        print(f"> horizontal: {horizontal}")
        print(f"> vertical: {vertical}")
        print(f"> yaw: {yaw}")
        print(f"> pitch: {pitch}")
        print(f"> dist: {dist}")
        print(f"> target: {target}")

    def set_visualizer_camera(
        self,
        cameraDistance=3,
        cameraYaw=30,
        cameraPitch=52,
        cameraTargetPosition=[0, 0, 0],
    ):
        p.resetDebugVisualizerCamera(
            cameraDistance=cameraDistance,
            cameraYaw=cameraYaw,
            cameraPitch=cameraPitch,
            cameraTargetPosition=cameraTargetPosition,
        )

    def get_num_joints(self):
        return p.getNumJoints(self.ur5eID)

    def get_joint_link_info(self):
        for i in range(self.numJoints):
            (
                jointIndex,
                jointName,
                jointType,
                qIndex,
                uIndex,
                flags,
                jointDamping,
                jointFriction,
                jointLowerLimit,
                jointUpperLimit,
                jointMaxForce,
                jointMaxVelocity,
                linkName,
                jointAxis,
                parentFramePos,
                parentFrameOrn,
                parentIndex,
            ) = p.getJointInfo(self.ur5eID, i)

            print(f"> ---------------------------------------------<")
            print(f"> jointIndex: {jointIndex}")
            print(f"> jointName: {jointName}")
            print(f"> jointType: {jointType}")
            print(f"> qIndex: {qIndex}")
            print(f"> uIndex: {uIndex}")
            print(f"> flags: {flags}")
            print(f"> jointDamping: {jointDamping}")
            print(f"> jointFriction: {jointFriction}")
            print(f"> jointLowerLimit: {jointLowerLimit}")
            print(f"> jointUpperLimit: {jointUpperLimit}")
            print(f"> jointMaxForce: {jointMaxForce}")
            print(f"> jointMaxVelocity: {jointMaxVelocity}")
            print(f"> linkName: {linkName}")
            print(f"> jointAxis: {jointAxis}")
            print(f"> parentFramePos: {parentFramePos}")
            print(f"> parentFrameOrn: {parentFrameOrn}")
            print(f"> parentIndex: {parentIndex}")

    def control_single_motor(self, jointIndex, jointPosition, jointVelocity=0):
        p.setJointMotorControl2(
            bodyIndex=self.ur5eID,
            jointIndex=jointIndex,
            controlMode=p.POSITION_CONTROL,
            targetPosition=jointPosition,
            targetVelocity=jointVelocity,
            positionGain=0.03,
        )

    def control_array_motors(
        self, jointPositions, jointVelocities=[0, 0, 0, 0, 0, 0]
    ):
        p.setJointMotorControlArray(
            bodyIndex=self.ur5eID,
            jointIndices=self.jointIDs,
            controlMode=p.POSITION_CONTROL,
            targetPositions=jointPositions,
            targetVelocities=jointVelocities,
            positionGains=[0.03, 0.03, 0.03, 0.03, 0.03, 0.03],
        )

    def get_single_joint_state(self):
        (
            jointPosition,
            jointVelocity,
            jointReactionForce,
            appliedJointMotorTorque,
        ) = p.getJointState(self.ur5eID, jointIndex=1)
        return (
            jointPosition,
            jointVelocity,
            jointReactionForce,
            appliedJointMotorTorque,
        )

    def get_array_joint_state(self):
        j1, j2, j3, j4, j5, j6 = p.getJointStates(
            self.ur5eID, jointIndices=self.jointIDs
        )
        return j1, j2, j3, j4, j5, j6

    def get_array_joint_positions(self):
        j1, j2, j3, j4, j5, j6 = self.get_array_joint_state()
        return (j1[0], j2[0], j3[0], j4[0], j5[0], j6[0])

    def forward_kin(self):
        (
            link_trn,
            link_rot,
            com_trn,
            com_rot,
            frame_pos,
            frame_rot,
            link_vt,
            link_vr,
        ) = p.getLinkState(
            self.ur5eID,
            self.gripperlinkid,
            computeLinkVelocity=True,
            computeForwardKinematics=True,
        )
        return link_trn, link_rot

    def inverse_kin(self, positions, quaternions):
        joint_angles = p.calculateInverseKinematics(
            self.ur5eID,
            self.gripperlinkid,
            positions,
            quaternions,
            lowerLimits=self.lower_limits,
            upperLimits=self.upper_limits,
            jointRanges=self.joint_ranges,
            jointDamping=self.joint_damp,
            restPoses=self.rest_poses,
        )
        return joint_angles

    def contact_point(self):
        contact_points = p.getContactPoints(bodyA=self.ur5eID, bodyB=self.tableID)
        print(f"> contact_points: {contact_points}")
        # for point in contact_points:
        #     print(f"Contact point details: {point}")

    def closest_point(self):
        closest_points = p.getClosestPoints(
            bodyA=self.ur5eID, bodyB=self.tableID, distance=0.5
        )
        print(f"> closest_points: {closest_points}")

    def reset_array_joint_state(self, targetValues):
        for i in range(6):
            p.resetJointState(
                self.ur5eID,
                jointIndex=self.jointIDs[i],
                targetValue=targetValues[i],
            )

    def reset_array_joint_state_ghost(self, targetValues, urdf_id):
        for i in range(6):
            p.resetJointState(
                urdf_id,
                jointIndex=self.jointIDs[i],
                targetValue=targetValues[i],
            )

    def collisioncheck(self):
        p.performCollisionDetection()


def simple_visualize():
    robot = UR5eBullet("gui")

    # camera for exp3
    robot.set_visualizer_camera(
        1.4,
        50.0,
        -35.0,
        (-0.037039175629615784, 0.08329583704471588, 0.2426416277885437),
    )

    try:
        while True:
            p.stepSimulation()
    except KeyboardInterrupt:
        p.disconnect()


def simple_visualize_change_color():
    robot = UR5eBullet("gui")
    robot.load_models_ghost()
    robot.load_slider()

    # camera for exp3
    robot.set_visualizer_camera(
        1.4,
        50.0,
        -35.0,
        (-0.037039175629615784, 0.08329583704471588, 0.2426416277885437),
    )

    try:
        while True:
            rgba = robot.read_slider()
            robot.change_color(robot.ghost_model[0], *rgba)
            p.stepSimulation()

    except KeyboardInterrupt:
        p.disconnect()


def collision_check():
    robot = UR5eBullet("gui")

    # camera for exp2
    robot.set_visualizer_camera(
        1.0,
        50.0,
        -35.0,
        (-0.037039175629615784, 0.08329583704471588, 0.2426416277885437),
    )

    try:
        while True:
            qKey = ord("c")
            keys = p.getKeyboardEvents()
            if qKey in keys and keys[qKey] & p.KEY_WAS_TRIGGERED:
                break
            robot.collisioncheck()
            robot.contact_point()
            robot.closest_point()

            p.stepSimulation()

    except KeyboardInterrupt:
        p.disconnect()


def joint_trajectory_visualize():
    robot = UR5eBullet("gui", loadobs=False)

    # camera for exp3
    robot.set_visualizer_camera(
        1.4,
        50.0,
        -35.0,
        (-0.037039175629615784, 0.08329583704471588, 0.2426416277885437),
    )

    # exp 2
    qs = [0.0, -np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, 0.0]
    qg1_short = [-1.12, -1.86, 1.87, 0.0, np.pi / 2, 0.0]
    qg2_long = [-1.12 + 2 * np.pi, -1.86, 1.87, 0.0, np.pi / 2, 0.0]
    n_steps = 100
    path_short = np.linspace(qs, qg1_short, n_steps)
    path_long = np.linspace(qs, qg2_long, n_steps)

    # exp 3
    qg_a = [0.0, -0.98, 1.57, -0.47, 1.57, 0.0]
    qg_b = [1.47, -0.11, -1.22, 3.53, -1.57, 6.23]
    qg_c = [-3.22, -1.09, 1.59, 5.86, 1.59, 0.0]
    qg_d = [-1.52, -1.02, 0.81, 5.35, 6.23, 2.36]
    path_seq = np.loadtxt("../build/zzz_path.csv", delimiter=",")

    path = path_short
    try:
        j = 0
        while True:
            nkey = ord("n")
            keys = p.getKeyboardEvents()
            if nkey in keys and keys[nkey] & p.KEY_WAS_TRIGGERED:
                q = path[j % n_steps]
                print(f"step {j}: {q}")
                robot.reset_array_joint_state(q)
                p.stepSimulation()
                j += 1

    except KeyboardInterrupt:
        p.disconnect()


def joint_trajectory_visualize_ghost():
    robot = UR5eBullet("gui", loadobs=False)
    robot.load_models_ghost(color=[0, 1, 0, 0.1])  # green ghost model
    robot.load_models_ghost(color=[1, 0, 0, 0.1])  # red ghost model

    # camera for exp3
    robot.set_visualizer_camera(
        1.0,
        61.20,
        -42.20,
        (-0.15094073116779327, 0.1758367419242859, 0.10792634636163712),
    )

    # exp 2
    qs = [0.0, -np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, 0.0]
    qg1_short = [-1.12, -1.86, 1.87, 0.0, np.pi / 2, 0.0]
    qg2_long = [-1.12 + 2 * np.pi, -1.86, 1.87, 0.0, np.pi / 2, 0.0]
    n_steps = 100
    path_short = np.linspace(qs, qg1_short, n_steps)
    path_long = np.linspace(qs, qg2_long, n_steps)

    path = path_short
    robot.reset_array_joint_state_ghost(path[0], robot.ghost_model[0])
    robot.reset_array_joint_state_ghost(path[-1], robot.ghost_model[-1])
    try:
        j = 0
        while True:
            nkey = ord("n")
            keys = p.getKeyboardEvents()
            if nkey in keys and keys[nkey] & p.KEY_WAS_TRIGGERED:
                q = path[j % n_steps]
                print(f"step {j}: {q}")
                robot.reset_array_joint_state(q)
                p.stepSimulation()
                j += 1

    except KeyboardInterrupt:
        p.disconnect()


if __name__ == "__main__":
    while True:
        func = [
            simple_visualize,
            simple_visualize_change_color,
            collision_check,
            joint_trajectory_visualize,
            joint_trajectory_visualize_ghost,
        ]

        for i, f in enumerate(func, start=1):
            print(f"{i}: {f.__name__}")

        arg = input("Enter argument number (` to exit): ")

        if arg == "`":
            print("Exiting...")
            break
        elif arg.isdigit() and 1 <= int(arg) <= len(func):
            func[int(arg) - 1]()
