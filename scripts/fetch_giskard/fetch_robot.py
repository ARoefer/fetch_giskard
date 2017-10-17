from giskardpy.robot import Robot
import operator

class Fetch(Robot):
    def __init__(self, urdf='fetch.urdf'):
        super(Fetch, self).__init__()
        self.load_from_urdf_path(urdf, 'base_link', ['gripper_link', 'head_camera_link'])
        for joint_name in self.joint_constraints:
            self.set_joint_weight(joint_name, 0.01)

        self.set_joint_weight('torso_lift_joint', 0.05)

        self.gripper = self.frames['gripper_link']
        self.eef     = self.gripper
        self.joint_state = {}

    def set_joint_state(self, joint_state):
        for i, joint_name in enumerate(joint_state.name):
            if joint_name in self.joints:
                self.joint_state[joint_name] = joint_state.position[i]


    def update_observables(self):
        return self.joint_state

    def __str__(self):
        return 'Fetch\'s state:\n' + reduce(operator.add, map(lambda t: t[0] + ': {:.3f}\n'.format(t[1]), self.joint_state.iteritems()), '')
