import operator
from giskardpy.robot import Robot, Joint
from collections import namedtuple
import giskardpy.symengine_wrappers as spw

Gripper = namedtuple('Gripper', ['name', 'pose', 'opening', 'height', 'max_opening'])

class Fetch(Robot):
	def __init__(self, urdf='fetch.urdf'):
		super(Fetch, self).__init__()
		self._joints['gripper_joint'] = Joint(spw.Symbol('gripper_joint'), 0.1, 0.0, 0.1, False)

		self.load_from_urdf_path(urdf, 'base_link', ['gripper_link', 'head_camera_link'])

		for joint_name in self.joint_constraints:
			self.set_joint_weight(joint_name, 0.01)

		self.set_joint_weight('torso_lift_joint', 0.05)

		self.gripper = Gripper(name='gripper',
							   pose=self.frames['gripper_link'],
							   opening=spw.Symbol('gripper_joint'),
							   height=0.03,
							   max_opening=0.1)

		self.eef     = self.gripper.pose
		self.camera  = self.frames['head_camera_link']

	def set_joint_state(self, joint_state):
		for joint_name, state in joint_state.items():
			if joint_name in self._state:
				self._state[joint_name] = state.position

	def do_gripper_fk(self, joint_state):
		js = {name: state.position for name, state in joint_state.items()}
		return Gripper(self.gripper.name,
					   self.gripper.pose.subs(js),
					   js['gripper_joint'],
					   self.gripper.height,
					   self.gripper.max_opening)

	def __str__(self):
		return 'Fetch\'s state:\n' + reduce(operator.add, map(lambda t: t[0] + ': {:.3f}\n'.format(t[1]), self.joint_state.iteritems()), '')
