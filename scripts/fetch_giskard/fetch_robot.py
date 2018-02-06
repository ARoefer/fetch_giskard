import operator
from gebsyas.dl_reasoning import SymbolicData, DLBodyPosture
from giskardpy.robot import Robot, Joint, Gripper, Camera
from gebsyas.utils import JointState
from collections import namedtuple
import giskardpy.symengine_wrappers as spw


class Fetch(Robot):
	def __init__(self, urdf='fetch.urdf'):
		super(Fetch, self).__init__()
		gripper_joint = Joint(spw.Symbol('gripper_joint'), 0.1, 0.0, 0.1, False)
		self._joints['gripper_joint'] = gripper_joint

		self.load_from_urdf_path(urdf, 'base_link', ['r_gripper_finger_link', 'l_gripper_finger_link', 'head_camera_link'])

		r_finger_joint = self._joints['r_gripper_finger_joint']
		l_finger_joint = self._joints['l_gripper_finger_joint']
		self._joints.pop('r_gripper_finger_joint', None)
		self._joints.pop('l_gripper_finger_joint', None)
		self.frames['r_gripper_finger_link'] = self.frames['r_gripper_finger_link'].subs({r_finger_joint.symbol: 0.5 * gripper_joint.symbol})
		self.frames['l_gripper_finger_link'] = self.frames['l_gripper_finger_link'].subs({l_finger_joint.symbol: 0.5 * gripper_joint.symbol})

		for joint_name, joint in self.joint_constraints.items():
			self.set_joint_weight(joint_name, 0.01)

		self.state = SymbolicData({jn: JointState(joint.symbol, 0, 0) for jn, joint in self._joints.items()}, self.do_js_resolve, ['joint_state'])


		self.set_joint_weight('torso_lift_joint', 0.05)

		self.gripper = Gripper(name='gripper',
							   pose=self.frames['gripper_link'],
							   opening=spw.Symbol('gripper_joint'),
							   height=0.03,
							   max_opening=0.1,
							   link_name='gripper_link')

		self.eef    = self.gripper.pose
		self.camera = Camera(name='head_camera',
							 pose=self.frames['head_camera_link'],
							 hfov=54.0,
							 near=0.35,
							 far=3.0)

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
					   self.gripper.max_opening,
					   self.gripper.link_name)

	def do_camera_fk(self, joint_state):
		js = {name: state.position for name, state in joint_state.items()}
		return Camera(name=self.camera.name,
					  pose=self.camera.pose.subs(js),
					  hfov=self.camera.hfov,
					  near=self.camera.near,
					  far=self.camera.far)

	def do_js_resolve(self, joint_state):
		return {jname: JointState(joint_state[jname].position, joint_state[jname].velocity, joint_state[jname].effort) for jname in self.state.data}

	def __str__(self):
		return 'Fetch\'s state:\n' + reduce(operator.add, map(lambda t: t[0] + ': {:.3f}\n'.format(t[1]), self.joint_state.iteritems()), '')
