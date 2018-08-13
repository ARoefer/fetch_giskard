import operator
from collections import namedtuple
import giskardpy.symengine_wrappers as spw
from giskardpy.symengine_robot import Robot, Joint, Gripper, Camera
from giskardpy.input_system    import JointStatesInput
from giskardpy.qp_problem_builder import HardConstraint, JointConstraint
from giskardpy.symengine_wrappers import pos_of
from gebsyas.dl_reasoning import SymbolicData, DLBodyPosture
from gebsyas.utils import JointState, symbol_formatter
from symengine import Symbol

class Fetch(Robot):
	def __init__(self, urdf='fetch.urdf'):
		super(Fetch, self).__init__(urdf, 0.6)


		self._joints['base_linear_joint'] = Joint(Symbol('base_linear_joint'),
												  0.4,
												  -1000,
												  1000,
												  'prismatic',
												  spw.translation3(Symbol('base_linear_joint'), 0,0))
		self._joints['base_angular_joint'] = Joint(Symbol('base_angular_joint'),
												  0.4,
												  None,
												  None,
												  'continuous',
												  spw.rotation3_rpy(0, 0, Symbol('base_angular_joint')))


		self.set_joint_symbol_map(JointStatesInput.prefix_constructor(symbol_formatter, 
									self.get_joint_names() + 
									['localization_x', 'localization_y', 'localization_z', 
									 'localization_z_ang', 'r_gripper_finger_joint', 'l_gripper_finger_joint', 'gripper_joint'], 
									 '', 
									 'position'))

		sj_lin = self._joints['base_linear_joint'].symbol
		sj_ang = self._joints['base_angular_joint'].symbol
		s_ang  = self.joint_states_input.joint_map['localization_z_ang']

		self.world_transform = spw.frame3_rpy(0, 0, s_ang + sj_ang, 
												[self.joint_states_input.joint_map['localization_x'] + spw.cos(s_ang) * sj_lin, 
												 self.joint_states_input.joint_map['localization_y'] + spw.sin(s_ang) * sj_lin, 
												 self.joint_states_input.joint_map['localization_z']])# * \
												#  .frame

		gripper_joint = self._joints['gripper_joint']

		self.state = SymbolicData({jn: JointState(self._joints[jn].symbol, 0, 0) for jn in self.get_joint_names()}, self.do_js_resolve, ['joint_state'])

		_torso_constraint = self.joint_constraints['torso_lift_joint']
		self.joint_constraints['torso_lift_joint'] = JointConstraint(_torso_constraint.lower, _torso_constraint.upper, 0.05)
		self.joint_constraints['base_linear_joint']  = JointConstraint(-0.4, 0.4, 1)
		self.joint_constraints['base_angular_joint'] = JointConstraint(-0.2, 0.2, 0.5)

		self.gripper = Gripper(name='gripper',
							   pose=self.get_fk_expression('map', 'gripper_link'),
							   opening=gripper_joint.symbol,
							   height=0.03,
							   max_opening=0.1,
							   link_name='gripper_link',
							   reach=1.2,
							   pivot_position=pos_of(self.get_fk_expression('map', 'shoulder_pan_link')))

		self.eef    = self.gripper.pose
		self.camera = Camera(name='head_camera',
							 pose=self.get_fk_expression('map', 'head_camera_link'),
							 hfov=54.0,
							 near=0.35,
							 far=3.0)


	def get_fk_expression(self, root_link, tip_link):
		if (root_link, tip_link) in self.fks:
			return self.fks[root_link, tip_link]

		if root_link == 'map':
			fk = self.world_transform * super(Fetch, self).get_fk_expression(self._urdf_robot.get_root(), tip_link)
			self.fks[root_link, tip_link] = fk
			return fk
		return super(Fetch, self).get_fk_expression(root_link, tip_link)

	def do_gripper_fk(self, joint_state):
		js = {self.joint_states_input.joint_map[name]: state.position for name, state in joint_state.items()}
		return Gripper(self.gripper.name,
					   self.gripper.pose.subs(js),
					   joint_state['gripper_joint'].position,
					   self.gripper.height,
					   self.gripper.max_opening,
					   self.gripper.link_name,
					   self.gripper.reach,
					   self.gripper.pivot_position.subs(js))

	def do_camera_fk(self, joint_state):
		js = {self.joint_states_input.joint_map[name]: state.position for name, state in joint_state.items()}
		return Camera(name=self.camera.name,
					  pose=self.camera.pose.subs(js),
					  hfov=self.camera.hfov,
					  near=self.camera.near,
					  far=self.camera.far)

	def do_js_resolve(self, joint_state):
		return {jname: JointState(joint_state[jname].position, joint_state[jname].velocity, joint_state[jname].effort) for jname in self.state.data}