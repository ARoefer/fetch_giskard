import giskardpy.symengine_wrappers as spw
from giskardpy.symengine_robot import Robot, Joint, Gripper, Camera
from giskardpy.input_system    import JointStatesInput
from giskardpy.qp_problem_builder import HardConstraint, JointConstraint, SoftConstraint
from giskardpy.symengine_wrappers import pos_of
from gebsyas.dl_reasoning import SymbolicData, DLBodyPosture
from gebsyas.utils import JointState, symbol_formatter, deg2rad
from symengine import Symbol


class Fetch(Robot):
	def __init__(self, urdf='fetch.urdf'):
		super(Fetch, self).__init__(urdf, 0.6)


		new_joints = {}
		for j, jd in self._joints.items():
			new_joints[j] = Joint(jd.symbol, min(jd.velocity_limit, 0.3), jd.lower, jd.upper, jd.type, jd.frame)
		self._joints = new_joints

		self._joints['base_linear_joint'] = Joint(Symbol('base_linear_joint'),
												  0.4,
												  -1000,
												  1000,
												  'prismatic',
												  spw.translation3(Symbol('base_linear_joint'), 0,0))
		self._joints['base_angular_joint'] = Joint(Symbol('base_angular_joint'),
												  0.2,
												  None,
												  None,
												  'continuous',
												  spw.rotation3_rpy(0, 0, Symbol('base_angular_joint')))


		self.set_joint_symbol_map(JointStatesInput.prefix_constructor(symbol_formatter,
									self.get_joint_names() +
									['localization_x', 'localization_y', 'localization_z',
									 'localization_z_ang', 'r_gripper_finger_joint', 'l_gripper_finger_joint', 'gripper_joint', 'linear_velocity_x', 'angular_velocity_z'],
									 '',
									 'position'))


		sj_lin = self._joints['base_linear_joint'].symbol
		sj_ang = self._joints['base_angular_joint'].symbol
		s_ang  = self.joint_states_input.joint_map['localization_z_ang']
		s_clv_x = self.joint_states_input.joint_map['linear_velocity_x']
		s_cav_z = self.joint_states_input.joint_map['angular_velocity_z']
		s_deltaT = Symbol('deltaT')

		l_accel_limit = 0.5
		a_accel_limit = 0.2

		self.world_transform = spw.frame3_rpy(0, 0, s_ang + sj_ang,
												[self.joint_states_input.joint_map['localization_x'] + (spw.cos(s_ang + sj_ang)) * sj_lin,
												 self.joint_states_input.joint_map['localization_y'] + (spw.sin(s_ang + sj_ang)) * sj_lin,
												 self.joint_states_input.joint_map['localization_z']])# * \
												#  .frame

		gripper_joint = self._joints['gripper_joint']

		self.state = SymbolicData({jn: JointState(self._joints[jn].symbol, 0, 0) for jn in self.get_joint_names()}, self.do_js_resolve, ['joint_state'])

		_torso_constraint = self.joint_constraints['torso_lift_joint']
		self.joint_constraints['torso_lift_joint'] = JointConstraint(_torso_constraint.lower, _torso_constraint.upper, 0.05)
		self.joint_constraints['base_linear_joint']  = JointConstraint(0, 1, 0.1)
		self.joint_constraints['base_angular_joint'] = JointConstraint(-1.6, 1.6, 0.1)


		self.gripper = Gripper(name='gripper',
							   pose=self.get_fk_expression('map', 'gripper_link'),
							   opening=gripper_joint.symbol,
							   height=0.03,
							   max_opening=0.1,
							   link_name='gripper_link',
							   reach=1,
							   pivot_position=pos_of(self.get_fk_expression('map', 'shoulder_pan_link')),
							   depth=0.06)

		self.eef    = self.gripper.pose
		self.camera = Camera(name='head_camera',
							 pose=self.get_fk_expression('map', 'head_camera_link'),
							 hfov=54.0 * deg2rad,
							 near=0.35,
							 far=6.0)

		self.soft_dynamics_constraints = {
			'dynamics_linear_base_accel' :
				SoftConstraint(-0.5 * s_deltaT, 0.5 * s_deltaT, 50, sj_lin),
			#'dynamics_angular_base_accel':
			#	SoftConstraint(-0.1 * s_deltaT, 0.1 * s_deltaT, 50, sj_ang),
			#'dynamics_torso_accel':
				#SoftConstraint(-0.2 * s_deltaT, 0.2 * s_deltaT, 50, self.joint_states_input.joint_map['torso_lift_joint'])
				}

		# 'torso_lift_link', 'wrist_roll_link'
		# Link names mapped to safety margin, AABB blow up and number of avoidance constraints
		self.collision_avoidance_links = {'base_collision_link': (0.15, 0.4, 4), 'elbow_flex_link': (0.08, 0.15, 2), 'wrist_flex_link': (0.05, 0.15, 2), 'gripper_link': (0.01, 0.1, 5), 'l_gripper_finger_link': (0.005, 0.05, 2), 'r_gripper_finger_link': (0.005, 0.05, 2)}

		self.self_collision_avoidance_pairs = {('gripper_link', 'torso_lift_link', 0.1), ('gripper_link', 'bellows_link', 0.1), ('gripper_link', 'base_collision_link', 0.1), ('gripper_link', 'box_link', 0.1), ('gripper_link', 'head_pan_link', 0.1), ('elbow_flex_link', 'torso_lift_link', 0.1)}


	def get_fk_expression(self, root_link, tip_link):
		if (root_link, tip_link) in self.fks:
			return self.fks[root_link, tip_link]

		if root_link == 'map':
			fk = self.world_transform * super(Fetch, self).get_fk_expression(self._urdf_robot.get_root(), tip_link)
			self.fks[root_link, tip_link] = fk
			return fk
		return super(Fetch, self).get_fk_expression(root_link, tip_link)

	def do_gripper_fk(self, joint_state):
		js = {self.joint_states_input.joint_map[name]: state.position for name, state in joint_state.items() if name in self.joint_states_input.joint_map}
		#js['deltaT'] = 0.0
		return Gripper(self.gripper.name,
					   self.gripper.pose.subs(js),
					   joint_state['gripper_joint'].position,
					   self.gripper.height,
					   self.gripper.max_opening,
					   self.gripper.link_name,
					   self.gripper.reach,
					   self.gripper.pivot_position.subs(js),
					   self.gripper.depth)

	def do_camera_fk(self, joint_state):
		js = {self.joint_states_input.joint_map[name]: state.position for name, state in joint_state.items() if name in self.joint_states_input.joint_map}
		#js['deltaT'] = 0.0
		return Camera(name=self.camera.name,
					  pose=self.camera.pose.subs(js),
					  hfov=self.camera.hfov,
					  near=self.camera.near,
					  far=self.camera.far)

	def do_js_resolve(self, joint_state):
		return {jname: JointState(joint_state[jname].position, joint_state[jname].velocity, joint_state[jname].effort) for jname in self.state.data}