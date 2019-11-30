from iai_bullet_sim.multibody import SimpleBaseDriver

class FetchDriver(SimpleBaseDriver):
    def update_velocities(self, robot_data, velocities_dict):
        """Updates a given velocity command."""
        super(FetchDriver, self).update_velocities(robot_data, velocities_dict)
        if 'gripper_joint' in velocities_dict:
            gripper_vel = velocities_dict['gripper_joint']
            velocities_dict['r_gripper_finger_joint'] = gripper_vel
            velocities_dict['l_gripper_finger_joint'] = gripper_vel

    @classmethod
    def factory(cls, config_dict):
        return super(FetchDriver, cls).factory(config_dict)