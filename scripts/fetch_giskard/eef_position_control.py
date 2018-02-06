#!/usr/bin/env python
import rospy
import tf
import sys
from fetch_giskard.fetch_robot import Fetch
from giskardpy.eef_position_controller import EEFPositionControl
from sensor_msgs.msg import JointState

def cmdDictToJointState(command):
    js = JointState()
    js.header.stamp = rospy.Time.now()
    for joint, vel in command.iteritems():
        js.name.append(str(joint))
        js.position.append(0)
        js.velocity.append(vel)
        js.effort.append(0)

    return js


class FetchEEFPositionControl(object):
    def __init__(self, urdf_path, goal_frame, command_topic='/simulator/commands'):
        super(FetchEEFPositionControl, self).__init__()

        self.fetch = Fetch(urdf_path)
        print('Controller creation starting now...')
        self.controller = EEFPositionControl(self.fetch)
        print('Finished instancing controller.')
        self.goal_frame = goal_frame
        self.tfListener = tf.TransformListener()
        self.publisher = rospy.Publisher(command_topic, JointState, queue_size=1)
        self.jsSubscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb, queue_size=1)


    def joint_state_cb(self, joint_state):
        self.fetch.set_joint_state(joint_state)

        try:
            now = rospy.Time.now()
            #self.tfListener.waitForTransform("base_link", self.goal_frame, now, rospy.Duration(0.5))
            (trans, rot) = self.tfListener.lookupTransform('odom', self.goal_frame, rospy.Time(0))
            self.controller.set_goal(trans)
            command = self.controller.get_next_command()
            cmdMsg = cmdDictToJointState(command)
            self.publisher.publish(cmdMsg)
        except (tf.LookupException, tf.ConnectivityException) as e:
            print('Lookup of goal frame "' + self.goal_frame + '" in "base_link" failed. Skipping update. Exception: ', e)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage: <URDF PATH> <GOAL FRAME> [<COMMAND TOPIC>]')
        exit(0)

    rospy.init_node('FetchEEFPController')

    if len(sys.argv) == 3:
        FEPC = FetchEEFPositionControl(sys.argv[1], sys.argv[2])
    else:
        FEPC = FetchEEFPositionControl(sys.argv[1], sys.argv[2], sys.argv[3])

    rospy.spin()
