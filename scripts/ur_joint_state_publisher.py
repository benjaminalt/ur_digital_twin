#!/usr/bin/env python

"""
Publish the current joint state of the URSim simulator to /ur_digital_twin/joint_state
"""

import rospy
from sensor_msgs.msg import JointState
from ur_joint_state_reader import URJointStateReader

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
               "wrist_2_joint", "wrist_3_joint"]


def main():
    node_name = "ur_joint_state_publisher"
    rospy.loginfo("{} starting".format(node_name))
    rospy.init_node(node_name, anonymous=False)

    config_file = rospy.get_param("ursim_rtde_config")
    host = rospy.get_param("ursim_host")
    port = rospy.get_param("ursim_port")
    sampling_interval = rospy.get_param("sampling_interval")
    rate = rospy.Rate(1/sampling_interval)

    pub = rospy.Publisher("/ur_digital_twin/joint_state", JointState, queue_size=10)
    with URJointStateReader(config_file, host, port, sampling_interval) as joint_state_reader:
        while not rospy.is_shutdown():
            current_state = joint_state_reader.receive(False)
            state = JointState()
            state.header.stamp = rospy.Time.now()
            state.name = JOINT_NAMES
            state.position = current_state.actual_q
            state.velocity = current_state.actual_qd
            pub.publish(state)
            rate.sleep()


if __name__ == "__main__":
    main()
