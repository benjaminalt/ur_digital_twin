#!/usr/bin/env python

"""
Publish the current joint state of the URSim simulator to /joint_group_position_controller/command
"""

import rospy
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, ListControllers, ListControllersRequest
from ur_joint_state_reader import URJointStateReader


def switch_to_position_controller():
    rospy.wait_for_service("/controller_manager/list_controllers")
    rate = rospy.Rate(1)
    list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)  
    position_controller_available = False
    while not position_controller_available:
        res = list_controllers(ListControllersRequest())
        position_controller_available = any([controller.name == "joint_group_position_controller" for controller in res.controller])
        rate.sleep()

    rospy.logdebug("Switching controllers")
    rospy.wait_for_service("/controller_manager/switch_controller")
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    req = SwitchControllerRequest()
    req.start_controllers = ["joint_group_position_controller"]
    req.stop_controllers = ["arm_controller"]
    req.strictness = 2
    switch_controller(req)


def main():
    node_name = "ur_joint_command_publisher"
    rospy.loginfo("{} starting".format(node_name))
    rospy.init_node(node_name, anonymous=False)

    config_file = rospy.get_param("ursim_rtde_config")
    host = rospy.get_param("ursim_host")
    port = rospy.get_param("ursim_port")
    sampling_interval = rospy.get_param("sampling_interval")
    rate = rospy.Rate(1/sampling_interval)

    switch_to_position_controller()
    pub = rospy.Publisher("/joint_group_position_controller/command", Float64MultiArray, queue_size=10)
    with URJointStateReader(config_file, host, port, sampling_interval) as joint_state_reader:
        while not rospy.is_shutdown():
            current_state = joint_state_reader.receive(False)
            state = Float64MultiArray()
            state.data = current_state.actual_q
            pub.publish(state)
            rate.sleep()


if __name__ == "__main__":
    main()
