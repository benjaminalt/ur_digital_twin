#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, ListControllers, ListControllersRequest
import math

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

def base_configuration():
    state = Float64MultiArray()
    state.data = [0.0, -2.0897, -1.6473, -2.5590, 3.6376, 5.3446]
    return state

def generate_joint_states(interval):
    state = base_configuration()
    vel = 1.0 # rad/s
    step = vel * interval # rad
    limit_min = - math.pi
    limit_max = math.pi
    increment = True
    while True:
        if increment:
            if state.data[0] + step > limit_max:
                increment = False
                continue
            state.data[0] += step
        else:
            if state.data[0] - step < limit_min:
                increment = True
                continue
            state.data[0] -= step
        yield state


def main():
    node_name = "joint_state_publisher"
    rospy.loginfo("{} starting".format(node_name))
    rospy.init_node(node_name, anonymous=False)

    try:
        switch_to_position_controller()

        pub = rospy.Publisher("/joint_group_position_controller/command", Float64MultiArray, queue_size=10)
        interval = 0.02
        rate = rospy.Rate(1/interval)
        for state in generate_joint_states(interval):
            if rospy.is_shutdown():
                break
            pub.publish(state)
            rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        return

if __name__ == "__main__":
    main()