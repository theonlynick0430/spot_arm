#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
import copy

    # rostopic pub /spot/go_to_pose geometry_msgs/PoseStamped "header:
    #   seq: 0
    #   stamp:
    #     secs: 0
    #     nsecs: 0
    #   frame_id: 'body'
    # pose:
    #   position:
    #     x: 0.5
    #     y: 0.0
    #     z: 0.0
    #   orientation:
    #     x: 0.0
    #     y: 0.0
    #     z: 0.0
    #     w: 1"

class Dummy(object):

    def __init__(self) -> None:
        rospy.init_node('Dummy', anonymous=True)
        self.pub = rospy.Publisher('/arm_joint_states', numpy_msg(JointState), queue_size=100)
        rospy.Subscriber('/joint_states', numpy_msg(JointState), self.callback_js)
        rospy.spin()

    def callback_js(self, data):
        msg = copy.deepcopy(data)
        # print('here')
        # print(data.name[-7:-1])
        msg.name = data.name[-7:]
        msg.position = data.position[-7:]
        msg.velocity = data.velocity[-7:]
        msg.effort = data.effort[-7:]
        self.pub.publish(msg)


def fake_js():
    rospy.init_node('Dummy2', anonymous=True)
    pub = rospy.Publisher('/joint_states', numpy_msg(JointState), queue_size=1)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # That command will have spot rotate on the spot at 0.3 radians/second. 
        js = JointState()
        js.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'arm_joint6', 'arm_gripper']
        js.position = np.zeros(7)
        js.velocity = np.zeros(7)
        js.effort = np.zeros(7)
        pub.publish(js)
        rate.sleep()


if __name__ == '__main__':
    try:
        d = Dummy()
        # fake_js()
    except rospy.ROSInterruptException:
        pass
