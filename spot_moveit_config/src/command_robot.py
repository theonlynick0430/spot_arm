#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
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
#  * Neither the name of SRI International nor the names of its
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
# Author: Acorn Pooley, Mike Lautman

# BEGIN_SUB_TUTORIAL imports
##
# To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
# This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
# and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
import pdb
# END_SUB_TUTORIAL

PYTHON_VERSION = sys.version_info.major
input if PYTHON_VERSION == 3 else raw_input

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonInteface(object):
    """SpotArmMoveGroupPythonInteface"""

    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        robot_description = "/spot_arm/robot_description"
        # BEGIN_SUB_TUTORIAL setup
        ##
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        # moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface', anonymous=True)

        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander(robot_description=robot_description)
        # pdb.set_trace()
        # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to a planning group (group of joints).  In this tutorial the group is the primary
        # arm joints in the Spot arm, so we set the group's name to "spot_arm".
        # If you are using a different robot, change this value to the name of your robot
        # arm planning group.
        # This interface can be used to plan and execute motions:
        group_name = "spot_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name, robot_description=robot_description)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # END_SUB_TUTORIAL

        # BEGIN_SUB_TUTORIAL basic_info
        ##
        # Getting Basic Information
        # ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        # print(robot.get_current_state())
        print("")
        # END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def get_joint_states(self):
        return self.move_group.get_current_joint_values()

    def get_ee_pose(self):
        return self.move_group.get_current_pose().pose

    def go_to_joint_states(self, joint_goals):
        # BEGIN_SUB_TUTORIAL plan_to_joint_states
        ##
        # Planning to a Joint Goal
        # ^^^^^^^^^^^^^^^^^^^^^^^^
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goals, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # END_SUB_TUTORIAL

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goals, current_joints, 0.01)

    def plan_path_to_ee_pose(self, ee_pose_goal):
        # BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        # Planning to a Pose Goal
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # We can plan a motion for this group to a desired pose for the
        # end-effector:
        self.move_group.set_pose_target(ee_pose_goal)

        # Now, we call the planner to compute the plan.
        if PYTHON_VERSION == 3:
            plan_success, plan, planning_time, error_code = self.move_group.plan()
            if not plan_success:
                print("WARNING: Planner failed to plan a valid path for the requested pose target")
            return plan
        else:
            return self.move_group.plan()

        # END_SUB_TUTORIAL

    def execute_plan(self, plan, ee_pose_goal):
        # BEGIN_SUB_TUTORIAL execute_plan
        ##
        # Executing a Plan
        # ^^^^^^^^^^^^^^^^
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(ee_pose_goal, current_pose, 0.01)

        # END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # BEGIN_SUB_TUTORIAL display_trajectory
        ##
        # Displaying a Trajectory
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        ##
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

        # END_SUB_TUTORIAL

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        # Ensuring Collision Updates Are Received
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        # For the purpose of this tutorial, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        # END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # BEGIN_SUB_TUTORIAL add_box
        ##
        # Adding Objects to the Planning Scene
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "finger"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.07  # slightly above the end effector
        self.box_name = "box"
        self.scene.add_box(self.box_name, box_pose, size=(0.1, 0.1, 0.1))

        # END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # BEGIN_SUB_TUTORIAL attach_object
        ##
        # Attaching Objects to the Robot
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        # robot be able to touch them without the planning scene reporting the contact as a
        # collision. By adding link names to the ``touch_links`` array, we are telling the
        # planning scene to ignore collisions between those links and the box. For the Panda
        # robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        # you should change this value to the name of your end effector group name.
        grasping_group = 'hand'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name,
                              touch_links=touch_links)
        # END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # BEGIN_SUB_TUTORIAL detach_object
        ##
        # Detaching Objects from the Robot
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can also detach and remove the object from the planning scene:
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)
        # END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # BEGIN_SUB_TUTORIAL remove_object
        ##
        # Removing Objects from the Planning Scene
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can remove the box from the world.
        self.scene.remove_world_object(self.box_name)

        # **Note:** The object must be detached before we can remove it from the world
        # END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
    try:
        input_func = input if sys.version_info.major == 3 else raw_input
        
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        print("============ Press `Enter` to begin by setting up the moveit_commander ...")
        input_func()
        tutorial = MoveGroupPythonInteface()

        print(
            "============ Press `Enter` to execute a movement using a joint state goal ...")
        input_func()
        # The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        # thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goals = tutorial.get_joint_states()
        joint_goals[0] = 2.617994
        joint_goals[1] = 0
        joint_goals[2] = 0
        joint_goals[3] = 0
        joint_goals[4] = 0
        joint_goals[5] = 0
        tutorial.go_to_joint_states(joint_goals)

        print("============ Press `Enter` to compute a plan using an end-effector pose goal ...")
        input_func()
        ee_pose_goal = tutorial.get_ee_pose()
        ee_pose_goal.orientation.w = 1.0
        ee_pose_goal.position.x = 0.4
        ee_pose_goal.position.y = 0.1
        ee_pose_goal.position.z = 0.4
        plan = tutorial.plan_path_to_ee_pose(ee_pose_goal)
        print(plan)

        print("============ Press `Enter` to display saved plan ...")
        input_func()
        tutorial.display_trajectory(plan)

        print("============ Press `Enter` to execute saved plan ...")
        input_func()
        tutorial.execute_plan(plan, ee_pose_goal)

        print("============ Press `Enter` to add a box to the planning scene ...")
        input_func()
        tutorial.add_box()

        print("============ Press `Enter` to attach a Box to the Spot arm robot ...")
        input_func()
        tutorial.attach_box()

        print("============ Press `Enter` to detach the box from the Spot arm robot ...")
        input_func()
        tutorial.detach_box()

        print("============ Press `Enter` to remove the box from the planning scene ...")
        input_func()
        tutorial.remove_box()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()

# BEGIN_TUTORIAL
# .. _moveit_commander:
# http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
# .. _MoveGroupCommander:
# http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
# .. _RobotCommander:
# http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
# .. _PlanningSceneInterface:
# http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
# .. _DisplayTrajectory:
# http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
# .. _RobotTrajectory:
# http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
# .. _rospy:
# http://docs.ros.org/melodic/api/rospy/html/
# CALL_SUB_TUTORIAL imports
# CALL_SUB_TUTORIAL setup
# CALL_SUB_TUTORIAL basic_info
# CALL_SUB_TUTORIAL plan_to_joint_state
# CALL_SUB_TUTORIAL plan_to_pose
# CALL_SUB_TUTORIAL plan_cartesian_path
# CALL_SUB_TUTORIAL display_trajectory
# CALL_SUB_TUTORIAL execute_plan
# CALL_SUB_TUTORIAL add_box
# CALL_SUB_TUTORIAL wait_for_scene_update
# CALL_SUB_TUTORIAL attach_object
# CALL_SUB_TUTORIAL detach_object
# CALL_SUB_TUTORIAL remove_object
# END_TUTORIAL
