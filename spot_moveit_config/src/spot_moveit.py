import rospy
import threading
import numpy as np
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from spot_msgs.srv import (
    ArmJointMovement,
    ArmJointMovementResponse,
    ArmJointMovementRequest,
)
from command_robot import MoveGroupPythonInteface


class SpotMoveit(object):
    """SpotMoveit"""

    def __init__(self,  planner, use_sim=False):

        self.planner = planner
        self.use_sim = use_sim

        # self.planner.wait_for_valid_joint_states()
        # self.planner.go_to_joint_states(self.spot_info_sub.get_joint_states())

        print("waiting for arm_joint_move srv ...")
        rospy.wait_for_service("/spot/arm_joint_move")
        print("arm_joint_move is ready")

    def move_to_goal(self, ee_pose_goal):
        # generate plan using KDL
        plan = self.planner.plan_path_to_ee_pose(ee_pose_goal)

        if self.use_sim:
            # execute plan if sim is running
            self.planner.execute_plan(plan, ee_pose_goal)
        else:
            # construct service to be sent to ROS driver
            try:
                arm_joint_move = rospy.ServiceProxy(
                    "/spot/arm_joint_move", ArmJointMovement)
                # arm_joint_move(plan.joint_trajectory.points[2].positions)
                for joint_trajectory_point in plan.joint_trajectory.points:
                    arm_joint_move(joint_trajectory_point.positions)
            except rospy.ServiceException as e:
                print("arm_joint_move service call failed: %s" % e)


class SpotInfoSubscriber(object):

    def __init__(self):

        rospy.init_node("spot_info_sub")


        



def main():

    planner = MoveGroupPythonInteface()
    # spot_info_sub_thread = threading.Thread(target=planner.run)
    # spot_info_sub_thread.start()

    spot_moveit = SpotMoveit(planner)
    ee_pose_goal = spot_moveit.planner.get_ee_pose()
    ee_pose_goal.orientation.w = 1.0
    ee_pose_goal.position.x = 0.4
    ee_pose_goal.position.y = 0.1
    ee_pose_goal.position.z = 0.4
    print("Press enter to move to specified goal")
    input()
    spot_moveit.move_to_goal(ee_pose_goal)

    # spot_info_sub_thread.join()

if __name__ == '__main__':
    main()
