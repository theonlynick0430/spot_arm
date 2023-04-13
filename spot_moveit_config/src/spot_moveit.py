import rospy
from spot_msgs.srv import (
    ArmJointMovement,
)
from spot_msgs.msg import JointTarget
from command_robot import MoveGroupPythonInteface


class SpotMoveit(object):
    """SpotMoveit"""

    def __init__(self, use_sim=False):

        self.planner = MoveGroupPythonInteface()
        self.use_sim = use_sim

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
                joint_targets = []
                for joint_trajectory_point in plan.joint_trajectory.points:
                    joint_target = JointTarget()
                    joint_target.positions = joint_trajectory_point.positions
                    joint_targets.append(joint_target)
                arm_joint_move(joint_targets)
            except rospy.ServiceException as e:
                print("arm_joint_move service call failed: %s" % e)   


def main():
    spot_moveit = SpotMoveit()
    ee_pose_goal = spot_moveit.planner.get_ee_pose()
    ee_pose_goal.orientation.w = 1.0
    ee_pose_goal.position.x = 0.4
    ee_pose_goal.position.y = 0.1
    ee_pose_goal.position.z = 0.4
    print("Press enter to move to specified goal")
    input()
    spot_moveit.move_to_goal(ee_pose_goal)


if __name__ == '__main__':
    main()
