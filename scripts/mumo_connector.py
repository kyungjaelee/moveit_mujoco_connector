#!/usr/bin/env python2.7
import rospy

from mujoco_moveit_connector.srv import *

from moveit_msgs.srv import *
import moveit_commander

# group_name = "both_arms" #"left_arm", "both_arms"
# group = moveit_commander.MoveGroupCommander(group_name)


def cartesian_planning_with_gripper_pose(req):
    group_name = req.group_name
    ntrial = req.ntrial
    gripper_pose = req.gripper_pose

    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planning_time(0.5)
    for _ in range(ntrial):
        move_group.set_start_state_to_current_state()
        plan_results = move_group.compute_cartesian_path([gripper_pose], 0.01, 0.0)

        planned_traj = plan_results[0]
        fraction = plan_results[1]

        result_flag = False
        if fraction > 0.8:
            result_flag = True
            rospy.loginfo("{} cartesian planning with gripper pose is done".format(group_name))
            break
    del move_group
    return MoveitPlanningGripperPoseResponse(result_flag, planned_traj.joint_trajectory)


def planning_with_arm_joints(req):
    group_name = req.group_name
    ntrial = req.ntrial
    joint_values = dict(zip(req.joint_names, req.joint_poses))

    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planning_time(0.5)
    for _ in range(ntrial):
        move_group.set_start_state_to_current_state()
        move_group.set_joint_value_target(joint_values)
        plan_results = move_group.plan()

        result_flag = plan_results[0]
        planned_traj = plan_results[1]

        if result_flag:
            rospy.loginfo("{} planning with joint values is done".format(group_name))
            break
    del move_group
    return MoveitPlanningJointValuesResponse(result_flag, planned_traj.joint_trajectory)


def planning_with_gripper_pose(req):
    group_name = req.group_name
    ntrial = req.ntrial
    gripper_pose = req.gripper_pose
    gripper_link = req.gripper_link

    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_planning_time(0.5)
    for _ in range(ntrial):
        move_group.set_pose_target(gripper_pose, end_effector_link=gripper_link)
        move_group.set_start_state_to_current_state()
        plan_results = move_group.plan()

        result_flag = plan_results[0]
        planned_traj = plan_results[1]

        if result_flag:
            rospy.loginfo("{} planning with gripper pose is done".format(group_name))
            break

    del move_group
    return MoveitPlanningGripperPoseResponse(result_flag, planned_traj.joint_trajectory)

# def send_joint_trajectory(req):
#     arm_pose = dict(zip(req.arm_joint_name, req.arm_pose))
#     group.set_joint_value_target(arm_pose)
#     plan_results = group.plan()
#     print(plan_results[0])
#     return plan_results[1].joint_trajectory

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('mujoco_moveit_connector_node', anonymous=True)
    rospy.Service('/cartesian_planning_with_gripper_pose', MoveitPlanningGripperPose, cartesian_planning_with_gripper_pose)
    rospy.Service('/planning_with_gripper_pose', MoveitPlanningGripperPose, planning_with_gripper_pose)
    rospy.Service('/planning_with_arm_joints', MoveitPlanningJointValues, planning_with_arm_joints)
    rospy.spin()




