 #!/usr/bin/env python

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import String, UInt8, UInt8MultiArray, Float32, Float32MultiArray
from tf.transformations import quaternion_from_euler
import math
import numpy as np

class Forward_kinematics:
    def __init__(self, name):
        self.name = name

    def S(self,szog):
        szinusz_num = math.sin(szog)
        return szinusz_num

    def C(self,szog):
        coszinusz_num = math.cos(szog)
        return coszinusz_num

    def dh_transform(self,dh_parms): # [alpha,a,theta,d]
        t_i=np.array([[self.C(dh_parms[2]),-1*self.S(dh_parms[2]),0,dh_parms[1]],
                    [self.C(dh_parms[0])*self.S(dh_parms[2]),self.C(dh_parms[0])*self.C(dh_parms[2]),-1*self.S(dh_parms[0]),-1*dh_parms[3]*self.S(dh_parms[0])],
                    [self.S(dh_parms[0])*self.S(dh_parms[2]),self.S(dh_parms[0])*self.C(dh_parms[2]),self.C(dh_parms[0]),dh_parms[3]*self.C(dh_parms[0])],
                    [0,0,0,1]])
        return t_i

    def forward_kinematics(self,joint_angles, r_TCP):
        """
        Calculates the TCP coordinates from the joint angles
        :param joint_angles: list, joint angles [j0, j1, j2, j3, j4, j5]
        :return: list, the list of TCP coordinates
        """
        # link distance from previous link
        l_1 = 0.290
        l_2 = 0.200
        l_2_x = 0.075
        l_2_z = 0.030
        l_3 = 0.150
        l_3_x = 0.035
        l_4 = 0.095
        l_5 = 0.050
        l_6 = 0.050

        # DH parameters
        # (alpha,a,theta,d)
        t_0_1 = [0,0,(joint_angles[0]),l_1]
        t_1_2 = [0, l_2_x + l_2 * self.S(joint_angles[1]), 0, - l_2 * self.C(joint_angles[1]) + l_2_z]
        t_2_3 = [0, l_3_x + l_3 * self.C(joint_angles[3]), 0, l_3 * self.S(joint_angles[3])]
        t_3_4 = [math.pi / 2, 0, joint_angles[4] + math.pi / 2, l_4]
        t_4_5 = [math.pi / 2, l_5, joint_angles[5], 0]
        t_5_6 = [-math.pi / 2, 0, joint_angles[6], l_6]

        # DH transformation matrix's
        t01 = self.dh_transform(t_0_1)
        t12 = self.dh_transform(t_1_2)
        t23 = self.dh_transform(t_2_3)
        t34 = self.dh_transform(t_3_4)
        t45 = self.dh_transform(t_4_5)
        t56 = self.dh_transform(t_5_6)

        t_01 = t01
        t_02 = np.dot(t_01, t12)
        t_03 = np.dot(t_02, t23)
        t_04 = np.dot(t_03, t34)
        t_05 = np.dot(t_04, t45)
        t_06 = np.dot(t_05, t56)

        r_02 = np.dot(t_01, t12)
        r_03 = np.dot(t_02, t23)
        r_04 = np.dot(t_03, t34)
        r_05 = np.dot(t_04, t45)
        r_06 = np.dot(t_05, t56)

        r_TCP_num = np.array([[r_TCP[0]], [r_TCP[1]], [r_TCP[2]], [1]])
        t_0_TCP = np.round(np.dot(t_06, r_TCP_num), 4)

        R_06 = np.array([[t_06[0][0], t_06[0][1], t_06[0][2]],
                          [t_06[1][0], t_06[1][1], t_06[1][2]],
                          [t_06[2][0], t_06[2][1], t_06[2][2]]])

        beta = math.atan2(-1 * R_06[2][0], math.sqrt(pow(R_06[0][0], 2) + pow(R_06[1][0], 2))) #*180/math.pi
        alpha = math.atan2(R_06[1][0]/math.cos(beta), R_06[0][0]/math.cos(beta)) #*180/math.pi
        gamma = math.atan2(R_06[2][1]/math.cos(beta), R_06[2][2]/math.cos(beta)) #*180/math.pi
        return t_0_TCP[0][0], t_0_TCP[1][0], t_0_TCP[2][0], gamma, beta, alpha 


class MoveGroupPythonInteface(object):
    """MoveGroupPythonInteface"""
    def __init__(self,group_name):
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous = True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Store parameters in corresponding variables
        self.robot = robot
        self.scene = scene
        self.move_group=move_group

    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

    def go_to_pose(self, x, y, z, alpha, beta, gamma):
        ## Planning to a Pose Goal
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        # set proper quaternion for the vertical orientation: https://quaternions.online/

        q = quaternion_from_euler(alpha, beta, gamma)
        #print(alpha*180/math.pi,beta*180/math.pi,gamma*180/math.pi)

        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.01)


class MoveitCommander:
    def __init__(self, group_name, shift_x, shift_y, shift_z):
        self.moveit_commander = MoveGroupPythonInteface(group_name)
        self.fwd=Forward_kinematics("forward")
        self.shift_x = shift_x
        self.shift_y = shift_y
        self.shift_z = shift_z

        # Set max velocity
        self.moveit_commander.move_group.set_max_velocity_scaling_factor(0.2)
        # Set tolerances, without that IK cannot do a valid plan
        self.moveit_commander.move_group.set_goal_position_tolerance(0.0005)
        self.moveit_commander.move_group.set_goal_orientation_tolerance(0.001)

    def callback(self, msg):
        jointangles = msg.data
        pos_TCP = self.fwd.forward_kinematics(jointangles[0:6], [0,0,0,1])
        self.moveit_commander.go_to_pose(
            pos_TCP[0] + self.shift_x,
            pos_TCP[1] + self.shift_y,
            pos_TCP[2] + self.shift_z,
            pos_TCP[3], 
            pos_TCP[4], 
            pos_TCP[5]
        )

def main():
  try:

    cmd_left = MoveitCommander("leftarm", 0.6, 0.2, 1)
    time.sleep(2)

    jointSub_left = rospy.Subscriber('/diploma_tele/leftarm/joint_angles', Float32MultiArray, cmd_left.callback, queue_size = 1)
    rospy.spin()
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()


