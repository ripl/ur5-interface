import rospy
import numpy as np
import signal
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
import moveit_msgs.msg
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from model_fitting.srv import segment
from enum import Enum

import sys
from time import sleep
from gripper import Gripper


class Arm():
    def __init__(self):
        # create a RobotCommander
        self.robot = RobotCommander()

        # create a PlanningSceneInterface object
        self.scene = PlanningSceneInterface()

        # create move group to interface with UR5 arm
        # Need to launch the ur.launch on husky cpu
        # and husky_ur_planning_execution.launch on laptop
        self.group = MoveGroupCommander("manipulator")

        # set planner
        self.group.set_planner_id("BKPIECEkConfigDefault")
\
        self.object_pub = rospy.Publisher('/segmentation/cylinder_marker', MarkerArray, queue_size=1)

        # create subscriber
        self.sub = rospy.Subscriber('/segmentation/cylinder_marker', MarkerArray, self.callback)

        # create `segmenter` service proxy object
        self.segment = rospy.ServiceProxy('segment', segment)

        # set FSM state
        self.fsm_state = FSM.SLEEP

        self.objects = None

        self.object = None

        self.grasp_mode = 'basic'

        self.is_shutdown = False

        self.approach_x = 0.34

        self.ee_link_to_center_gripper = 0.16

        self.release_configuration = None

        # create service client
        # self._service = self.segment_service_client()

        # # Move to start pose and wait for objects to grasp
        # self.move_to_ready_pose()

        # create gripper object
        self.gripper = Gripper()


    def get_robot_state(self):
        return self.robot.get_current_state()

    def get_group_names(self):
        return self.robot.get_group_names()

    def get_planning_frame(self):
        return self.group.get_planning_frame()

    def get_joint_values(self):
        return self.group.get_current_joint_values()

    def get_pose(self):
        return self.group.get_current_pose().pose

    def clear_pose_targets(self):
        self.group.clear_pose_targets()

    def plan_to_pose(self, pose):
        '''Plan to a given pose for the end-effector
        '''
        # Clear old pose targets
        self.clear_pose_targets()
        # Set target pose
        self.group.set_pose_target(pose)

        numTries = 0;
        while numTries < 5:
            plan = self.group.plan()
            numTries+=1
            if len(plan.joint_trajectory.points) > 0:
                print('succeeded in %d tries' % numTries)
                return True
        # check that planning succeeded

         # If we get here, we couldn't find a plan in max number of tries
        print('Planning failed')
        return False

    def plan_to_joints(self, joint_angles):
        '''Plan to a given joint configuration'''
        # Clear old pose targets
        self.clear_pose_targets()
        # Set Joint configuration target
        self.group.set_joint_value_target(joint_angles)
        numTries = 0
        while numTries < 5:
            plan = self.group.plan()
            numTries+=1
            if len(plan.joint_trajectory.points) > 0:
                print('succeeded in %d tries' % numTries)
                return True
        print("Planning failed")
        return False

    def _change_planner(self, planner):
        planners = [
            "SBLkConfigDefault",
            "ESTkConfigDefault",
            "LBKPIECEkConfigDefault",
            "BKPIECEkConfigDefault",
            "KPIECEkConfigDefault",
            "RRTkConfigDefault",
            "RRTConnectkConfigDefault",
            "RRTstarkConfigDefault",
            "TRRTkConfigDefault",
            "PRMkConfigDefault",
            "PRMstarkConfigDefault"
        ]
        if planner in planners:
            self.group.set_planner_id(planner)
        return
