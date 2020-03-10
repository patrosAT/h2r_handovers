#!/usr/bin/env python

# -- IMPORT --
import time
import math
import numpy as np
import cv2
# Ros
import rospy
import tf
import actionlib
from std_msgs.msg import Empty
from geometry_msgs.msg import TwistStamped, Pose
# Ros ACRV
from rv_msgs.msg import ManipulatorState
from rv_msgs.msg import ServoToPoseAction, ServoToPoseGoal
from rv_msgs.msg import MoveToNamedPoseAction, MoveToNamedPoseGoal
from rv_msgs.msg import ActuateGripperAction, ActuateGripperGoal
# Ros RTHTR
from ggcnn_humanseg_ros.msg import GraspPrediction
import helper.tf_helpers as tfh


class RTHTR:

    def __init__(self):
      
        # Parameter
        self.state_move = 'startup'
        self.state_init = False
        self.ggcnn_init = False
        self.goal_set = False

        self.ggcnn_topic = rospy.get_param('/human_robot_handover_ros/ggcnn/topic')
        self.ggcnn_window = rospy.get_param('/human_robot_handover_ros/ggcnn/window')
        self.ggcnn_dev_pos = rospy.get_param('/human_robot_handover_ros/ggcnn/deviation_position')
        self.ggcnn_dev_orient = rospy.get_param('/human_robot_handover_ros/ggcnn/deviation_orientation')

        self.arm_state = rospy.get_param('/human_robot_handover_ros/robot/arm_state')
        self.arm_servo_pose = rospy.get_param('/human_robot_handover_ros/robot/arm_servo_pose')
        self.arm_named_pose = rospy.get_param('/human_robot_handover_ros/robot/arm_named_pose')
        self.arm_gripper = rospy.get_param('/human_robot_handover_ros/robot/arm_gripper')
        
        self.position_error = rospy.get_param('/human_robot_handover_ros/movement/position_error')
        self.move_x = rospy.get_param('/human_robot_handover_ros/movement/move_x')
        self.move_y = rospy.get_param('/human_robot_handover_ros/movement/move_y')
        self.move_z = rospy.get_param('/human_robot_handover_ros/movement/move_z')
        self.dist_ggcnn = rospy.get_param('/human_robot_handover_ros/movement/dist_ggcnn')
        self.dist_final = rospy.get_param('/human_robot_handover_ros/movement/dist_final')
        self.move_final = rospy.get_param('/human_robot_handover_ros/movement/move_final')
        self.speed_approach = rospy.get_param('/human_robot_handover_ros/movement/speed_approach')
        self.scaling_handover = rospy.get_param('/human_robot_handover_ros/movement/scaling_handover')

        self.gripper_open = rospy.get_param('/human_robot_handover_ros/gripper/gripper_open')

        self.visualization_type = rospy.get_param('/human_robot_handover_ros/visualization/activated')
        self.visualization_topic = rospy.get_param('/human_robot_handover_ros/visualization/topic')

        self.servo_goal = ServoToPoseGoal()
        self.error_code = 0
        self.ggcnn_xP_arr = []
        self.ggcnn_yP_arr = []
        self.ggcnn_zP_arr = []
        self.ggcnn_xO_arr = []
        self.ggcnn_yO_arr = []
        self.ggcnn_zO_arr = []
        self.ggcnn_wO_arr = []

        self.xP = None
        self.yP = None
        self.zP = None
        self.P_off = None
        self.xP_off = None
        self.yP_off = None
        self.zP_off = None

        self.grab_xP = None
        self.grab_yP = None
        self.grab_zP = None
        self.grab_xO = None
        self.grab_yO = None
        self.grab_zO = None
        self.grab_wO = None
        self.grab_width = None
        self.grab_quality = None

        # Robot interface
        self.servo_pose_client = actionlib.SimpleActionClient(self.arm_servo_pose, ServoToPoseAction)
        self.servo_pose_client.wait_for_server()
        print("STARTUP -> named_pose_client OK")
        
        self.named_pose_client = actionlib.SimpleActionClient(self.arm_named_pose, MoveToNamedPoseAction)
        self.named_pose_client.wait_for_server()
        print("STARTUP -> named_pose_client OK")

        self.gripper_client = actionlib.SimpleActionClient(self.arm_gripper, ActuateGripperAction)
        self.gripper_client.wait_for_server()
        print("STARTUP -> gripper_client OK")

        # Subsriber
        rospy.Subscriber(self.arm_state, ManipulatorState, self._callback_state, queue_size=1)
        rospy.Subscriber(self.ggcnn_topic, GraspPrediction, self._callback_ggcnn, queue_size=1)
        
        # Visualization
        if (self.visualization_type):
            self.pub_visualization = rospy.Publisher(self.visualization_topic, Empty, queue_size=1)
            rospy.Subscriber(self.visualization_topic, Empty, self._callback_visualization, queue_size=1)


    #### FIXED MOVEMENTS ####
    def _move_start(self):
        self.named_pose_client.send_goal(MoveToNamedPoseGoal(pose_name='patros_start', speed=self.speed_approach))
        self.named_pose_client.wait_for_result()

    def _move_home(self):
        self.named_pose_client.send_goal(MoveToNamedPoseGoal(pose_name='patros_home', speed=self.speed_approach))
        self.named_pose_client.wait_for_result()

    def _move_bin(self):
        self.named_pose_client.send_goal(MoveToNamedPoseGoal(pose_name='patros_bin', speed=self.speed_approach))
        self.named_pose_client.wait_for_result()

    def _gripper_open(self, width):
        self.gripper_client.send_goal(ActuateGripperGoal(mode=ActuateGripperGoal.MODE_STATIC, width=width))
        self.gripper_client.wait_for_result()

    def _gripper_close(self, width):
        self.gripper_client.send_goal(ActuateGripperGoal(mode=ActuateGripperGoal.MODE_GRASP, width=width))
        self.gripper_client.wait_for_result()

    #### HELPER FUNCTIONS ####

    # HELPER to reset all parameter
    def _reset_parameter(self):
        self.state_init = False
        self.ggcnn_init = False
        self.goal_set = False

        self.servo_goal = ServoToPoseGoal()        
        self.error_code = 0
        self.ggcnn_xP_arr = []
        self.ggcnn_yP_arr = []
        self.ggcnn_zP_arr = []
        self.ggcnn_xO_arr = []
        self.ggcnn_yO_arr = []
        self.ggcnn_zO_arr = []
        self.ggcnn_wO_arr = []

        self.xP = None
        self.yP = None
        self.zP = None
        self.P_off = None
        self.xP_off = None
        self.yP_off = None
        self.zP_off = None

        self.grab_xP = None
        self.grab_yP = None
        self.grab_zP = None
        self.grab_xO = None
        self.grab_yO = None
        self.grab_zO = None
        self.grab_wO = None
        self.grab_width = None
        self.grab_quality = None


    # HELPER to define and send goal
    def _set_goal(self):

        # Define new goal
        self.servo_goal = ServoToPoseGoal()
        self.servo_goal.stamped_pose.header.frame_id = 'panda_link0'
        self.servo_goal.stamped_pose.pose.position.x = self.grab_xP
        self.servo_goal.stamped_pose.pose.position.y = self.grab_yP
        self.servo_goal.stamped_pose.pose.position.z = self.grab_zP
        self.servo_goal.stamped_pose.pose.orientation.x = self.grab_xO
        self.servo_goal.stamped_pose.pose.orientation.y = self.grab_yO
        self.servo_goal.stamped_pose.pose.orientation.z = self.grab_zO
        self.servo_goal.stamped_pose.pose.orientation.w = self.grab_wO
        self.servo_goal.scaling = self.scaling_handover

        # Send goal
        self.servo_pose_client.send_goal(self.servo_goal)
        self.goal_set = True
        print("MOVE - Goal:")
        print(self.servo_goal.stamped_pose.pose)

        # Publish result
        if (self.visualization_type):
            self.pub_visualization.publish(Empty())


    #### OBJECT-BASED MOVEMENT ####
    def move(self):

        while not rospy.is_shutdown():

            ## HANDLE ERRORS ##
            if (self.error_code != 0):
                rospy.logerr('ERROR CODE: %d' % (self.error_code))

                # Stop any movement
                self.servo_pose_client.cancel_all_goals()

                # Move to start
                self._move_start()
                rospy.sleep(0.5)

                # Eliminate previous measuements (unuseful)
                self._reset_parameter()
                continue
            

            ## STARTUP POSITION ##
            if(self.state_move == 'startup'):

                # Move to start
                self._move_start()
                self.state_move = 'move_ggcnn'

                # Eliminate previous measuements (unuseful)
                self._reset_parameter()


            ## CHECK VIABLE STATE ##
            if not (self.state_init) or not (self.ggcnn_init):
                print('State init:', self.state_init, 'GGCNN init:', self.ggcnn_init)
                rospy.sleep(0.5)
                continue

            
            ## MOVEMENT ##
            if(self.state_move == 'move_ggcnn'):

                # STAGE 1: P_off > dist_ggcnn
                if (abs(self.P_off) > self.dist_ggcnn):
                    if not (self.goal_set):
                        self._set_goal()

                # STAGE 2: dist_ggcnn > P_off > dist_final
                elif (abs(self.P_off) > self.dist_final):
                    pass

                # STAGE 3: dist_final > P_off
                else:
                    print('Stage 3')
                    self.servo_pose_client.wait_for_result()
                    self.goal_set = False
                    print('Goal reached')

                    # STAGE 4: Grab object and drop it
                    self._gripper_close(0.01)
                    self._move_home()
                    self._move_bin()
                    self._gripper_open(self.gripper_open)

                    # STAGE 5: Restart: move to start & reset parameter
                    self._move_start()
                    self._reset_parameter()


    #### CALLBACK GGCNN ####
    def _callback_ggcnn(self, msg):

        # Check for ggcnn-activate region
        if (self.P_off == None) or (abs(self.P_off) > self.dist_ggcnn):

            # Sliding window over last X images -> detect outliers
            self.ggcnn_xP_arr.append(msg.pose.position.x)
            self.ggcnn_yP_arr.append(msg.pose.position.y)
            self.ggcnn_zP_arr.append(msg.pose.position.z)
            self.ggcnn_xO_arr.append(msg.pose.orientation.x)
            self.ggcnn_yO_arr.append(msg.pose.orientation.y)
            self.ggcnn_zO_arr.append(msg.pose.orientation.z)
            self.ggcnn_wO_arr.append(msg.pose.orientation.w)

            # Drop outliers & calculate grasping point
            if (len(self.ggcnn_xP_arr) >= self.ggcnn_window):

                # Calculate mean
                mean_xP = np.nanmean(self.ggcnn_xP_arr)
                mean_yP = np.nanmean(self.ggcnn_yP_arr)
                mean_zP = np.nanmean(self.ggcnn_zP_arr)
                mean_xO = np.nanmean(self.ggcnn_xO_arr)
                mean_yO = np.nanmean(self.ggcnn_yO_arr)
                mean_zO = np.nanmean(self.ggcnn_zO_arr)
                mean_wO = np.nanmean(self.ggcnn_wO_arr)

                # Drop values outside region of interest
                delete_upp = np.any([[(self.ggcnn_xP_arr > (mean_xP + self.ggcnn_dev_pos))], 
                                    [(self.ggcnn_yP_arr > (mean_yP + self.ggcnn_dev_pos))], 
                                    [(self.ggcnn_zP_arr > (mean_zP + self.ggcnn_dev_pos))], 
                                    [(self.ggcnn_xO_arr > (mean_xO + self.ggcnn_dev_orient))], 
                                    [(self.ggcnn_yO_arr > (mean_yO + self.ggcnn_dev_orient))], 
                                    [(self.ggcnn_zO_arr > (mean_zO + self.ggcnn_dev_orient))], 
                                    [(self.ggcnn_wO_arr > (mean_wO + self.ggcnn_dev_orient))]], axis=0)
                delete_low =  np.any([[(self.ggcnn_xP_arr < (mean_xP - self.ggcnn_dev_pos))], 
                                      [(self.ggcnn_yP_arr < (mean_yP - self.ggcnn_dev_pos))], 
                                      [(self.ggcnn_zP_arr < (mean_zP - self.ggcnn_dev_pos))], 
                                      [(self.ggcnn_xO_arr < (mean_xO - self.ggcnn_dev_orient))], 
                                      [(self.ggcnn_yO_arr < (mean_yO - self.ggcnn_dev_orient))], 
                                      [(self.ggcnn_zO_arr < (mean_zO - self.ggcnn_dev_orient))], 
                                      [(self.ggcnn_wO_arr < (mean_wO - self.ggcnn_dev_orient))]], axis=0)
                delete = np.any([delete_upp, delete_low], axis=0)

                ggcnn_xP_arr_new = []     # Values are stored in a new array to allow a reaction to drastcial changes
                ggcnn_yP_arr_new = []
                ggcnn_zP_arr_new = []
                ggcnn_xO_arr_new = []
                ggcnn_yO_arr_new = []
                ggcnn_zO_arr_new = []
                ggcnn_wO_arr_new = []
                for i in range(len(delete[0])):
                    if (delete[0][i] == False):
                        ggcnn_xP_arr_new.append(self.ggcnn_xP_arr[i])
                        ggcnn_yP_arr_new.append(self.ggcnn_yP_arr[i])
                        ggcnn_zP_arr_new.append(self.ggcnn_zP_arr[i])
                        ggcnn_xO_arr_new.append(self.ggcnn_xO_arr[i])
                        ggcnn_yO_arr_new.append(self.ggcnn_yO_arr[i])
                        ggcnn_zO_arr_new.append(self.ggcnn_zO_arr[i])
                        ggcnn_wO_arr_new.append(self.ggcnn_wO_arr[i])

                # Check for empty array
                if (len(ggcnn_xP_arr_new) > (self.ggcnn_window - 2)):

                    # Calculate grasping point
                    self.grab_xP = np.nanmean(ggcnn_xP_arr_new)
                    self.grab_yP = np.nanmean(ggcnn_yP_arr_new)
                    self.grab_zP = np.nanmean(ggcnn_zP_arr_new)
                    self.grab_xO = np.nanmean(ggcnn_xO_arr_new)
                    self.grab_yO = np.nanmean(ggcnn_yO_arr_new)
                    self.grab_zO = np.nanmean(ggcnn_zO_arr_new)
                    self.grab_wO = np.nanmean(ggcnn_wO_arr_new)
                    self.grab_width = msg.width
                    self.grab_quality = msg.quality

                    self.ggcnn_init = True

            # Keep window size fixed
            if (len(self.ggcnn_xP_arr) >= self.ggcnn_window):
                self.ggcnn_xP_arr.pop(0)
                self.ggcnn_yP_arr.pop(0)
                self.ggcnn_zP_arr.pop(0)
                self.ggcnn_xO_arr.pop(0)
                self.ggcnn_yO_arr.pop(0)
                self.ggcnn_zO_arr.pop(0)
                self.ggcnn_wO_arr.pop(0)

    
    #### CALLBACK ARM STATE ####
    def _callback_state(self, msg):

        # Save current position & error
        self.xP = msg.ee_pose.pose.position.x
        self.yP = msg.ee_pose.pose.position.y
        self.zP = msg.ee_pose.pose.position.z
        self.error_code = msg.errors

        if (self.ggcnn_init) and (self.xP != None) and (self.yP != None) and (self.zP != None):

            self.xP_off = self.grab_xP - self.xP
            self.yP_off = self.grab_yP - self.yP
            self.zP_off = self.grab_zP - self.zP
            self.P_off = math.sqrt(self.xP_off**2 + self.yP_off**2 + self.zP_off**2)

            self.state_init = True


    #### CALLBACK VISUALIZATION ####
    def _callback_visualization(self, msg):
            
        ggcnn_G = Pose()
        ggcnn_G.position.x = self.grab_xP
        ggcnn_G.position.y = self.grab_yP
        ggcnn_G.position.z = self.grab_zP
        ggcnn_G.orientation.x = self.grab_xO
        ggcnn_G.orientation.y = self.grab_yO
        ggcnn_G.orientation.z = self.grab_zO
        ggcnn_G.orientation.w = self.grab_wO
        tfh.publish_pose_as_transform(ggcnn_G, 'panda_link0', 'ggcnn', 0.3)

        goal_G = Pose()
        goal_G.position.x = self.servo_goal.stamped_pose.pose.position.x
        goal_G.position.y = self.servo_goal.stamped_pose.pose.position.y
        goal_G.position.z = self.servo_goal.stamped_pose.pose.position.z
        goal_G.orientation.x = self.servo_goal.stamped_pose.pose.orientation.x
        goal_G.orientation.y = self.servo_goal.stamped_pose.pose.orientation.y
        goal_G.orientation.z = self.servo_goal.stamped_pose.pose.orientation.z
        goal_G.orientation.w = self.servo_goal.stamped_pose.pose.orientation.w
        tfh.publish_pose_as_transform(goal_G, 'panda_link0', 'GOAL', 0.3)


#### MAIN ####
if __name__ == '__main__':

    rospy.init_node('rthtr_driver_closed_loop')
    rthtr = RTHTR()
    rthtr.move()
    rospy.spin()