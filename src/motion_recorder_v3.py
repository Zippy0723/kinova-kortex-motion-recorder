#!/usr/bin/env python3
###
# Written and Maintained by Thomas Haley, Slippery Rock University, tjh1019@sru.edu
# Using example code from Kinova inc, Copyright 2019
#
#
#
###

import sys
import os
import rospy
import time
import json
import threading

from kortex_driver.srv import *
from kortex_driver.msg import *

timer = None

class MotionRecorder:
    def __init__(self):
        try:
            rospy.init_node('motion_recorder_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "kinova_arm")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
    
    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

    def subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def do_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def get_cartesian_cords(self):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        res = []

        
        res.append(feedback.base.commanded_tool_pose_x)
        res.append(feedback.base.commanded_tool_pose_y)
        res.append(feedback.base.commanded_tool_pose_z)
        res.append(feedback.base.commanded_tool_pose_theta_x)
        res.append(feedback.base.commanded_tool_pose_theta_y)
        res.append(feedback.base.commanded_tool_pose_theta_z)

        return res

    def do_gripper_command(self, value):
        # Initialize the request
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def playback_cartesian_pos_list(self,cartesian_pos_list):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()

        for pos in cartesian_pos_list:
            trajectory.waypoints.append(self.FillCartesianWaypoint(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],0))

        global timer
        timer = threading.Timer(15, self.timeout_abort)
        timer.start()

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # Call the service
        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            #This will lock-up if it cannot find a valid motion path, so a timeout is needed
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()
            
    def timeout_abort(self):
        print("Motion timed out, aborting! Your states may be too far apart, ensure you are frequently recording states.")
        os._exit(1)

    def main(self):
        success = self.is_init_success

        if success:
            # Clear faults and set up notifications
            success &= self.do_clear_faults()
            success &= self.subscribe_to_a_robot_notification()

            action_queue = []
            
            success &= self.home_the_robot()
            self.do_gripper_command(0)

            action_queue.append(("TYPE_MOVE",self.get_cartesian_cords()))
            print("Starting Position Recorded")
            
            while True:
                user_token = input("Enter 'N' to record the current position as the next state. Enter 'P' to stop recording and attempt playback. Enter O or C to add a gripper open/close command")
                if user_token == 'N':
                    action_queue.append(("TYPE_MOVE",self.get_cartesian_cords()))
                    print("Position Recorded")
                    
                elif user_token == 'P':
                    success &= self.home_the_robot()
                    self.do_gripper_command(0)
                    print("Attempting to playback your motion...")
                    global timer

                    for action in action_queue:
                    	#Times out if playback function locks up, which it will do if it cannot find a simple path to the next state. Ensure state changes are gradual, your motion should consist of many states
                        if action[0] == "TYPE_MOVE":
                            success &= self.playback_cartesian_pos_list([action[1]])
                            try:
                                timer.cancel()
                            except:
                                pass

                        if action[0] == "TYPE_GRIP":
                            success &= self.do_gripper_command(action[1])
                         
                    break #Breaks the top while loop
                    	
                elif user_token == "C":
                    self.do_gripper_command(1)
                    action_queue.append(("TYPE_GRIP",1))
                    
                elif user_token == "O":
                    self.do_gripper_command(0)
                    action_queue.append(("TYPE_GRIP",0))
                    

        if not success:
            rospy.logerr("The program has encountered an error. Please check the readme")
        else:
            while True:
                print("Succesfully played back recorded Actions. Do you want to save the action sequence?")
                save_file = input("Y/N: ")
                if save_file == "Y":
                    file_path = ""
                    file_path = file_path + input("File name?: ")
                    with open(file_path,"w") as file:
                        json.dump(action_queue,file)

                    print("Action sequence saved succesfully")
                    break
                else:
                    break


if __name__ == "__main__":
    ex = MotionRecorder()
    ex.main()
