#include <iostream>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>

class List_Controller{

private:
    ros::NodeHandle nh;
    ros::ServiceServer list_controll_service;

public:

    List_Controller(){
        this->list_controll_service = nh.advertiseService("/hsrb/controller_manager/list_controllers", &List_Controller::list_controll_server, this);
    }

    ~List_Controller(void){};

    bool list_controll_server(controller_manager_msgs::ListControllers::Request &input, controller_manager_msgs::ListControllers::Response &output){

        //**********  joint_state_controller **********
        controller_manager_msgs::HardwareInterfaceResources joint_state_claimed_resources;
        joint_state_claimed_resources.hardware_interface = "hardware_interface::JointStateInterface";

        controller_manager_msgs::ControllerState joint_state_controller;
        joint_state_controller.name = "joint_state_controller";
        joint_state_controller.state = "running";
        joint_state_controller.type = "joint_state_controller/JointStateController";
        joint_state_controller.claimed_resources.push_back(joint_state_claimed_resources);


        //**********  arm_trajectory_controller **********
        controller_manager_msgs::HardwareInterfaceResources arm_claimed_resources;
        arm_claimed_resources.hardware_interface = "hardware_interface::PositionJointInterface";
        arm_claimed_resources.resources.push_back("arm_flex_joint");
        arm_claimed_resources.resources.push_back("arm_lift_joint");
        arm_claimed_resources.resources.push_back("arm_roll_joint");
        arm_claimed_resources.resources.push_back("wrist_flex_joint");
        arm_claimed_resources.resources.push_back("wrist_roll_joint");

        controller_manager_msgs::ControllerState arm_controller;
        arm_controller.name = "arm_trajectory_controller";
        arm_controller.state = "running";
        arm_controller.type = "position_controllers/JointTrajectoryController";
        arm_controller.claimed_resources.push_back(arm_claimed_resources);


        //**********  head_trajectory_controller **********
        controller_manager_msgs::HardwareInterfaceResources head_claimed_resources;
        head_claimed_resources.hardware_interface = "hardware_interface::PositionJointInterface";
        head_claimed_resources.resources.push_back("head_pan_joint");
        head_claimed_resources.resources.push_back("head_tilt_joint");

        controller_manager_msgs::ControllerState head_controller;
        head_controller.name = "head_trajectory_controller";
        head_controller.state = "running";
        head_controller.type = "position_controllers/JointTrajectoryController";
        head_controller.claimed_resources.push_back(head_claimed_resources);


        //**********  gripper_controller **********
        controller_manager_msgs::HardwareInterfaceResources gripper_claimed_resources;
        gripper_claimed_resources.hardware_interface = "hsrb_hardware_interface::HrhGripperInterface";
        gripper_claimed_resources.resources.push_back("hand_motor_joint");

        controller_manager_msgs::ControllerState gripper_controller;
        gripper_controller.name = "gripper_controller";
        gripper_controller.state = "running";
        gripper_controller.type = "hsrb_gripper_controller/HrhGripperController";
        gripper_controller.claimed_resources.push_back(gripper_claimed_resources);


        //**********  omni_base_controller **********
        controller_manager_msgs::HardwareInterfaceResources omni_pos_claimed_resources;
        omni_pos_claimed_resources.hardware_interface = "hardware_interface::PositionJointInterface";
        omni_pos_claimed_resources.resources.push_back("base_roll_joint");
        controller_manager_msgs::HardwareInterfaceResources omni_vel_claimed_resources;
        omni_vel_claimed_resources.hardware_interface = "hardware_interface::VelocityJointInterface";
        omni_vel_claimed_resources.resources.push_back("base_l_drive_wheel_joint");
        omni_vel_claimed_resources.resources.push_back("base_r_drive_wheel_joint");

        controller_manager_msgs::ControllerState omni_controller;
        omni_controller.name = "omni_base_controller";
        omni_controller.state = "running";
        omni_controller.type = "hsrb_base_controllers/OmniBaseController";
        omni_controller.claimed_resources.push_back(omni_pos_claimed_resources);
        omni_controller.claimed_resources.push_back(omni_vel_claimed_resources);


        // set result value.
        output.controller.push_back(joint_state_controller);
        output.controller.push_back(arm_controller);
        output.controller.push_back(head_controller);
        output.controller.push_back(gripper_controller);
        output.controller.push_back(omni_controller);

        return true;

    }//list_controll_server

};

int main(int argc, char** argv){
    ros::init(argc, argv, "sigverse_hsr_list_controller_server");
    List_Controller lc;
    ROS_INFO("sigverse_hsr_list_controller ok.");
    ros::spin();
    return 0;
}
