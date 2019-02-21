#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64.h>

class HSRB_Gripper_Action
{
protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    std::string action_name_;

    // create messages that are used to published feedback/result
    control_msgs::FollowJointTrajectoryFeedback feedback;
    control_msgs::FollowJointTrajectoryResult result;
    ros::Publisher pub_gripper_command;
    ros::Subscriber sub_joint_state;

    //joint angle
    double hand_motor_joint_pos;

public:

    HSRB_Gripper_Action(std::string name) : as_(nh, name, boost::bind(&HSRB_Gripper_Action::executeCB, this, _1), false), action_name_(name)
    {
        this->as_.start();
        this->pub_gripper_command = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/gripper_trajectory_controller/command", 10);
        this->sub_joint_state = nh.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &HSRB_Gripper_Action::jointStateCallback, this);
    }

    ~HSRB_Gripper_Action(void){}

    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal_msg)
    {
        std::cout << "msg.size() = " << goal_msg->trajectory.points.size() << std::endl;
        bool success = true;
        for(int i = 0; i<goal_msg->trajectory.points.size(); i++){

            std::cout << "call" << std::endl;

            if (as_.isPreemptRequested() || !ros::ok()){
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }//if_cancel

            //send sigverse
            trajectory_msgs::JointTrajectory traj;
            traj.header = goal_msg->trajectory.header;
            //traj.joint_names = goal_msg->trajectory.joint_names;
            traj.joint_names.push_back("hand_l_proximal_joint");
            traj.joint_names.push_back("hand_r_proximal_joint");

            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.push_back(goal_msg->trajectory.points[i].positions[0]);
            point.positions.push_back(-goal_msg->trajectory.points[i].positions[0]);
            point.velocities.push_back(0.0);
            point.velocities.push_back(0.0);
            point.time_from_start = ros::Duration(2);

            traj.points.push_back(point);
            this->pub_gripper_command.publish(traj);

            //send feedback
            feedback.header = goal_msg->trajectory.header;
            feedback.joint_names = goal_msg->trajectory.joint_names;
            feedback.desired.positions = goal_msg->trajectory.points[i].positions;
            feedback.actual.positions.clear();
            feedback.actual.positions.push_back(this->hand_motor_joint_pos);
            as_.publishFeedback(feedback);

            //sleep
            goal_msg->trajectory.points[i].time_from_start.sleep();
        }//for_trajectory

        if(success)
        {
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          // set the action state to succeeded
          as_.setSucceeded(result);
        }
    }//executeCB

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state){
        for(int i=0; i<joint_state->name.size(); i++){
            if(joint_state->name[i] == "hand_motor_joint"){
                this->hand_motor_joint_pos = joint_state->position[i];
            }
        }//for
    }//jointStateCallback

};//HSRB_Gripper_Action


int main(int argc, char** argv){

    ros::init(argc, argv, "sigverse_hsr_gripper_controller");
    HSRB_Gripper_Action gripper_controller("/hsrb/gripper_controller/follow_joint_trajectory");
    ROS_INFO("sigverse_hsr_gripper_controller ok.");
    ros::spin();

    return 0;
}//main
