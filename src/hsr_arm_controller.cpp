#include <iostream>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class HSRB_ARM_Action
{
protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    std::string action_name_;

    // create messages that are used to published feedback/result
    control_msgs::FollowJointTrajectoryFeedback feedback;
    control_msgs::FollowJointTrajectoryResult result;
    ros::Publisher pub_arm_command;
    ros::Subscriber sub_joint_state;

    //joint angle
    double arm_lift_joint_pos;
    double arm_flex_joint_pos;
    double arm_roll_joint_pos;
    double wrist_flex_joint_pos;
    double wrist_roll_joint_pos;

public:

    HSRB_ARM_Action(std::string name) : as_(nh, name, boost::bind(&HSRB_ARM_Action::executeCB, this, _1), false), action_name_(name)
    {
        this->as_.start();
        this->pub_arm_command = nh.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
        this->sub_joint_state = nh.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &HSRB_ARM_Action::jointStateCallback, this);
    }

    ~HSRB_ARM_Action(void){}

    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal_msg)
    {

        bool success = true;
        for(int i = 0; i < goal_msg->trajectory.points.size(); i++){
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
            traj.joint_names = goal_msg->trajectory.joint_names;
            traj.points.push_back(goal_msg->trajectory.points[i]);
            this->pub_arm_command.publish(traj);

            //send feedback
            feedback.header = goal_msg->trajectory.header;
            feedback.joint_names = goal_msg->trajectory.joint_names;
            feedback.desired.positions = goal_msg->trajectory.points[i].positions;
            feedback.actual.positions.clear();
            feedback.actual.positions.push_back(this->arm_lift_joint_pos);
            feedback.actual.positions.push_back(this->arm_flex_joint_pos);
            feedback.actual.positions.push_back(this->arm_roll_joint_pos);
            feedback.actual.positions.push_back(this->wrist_flex_joint_pos);
            feedback.actual.positions.push_back(this->wrist_roll_joint_pos);
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
            if(joint_state->name[i] == "arm_lift_joint"){
                this->arm_lift_joint_pos = joint_state->position[i];
            }else if(joint_state->name[i] == "arm_flex_joint"){
                this->arm_flex_joint_pos = joint_state->position[i];
            }else if(joint_state->name[i] == "arm_roll_joint"){
                this->arm_roll_joint_pos = joint_state->position[i];
            }else if(joint_state->name[i] == "wrist_flex_joint"){
                this->wrist_flex_joint_pos = joint_state->position[i];
            }else if(joint_state->name[i] == "wrist_roll_joint"){
                this->wrist_roll_joint_pos = joint_state->position[i];
            }//if
        }//for
    }//jointStateCallback

};//HSRB_ARM_Action


int main(int argc, char** argv){

    ros::init(argc, argv, "sigverse_hsr_arm_controller");
    HSRB_ARM_Action arm_controller("/hsrb/arm_trajectory_controller/follow_joint_trajectory");
    ROS_INFO("sigverse_hsr_arm_controller ok.");
    ros::spin();

    return 0;
}//main
