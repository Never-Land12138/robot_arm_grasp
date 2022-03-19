#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

bool move_finished = false;
bool msg_received = false;
geometry_msgs::Pose target_pose;
 int cnt = 0;

void PoseCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
        if(move_finished)
        {
                cnt = 0;
                target_pose = *msg;
                msg_received = true;
                ROS_INFO("Target Pose Received!");
        }
}

void GoHomeCallBack(const std_msgs::Bool::ConstPtr& msg)
{
        if((*msg).data)
        {
                static const std::string PLANNING_GROUP = "gluon";
                moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

                std::vector<double> joint_home_position(6,0);
                move_group.setJointValueTarget(joint_home_position);

                moveit::planning_interface::MoveGroupInterface::Plan go_home;
                bool success = (move_group.plan(go_home)==moveit::planning_interface::MoveItErrorCode::SUCCESS);

                ROS_INFO("Go to home");
                move_group.execute(go_home);
        }
}

int main(int argc, char** argv)
{
        ros::init(argc, argv, "robot_arm_server");
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::Publisher target_achieved_pub = nh.advertise<std_msgs::Bool>("/robot/pose_achieved", 10);

        ros::Subscriber pose_sub = nh.subscribe("/robot/pose", 10, PoseCallBack);
        ros::Subscriber go_home_sub = nh.subscribe("/robot/go_home", 10, GoHomeCallBack);

        static const std::string PLANNING_GROUP = "gluon";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const moveit::core::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        const std::string end_effector_name = move_group.getEndEffectorLink();
        std::cout<<"current pose: "<<move_group.getCurrentPose().pose<<std::endl;

        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
        visual_tools.deleteAllMarkers();
        visual_tools.loadRemoteControl();
        
        visual_tools.trigger();

        ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
        std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));

        std::vector<double> joint_home_position(6,0.);
        move_group.setJointValueTarget(joint_home_position);

        moveit::planning_interface::MoveGroupInterface::Plan start;
        bool success = (move_group.plan(start)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Go to home");
        move_group.execute(start);

        move_finished = true;

        while(ros::ok())
        {
                if(move_finished&&msg_received)
                {
                        move_finished = false;
                        msg_received = false;

                        std::cout<<"target position: "<<target_pose.position<<std::endl;

                        move_group.setPoseTarget(target_pose);

                        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                        success = (move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                        
                        move_group.execute(my_plan);

                        move_finished = true;

                        geometry_msgs::Pose curr_pose = move_group.getCurrentPose().pose;

                        if(abs(curr_pose.position.x - target_pose.position.x) + abs(curr_pose.position.y - target_pose.position.y) + abs(curr_pose.position.z - target_pose.position.z) < 0.1&&cnt == 0)
                        {
                                cnt++; 
                                std_msgs::Bool msg;
                                msg.data = 1;
                                target_achieved_pub.publish(msg);
                        }
                }
                ros::spinOnce();
        }

        success = (move_group.plan(start)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Go to home");
        move_group.execute(start);

        return 0;
}