#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
        ros::init(argc, argv, "move_group_interface_tutorial");
        ros::NodeHandle node_handle;
        ros::AsyncSpinner spinner(1);
        spinner.start();

        static const std::string PLANNING_GROUP = "gluon";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const moveit::core::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
        visual_tools.deleteAllMarkers();
        visual_tools.loadRemoteControl();
        
        visual_tools.trigger();

        ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
        std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));

        while(ros::ok())
        {
                visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go to home");
                std::vector<double> joint_home_position(6,0.);
                move_group.setJointValueTarget(joint_home_position);

                moveit::planning_interface::MoveGroupInterface::Plan start;
                bool success = (move_group.plan(start)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ROS_INFO("Go to home");
                move_group.execute(start);
                
                // geometry_msgs::Pose target_pose1;
                // target_pose1.orientation.w = 1.;
                // target_pose1.position.x = 0.1;
                // target_pose1.position.y = 0.3;
                // target_pose1.position.z = 0.4;
                // move_group.setPoseTarget(target_pose1);

                // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                // bool success = (move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);

                // ROS_INFO("Move to target pose1");
                // move_group.execute(my_plan);

                for(int i = 0; i < 1; i++)
                {
                        geometry_msgs::Pose target_pose;

                        target_pose.orientation.w = 1.;
                        target_pose.position.y = 0.3 - 0.1*(i%3);
                        target_pose.position.x= -0.1 + 0.1*(i/3);
                        target_pose.position.z = 0.3;

                        std::cout<<"target position: "<<target_pose.position<<std::endl;

                        move_group.setPoseTarget(target_pose);

                        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                        success = (move_group.plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                        
                        move_group.execute(my_plan);

                        sleep(2);
                }
        }
        return 0;
}
