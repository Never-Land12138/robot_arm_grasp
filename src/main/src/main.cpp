#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

bool target_achieved = false;
bool chessboard_detected = false;

void targetAchieveCallBack(const std_msgs::Bool::ConstPtr& msg)
{
        if((*msg).data)
        {
                target_achieved = true;
        }
}

void chessboardDetectedCallBack(const std_msgs::Bool::ConstPtr& msg)
{
        if((*msg).data)
        {
                chessboard_detected = true;
        }
}

int main(int argc, char** argv)
{
        ros::init(argc, argv, "main");
        ros::NodeHandle nh;

        ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::Pose>("/robot/pose", 10);
        ros::Publisher start_detect_chessboard = nh.advertise<std_msgs::Bool>("/vision/chessboard/detect", 10);

        ros::Subscriber target_achieve_sub = nh.subscribe("/robot/pose_achieved", 10, targetAchieveCallBack);
        ros::Subscriber chessboard_detect_sub = nh.subscribe("/vision/chessboard/had_detected", 10, chessboardDetectedCallBack);

        ros::Rate rate(20);

        std::vector<geometry_msgs::Pose> target_pose_list;
        for(int i = 0; i<9; i ++)
        {
                geometry_msgs::Pose target_pose;

                target_pose.position.x = -0.1 + 0.1*(i/3);
                target_pose.position.y = 0.3 - 0.1*(i%3);
                target_pose.position.z = 0.35;

                target_pose.orientation.w = 1.;

                target_pose_list.push_back(target_pose);
        }

        for(int i = 0; i < target_pose_list.size(); i++)
        {
                bool msg_send = false;
                while(ros::ok()&&(!target_achieved))
                {
                        if(!msg_send)
                        {
                                target_pose_pub.publish(target_pose_list[i]);
                                msg_send = false;
                        }
                        ros::spinOnce();
                        rate.sleep();
                }
                target_achieved = false;
                //ROS_INFO("Target Achieved! Coordinate: [ %s, %s, %s]", target_pose_list[i].position.x, target_pose_list[i].position.y, target_pose_list[i].position.z);

                std_msgs::Bool msg;
                msg.data = true;
                start_detect_chessboard.publish(msg);

                ros::spinOnce();

                while(ros::ok()&&(!chessboard_detected))
                {
                        ros::spinOnce();
                        rate.sleep();
                }
                chessboard_detected = false;
        }
        return 0;
}