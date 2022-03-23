#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

bool target_achieved = false;
bool chessboard_detected = false;

geometry_msgs::Pose cam2robot;
cv::Mat T = cv::Mat::eye(cv::Size(4,4), CV_64F);

void fromQuaternion2RPY(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw)
{
        double w = pose.orientation.w;
        double x = pose.orientation.x;
        double y = pose.orientation.y;
        double z = pose.orientation.z;

        roll = atan2(2*(w*x+y*z), pow(w,2) - pow(x,2) - pow(y,2) + pow(z,2));
        pitch = asin(2*(w*y-x*z));
        yaw = atan2(2*(w*z+x*y), pow(w,2) + pow(x,2) - pow(y,2) - pow(z,2));
}

void fromRPY2R(const double& roll, const double& pitch, const double& yaw, cv::Mat& R)
{
        cv::Mat Rx = cv::Mat::eye(cv::Size(3,3), CV_64FC1);
        cv::Mat Ry = cv::Mat::eye(cv::Size(3,3), CV_64FC1);
        cv::Mat Rz = cv::Mat::eye(cv::Size(3,3), CV_64FC1);

        Rx.at<double>(1,1) = std::cos(roll);
        Rx.at<double>(1,2) = -std::sin(roll);
        Rx.at<double>(2,1) = std::sin(roll);
        Rx.at<double>(2,2) = std::cos(roll);

        Ry.at<double>(0,0) = std::cos(pitch);
        Ry.at<double>(0,2) = std::sin(pitch);
        Ry.at<double>(2,0) = -std::sin(pitch);
        Ry.at<double>(2,2) = std::cos(pitch);

        Rz.at<double>(0,0) = std::cos(yaw);
        Rz.at<double>(0,1) = -std::sin(yaw);
        Rz.at<double>(1,0) = std::sin(yaw);
        Rz.at<double>(1,1) = std::cos(yaw);

        R = Rz*Ry*Rx; 
}

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

void robotPoseCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
        cam2robot = *msg;
        
        ros::NodeHandle nh;
        ros::Publisher go_home_pub = nh.advertise<std_msgs::Bool>("/robot/go_home", 10);

        std_msgs::Bool go_home_msg;
        go_home_msg.data = true;
        go_home_pub.publish(go_home_msg);

        double  roll, pitch, yaw;
        fromQuaternion2RPY(cam2robot, roll, pitch, yaw);
        
        cv::Mat R = cv::Mat::eye(cv::Size(3,3), CV_64FC1);
        fromRPY2R(roll, pitch, yaw, R);
        
        for(int i = 0; i < 9; i ++)
        {
                T.at<double>(i/3, i%3) = R.at<double>(i/3, i%3);
        }

        T.at<double>(0,3) = cam2robot.position.x;
        T.at<double>(1,3) = cam2robot.position.y;
        T.at<double>(2,3) = cam2robot.position.z;
}

int main(int argc, char** argv)
{
        ros::init(argc, argv, "main");
        ros::NodeHandle nh;

        ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::Pose>("/robot/pose", 10);
        ros::Publisher start_detect_chessboard = nh.advertise<std_msgs::Bool>("/vision/chessboard/detect", 10);

        ros::Subscriber target_achieve_sub = nh.subscribe("/robot/pose_achieved", 10, targetAchieveCallBack);
        ros::Subscriber chessboard_detect_sub = nh.subscribe("/vision/chessboard/had_detected", 10, chessboardDetectedCallBack);
        ros::Subscriber robot_pose_sub = nh.subscribe("/vision/robot_pose", 10, robotPoseCallBack);

        ros::Rate rate(20);

        std::vector<geometry_msgs::Pose> target_pose_list;
        for(int i = 0; i<27; i ++)
        {
                geometry_msgs::Pose target_pose;

                target_pose.position.x = -0.1 + 0.1*((i%9)/3);
                target_pose.position.y = 0.3 - 0.1*(i%3);
                target_pose.position.z = 0.25 + 0.05*(i/9);

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
                ROS_INFO("Target Achieved! Coordinate: [ %lf, %lf, %lf]", target_pose_list[i].position.x, target_pose_list[i].position.y, target_pose_list[i].position.z);

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

        ros::spin();
        return 0;
}