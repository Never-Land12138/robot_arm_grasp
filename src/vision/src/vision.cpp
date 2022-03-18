#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

#define m2mm (1000)

cv::Mat rgb;
cv::Mat depth;
cv::Size board_size = cv::Size(3,3);

bool start_detect = false;
bool chessboard_detected = false; 

void RGBCallBack(const sensor_msgs::Image::ConstPtr& msg)
{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::TYPE_8UC3);
        rgb = cv_ptr->image;
        //std::cout<<"RGB Received! Size:" << rgb.size << std::endl;
}

void DepthCallBack(const sensor_msgs::Image::ConstPtr& msg)
{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::TYPE_16UC1);
        depth = cv_ptr->image;
        //std::cout << "DEPTH RECEIVED! Size:" <<  depth.size << std::endl;
}

void startDetectCallBack(const std_msgs::Bool::ConstPtr& msg)
{
        if((*msg).data)
        {
                start_detect = true;
        }
}

bool getCornerList(const cv::Mat& rgb, std::vector<cv::Point2f>& corners)
{
        if(rgb.empty())
        {
                ROS_WARN("No Image Received!");
                return false;
        }
        cv::Mat gray;
        cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);

        cv::Size patternSize = board_size;

        bool chessboardFound = cv::findChessboardCorners(gray, patternSize, corners);
        if(!chessboardFound) 
        {
                ROS_WARN("No Chessboard Found!");        
                return false;
        }

        //std::cout<<depth.at<uint16_t>(corners[0].x, corners[0].y)<<std::endl;
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01);
        cv::cornerSubPix(gray, corners, cv::Size(5,5), cv::Size(-1, -1), criteria);

        cv::drawChessboardCorners(rgb, patternSize, corners, chessboardFound);
        
        cv::imshow("chessboard", rgb);
        
        return true;
}

void getRT(const std::vector<cv::Point2f>& corners, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, 
                        cv::Mat& R, cv::Mat& T)
{
        if(corners.empty() || cameraMatrix.empty() || distCoeffs.empty())return;

        std::vector<cv::Point3f> objectPoints;
        
        for(int i = 0; i < board_size.width; i++)
        {
                for(int j = 0; j < board_size.height; j++)
                {
                        cv::Point3f realPoint;
                        realPoint.x = (double)(i-1)*10.;
                        realPoint.y = (double)(j-1)*10.;
                        realPoint.z = 0.;
                        objectPoints.push_back(realPoint);
                }
        }

        cv::Mat rvec;
        cv::solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, T);
        cv::Rodrigues(rvec, R);
}

void getCenter(const std::vector<cv::Point2f>& corners, const cv::Mat& cameraMatrix, cv::Point3f& center)
{
        if(corners.empty() || cameraMatrix.empty())return;

        cv::Point2f center_pixel = corners.at(corners.size()/2);
        
        center.z =  (float)depth.at<uint16_t>(center_pixel.x, center_pixel.y);
        center.x = (center_pixel.x - cameraMatrix.at<double>(0,2))*(center.z/cameraMatrix.at<double>(0,0));  
        center.y = (center_pixel.y - cameraMatrix.at<double>(1,2))*(center.z/cameraMatrix.at<double>(1,1));
}

void getCameraInfo(cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
        cameraMatrix = cv::Mat::eye(3,3,CV_64F);
        cameraMatrix.at<double>(0,0) = 609.7652587890625;
        cameraMatrix.at<double>(0,2) =  319.5198974609375;
        cameraMatrix.at<double>(1,1) =  609.3673706054688;
        cameraMatrix.at<double>(1,2) = 242.18905639648438;

        std::vector<double> distCoeffs_vec{6.8641294614044765e-02, 
                                                                                3.2902097109600531e-01,
                                                                                1.8755845953358030e-03, 
                                                                                2.2316380587360627e-03,
                                                                                -1.7254670555737144e+00};
        distCoeffs = cv::Mat(distCoeffs_vec, true);
}

void fromR2rpy(const cv::Mat& R, double& roll, double& pitch, double& yaw)
{
        roll = atan2(R.at<double>(2,1), R.at<double>(2,2));
        pitch = atan2(-R.at<double>(2,0), sqrt(pow(R.at<double>(0,0),2)+pow(R.at<double>(1,0),2)));
        yaw = atan2(R.at<double>(1,0),R.at<double>(0,0));
}

bool ICP(const std::vector<cv::Point3d>& src_points, const std::vector<cv::Point3d>& dst_points, cv::Mat& T, cv::Mat& R, cv::Mat& t)
{
        T = cv::Mat::eye(cv::Size(4,4), CV_64FC1);
        if(src_points.size() != dst_points.size() || src_points.size() < 3)
        {
                ROS_WARN("ICP Wrong!");
                return false;
        }
        int points_num = src_points.size();

        cv::Point3d src_center = cv::Point3d(0,0,0);
        cv::Point3d dst_center = cv::Point3d(0,0,0);
        cv::Mat _src_center(3,1,CV_64FC1);
        cv::Mat _dst_center(3,1,CV_64FC1);

        for(int i = 0; i < points_num; i++)
        {
                src_center.x += ((double)src_points[i].x)/points_num;
                src_center.y += ((double)src_points[i].y)/points_num;
                src_center.z += ((double)src_points[i].z)/points_num;
                _src_center.at<double>(0,0) = src_center.x;
                _src_center.at<double>(1,0) = src_center.y;
                _src_center.at<double>(2,0) = src_center.z;

                dst_center.x += ((double)dst_points[i].x)/points_num;
                dst_center.y += ((double)dst_points[i].y)/points_num;
                dst_center.z += ((double)dst_points[i].z)/points_num;
                _dst_center.at<double>(0,0) = dst_center.x;
                _dst_center.at<double>(1,0) = dst_center.y;
                _dst_center.at<double>(2,0) = dst_center.z;
        }

        cv::Mat src_decentroid_coordinate(3, points_num, CV_64FC1);
        cv::Mat dst_decentroid_coordinate(3, points_num, CV_64FC1);

        for(int i = 0; i < points_num; i++)
        {
                src_decentroid_coordinate.at<double>(0,i) = src_points[i].x - src_center.x;
                src_decentroid_coordinate.at<double>(1,i) = src_points[i].y - src_center.y;
                src_decentroid_coordinate.at<double>(2,i) = src_points[i].z - src_center.z;
                
                dst_decentroid_coordinate.at<double>(0,i) = dst_points[i].x - dst_center.x;
                dst_decentroid_coordinate.at<double>(1,i) = dst_points[i].y - dst_center.y;
                dst_decentroid_coordinate.at<double>(2,i) = dst_points[i].z - dst_center.z;
        }

        cv::Mat S = src_decentroid_coordinate * dst_decentroid_coordinate.t();

        cv::Mat u, w, vt;
        cv::SVDecomp(S, w, u, vt);

        R = vt.t()*u.t();        
        t = _dst_center - R*_src_center;

        for(int i = 0; i < 3; i++)
        {
                for(int j = 0; j < 3; j++)
                {
                        T.at<double>(i,j) = R.at<double>(i,j);
                }
                T.at<double>(i,3) = t.at<double>(i,0);
        }

        return true;
}

int main(int argc, char** argv)
{
        ros::init(argc, argv, "vision");
        ros::NodeHandle nh;

        ros::Publisher chessboard_detected_pub = nh.advertise<std_msgs::Bool>("/vision/chessboard/had_detected", 10);

        ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 10,  RGBCallBack);
        ros::Subscriber  depth_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 10, DepthCallBack);
        ros::Subscriber  start_detect_sub = nh.subscribe("/vision/chessboard/detect", 10, startDetectCallBack);
        
        cv::Mat cameraMatrix, distCoeffs;
        getCameraInfo(cameraMatrix, distCoeffs);

        ros::Rate rate(50);

        //ICP test
        // while(ros::ok())
        // {
        //         cv::Point3d srcPoint1(0,0,0), srcPoint2(1,1,1), srcPoint3(2,1,2);
        //         cv::Point3d dstPoint1(0,0,0), dstPoint2(1,-1,1), dstPoint3(1,-2,2);

        //         std::vector<cv::Point3d> src_points{srcPoint1, srcPoint2, srcPoint3}, dst_points{dstPoint1, dstPoint2, dstPoint3};

        //         cv::Mat T = cv::Mat::eye(cv::Size(4,4), CV_64FC1);
        //         cv::Mat R = cv::Mat::eye(cv::Size(3,3), CV_64FC1);
        //         cv::Mat t;
        //         double roll, pitch, yaw;
        //         ICP(src_points, dst_points, T, R, t);
        //         fromR2rpy(R, roll, pitch, yaw);
        //         std::cout<<T<<std::endl;
        //         std::cout<<"roll: "<<roll/M_PI*180<<"\tpitch:"<<pitch/M_PI*180<<"\tyaw:"<<yaw/M_PI*180<<std::endl;
        // }

        std::vector<cv::Point3d> robot_points_list;
        for(int i = 0; i < 9; i ++)
        {
                cv::Point3d robot_point;
                robot_point.x = (-0.1+0.1*(i/3))*m2mm;
                robot_point.y = (0.3-0.1*(i%3))*m2mm;
                robot_point.z = 0.3*m2mm;

                robot_points_list.push_back(robot_point);
        }

        std::vector<cv::Point3d> camera_points_list;
        for(int i = 0; i < 9; i++)
        {
                while(ros::ok())
                {
                        if(start_detect)
                        {
                                ROS_INFO("Start Detect Chessboard!");
                                start_detect = false;
                                std::vector<cv::Point2f>corners;

                                if(!rgb.empty())
                                {
                                        cv::imshow("chessboard", rgb);
                                        cv::waitKey(10);
                                }

                                while(ros::ok())
                                {
                                        bool  getCornerListSuccess = getCornerList(rgb, corners);
                                        if(getCornerListSuccess)break;
                                }

                                cv::Point3f center(0.,0.,0.), last_center(0., 0., 0.);
                                while(ros::ok())
                                {
                                        getCenter(corners, cameraMatrix, center);

                                        if(abs(center.z - last_center.z)<10 && center.z > 100 && center.z < 10000 )break;
                                        
                                        last_center = center;
                                }
                                ROS_INFO("Chessboard Detected! Center Coordinate: [%s, %s, %s]", center.x, center.y, center.z);
                                camera_points_list.push_back((cv::Point3d)center);
                                
                                std_msgs::Bool msg;
                                msg.data = true;
                                chessboard_detected_pub.publish(msg);
                        }

                        ros::spinOnce();
                        rate.sleep();
                }
        }

        cv::Mat T = cv::Mat::eye(cv::Size(4,4), CV_64F);
        cv::Mat R = cv::Mat::eye(cv::Size(3,3), CV_64F);
        cv::Mat t;
        double roll, pitch, yaw;

        ICP(camera_points_list, robot_points_list, T, R, t);
        fromR2rpy(R, roll, pitch, yaw);

        std::cout<<"x: " << t.at<double>(0,0)<< "\ty: "<<t.at<double>(1,0)<<"\tz: "<<t.at<double>(2,0)<<"\troll: "<<roll<<"\tpitch:"<<pitch<<"\tyaw:"<<yaw<<std::endl;


        // while(ros::ok())
        // {
        //         std::vector<cv::Point2f> corners;

        //         getCornerList(rgb, corners);
                
        //         cv::Point3f center;
        //         getCenter(corners, cameraMatrix, center);
        //         if((last_center.x - center.x) < 10&&(last_center.y - center.y) < 10&&(last_center.z - center.z)<10
        //                 &&center.z<3000&&center.z>100)
        //         {
        //                 //std::cout<<center<<std::endl;
        //         }

        //         last_center = center;

        //         double roll, pitch, yaw;
        //         cv::Mat R = cv::Mat::eye(cv::Size(3,3),CV_64F);
        //         cv::Mat T;
        //         getRT(corners, cameraMatrix, distCoeffs, R, T);
        //         fromR2rpy(R, roll, pitch, yaw);

        //         //std::cout<<"roll: "<<roll/M_PI*180<<"\tpitch:"<<pitch/M_PI*180<<"\tyaw:"<<yaw/M_PI*180<<std::endl;

        //         if(!rgb.empty())
        //                 cv::imshow("chessboard", rgb);

        //         cv::waitKey(1);

        //         rate.sleep();
        //         ros::spinOnce();
        // }
}