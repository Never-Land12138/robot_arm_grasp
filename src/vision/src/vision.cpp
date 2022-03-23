#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>

#define m2mm (1000)
#define rad2ang (180./M_PI)
#define _CALIBRATE_POINT_NUM  (27)

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
        //std::cout << "DEPTH Received! Size:" <<  depth.size << std::endl;
}

void startDetectCallBack(const std_msgs::Bool::ConstPtr& msg)
{
        if((*msg).data)
        {
                start_detect = true;
        }
}

void getCornerList(const cv::Mat& rgb, std::vector<cv::Point2f>& corners, bool& chessboard_detected)
{
        if(rgb.empty())
        {
                ROS_WARN("No Image Received!");
                chessboard_detected = false;
                return;
        }

        //cv::imshow("chessboard", rgb);

        cv::Mat gray;
        cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);

        cv::Size patternSize = board_size;

        bool chessboardFound = cv::findChessboardCorners(gray, patternSize, corners);
        if(!chessboardFound) 
        {
                ROS_WARN("No Chessboard Found!");        
                chessboard_detected = false;
                return;
        }

        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01);
        cv::cornerSubPix(gray, corners, cv::Size(5,5), cv::Size(-1, -1), criteria);

        cv::drawChessboardCorners(gray, patternSize, corners, chessboardFound);
        
        //cv::imshow("chessboard", gray);
        
        chessboard_detected = true;
        return;
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
        
        center.z =  depth.at<uint16_t>(center_pixel.y, center_pixel.x);
        //center.z =  (float)depth.at<uint16_t>(320, 240);

        center.x = (center_pixel.x - cameraMatrix.at<double>(0,2))*(center.z/cameraMatrix.at<double>(0,0));  
        center.y = (center_pixel.y - cameraMatrix.at<double>(1,2))*(center.z/cameraMatrix.at<double>(1,1));
}

void getCameraInfo(cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
        cameraMatrix = cv::Mat::eye(3,3,CV_64F);
        cameraMatrix.at<double>(0,0) = 914.6478881835938;
        cameraMatrix.at<double>(0,2) =  639.2798461914062;
        cameraMatrix.at<double>(1,1) =  914.0510864257812;
        cameraMatrix.at<double>(1,2) = 363.2835693359375;

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
        pitch = atan2(-R.at<double>(2,0), sqrt(pow(R.at<double>(2,1),2)+pow(R.at<double>(2,2),2)));
        yaw = atan2(R.at<double>(1,0),R.at<double>(0,0));
}

void fromrpy2Quaternion(const double& roll, const double& pitch, const double&yaw, geometry_msgs::Pose& pose)
{
        cv::Mat mat1(4, 1, CV_64FC1);
        cv::Mat mat2(4, 1, CV_64FC1);
        cv::Mat mat3(4, 1, CV_64FC1);

        mat1.at<double>(0,0)=std::cos(yaw/2);
        mat1.at<double>(3,0)=std::sin(yaw/2);

        mat2.at<double>(0,0)=std::cos(pitch/2);
        mat2.at<double>(2,0)=std::sin(pitch/2);

        mat3.at<double>(0,0)=std::cos(roll/2);
        mat3.at<double>(1,0)=std::sin(roll/2);

        cv::Mat q = mat1*mat2*mat3;

        pose.orientation.w=q.at<double>(0,0);
        pose.orientation.x=q.at<double>(1,0);
        pose.orientation.y=q.at<double>(2,0);
        pose.orientation.z=q.at<double>(3,0);
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

        cv::Mat s = src_decentroid_coordinate * dst_decentroid_coordinate.t();

        cv::Mat u, w, vt;
        cv::SVDecomp(s, w, u, vt);

        cv::Mat mat_temp = u*vt;
        double det = cv::determinant(mat_temp);
        double datM[]={1,0,0,0,1,0,0,0,det};
        cv::Mat matM(3,3,CV_64FC1, datM);

        R = vt.t()*matM*u.t();     
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
        ros::Publisher  robot_pose_pub = nh.advertise<geometry_msgs::Pose>("/vision/robot_pose", 10);

        ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 10,  RGBCallBack);
        ros::Subscriber  depth_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 10, DepthCallBack);
        ros::Subscriber  start_detect_sub = nh.subscribe("/vision/chessboard/detect", 10, startDetectCallBack);
        
        cv::Mat cameraMatrix, distCoeffs;
        getCameraInfo(cameraMatrix, distCoeffs);

        ros::Rate rate(50);

        std::vector<cv::Point3d> robot_points_list;
        for(int i = 0; i < _CALIBRATE_POINT_NUM; i ++)
        {
                cv::Point3d robot_point;
                robot_point.x = (-0.1+0.1*((i%9)/3))*m2mm;
                robot_point.y = (0.3-0.1*(i%3))*m2mm;
                robot_point.z = (0.25+0.05*(i/9))*m2mm;

                robot_points_list.push_back(robot_point);
        }

        std::vector<cv::Point3d> camera_points_list;

        while(ros::ok())
        {
                if(start_detect)
                {
                        for(int k = 0; k < 100; k ++)
                        {
                                rate.sleep();
                                ros::spinOnce();
                        }
                        ROS_INFO("Start Detect Chessboard!");
                        start_detect = false;
                        std::vector<cv::Point2f>corners;

                        while(ros::ok())
                        {                        
                                if(!rgb.empty())
                                {
                                        cv::imshow("rgb", rgb);
                                        cv::waitKey(10);
                                }

                                bool  getCornerListSuccess = false;
                                getCornerList(rgb, corners, getCornerListSuccess);
                                        
                                if(getCornerListSuccess)break;
                                        
                                cv::waitKey(10);
                                ros::spinOnce();
                        }
                                
                        cv::Point3f  real_center(0.,0.,0.), temp_center(0.,0.,0.);
                        int cnt = 0;
                        while(ros::ok())
                        {
                                cnt++;
                                if(cnt >= 100)
                                {
                                        real_center /= cnt;
                                        break;
                                }
                                
                                if(!rgb.empty())
                                {
                                        cv::circle(rgb, cv::Point2f(rgb.size().width/2, rgb.size().height/2),2,cv::Scalar(0,0,255),2);
                                        cv::circle(rgb,corners[corners.size()/2],2,cv::Scalar(255,0,0),2);
                                        cv::imshow("rgb", rgb);
                                        //cv::imshow("depth", depth);

                                        cv::waitKey(10);
                                }

                                getCenter(corners, cameraMatrix, temp_center);

                                real_center+=temp_center;

                                ros::spinOnce();
                        }

                        ROS_INFO("Chessboard Detected! Center Coordinate: [ %lf, %lf, %lf]", real_center.x,real_center.y, real_center.z);
                        camera_points_list.push_back((cv::Point3d)real_center);
                                
                        std_msgs::Bool msg;
                        msg.data = true;
                        chessboard_detected_pub.publish(msg);

                        ROS_INFO("Camera Points List Size: [%d]", camera_points_list.size());
                        if(camera_points_list.size() == _CALIBRATE_POINT_NUM)break;
                }

                ros::spinOnce();
                rate.sleep();
        }

        cv::Mat T = cv::Mat::eye(cv::Size(4,4), CV_64F);
        cv::Mat R = cv::Mat::eye(cv::Size(3,3), CV_64F);
        cv::Mat t;
        double roll, pitch, yaw;

        ICP(camera_points_list, robot_points_list, T, R, t);
        fromR2rpy(R, roll, pitch, yaw);

        ROS_INFO("x: [%lf], y: [%lf], z: [%lf], roll: [%lf], pitch: [%lf], yaw: [%lf]", t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0), roll*rad2ang, pitch*rad2ang, yaw*rad2ang);
        std::cout<<"Calculated T:"<<std::endl<<T<<std::endl;

        // for(int i = 0; i <_CALIBRATE_POINT_NUM; i ++)
        // {
        //         cv::Mat homo(4,1,CV_64F);
                
        //         homo.at<double>(0,0) = (double)camera_points_list[i].x;
        //         homo.at<double>(1,0) = (double)camera_points_list[i].y;
        //         homo.at<double>(2,0) = (double)camera_points_list[i].z;
        //         homo.at<double>(3,0) = 1.;

        //         std::cout<<"Calculate Transform ["<<i <<"]: "<<(T*homo)<<std::endl;
        //         std::cout<<robot_points_list[i]<<std::endl;
        // }

        geometry_msgs::Pose cam2robot;
        cam2robot.position.x = t.at<double>(0,0);
        cam2robot.position.y = t.at<double>(1,0);
        cam2robot.position.z = t.at<double>(2,0);
        fromrpy2Quaternion(roll, pitch, yaw, cam2robot);
        robot_pose_pub.publish(cam2robot);

        ros::spin();
        return 0;
}