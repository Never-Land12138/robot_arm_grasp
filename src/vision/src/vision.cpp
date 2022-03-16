#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

cv::Mat rgb;
cv::Mat depth;
cv::Size board_size = cv::Size(3,3);

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

void getCornerList(const cv::Mat& rgb, std::vector<cv::Point2f>& corners)
{
        if(rgb.empty())return;

        cv::Mat gray;
        cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);

        cv::Size patternSize = board_size;

        bool chessboardFound = cv::findChessboardCorners(gray, patternSize, corners);
        if(!chessboardFound) return;

        //std::cout<<depth.at<uint16_t>(corners[0].x, corners[0].y)<<std::endl;
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01);
        cv::cornerSubPix(gray, corners, cv::Size(5,5), cv::Size(-1, -1), criteria);

        cv::drawChessboardCorners(rgb, patternSize, corners, chessboardFound);
        
        //cv::imshow("chessboard", rgb);
        
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

int main(int argc, char** argv)
{
        ros::init(argc, argv, "vision");
        ros::NodeHandle nh;

        ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 10,  RGBCallBack);
        ros::Subscriber  depth_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 10, DepthCallBack);
        
        cv::Mat cameraMatrix, distCoeffs;
        getCameraInfo(cameraMatrix, distCoeffs);
        cv::Point3f last_center;
        ros::Rate rate(50);
        while(ros::ok())
        {
                std::vector<cv::Point2f> corners;

                getCornerList(rgb, corners);
                
                cv::Point3f center;
                getCenter(corners, cameraMatrix, center);
                if((last_center.x - center.x) < 10&&(last_center.y - center.y) < 10&&(last_center.z - center.z)<10
                        &&center.z<3000&&center.z>100)
                {
                        std::cout<<center<<std::endl;
                }

                last_center = center;

                double roll, pitch, yaw;
                cv::Mat R = cv::Mat::eye(cv::Size(3,3),CV_64F);
                cv::Mat T;
                getRT(corners, cameraMatrix, distCoeffs, R, T);
                fromR2rpy(R, roll, pitch, yaw);

                //std::cout<<"roll: "<<roll/M_PI*180<<"\tpitch:"<<pitch/M_PI*180<<"\tyaw:"<<yaw/M_PI*180<<std::endl;

                if(!rgb.empty())
                cv::imshow("chessboard", rgb);

                cv::waitKey(1);

                rate.sleep();
                ros::spinOnce();
        }
}