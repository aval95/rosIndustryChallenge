#ifndef ROS__NAVIGATION__H
#define ROS__NAVIGATION__H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <move_base/move_base.h>
#include <cstdlib>
#include <limits>
#include <laser_line_extraction/LineSegment.h>
#include <laser_line_extraction/LineSegmentList.h>
#include <g06_perception/ROSUtils.h>

namespace ros_hw_utils {
    class ROSNavigation {
        public:

            ROSNavigation(ros::NodeHandle nodehandle, bool simulated, bool is360laser);

            void setUtils(ros_hw_utils::ROSUtils *utils);

            geometry_msgs::PoseWithCovariance getCurrentRobotPosition(bool useOdom = false);

            geometry_msgs::Pose setRobotGoalPosition(geometry_msgs::Pose pose, bool execute = false);

            geometry_msgs::TwistWithCovariance getCurrentRobotVelocity();

            geometry_msgs::Twist setRobotVelocity(geometry_msgs::Twist twist, const double time, bool execute = false);

            sensor_msgs::LaserScan getLaserScan();

            sensor_msgs::LaserScan getFrontLaserData(sensor_msgs::LaserScan inputScan);

            sensor_msgs::LaserScan getRearLaserData(sensor_msgs::LaserScan inputScan);

            sensor_msgs::LaserScan getRightLaserData(sensor_msgs::LaserScan inputScan);

            sensor_msgs::LaserScan getLeftLaserData(sensor_msgs::LaserScan inputScan);

            double getLaserScan(int scanID);

            double getLaserRightDistance(int numberOfValuesForMean = 40);

            double getLaserLeftDistance(int numberOfValuesForMean = 40);

            double getLaserFrontDistance(int numberOfValuesForMean = 80);

            double getLaserRightDistance(sensor_msgs::LaserScan inputScan, int numberOfValuesForMean = 40);

            double getLaserLeftDistance(sensor_msgs::LaserScan inputScan, int numberOfValuesForMean = 40);

            double getLaserFrontDistance(sensor_msgs::LaserScan inputScan, int numberOfValuesForMean = 80);

            double getLaserRightDistanceA(sensor_msgs::LaserScan inputScan);

            double getLaserRightDistanceB(sensor_msgs::LaserScan inputScan);

            double getLaserRightDistanceC(sensor_msgs::LaserScan inputScan);

            double getLaserRightDistanceD(sensor_msgs::LaserScan inputScan);

            double getLaserLeftDistanceA(sensor_msgs::LaserScan inputScan);

            double getLaserLeftDistanceB(sensor_msgs::LaserScan inputScan);

            double getLaserLeftDistanceC(sensor_msgs::LaserScan inputScan);

            double getLaserLeftDistanceD(sensor_msgs::LaserScan inputScan);

            int getObstacleRelativePosition(sensor_msgs::LaserScan inputScan);

            laser_line_extraction::LineSegmentList getRightLines(laser_line_extraction::LineSegmentList linesList);

            laser_line_extraction::LineSegmentList getLeftLines(laser_line_extraction::LineSegmentList linesList);

            laser_line_extraction::LineSegment getFrontLine(laser_line_extraction::LineSegmentList linesList);

            laser_line_extraction::LineSegment getRightLine(laser_line_extraction::LineSegmentList linesList);

            laser_line_extraction::LineSegment getLeftLine(laser_line_extraction::LineSegmentList linesList);

            void mnav_goStraight(const double &time, int sign, double linear_speed = 0.1);

            void mnav_goStraightWithCurve(const double &time, int sign, double linear_speed = 0.1, double angular_speed = 0.2);

            void mnav_turn(const double &rotation_time, int sign, double angular_speed = M_PI / 8);

            void mnav_turnAngleRadians(const double radians, int sign, double angular_speed = M_PI / 8, bool then_stop = false);

            void mnav_turnAngleDegrees(const double degrees, int sign, double angular_speed = M_PI / 8, bool then_stop = false);

            void mnav_stop();

        private:

            sensor_msgs::LaserScan preprocessLaserData(sensor_msgs::LaserScan inputScan);

            int neg_mod(int num1, int num2);

            ros::NodeHandle m_nh;
            geometry_msgs::Twist m_velocity;
            double m_linear = 0.1;
            double m_angular = 0;
            ros::Publisher m_vel_pub;
            bool m_simulated;
            bool m_is_360degrees_laser;
            std::string m_cmd_vel_topic;
            std::string m_odom_topic;
            ros_hw_utils::ROSUtils *m_utils;
            ros::Publisher m_laser_pub;

    };

};

#endif