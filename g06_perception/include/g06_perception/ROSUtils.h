#ifndef ROS__UTILS__H
#define ROS__UTILS__H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <map>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace ros_hw_utils {
    class ROSUtils {
        public:

            ROSUtils(ros::NodeHandle nodehandle, bool simulated);

            std::map<std::string, double> loadConfigParams();

            double getParam(std::string name);

            int getPrimitiveShape(int id);

            std::string convertIdToFrameId(int id);

            int convertFrameIdToId(std::string frame_id);

            std::string getModelNameById(int id);

            apriltags_ros::AprilTagDetection applyOffset(const apriltags_ros::AprilTagDetection detection);

            geometry_msgs::Pose createPose(double x, double y, double z, double ox, double oy, double oz, double ow);

            void publishPoses(const std::string topicName, const apriltags_ros::AprilTagDetectionArray foundItems, bool continuePublishing);

            void publishAllAndRequestedPoses(const std::string topicNameAll, const apriltags_ros::AprilTagDetectionArray foundItems, const std::string topicNameRequested, const apriltags_ros::AprilTagDetectionArray foundRequestedItems, bool continuePublishing);

            geometry_msgs::Pose doTransform(geometry_msgs::Pose original_pose, std::string toFrame = "world", std::string fromFrame = "camera_rgb_optical_frame", std::string publishName = "tr_");
            
            static const int SHAPE_CUBE = 0;
            static const int SHAPE_CYLINDER = 1;
            static const int SHAPE_TRIANGLE = 2;

        private:

			ros::NodeHandle m_nh;
            bool m_simulated;
            int NUM_OBJECTS = 16;
            std::vector<std::string> m_frame_ids;
            std::vector<std::string> m_model_ids;
            std::map<std::string, double> m_params;

    };
};

#endif