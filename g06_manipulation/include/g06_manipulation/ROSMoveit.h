#ifndef ROS__MOVEIT__H
#define ROS__MOVEIT__H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <map>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <shape_msgs/SolidPrimitive.h>
#include <geometric_shapes/shape_operations.h>

#include <robotiq_s_model_control/SModel_robot_output.h>
#include <gazebo_ros_link_attacher/Attach.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <fstream>
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>	// POSIX terminal control definitions

// Servizi
#include <g06_fsm/ManipulationReady.h>
#include <g06_fsm/ManipulationToFSM.h>

namespace ros_hw_utils {
    class ROSMoveit {
        public:

			ROSMoveit(ros::NodeHandle nodehandle, std::string move_group_name, bool simulated = true);
			
			void initPlanningAndMove(std::string move_group_name);
			
			bool setPose(double x, double y, double z, double ox, double oy, double oz, double ow, bool execute = false);

			bool setPose(geometry_msgs::Pose pose, bool execute = false);

			int getCollisionObjectIndexById(std::string id);

			bool setPoseWithCartesianPath(std::vector<geometry_msgs::Pose> waypoints, bool execute = false);

			bool setPoseWithJoints(std::vector<double> joint_group_positions, bool execute = false);
			
			std::string addCollisionObject(moveit_msgs::CollisionObject co, bool apply = false);
			
			std::string addCollisionObject(const shapes::Mesh* m, std::string id, geometry_msgs::Pose pose, bool apply = false);

			int applyCollisionObjectsToScene();

			moveit_msgs::CollisionObject removeCollisionObject(std::string id, bool apply = false);

			void removeCollisionObjects();
			
			moveit_msgs::CollisionObject createWall(std::string id, geometry_msgs::Pose pose, double dim1, double dim2, double dim3, bool addAsCollisionObject = true);
			
			moveit_msgs::CollisionObject createBasket(std::string id, geometry_msgs::Pose pose, double dim1, double dim2, double dim3, bool addAsCollisionObject = true);

			void attachObject(std::string id);
			
			void detachObject();
			
			bool hasObjectAttached();
			
			moveit_msgs::CollisionObject getAttachedObject();

			std::vector<double> setEndEffectorOrientation(double angle, bool execute = false);

			std::vector<double> getEndEffectorRPY();

			geometry_msgs::Pose getEndEffectorPose();

			void setGripperState(robotiq_s_model_control::SModel_robot_output gripper_state);

			void openGripper();

			void closeGripper();

        private:

			void attachObject(moveit_msgs::CollisionObject co);

			bool writeToSerial(const unsigned char cmd[], const unsigned int cmd_size);
			
			ros::NodeHandle m_nh;
			bool m_simulated;
			std::vector<moveit_msgs::CollisionObject> m_collision_objects;
			int m_attached_object_index = -1;
			const std::string m_move_group_name;
			moveit::planning_interface::MoveGroupInterface m_move_group;
			moveit::planning_interface::PlanningSceneInterface m_planning_scene_interface;
			const robot_state::JointModelGroup *m_joint_model_group;
			moveit::core::RobotStatePtr m_current_robot_state;
			bool m_robotiq_hand;

    };
};

#endif
