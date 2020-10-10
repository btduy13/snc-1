#ifndef FETCH_PUREPURSUIT_H_
#define FETCH_PUREPURSUIT_H_

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <thread>
#include <atomic>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <iostream>
#include "pure_pursuit.h"


class FetchPurePursuit : public PurePursuit
{
    public:

        static const double MAX_LINEAR_VEL;
        static const double MAX_ANGULAR_VEL;

        FetchPurePursuit(ros::NodeHandle &nh);
        ~FetchPurePursuit();

        void operate();

    private:

        ros::NodeHandle nh_;
        ros::Rate rate_;

        ros::Publisher cmd_vel_pubisher_;
        ros::Subscriber odom_subscriber_;
        ros::Subscriber target_pose_subsciber_;
        ros::Subscriber turtlebot_pose_subscriber_;

        ros::Subscriber tf_subcriber_;

        ros::Subscriber marker_state_subcriber_;

        geometry_msgs::Pose raw_current_pose_;
        geometry_msgs::Pose raw_target_pose_;
        geometry_msgs::Pose raw_turtlebot_pose_;
        
        geometry_msgs::Twist current_velocity_;

        tf2_msgs::TFMessage transform_;

        std_msgs::Int8 current_marker_state_;

        // std::thread *plan_velocity_thread_;

        std::atomic<bool> current_received_;
        std::atomic<bool> target_received_;
        std::atomic<bool> turtlebot_received_;
        std::atomic<bool> tf_received_;

        void odomCallback(const nav_msgs::OdometryConstPtr &odom_message);
        void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_message);
        void turtleBotOdomCallback(const nav_msgs::OdometryConstPtr &odom_message);
        void tfCallback(const tf2_msgs::TFMessageConstPtr &tf_message);
        void stateCallback(const std_msgs::Int8ConstPtr &state_message);
        
        void publishTwistCommand(geometry_msgs::Twist message);

        void planVelocity();
};

#endif
