#include "fetch_purepursuit/fetch_purepursuit.h"

const double FetchPurePursuit::MAX_LINEAR_VEL = 0.22;
const double FetchPurePursuit::MAX_ANGULAR_VEL = 2.5;

void FetchPurePursuit::odomCallback(const nav_msgs::OdometryConstPtr &odom_message)
{
    raw_current_pose_ = (*odom_message).pose.pose;
    current_received_ = true;
}

void FetchPurePursuit::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_message)
{
    raw_target_pose_ = (*pose_message).pose;
    target_received_ = true;
}

void FetchPurePursuit::turtleBotOdomCallback(const nav_msgs::OdometryConstPtr &odom_message)
{
    raw_turtle_pose_ = (*odom_message).pose.pose;
    turtlebot_received_ = true;
}
void FetchPurePursuit::tfCallback(const tf2_msgs::TFMessageConstPtr &tf_message)
{
    current_tf_ = *tf_message;
    // std::cout<<"Size: "<<current_tf_.transforms.size()<<std::endl;
    for(int i = 0;i<current_tf_.transforms.size();i++)
    {
        // std::cout<<"FrameID:"<<current_tf_.transforms.at(i).header.frame_id<<std::endl;
        // std::cout<<"ChildframeID:"<<current_tf_.transforms.at(i).child_frame_id<<std::endl;
    }
    tf_received_ = true;
}

void FetchPurePursuit::stateCallback(const std_msgs::Int8ConstPtr &state_message)
{
    current_marker_state_ = *state_message;
    // std::cout<<"State: "<<current_marker_state_.data<<std::endl;
}

void FetchPurePursuit::planVelocity()
{
    // while(true)
    // // {
        if (current_received_ == true && target_received_ == true && turtlebot_received_ == true)
        {
            // current_pose_.position.x = raw_current_pose_.position.x;
            // current_pose_.position.y = raw_current_pose_.position.y;
            // current_pose_.orientation = tf::getYaw(raw_current_pose_.orientation);

            // std::cout<<"State: "<<current_marker_state_.data<<std::endl;

            if (current_marker_state_.data == 3)
            {
                std::cout<<"Yes"<<std::endl;
                current_pose_.position.x = 0;
                current_pose_.position.y = 0;
                current_pose_.orientation = 0;

                transformed_desired_pose_.position.x = raw_target_pose_.position.z + 0.5;
                transformed_desired_pose_.position.y = -raw_target_pose_.position.x;
                transformed_desired_pose_.orientation = tf::getYaw(raw_target_pose_.orientation);
            }
            else
            {
                std::cout<<"No"<<std::endl;
                Pose2d turtle_pose;

                turtle_pose.position.x = raw_turtle_pose_.position.x;
                turtle_pose.position.y = raw_turtle_pose_.position.y;
                turtle_pose.orientation = tf::getYaw(raw_turtle_pose_.orientation);

                current_pose_.position.x = raw_current_pose_.position.x;
                current_pose_.position.y = raw_current_pose_.position.y;
                current_pose_.orientation = tf::getYaw(raw_current_pose_.orientation);

                // transformed_desired_pose_.position.x = raw_turtle_pose_.position.x;
                // transformed_desired_pose_.position.y = raw_turtle_pose_.position.y;
                // transformed_desired_pose_.orientation = tf::getYaw(raw_turtle_pose_.orientation);

                transformed_desired_pose_ = calculateTransformedPose(current_pose_,turtle_pose.position);
                // transformed_desired_pose_.position.y = -transformed_desired_pose_.position.y;

            }

            double angular = calculateAngularVelocity();
            double linear = calculateLinearVelocity();

            if (linear > MAX_LINEAR_VEL)
            {
                linear = MAX_LINEAR_VEL;
            }

            if (fabs(angular) > MAX_ANGULAR_VEL)
            {
                double check = angular;
                angular = (check/fabs(check))*MAX_ANGULAR_VEL;
            }
            std::cout<<"Linear: "<<linear<<std::endl;
            std::cout<<"Angular: "<<angular<<std::endl;
            std::cout<<"-------"<<std::endl;

            setDesiredAngularVelocity(angular);
            setDesiredLinearVelocity(linear);

            current_velocity_.linear.x = linear;
            current_velocity_.angular.z = angular;

            cmd_vel_pub_.publish(current_velocity_);
        }

}

void FetchPurePursuit::operate()
{
    planVelocity();
}

FetchPurePursuit::FetchPurePursuit(ros::NodeHandle &nh)
    : nh_(nh)
    , rate_(100)
{
    current_received_ = false;
    target_received_ = false;
    turtlebot_received_ = false;
    tf_received_ = false;
    current_marker_state_.data = 0;

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    odom_sub_ = nh.subscribe("/odom", 100, &FetchPurePursuit::odomCallback, this);
    target_pose_sub_ = nh.subscribe("/visp_auto_tracker/object_position", 1, &FetchPurePursuit::targetPoseCallback, this);
    turtle_pose_sub_ = nh.subscribe("/tb3_0/odom", 100, &FetchPurePursuit::turtleBotOdomCallback, this);
    tf_sub_ = nh.subscribe("/tf",100,&FetchPurePursuit::tfCallback, this);
    marker_state_sub_ = nh.subscribe("/visp_auto_tracker/status",100,&FetchPurePursuit::stateCallback, this);
    // plan_velocity_thread_ = new std::thread(&FetchPurePursuit::planVelocity,this);
}

FetchPurePursuit::~FetchPurePursuit()
{

}
