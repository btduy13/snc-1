#include "fetch_purepursuit/fetch_purepursuit.h"

const double FetchPurePursuit::MAX_LINEAR_VEL = 0.22;
const double FetchPurePursuit::MAX_ANGULAR_VEL = 2.5;

void FetchPurePursuit::odomCallback(const nav_msgs::OdometryConstPtr &odom_message){
    raw_current_pose_ = (*odom_message).pose.pose;
    current_received_ = true;
}

void FetchPurePursuit::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_message){
    raw_target_pose_ = (*pose_message).pose;
    target_received_ = true;
}

void FetchPurePursuit::turtleBotOdomCallback(const nav_msgs::OdometryConstPtr &odom_message){
    raw_turtlebot_pose_ = (*odom_message).pose.pose;
    turtlebot_received_ = true;
}
void FetchPurePursuit::tfCallback(const tf2_msgs::TFMessageConstPtr &tf_message){
    transform_ = *tf_message;
   
    for(int i = 0;i<transform_.transforms.size();i++){
      
    }
    tf_received_ = true;
}

void FetchPurePursuit::stateCallback(const std_msgs::Int8ConstPtr &state_message){
    current_marker_state_ = *state_message;
    // std::cout<<"State: "<<current_marker_state_.data<<std::endl;
}

void FetchPurePursuit::planVelocity()
{
    // while(true)
    // // {
        if (current_received_ == true && target_received_ == true && turtlebot_received_ == true){
           

            if (current_marker_state_.data == 3)            {
                std::cout<<"Yes"<<std::endl;
                current_pose_.point.x = 0;
                current_pose_.point.y = 0;
                current_pose_.orientation = 0;

                transformed_target_pose_.point.x = raw_target_pose_.position.z + 0.5;
                transformed_target_pose_.point.y = -raw_target_pose_.position.x;
                transformed_target_pose_.orientation = tf::getYaw(raw_target_pose_.orientation);
            }
            else            {
                std::cout<<"No"<<std::endl;
                pose turtlebot_pose;

                turtlebot_pose.point.x = raw_turtlebot_pose_.position.x;
                turtlebot_pose.point.y = raw_turtlebot_pose_.position.y;
                turtlebot_pose.orientation = tf::getYaw(raw_turtlebot_pose_.orientation);

                current_pose_.point.x = raw_current_pose_.position.x;
                current_pose_.point.y = raw_current_pose_.position.y;
                current_pose_.orientation = tf::getYaw(raw_current_pose_.orientation);

                transformed_target_pose_ = computeTransformedPose(current_pose_,turtlebot_pose.point);


            }

            double angular = computeAngular();
            double linear =  computeLinear();

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

            setDestinateAngularVel(angular);
            setDestinateLinearVel(linear);

            current_velocity_.linear.x = linear;
            current_velocity_.angular.z = angular;

            cmd_vel_pubisher_.publish(current_velocity_);
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

    cmd_vel_pubisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    odom_subscriber_ = nh.subscribe("/odom", 100, &FetchPurePursuit::odomCallback, this);
    target_pose_subsciber_ = nh.subscribe("/visp_auto_tracker/object_position", 1, &FetchPurePursuit::targetPoseCallback, this);
    turtlebot_pose_subscriber_ = nh.subscribe("/tb3_0/odom", 100, &FetchPurePursuit::turtleBotOdomCallback, this);
    tf_subcriber_ = nh.subscribe("/tf",100,&FetchPurePursuit::tfCallback, this);
    marker_state_subcriber_ = nh.subscribe("/visp_auto_tracker/status",100,&FetchPurePursuit::stateCallback, this);
    // plan_velocity_thread_ = new std::thread(&FetchPurePursuit::planVelocity,this);
}

FetchPurePursuit::~FetchPurePursuit(){

}
