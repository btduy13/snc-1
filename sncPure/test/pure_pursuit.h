#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <cmath>
#include <iostream>
#include "purepursuitinterface.h"

/**
 * @class PurePursuitController 
 * @brief PurePursuitController sets the characteristics required for a typical pure pursuit controller that can be applied to multiple scenario
 * @details This class is inheritted from the base PurePursuitInterface. It also has the following propertises:
 * + The sharp turn angle can be adjusted to increase the performance of the controller
 * + The linear velocity to pursuit the target is decided by taking the distance between the current pose of the pursuitter and the target pose times a p controller.
 * + The angular velocity to pursuit the target is decided by:
 *  - If the angle from the current orientation of the pursuitter to the target pose is greater than a threshold, the angular velocity will be that angle times another p controller. If not, it will be the arc from the current pose to the target pose times the linear velocity.
 * + The target pose is the pose in the coordinate frame of the pursuitter. 2D homogenous transformation is applied to transform the point from the world coordinate frame to the local coordinate frame.
 * @note In order for pure pursuit to work, the pose of the target must be within the coordinate frame of the pursuitter. Therefore, in this scenario, a 2D homogenous transformation is needed to convert from the world coordinate frame to the local coordinate frame of the pursuitter.
 */
class PurePursuitController : public PurePursuitInterface
{
    public:
        PurePursuitController();
        ~PurePursuitController();
        const double getLinearVelocity(void);
        const double getAngularVelocity(void);
        const double getTurnThreshold(void);
        Pose2d getTargetPose();
        void setCurrentPose(Pose2d current_pose);
        void setTargetPose(Pose2d target_pose);
        void setSharpTurnThreshold(double angle);
        void setTargetVelocity(double linear_vel, double angular_vel);
        void setLinearPController(double p_controller_);
        void setAngularPController(double p_controller_);
        void setAimAhead(bool aim_ahead);
        void pursuit();
    protected:
        static const double DEFAULT_TURN_THRESHOLD; //ok
        static const double DEFAULT_LINEAR_P_CONTROLLER;  //ok 
        static const double DEFAULT_ANGULAR_P_CONTROLLER; //ok
        static const double TIME_CONSTRAINT;    //ok
        double turn_threshold_;  //ok
        double linear_velocity_;  //ok
        double angular_velocity_;  //ok
        double arc_;  //ok
        double linear_p_controller_;  //ok
        double angular_p_controller_;  //ok
        double target_linear_;     //ok
        double target_angular_;     //ok
        Pose2d current_pose_;    //ok
        Pose2d desired_pose_;    //ok 
        Pose2d transformed_desired_pose_;
        bool aim_ahead_;
        void setDesiredLinearVelocity(double linear_velocity);     
        void setDesiredAngularVelocity(double angular_velocity);
        double calculateArc(GlobalOrd point);
        double calculateDistance(GlobalOrd point);
        double calculateAngularVelocity();
        double calculateLinearVelocity();
        void getInverseMatrix(double matrix[3][3], double (&inverse_mat)[3][3]);
        void getTransformedPose(double transformMatrix[3][3], double origin_pose[3][1],double (&new_pose)[3][1]);
        Pose2d calculateTransformedPose(Pose2d pose, GlobalOrd point);
        
};
#endif