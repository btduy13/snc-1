#ifndef PURE_PURSUIT_INTERFACE_H
#define PURE_PURSUIT_INTERFACE_H

#include <cmath>

struct GlobalOrd
{
    double x;
    double y;
};

struct Pose2d
{
    GlobalOrd position;
    double orientation;
};

/**
 * @class PurePursuitInterface
 * @brief PurePursuitInterface sets the bare minimum requirement for a typical pure pursuit controller that can be applied for different cases.
 * @details PurePursuitInterface has the following propertises:
 * + The controller takes in a current pose and a target pose with a target velocities (linear and angular) and calculate the linear and angular velocity required in order to pursuit the target.
 * + The controller can also aim head based on the current linear velocity and angular velocity of the target. This option can be turned on or off by setting the setAimAhead() member function.
 */
class PurePursuitInterface
{
    public:

        PurePursuitInterface();

        ~PurePursuitInterface();

        /**
         * @brief Member function that sets the current position of the pursuitter
         * 
         * @param pose: current pose of the pursuitter: x y coordinate and orientation
         */
        virtual void setCurrentPose(Pose2d pose) = 0;

        /**
         * @brief Member function that sets the pose of the target. If aim ahead is turned on, this function will recalculate the pose based on the velocities of the target.
         * 
         * @param pose: pose of the target: x y coordinate and orientation
         */
        virtual void setTargetPose(Pose2d pose) = 0;

        /**
         * @brief Member function that sets velocities of the target
         * 
         * @param linear_velocity 
         * @param angular_velocity 
         */
        virtual void setTargetVelocity(double linear_velocity, double angular_velocity) = 0;

        /**
         * @brief Member function that turns on or off aim ahead option
         * 
         * @param status: turn on or off by setting this value true or false respectively
         */
        virtual void setAimAhead(bool status) = 0;

        /**
         * @brief Member function that calculates the linear and angular velocity based on the given current and target pose and velocites
         * 
         */
        virtual void pursuit() = 0;
};

#endif