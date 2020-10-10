#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H
#include<cmath>
#include <iostream>
#include <vector>
using namespace std;
struct Point
{
    double x;
    double y;
};
struct pose
{
    Point point;
    double orientation;
};
struct Velocity
{
    double linear;
    double angular;
};
struct Pcontroller
{
    double linear;
    double angular;
};
class PurePursuit
{
protected:
    double linear_vel_;
    double angular_vel_;
    double turn_limit_;
    pose current_pose_;
    pose target_pose_;
    Pcontroller controller_;
    Velocity velocity_;
    Velocity target_vel_;
    bool state_;
    const double TURN_LIMIT = M_PI/12;
    const double LINEAR_CONTROLLER = 2;
    const double ANGULAR_CONTROLLER =5;
    const double TIME_CONSTRAINT =0.5;
    double arc_;
    pose transformed_target_pose_;
    
public:
    PurePursuit();
    ~PurePursuit(){};
    
    double getLinearVel()    {        
        return linear_vel_;    }
    
    double getAngularVel()    {        
        return angular_vel_;    }
    
    double getTurnLimit() {   
        return turn_limit_;    }
    
    void setTurnLimit(double angle){
        turn_limit_ = angle;    }
    
    void setDestinateLinearVel(double linear_vel){
        linear_vel_ = linear_vel;    }
    
    void setDestinateAngularVel(double angular_vel){
        angular_vel_ = angular_vel;    }
    
    void CurrentPoseSet(pose pose){
        current_pose_ =pose;    }
    
    void TargetPoseSet(pose pose);
    
    pose getTargetPose(){
        return target_pose_;    }
    
    void LinearPController (double p_controller)
    {
        controller_.linear = p_controller;
    }
    
    void AngularPController (double p_controller)
    {
        controller_.angular = p_controller;
    }
    
    void TargetVel(Velocity vel) //set target velocity
    {
        target_vel_ = vel;
    }

    void AimheadSet (bool state)
    {
        state_ =state;
    }

    double computeArc (Point point)
    {
        double l = computeDistance(point);
        double y = point.y;
        return (2*y)/(l*l);
    }

    double computeDistance (Point point)
    {
        return sqrt(point.x*point.x + point.y*point.y);
    }

    double computeLinear ();

    double computeAngular ();

    void PurePurSuitStart();

    pose computeTransformedPose(pose pose, Point point);

    void getInverseMatrix (double mat[3][3],double (&inverse_mat)[3][3]);

    void TransformedPoseGet(double transformMatrix[3][3], double origin_pose[3][1],double (&new_pose)[3][1]);
    
};


#endif