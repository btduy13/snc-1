#include "fetch_purepursuit/pure_pursuit.h"

const double PurePursuitController::DEFAULT_TURN_THRESHOLD = M_PI/12;
const double PurePursuitController::DEFAULT_LINEAR_P_CONTROLLER = 2;
const double PurePursuitController::DEFAULT_ANGULAR_P_CONTROLLER = 5;
const double PurePursuitController::TIME_CONSTRAINT = 0.5;

const double PurePursuitController::getLinearVelocity(void)
{
    return linear_velocity_;
}

const double PurePursuitController::getAngularVelocity(void)
{
    return angular_velocity_;
}

const double PurePursuitController::getTurnThreshold(void)
{
    return turn_threshold_;
}

void PurePursuitController::setSharpTurnThreshold(double angle)
{
    turn_threshold_ = angle;
}

void PurePursuitController::setDesiredLinearVelocity(double linear_velocity)
{
    linear_velocity_ = linear_velocity;
}

void PurePursuitController::setDesiredAngularVelocity(double angular_velocity)
{
    angular_velocity_ = angular_velocity;
}

void PurePursuitController::setCurrentPose(Pose2d current_pose)
{
    current_pose_ = current_pose;
}

void PurePursuitController::setTargetPose(Pose2d target_pose)
{
    if (aim_ahead_)
    {
        double theta = target_pose.orientation + target_angular_*TIME_CONSTRAINT;
        double x = target_pose.position.x + target_linear_*cos(theta)*TIME_CONSTRAINT;
        double y = target_pose.position.y + target_linear_*sin(theta)*TIME_CONSTRAINT;

        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.orientation = theta;

        desired_pose_ = target_pose;
    }
    else desired_pose_ = target_pose;
    
}

Pose2d PurePursuitController::getTargetPose()
{
    return desired_pose_;
}

void PurePursuitController::setLinearPController(double p_controller)
{
    linear_p_controller_ = p_controller;
}

void PurePursuitController::setAngularPController(double p_controller)
{
    angular_p_controller_ = p_controller;
}

void PurePursuitController::setTargetVelocity(double linear_vel, double angular_vel)
{
    target_angular_ = angular_vel;
    target_linear_ = linear_vel;
}

void PurePursuitController::setAimAhead(bool aim_ahead)
{
    aim_ahead_ = aim_ahead;
}

double PurePursuitController::calculateArc(GlobalOrd point)
{
    double l = calculateDistance(point);
    double y = point.y;
    return (2*y)/(l*l);
}

double PurePursuitController::calculateDistance(GlobalOrd point)
{
    double x = point.x;
    double y = point.y;

    return sqrt(x*x + y*y);
}

double PurePursuitController::calculateLinearVelocity()
{
    double x1 = 0;
    double y1 = 0;
    double theta1 = M_PI/2;
    double x2 = transformed_desired_pose_.position.x;
    double y2 = transformed_desired_pose_.position.y;
    double theta2 = transformed_desired_pose_.orientation;

    double distance = calculateDistance(transformed_desired_pose_.position);
    
    distance = distance - 0.6;

    if (distance < 0)
    {

        distance = 0;
    }

    linear_velocity_ = linear_p_controller_ * distance;
    return linear_velocity_;
}

double PurePursuitController::calculateAngularVelocity()
{
    double x1 = 0;
    double y1 = 0;
    double theta1 = 0;
    double x2 = transformed_desired_pose_.position.x;
    double y2 = transformed_desired_pose_.position.y;

    double alpha = atan2(y2-y1,x2-x1) - theta1;
    arc_ = calculateArc(transformed_desired_pose_.position);
    
    // if (fabs(alpha) > turn_threshold_)
    // {
    //     angular_velocity_ = angular_p_controller_*alpha;
    // }
    // else angular_velocity_ = linear_velocity_*arc_;

    angular_velocity_ = angular_p_controller_*alpha;

    return angular_velocity_;
}

void PurePursuitController::pursuit()
{
    transformed_desired_pose_ = calculateTransformedPose(current_pose_,desired_pose_.position);

    double angular = calculateAngularVelocity();
    double linear = calculateLinearVelocity();

    setDesiredAngularVelocity(angular);
    setDesiredLinearVelocity(linear);
}

PurePursuitController::PurePursuitController()
{
    turn_threshold_ = DEFAULT_TURN_THRESHOLD;
    target_angular_ = 0;
    target_linear_ = 0;

    linear_p_controller_ = DEFAULT_LINEAR_P_CONTROLLER;
    angular_p_controller_ = DEFAULT_ANGULAR_P_CONTROLLER;
    aim_ahead_ = false;
}


PurePursuitController::~PurePursuitController()
{

}

Pose2d PurePursuitController::calculateTransformedPose(Pose2d pose, GlobalOrd point)
{
    double x = pose.position.x;
    double y = pose.position.y;
    double theta = pose.orientation;

    double xp = point.x;
    double yp = point.y;

    double transform_matrix[3][3] = {{cos(theta),-sin(theta),x},{sin(theta),cos(theta),y},{0,0,1}};
    double inverse_transform[3][3];
    double origin_vector[3][1] = {{xp},{yp},{1}};
    double new_vector[3][1];

    getInverseMatrix(transform_matrix,inverse_transform);
    getTransformedPose(inverse_transform,origin_vector,new_vector);

    Pose2d transformed_pose;
    transformed_pose.position.x = new_vector[0][0];
    transformed_pose.position.y = new_vector[1][0];
    transformed_pose.orientation = 0;

    return transformed_pose;
}

void PurePursuitController::getInverseMatrix(double mat[3][3], double (&inverse_mat)[3][3])
{
    double det = 0;
    int i,j;
    
    for(i = 0; i < 3; i++)
		det = det + (mat[0][i] * (mat[1][(i+1)%3] * mat[2][(i+2)%3] - mat[1][(i+2)%3] * mat[2][(i+1)%3]));
	
    for(i = 0; i < 3; i++)
    {
		for(j = 0; j < 3; j++)
        {
			inverse_mat[i][j] = ((mat[(j+1)%3][(i+1)%3] * mat[(j+2)%3][(i+2)%3]) - (mat[(j+1)%3][(i+2)%3] * mat[(j+2)%3][(i+1)%3]))/ det;
        }
	}
}

void PurePursuitController::getTransformedPose(double transformMatrix[3][3], double origin_pose[3][1],double (&new_pose)[3][1])
{
    for(int i=0; i<3; i++)
    {
        new_pose[i][0]= 0.0;
        for(int j=0; j<3; j++)
        {
            new_pose[i][0] += (transformMatrix[i][j] * origin_pose[j][0]);
        }
    }
}

