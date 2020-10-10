#include "purepursuit.h"
PurePursuit::PurePursuit()
{
    turn_limit_ = TURN_LIMIT;
    target_vel_.angular =0;
    target_vel_.linear =0;
    controller_.angular  =LINEAR_CONTROLLER;
    controller_.linear =ANGULAR_CONTROLLER;
    state_ = false;
}
void PurePursuit::TargetPoseSet(pose pose)
{
    if (state_ == true)
    {
        double alpha = pose.orientation + target_vel_.angular * TIME_CONSTRAINT;
        double x = pose.point.x + target_vel_.linear *cos(alpha)*TIME_CONSTRAINT;
        double y = pose.point.y + target_vel_.linear *sin(alpha)*TIME_CONSTRAINT;
        target_pose_.orientation = alpha;
        target_pose_.point.x =x;
        target_pose_.point.y =y;
    }
    else
    {
        target_pose_ = pose;
    }
}

double PurePursuit::computeLinear()
{
    double x1 =0;
    double y1 =0;
    // double alpha = M_PI/2;
    double x2 = transformed_target_pose_.point.x;
    double y2 = transformed_target_pose_.point.y;
    double alpha = transformed_target_pose_.orientation;
    double distance_compute = computeDistance(transformed_target_pose_.point);
    distance_compute -= 0.6;
    if (distance_compute < 0)
    {
        distance_compute =0;
    }
    linear_vel_ = controller_.linear *distance_compute;
    return linear_vel_;
}

double PurePursuit::computeAngular()
{
    double x1=0;
    double y1=0;
    double alpha =0;
    double x2 = transformed_target_pose_.point.x;
    double y2 = transformed_target_pose_.point.y;
    double alpha_1 = atan2(y2-y1,x2-x1) - alpha;
    arc_ = computeArc (transformed_target_pose_.point);
    angular_vel_= controller_.angular *alpha_1;
    return angular_vel_;
}
void PurePursuit::PurePurSuitStart()
{
    transformed_target_pose_ = computeTransformedPose(current_pose_,target_pose_.point);
    double angular = computeAngular();
    double linear  = computeLinear();
    setDestinateAngularVel(angular);
    setDestinateLinearVel(linear);
}
pose PurePursuit::computeTransformedPose(pose posee, Point point)
{
    double x = posee.point.x;
    double y = posee.point.y;
    double alpha = posee.orientation;

    double xp= point.x;
    double yp = point.y;

    double transformed_mat[3][3] = {{cos(alpha),-sin(alpha),x},{sin(alpha),cos(alpha),y},{0,0,1}};
    double inverse_transform[3][3];
    double origin_vector[3][1] = {{xp},{yp},{1}};
    double new_vector[3][1];
    getInverseMatrix(transformed_mat,inverse_transform);
    TransformedPoseGet(inverse_transform,origin_vector,new_vector);
    pose transformed_pose;
    transformed_pose.point.x  =new_vector[0][0];
    transformed_pose.point.y  =new_vector[1][0];
    transformed_pose.orientation =0;
    return transformed_pose;
}

void PurePursuit::getInverseMatrix(double mat[3][3], double (&inverse_mat)[3][3])
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

void PurePursuit::TransformedPoseGet(double transformMatrix[3][3], double origin_pose[3][1],double (&new_pose)[3][1])
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
