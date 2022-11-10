/*
	EE3305/ME3243
	Name: Chen Guoyi (guoyi@comp.nus.edu.sg)
	Matric number: A0262311W
*/

#include "botcontrol.hpp"
#include <ros/ros.h>

using namespace botcontrol;

BotControl::BotControl(ros::NodeHandle& nh) : nodehandle_(nh){

	//load the param
	if(!loadParam()){
		ROS_ERROR("Error in loading the parameters.");
		// ros::requestShutdown();
	}

	// declare all the subscriber and publisher
	true_sub_ = nodehandle_.subscribe("/gazebo/link_states", 1, &BotControl::trueCallBack, this);
	
	vel_pub_ = nodehandle_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 200);
	error_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_forward", 1); 
	error_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_angle", 1);
	control_signal_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_forward", 1);
	control_signal_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_angle", 1);

	//initialize variables
	error_forward_ = 0;
	error_angle_ = 0;
	error_forward_prev_ = 0;
	error_angle_prev_ = 0;
	I_forward_ = 0;
	I_angle_ = 0;
	D_forward_ = 0;
	D_angle_ = 0;

	ROS_INFO("Node Initialized");
}

BotControl::~BotControl(){}

void BotControl::trueCallBack(const gazebo_msgs::LinkStates& msg){
    // get index
    int i = 0;
    for (; i<msg.name.size(); ++i)
        if (msg.name[i] == "husky::base_link")
            break;
    if (i >= msg.name.size())
        return; // does not exist
        
    geometry_msgs::Pose pose = msg.pose[i];
    pos_x_ = pose.position.x;
    pos_y_ = pose.position.y;
    double qz = pose.orientation.z;
    double qx = pose.orientation.x;
    double qy = pose.orientation.y;
    double qw = pose.orientation.w;
	double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    ang_z_ = atan2(siny_cosp, cosy_cosp);
}
void BotControl::pidAlgorithm(){
	std_msgs::Float32 linear_error;
	std_msgs::Float32 angle_error;
	std_msgs::Float32 linear_velocity; // command
	std_msgs::Float32 angle_velocity; // command

    double Dx = pillar_x - pos_x_; // pos_x_ from odom (true sim. position)
    double Dy = pillar_y - pos_y_; // pos_y_ from odom (true sim. position)
    
	// update the pid status
	error_forward_prev_ = error_forward_;
	error_angle_prev_ = error_angle_;
	
	error_forward_ = sqrt(Dx*Dx + Dy*Dy) - target_distance;
	error_angle_ = atan2(Dy, Dx) - ang_z_;
    
	// regularize the error_angle_ within [-PI, PI]
	if(error_angle_ < -PI) error_angle_ += 2*PI;
	if(error_angle_ > PI) error_angle_ -= 2*PI;

	// integral term
	I_forward_ += dt * error_forward_;
	I_angle_ += dt * error_angle_;

	// derivative term
	D_forward_ = (-error_forward_prev_ + error_forward_) / dt;
	D_angle_ = (-error_angle_prev_ + error_angle_) / dt;

	// ENTER YOUR CODE HERE

	//motoring contral
	trans_forward_ = error_forward_ * Kp_f + I_forward_ * Ki_f + D_forward_ * Kd_f;


	//steering control
	trans_angle_ = error_angle_ * Kp_a + I_angle_ * Ki_a + D_angle_ * Kd_a;

	// END OF YOUR CODE HERE


	ROS_INFO("1----Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Linear_error: %f",  
		trans_forward_, trans_angle_, error_angle_, error_forward_);


	//publish all
	vel_cmd_.linear.x = trans_forward_;
	vel_cmd_.angular.z = trans_angle_; //euler angle
	vel_pub_.publish(vel_cmd_);

	linear_error.data = error_forward_;
	error_forward_pub_.publish(linear_error);

	linear_velocity.data = trans_forward_;
	control_signal_angle_pub_.publish(linear_velocity);

	angle_error.data = error_angle_;
	error_angle_pub_.publish(angle_error);

	angle_velocity.data = trans_angle_;
	control_signal_angle_pub_.publish(angle_velocity);

}

void BotControl::spin(){
	ros::Rate loop_rate(1/dt);

    //# sleep at start to wait for windows to load
	ros::Rate init_rate(1);
    for (int i=3; i>0; --i) {
        ROS_INFO("%d", i);
        init_rate.sleep();
    } // sleep for # seconds where i=# above
	
	while(ros::ok()){
		ros::spinOnce();
        pidAlgorithm(); 
		loop_rate.sleep();
	}

}

bool BotControl::loadParam(){


	if(!nodehandle_.getParam("/Kd_a", Kd_a)){
		ROS_ERROR("Kd_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kp_f", Kp_f)){
		ROS_ERROR("Kp_f Load Error");
		return false;
	}

	if(!nodehandle_.getParam("/Ki_f", Ki_f)){
		ROS_ERROR("Ki_f Load Error");
		return false;
	}

	if(!nodehandle_.getParam("/Kd_f", Kd_f)){
		ROS_ERROR("Kd_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kp_a", Kp_a)){
		ROS_ERROR("Kp_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Ki_a", Ki_a)){
		ROS_ERROR("Ki_a Load Error");
		return false;
	}

	if(!nodehandle_.getParam("/target_angle", target_angle)){
		ROS_ERROR("target_angle Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/target_distance", target_distance)){
		ROS_ERROR("target_distance Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pillar_x", pillar_x)){ //#
		ROS_ERROR("pillar_x Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/pillar_y", pillar_y)){ //#
		ROS_ERROR("pillar_y Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/dt", dt)){
		ROS_ERROR("dt Load Error");
		return false;
	}

	return true;

}
