
# include "followb.h"



FollowB::FollowB(int argc,char **argv)
{
	if(argc != 3)
	{
		ROS_ERROR(
		"Usage : stdr_obstacle avoidance <robot_frame_id> <laser_frame_id>");
		exit(0);
	}
	laser_topic_ = std::string("/")  +  std::string(argv[1]) + std::string("/") + std::string(argv[2]);
	speeds_topic_ = std::string("/") + std::string(argv[1]) + std::string("/cmd_vel");
		
	subscriber_ = n_.subscribe( laser_topic_.c_str(), 1, &FollowB::callback,this);
		
	cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>(speeds_topic_.c_str(), 1);
}


FollowB::~FollowB(void)
{

}


void FollowB::callback(const sensor_msgs::LaserScan& msg)
{

	scan_ = msg;

	std::cout << "LaserScan angle(min.): " << scan_.angle_min  << " LaserScan angle(max.): " << scan_.angle_min << " angle (inc.) " << scan_.angle_increment << std::endl;

	float linear = 0, rotational = 0;
	for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
	{
		float real_dist = scan_.ranges[i];
		linear -= cos(scan_.angle_min + i * scan_.angle_increment) / (1.0 + real_dist * real_dist);
		rotational -= sin(scan_.angle_min + i * scan_.angle_increment) 	/ (1.0 + real_dist * real_dist);
	}

	geometry_msgs::Twist cmd;

	linear /= scan_.ranges.size();
	rotational /= scan_.ranges.size();

	//~ ROS_ERROR("%f %f",linear,rotational);

	if(linear > 0.3)
	{
		linear = 0.3;
	}
	else if(linear < -0.3)
	{
		linear = -0.3;
	}

	cmd.linear.x = 0.3 + linear;
	cmd.angular.z = rotational;
	
	std::cout << " Twist linear (x): "  << cmd.linear.x << " linear (y) : "   << cmd.linear.y  << " linear (z): "  << cmd.linear.z << std::endl;
	std::cout << " Twist angular (x): " << cmd.angular.x << " angular (y) : " << cmd.angular.y << " angular (z): " << cmd.angular.z << std::endl;

	cmd_vel_pub_.publish(cmd);
}
