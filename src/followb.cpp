#include <boost/bind.hpp>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

ros::Subscriber g_sub_pose;
turtlesim::Pose g_turtlesim_pose;

#define PI 3.141592
float UPDATE_RATE = 1.016; //0.016 = 60 fps
float ROT_SPEED = 3;
double NEW_ANGLE = 0;
bool bClockwise = false;

void drawSpiral(ros::Publisher &g_pub_velocity) {
	geometry_msgs::Twist vel_msg;

	if ((g_turtlesim_pose.x < 10) && (g_turtlesim_pose.y < 10)) { // just to make beautiful spirals

		NEW_ANGLE += 0.5;

		vel_msg.linear.x = NEW_ANGLE;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;

		std::cout << vel_msg.linear.x << std::endl;

		if (bClockwise)
			vel_msg.angular.z = -std::abs(ROT_SPEED);
		else
			vel_msg.angular.z = std::abs(ROT_SPEED);

		g_pub_velocity.publish(vel_msg);
	} else {
		vel_msg.linear.x = 0;
		g_pub_velocity.publish(vel_msg);
	}

}

void timerCallback(const ros::TimerEvent&, ros::Publisher twist_pub) {
	drawSpiral(twist_pub);
}

/* Get current position of the turtle*/

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message) {
	g_turtlesim_pose.x = pose_message->x;
	g_turtlesim_pose.y = pose_message->y;
}

/*
 * ROS uses SI units, specifically MKS.
 * That means for angular rate the default is radians/second.
 *
*/
double degrees2radians(double angle_in_degrees) {
	return angle_in_degrees * PI / 180.0;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "draw_spiral");

	ros::NodeHandle nh;

	g_sub_pose = nh.subscribe("turtle1/pose", 1, poseCallback);
	ros::Publisher g_pub_velocity = nh.advertise<geometry_msgs::Twist>(
			"turtle1/cmd_vel", 1);
	ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("reset");

	double angle = 0;
	std::cout << "Choose your angle in degrees, e.g. 10 (double) :" << std::endl;
	std::cin >> angle;

	bool bClockwise;
	std::cout << "Clockwise (1 = True and 0 = False) ? \n";
	while (!(std::cin >> bClockwise)) {
		std::cin.clear();
		std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}

	NEW_ANGLE = degrees2radians(angle);

	std::cout << "Start drawing ..." << std::endl;
	ros::Timer timer = nh.createTimer(ros::Duration(UPDATE_RATE),
			boost::bind(timerCallback, _1, g_pub_velocity));

	std_srvs::Empty empty;
	reset.call(empty);

	ros::spin();
}
