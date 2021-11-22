#include "followb.h"

FollowB::FollowB(int argc, char **argv)
{
	if (argc != 4)
	{
		ROS_ERROR(
			"Usage : followb <robot_frame_id> <sensor_type> <algorithm>");
		exit(0);
	}

	setupReactiveAlgorithm(std::string(argv[2]), std::string(argv[3]));
	setupSubscribers(std::string(argv[1]), std::string(argv[2]));
	setupPublisher(std::string(argv[1]));
}

FollowB::~FollowB(void)
{
}

// Check if the sensor support the algorithm
void FollowB::setupReactiveAlgorithm(std::string sensorType, std::string algo)
{
	std::cout << sensorType << " - " << algo << std::endl;
	if (sensorType == "sonar" && algo == "vwall")
	{
		std::cout << "Sonar doesn't support vwall algorithm." << std::endl;
		exit(0);
	}
	else if (algo != "vwall" && algo != "pwall")
	{
		std::cout << algo << std::endl;
		std::cout << "Algorithm not supported. Please, choose pwall or vwall as parameter." << std::endl;
		exit(0);
	}
	else
	{
		algorithm_ = algo;
		std::cout << "Start navigation with algorithm:" << algorithm_ << std::endl;
	}
}

/*
 Subscribe the data from the sensors and set a callback to reveive their data:
 laser_0 (is the laser)
 sonar_0 (sonar 1)
 sonar_1 (sonar 2)
 */
void FollowB::setupSubscribers(std::string robotId, std::string sensorType)
{
	if (sensorType == std::string("laser"))
	{
		std::string laserTopic_ = std::string("/") + robotId + std::string("/") + std::string(sensorType).append(std::string("_0"));
		subscriberSensor_ = nodeHandler_.subscribe(laserTopic_.c_str(), 1, &FollowB::laserCallback, this);
	}
	else if (sensorType == std::string("sonar"))
	{
		std::string sonarTopic_;

		sonarTopic_ = std::string("/") + robotId + std::string("/") + std::string(sensorType).append(std::string("_") + std::to_string(0));
		subscribeSonar0_ = nodeHandler_.subscribe(sonarTopic_.c_str(), 1, &FollowB::sonarCallback, this);

		sonarTopic_ = std::string("/") + robotId + std::string("/") + std::string(sensorType).append(std::string("_") + std::to_string(1));
		subscribeSonar1_ = nodeHandler_.subscribe(sonarTopic_.c_str(), 1, &FollowB::sonarCallback, this);
	}
	else
	{
		std::cout << "Sensor not recognized: " << sensorType << std::endl;
		exit(0);
	}
}

// Setup the the Publisher
void FollowB::setupPublisher(std::string robotId)
{
	std::string speedsTopic_ = std::string("/") + robotId + std::string("/cmd_vel");
	cmdVelPub_ = nodeHandler_.advertise<geometry_msgs::Twist>(speedsTopic_.c_str(), 1);
}

// Get the ray with the minimal distance to the obstacle and the angle
HitRay FollowB::getMinDistanceLaserHit(const sensor_msgs::LaserScan &msg)
{
	HitRay laserHit = {FLT_MAX, 0.0f};

	for (int i = 0; i < msg.ranges.size(); i++)
	{
		if (msg.ranges[i] < laserHit.distance)
		{
			laserHit.distance = msg.ranges[i];
			laserHit.angle = msg.angle_min + (msg.angle_increment * float(i));
			laserHit.index = i;
		}
	}
	return laserHit;
}

// Calculate the robot's heading using the 'parallel' wall follower technique 
void FollowB::parallelWallFollowing(const HitRay &hray, float targetDistance, float turningRate, float rangeMax)
{
	if (hray.distance <= rangeMax)
	{
		const float alpha = degrees2radians(90) - fabs(hray.angle);
		const float z = -turningRate * (sin(alpha) - (hray.distance - targetDistance)) * LINEAR_VEL;
		setRobotHeading(z);
	}
	else
	{
		setRobotHeading(0);
	}
}


// Calculate the robot's heading using the virtual right triangle wall follower technique  
void FollowB::virtualRightTriangleWallFollower(const sensor_msgs::LaserScan &msg)
{
	HitRay laserHitRay1 = getMinDistanceLaserHit(msg);
	HitRay laserHitRay2 = {FLT_MAX, 0.0f};

	int INC = ((laserHitRay1.index + 5) > msg.ranges.size() - 1) ? INC = -3 : INC = 3;

	laserHitRay2.distance = msg.ranges[laserHitRay1.index + INC];
	laserHitRay2.angle = msg.angle_min + (msg.angle_increment * float(laserHitRay1.index + INC));
	laserHitRay2.index = laserHitRay1.index + INC;

	const float x0 = convertPolarToCartesianX(laserHitRay1);
	const float y0 = convertPolarToCartesianY(laserHitRay1);

	const float x1 = convertPolarToCartesianX(laserHitRay2);
	const float y1 = convertPolarToCartesianY(laserHitRay2);

	const float alpha = atan2((y1 - DWALL), (x1 + WALL_LEAD - y0));

	setRobotHeading(alpha);
}

// Receive the messages from the laser sensors 
void FollowB::laserCallback(const sensor_msgs::LaserScan &msg)
{
	if (algorithm_ == std::string("pwall"))
	{
		parallelWallFollowing(getMinDistanceLaserHit(msg), TARGET_DIST_LASER, TURNING_RATE_LASER, msg.range_max);
	}
	else if (algorithm_ == std::string("vwall"))
	{
		virtualRightTriangleWallFollower(msg);
	}
}

//Receive the messages from the sonar sensors
void FollowB::sonarCallback(const sensor_msgs::Range::ConstPtr &msg)
{
	if (msg->range != std::numeric_limits<sensor_msgs::Range::_range_type>::infinity())
	{
		HitRay hray{msg->range, msg->field_of_view / 2, 0};
		parallelWallFollowing(hray, TARGET_DIST_SONAR, TURNING_RATE_SONAR, msg->max_range);
	}
}


// Compare two floats 
bool FollowB::compareFloat(double a, double b)
{
	return fabs(a - b) < EPSILON;
}

// Convert degress to radians 
float FollowB::degrees2radians(float angle_in_degrees)
{
	return angle_in_degrees * (M_PI / 180.0);
}

// Convert radians to degrees
float FollowB::radians2degrees(float angle_in_radians)
{
	return angle_in_radians * (180 / M_PI);
}

// Get x coordinate from an polar angle 
float FollowB::convertPolarToCartesianX(const HitRay &hit)
{
	return hit.distance * cos((hit.angle));
}

// Get y coordinate from an polar angle 
float FollowB::convertPolarToCartesianY(const HitRay &hit)
{
	return hit.distance * sin((hit.angle));
}

// Set the hading of the reactive robots
void FollowB::setRobotHeading(const float& angle){

	geometry_msgs::Twist cmd;
	cmd.linear.x = LINEAR_VEL;
	cmd.angular.z = angle;
	cmdVelPub_.publish(cmd);
}