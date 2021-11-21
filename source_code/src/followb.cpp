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

void FollowB::setupPublisher(std::string robotId)
{
	std::string speedsTopic_ = std::string("/") + robotId + std::string("/cmd_vel");
	cmdVelPub_ = nodeHandler_.advertise<geometry_msgs::Twist>(speedsTopic_.c_str(), 1);
}

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

bool FollowB::compareFloat(double a, double b)
{
	return fabs(a - b) < EPSILON;
}

float FollowB::degrees2radians(float angle_in_degrees)
{
	return angle_in_degrees * (M_PI / 180.0);
}

float FollowB::radians2degrees(float angle_in_radians)
{
	return angle_in_radians * (180 / M_PI);
}

float FollowB::convertPolarToCartesianX(const HitRay &hit)
{
	return hit.distance * cos((hit.angle));
}

float FollowB::convertPolarToCartesianY(const HitRay &hit)
{
	return hit.distance * sin((hit.angle));
}

void FollowB::paralellWallFollowing(const HitRay &hray, float targetDistance, float turningRate, float rangeMax)
{
	geometry_msgs::Twist cmd;
	if (hray.distance <= rangeMax)
	{
		float alpha = degrees2radians(90) - fabs(hray.angle);
		cmd.linear.x = LINEAR_VEL;
		cmd.angular.z = -turningRate * (sin(alpha) - (hray.distance - targetDistance)) * LINEAR_VEL;
		cmdVelPub_.publish(cmd);
	}
	else
	{
		cmd.linear.x = LINEAR_VEL;
		cmdVelPub_.publish(cmd);
	}
}

void FollowB::virtualTriangleWallFollowing(const sensor_msgs::LaserScan &msg)
{
	HitRay laserHitRay1 = {FLT_MAX, 0.0f};
	HitRay laserHitRay2 = {FLT_MAX, 0.0f};

	int minIdx = 0;

	for (int i = 0; i < msg.ranges.size(); i++)
	{
		if (msg.ranges[i] < laserHitRay1.distance)
		{
			minIdx = i;
			laserHitRay1.distance = msg.ranges[i];
		}
	}

	laserHitRay1.distance = msg.ranges[minIdx];
	laserHitRay1.angle = msg.angle_min + (msg.angle_increment * float(minIdx));
	laserHitRay1.index = minIdx;

	int INC = (minIdx == msg.ranges.size() - 1) ? INC = -1 : INC = 1;

	laserHitRay2.distance = msg.ranges[minIdx + INC];
	laserHitRay2.angle = msg.angle_min + (msg.angle_increment * float(minIdx + INC));
	laserHitRay2.index = minIdx + INC;

	float x0 = convertPolarToCartesianX(laserHitRay1);
	float y0 = convertPolarToCartesianY(laserHitRay1);

	float x1 = convertPolarToCartesianX(laserHitRay2);
	float y1 = convertPolarToCartesianY(laserHitRay2);

	float alpha = atan2((y1 - DWALL), (x1 + WALL_LEAD - y0));

	geometry_msgs::Twist cmd;
	cmd.linear.x = LINEAR_VEL;
	cmd.angular.z = alpha;
	cmdVelPub_.publish(cmd);
}

void FollowB::laserCallback(const sensor_msgs::LaserScan &msg)
{
	//ROS_INFO("Laser Seq: [%d]", msg.header.seq);
	//ROS_INFO("Laser Scan time: [%f]", msg.scan_time);

	if (algorithm_ == std::string("pwall"))
	{
		paralellWallFollowing(getMinDistanceLaserHit(msg), TARGET_DIST_LASER, TURNING_RATE_LASER, msg.range_max);
	}
	else if (algorithm_ == std::string("vwall"))
	{
		virtualTriangleWallFollowing(msg);
	}
}

void FollowB::sonarCallback(const sensor_msgs::Range::ConstPtr &msg)
{
	//ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	//ROS_INFO("Sonar Range: [%f]", msg->range);

	if (msg->range != std::numeric_limits<sensor_msgs::Range::_range_type>::infinity())
	{
		HitRay hray{msg->range, msg->field_of_view / 2, 0};
		paralellWallFollowing(hray, TARGET_DIST_SONAR, TURNING_RATE_SONAR, msg->max_range);
	}
}
