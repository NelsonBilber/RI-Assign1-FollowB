# include "followb.h"

FollowB::FollowB(int argc,char **argv)
{
	if(argc != 4)
	{
		ROS_ERROR(
		"Usage : followb <robot_frame_id> <sensor_frame_id> <algorithm>");
		exit(0);
	}

	setupReactiveAlgorithm(argv[3]);
	setupSubscribers(std::string(argv[1]), std::string(argv[2]));
	setupPublisher(std::string(argv[1]));
}


FollowB::~FollowB(void)
{}

void FollowB::setupReactiveAlgorithm(std::string algo)
{	
	if(algo.compare(std::string("vwall")) != 0 &&  algo.compare(std::string("pwall")) != 0)
	{
		std::cout << algo << std::endl;
		std::cout << "Algorithm not supported. Please, choose pwall or vwall as parameter." << std::endl;
		exit(0);
	}
	else
	{
		algorithm_ = algo;
		std::cout <<"Start navigation with algorithm:"<< algorithm_ <<std::endl;
	}
}

void FollowB::setupSubscribers(std::string robotId, std::string sensorId)
{	
	if (sensorId.substr(0, 5) == std::string("laser"))
	{
		std::string laserTopic_ = std::string("/")  +  robotId + std::string("/") + std::string(sensorId);
		subscriberSensor_ = nodeHandler_.subscribe( laserTopic_.c_str(), 1, &FollowB::laserCallback,this);
		std::cout << "Subscribe from robot: " << robotId << " data from: " << sensorId << std::endl;
	}
	else if (sensorId.substr(0, 5) == std::string("sonar"))
	{
		//@Todo:: http://wiki.ros.org/evarobot_sonar/Tutorials/indigo/Writing%20a%20Simple%20Subscriber%20for%20Sonar%20Sensor
		
		std::string sonarTopic_ = std::string("/")  +  robotId + std::string("/") + std::string(sensorId);
		subscriberSensor_ = nodeHandler_.subscribe(sonarTopic_.c_str(), 1, &FollowB::sonarCallback, this);
		std::cout << "Subscribe from robot: " << robotId << " data from: " << sensorId << std::endl;
	}
	else
	{
		std::cout << "Sensor not recognized: " << sensorId << std::endl;		
	}

}

void FollowB::setupPublisher(std::string robotId)
{	
	std::string speedsTopic_ = std::string("/") + robotId + std::string("/cmd_vel");	
	cmdVelPub_ = nodeHandler_.advertise<geometry_msgs::Twist>(speedsTopic_.c_str(), 1);	
}

LaserHit FollowB::getMinDistanceLaserHit(const sensor_msgs::LaserScan& msg)
{
	LaserHit laserHit = {FLT_MAX, 0.0f};

    for(int i = 0; i < msg.ranges.size(); i++) 
	{
		if(msg.ranges[i] < laserHit.distance) 
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

float FollowB::radians2degrees(float angle_in_radians){
	return angle_in_radians * (180/M_PI);
}

float FollowB::convertPolarToCartesianX(const LaserHit &hit)
{
	return hit.distance * cos ((hit.angle));
}
    
float FollowB::convertPolarToCartesianY(const LaserHit &hit)
{
	return hit.distance * sin ((hit.angle));
}

void FollowB::paralellWallFollowing(const sensor_msgs::LaserScan& msg)
{
	LaserHit laserHit = getMinDistanceLaserHit(msg);

	if(laserHit.distance <= msg.range_max) 
	{
		float alpha =  degrees2radians(90) - fabs(laserHit.angle);
		geometry_msgs::Twist cmd;
		cmd.linear.x = LINEAR_VEL;
		cmd.angular.z = - TURNING_RATE * (sin(alpha) - (laserHit.distance - TARGET_DIST)) * LINEAR_VEL;
		cmdVelPub_.publish(cmd);
    }	
    else 
	{
		std::cout << "=> Laser don't touch anything." << std::endl;
    }
}

void FollowB::virtualTriangleWallFollowing(const sensor_msgs::LaserScan& msg)
{
	LaserHit laserHitRay1 = {FLT_MAX, 0.0f};
	LaserHit laserHitRay2 = {FLT_MAX, 0.0f};

	int minIdx = 0;
	
    for(int i = 0; i < msg.ranges.size(); i++) 
	{
		if(msg.ranges[i] < laserHitRay1.distance) 
		{
			minIdx = i;
			laserHitRay1.distance = msg.ranges[i];
		}
    }
	
	laserHitRay1.distance = msg.ranges[minIdx];
	laserHitRay1.angle = msg.angle_min + (msg.angle_increment * float(minIdx));
	laserHitRay1.index = minIdx;

	int INC = (minIdx == msg.ranges.size()-1) ? INC = -1: INC = 1;

	laserHitRay2.distance = msg.ranges[minIdx+INC];
	laserHitRay2.angle = msg.angle_min + (msg.angle_increment * float(minIdx+INC));
	laserHitRay2.index = minIdx+INC;
	
	float x0 = convertPolarToCartesianX(laserHitRay1);
	float y0 = convertPolarToCartesianY(laserHitRay1);

	float x1 = convertPolarToCartesianX(laserHitRay2);
	float y1 = convertPolarToCartesianY(laserHitRay2);

	float alpha = atan2 ((y1-DWALL), (x1 + WALL_LEAD - y0)) ;

	geometry_msgs::Twist cmd;
	cmd.linear.x = LINEAR_VEL;
	cmd.angular.z = alpha;
	cmdVelPub_.publish(cmd);

}

void FollowB::laserCallback(const sensor_msgs::LaserScan& msg)
{
	if(algorithm_ == std::string("pwall"))
	{
		paralellWallFollowing(msg);
	}
	else if(algorithm_ == std::string("vwall"))
	{
		virtualTriangleWallFollowing(msg);
	}
}

// @Read
// http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Range.html
void FollowB::sonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	ROS_INFO("Sonar Range: [%f]", msg->range);
}
