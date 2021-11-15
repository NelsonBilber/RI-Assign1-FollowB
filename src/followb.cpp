# include "followb.h"

FollowB::FollowB(int argc,char **argv)
{
	if(argc != 3)
	{
		ROS_ERROR(
		"Usage : followb <robot_frame_id> <sensor_frame_id>");
		exit(0);
	}

	lastPose_ = {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f};
	setupSubscribers(std::string(argv[1]), std::string(argv[2]));
	setupPublisher(std::string(argv[1]));
}


FollowB::~FollowB(void)
{}

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

	//odometry
	std::string odomTopic = std::string("/") + robotId + std::string("/odom");    
    subscriberOdometry_ = nodeHandler_.subscribe(odomTopic, 1, &FollowB::odometryCallback, this);
}

bool FollowB::compareFloat(double a, double b)
{
    return fabs(a - b) < EPSILON;
}

void FollowB::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
/*
	ROS_INFO("Seq: [%d]", msg->header.seq);
	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

	std::cout << "robot is not stuck " << lastPose_.positionx << " " << msg->pose.pose.position.x  << std::endl;
	std::cout << "robot is not stuck " << lastPose_.positiony << " " << msg->pose.pose.position.y  << std::endl;
	std::cout << "robot is not stuck " << lastPose_.positionz << " " << msg->pose.pose.position.z  << std::endl;
*/
	if( compareFloat(lastPose_.positionx, msg->pose.pose.position.x) &&
		compareFloat(lastPose_.positiony, msg->pose.pose.position.y) &&
		compareFloat(lastPose_.positionz, msg->pose.pose.position.z) &&
		compareFloat(lastPose_.orientationx, msg->pose.pose.orientation.x) &&
		compareFloat(lastPose_.orientationy, msg->pose.pose.orientation.y) &&
		compareFloat(lastPose_.orientationz, msg->pose.pose.orientation.z))
	{
			/*
			rotateItSelf = true;

			inTheSamePosition++;
			if(inTheSamePosition == 100)
			{
				std::cout << "robot is stuck (update position)" << inTheSamePosition  << std::endl;
				
				geometry_msgs::Twist cmd;
				cmd.linear.x = LINEAR_VEL;
				cmd.angular.z =  0;
			
				//cmdVelPub_.publish(cmd);
				rotateItSelf = false;
				inTheSamePosition = 0;
			}
			*/

			//std::cout << "robot is stuck " << inTheSamePosition  << std::endl;
			//std::cout << "angular (z): " << msg->pose.pose.orientation.z << std::endl;
	}
	else{
		inTheSamePosition = 0;
		lastPose_.positionx = msg->pose.pose.position.x ;
		lastPose_.positiony = msg->pose.pose.position.y ;
		lastPose_.positionz = msg->pose.pose.position.z ;
		lastPose_.orientationx =  msg->pose.pose.orientation.x ;
		lastPose_.orientationy =  msg->pose.pose.orientation.y ;
		lastPose_.orientationz =  msg->pose.pose.orientation.z;
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

	//std::cout << "Begin distance: " << laserHit.distance << " angle: " << laserHit.angle << std::endl;
	//std::cout << "Max ranges: " << msg.ranges.size() << std::endl;

    for(int i = 0; i < msg.ranges.size(); i++) 
	{
		//std::cout << "Range (i) : " << i << " range " << msg.ranges[i] << " angle inc. : " << msg.angle_increment << std::endl;

		if(msg.ranges[i] < laserHit.distance) 
		{
			laserHit.distance = msg.ranges[i];
			laserHit.angle = msg.angle_min + (msg.angle_increment * float(i));

			//std::cout << "=> New min. distance: " << laserHit.distance << " angle: " << laserHit.angle << std::endl;
		}
    }

	//std::cout << std::endl;

	return laserHit;
}

float FollowB::degrees2radians(float angle_in_degrees) 
{
	return angle_in_degrees * M_PI / 180.0;
}

// @Read
// http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
void FollowB::laserCallback(const sensor_msgs::LaserScan& msg)
{
	LaserHit laserHit = getMinDistanceLaserHit(msg);
	
	//std::cout << "=> Compare with: " << laserHit.distance << " TargetDistance: " << TARGET_DIST << std::endl;
	std::cout << "=> Compare with: " << laserHit.distance 
			  << " => max range: "	 << msg.range_max
			  << " TargetDistance: " << TARGET_DIST 
			  << std::endl;
	
	if(laserHit.distance <= msg.range_max) 
	{
		touching_wall = true;

		float alpha =  degrees2radians(90) - fabs(laserHit.angle);
		//float alpha =  float(M_PI/2.0) - fabs(laserHit.angle);

		std::cout << "target dst: " << (laserHit.distance - TARGET_DIST) << std::endl;
		std::cout << "alpha: " << alpha << std::endl;
		std::cout << "sin(alpha): " << sin(alpha) << std::endl;

		geometry_msgs::Twist cmd;
		cmd.linear.x = LINEAR_VEL;
		cmd.angular.z = - TURNING_RATE * (sin(alpha) - (laserHit.distance - TARGET_DIST)) * LINEAR_VEL;

		std::cout << "linear vel: " << cmd.linear.x << "\n";
		std::cout << "angular vel: " << cmd.angular.z << "\n";
		std::cout << "laser dist: " << laserHit.distance << "\n";
		std::cout << "alpha: " << alpha << "\n";
		std::cout << std::endl;
		std::cout << "Touching the wall" << std::endl;

		if(!rotateItSelf)
			cmdVelPub_.publish(cmd);
    }	
    else 
	{
    	touching_wall = false;
		std::cout << "=> not touching the wall" << std::endl;
    }
}

// @Read
// http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Range.html
void FollowB::sonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	ROS_INFO("Sonar Range: [%f]", msg->range);
}



/*
File: sensor_msgs/LaserScan.msg
Raw Message Definition
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.
*/
