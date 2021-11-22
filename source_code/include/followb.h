#ifndef FOLLOWB
#define FOLLOWB

#include <iostream>
#include <cstdlib>
#include <cmath>

#include <ros/package.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

/* Laser sensor constants */
const float TARGET_DIST_LASER = 0.3f; /* 0.24f for value inf. it takes time to rotate*/
const float TURNING_RATE_LASER = 15.0f;

/* Sonar sensor constants */
const float TARGET_DIST_SONAR = 0.5f;
const float TURNING_RATE_SONAR = 15.0f;

/* Velocities */
const float LINEAR_VEL = 0.3f;
const float ANGULAR_VEL = 0.6f;

/* Distances */
const float DWALL = 1.1f;
const float WALL_LEAD = 2.6f;

const float EPSILON = 0.00001f;

struct HitRay
{
  float distance;
  float angle;
  int index;
};

class FollowB
{
private:

  // Calculate the robot's heading using the virtual right triangle wall follower technique  
  void virtualRightTriangleWallFollower(const sensor_msgs::LaserScan &msg);

  // Calculate the robot's heading using the 'parallel' wall follower technique
  void parallelWallFollowing(const HitRay &hray, float targetDistance, float turningRate, float rangeMax);

  // Set the heading of the reactive robots
  void setRobotHeading(const float& angle);

  // Compare two floats 
  bool compareFloat(double a, double b);

  // Get x cartesian coordinate from a polar coordinate 
  float convertPolarToCartesianX(const HitRay &hit);

  // Get y coordinate coordinate from a polar coordinate
  float convertPolarToCartesianY(const HitRay &hit);

  // Convert degress to radians 
  float degrees2radians(float angle_in_degrees);

  // Convert radians to degree
  float radians2degrees(float angle_in_radians);

  // Get the ray with the minimal distance to the obstacle and the angle
  HitRay getMinDistanceLaserHit(const sensor_msgs::LaserScan &msg);

  // Check if the sensor support the algorithm
  void setupReactiveAlgorithm(std::string sensorType, std::string algo);

  /*
  Subscribe the data from the sensors and set a callback to reveive their data:
  laser_0 (is the laser)
  sonar_0 (sonar 1)
  sonar_1 (sonar 2)
  */
  void setupSubscribers(std::string robotId, std::string sensorType);

  // Setup the the Publisher
  void setupPublisher(std::string robotId);

  sensor_msgs::LaserScan scan_;

  ros::Subscriber subscriberSensor_;

  ros::Subscriber subscribeSonar0_;

  ros::Subscriber subscribeSonar1_;

  ros::NodeHandle nodeHandler_;

  ros::Publisher cmdVelPub_;

  std::string algorithm_;

public:
  FollowB(int argc, char **argv);

  ~FollowB(void);

  // Receive the messages from the laser sensors 
  void laserCallback(const sensor_msgs::LaserScan &msg);

  // Receive the messages from the sonar sensors
  void sonarCallback(const sensor_msgs::Range::ConstPtr &msg);

};

#endif
