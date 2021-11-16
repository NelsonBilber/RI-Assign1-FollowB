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

const float TARGET_DIST = 0.3f; /* 0.24f for value inf. it takes time to rotate*/
const float TURNING_RATE = 15.0f;
const float LINEAR_VEL = 0.3f;
const float ANGULAR_VEL = 0.6f;

const float EPSILON = 0.00001f;


struct LaserHit{
  float distance;
  float angle;
  int index;
};

struct Pose {
  float positionx; 
  float positiony;
  float positionz;
  float orientationx;
  float orientationy;
  float orientationz;
};

class FollowB
{
  private:    

    float convertPolarToCartesianX(const LaserHit &hit);
    
    float convertPolarToCartesianY(const LaserHit &hit);
    

    void virtualTriangleWallFollowing(const sensor_msgs::LaserScan& msg); 

    void paralellWallFollowing(const sensor_msgs::LaserScan& msg);

    bool compareFloat(double a, double b);
    
    float degrees2radians(float angle_in_degrees);

    float radians2degrees(float angle_in_radians);

    LaserHit getMinDistanceLaserHit(const sensor_msgs::LaserScan& msg);

    void setupSubscribers(std::string robotId, std::string sensorId);

    void setupPublisher(std::string robotId);

    sensor_msgs::LaserScan scan_;   

    ros::Subscriber subscriberSensor_; 

    ros::Subscriber subscriberOdometry_;     

    ros::NodeHandle nodeHandler_;      

    ros::Publisher cmdVelPub_;

    bool touching_wall;

    Pose lastPose_;

    int inTheSamePosition = 0;

    bool rotateItSelf = false;
    
  public:
    
    FollowB(int argc,char **argv);
        
    ~FollowB(void);   

    void laserCallback(const sensor_msgs::LaserScan& msg);

    void sonarCallback(const sensor_msgs::Range::ConstPtr& msg);

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
};


#endif