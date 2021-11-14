# include "followb.h"


int main(int argc,char **argv)
{
  ros::init(argc, argv, "stdr_obstacle_avoidance", ros::init_options::AnonymousName);
  FollowB obj(argc, argv);
  ros::spin();
  return 0;
}