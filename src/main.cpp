# include "followb.h"


int main(int argc,char **argv)
{
  ros::init(argc, argv, "follow_b_shape", ros::init_options::AnonymousName);
  FollowB obj(argc, argv);
  ros::spin();
  return 0;
}