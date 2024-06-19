#include <ros/ros.h>

#include <uwb_signal_processing/uwb_filter.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uwb_filter_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  uwb_signal_processing::UWB_Filter uwb_filter(nh,nh_private);

  ros::spin();

  return 0;
}
