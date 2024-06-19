#ifndef _UWB_FILTER_H
#define _UWB_FILTER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <deque>
#include <iostream>
#include <iomanip>
#include <chrono>

#include <nlink_parser/LinktrackAoaNode0.h>
#include <nlink_parser/LinktrackAoaNodeframe0.h>

namespace uwb_signal_processing {

class UWB_Filter 
{
public:
  UWB_Filter( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private );

  ~UWB_Filter() {}

  void NLink_Parser_Callback( const nlink_parser::LinktrackAoaNodeframe0 &msg );

private:
  std::string formatTimestamp(double timestamp);

  bool handleMissingData( float& angle, float& distance );

  void updateThreshold( float &cur_dis );

  bool schwellenwertFilter( float &angle );

  bool inActiveArea( float &angle );

  void moving_average_filter( float &cur_data, float &pre_data );

  void publish();

  // ROS Parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber nlink_parser_sub_;
  ros::Publisher uwb_pub_;

  std::deque<float> angles_;
  std::deque<float> distances_;
  std::deque<float> local_times_;
  double timestamp_;

  // 设置最大长度
  // const size_t PYWT_MAX_LENGTH = 128;  
  // const size_t DIST_MAX_LENGTH = 10;
  const size_t MAX_LENGTH_ = 5;
  static float kAlpha_;

  float max_velocity_;
  float yaw_thresh_;
};

}  // namespace uwb_signal_processing

#endif // _UWB_FILTER_H