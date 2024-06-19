#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <uwb_signal_processing/uwb_filter.h>
#include <uwb_signal_processing/common.h>

namespace uwb_signal_processing {

float UWB_Filter::kAlpha_;

UWB_Filter::UWB_Filter( const ros::NodeHandle& nh, const ros::NodeHandle& nh_private )
    : nh_(nh),
    nh_private_(nh_private)
{   
    double max_velocity, kAlpha;
    nh_private_.param("uwb/max_velocity", max_velocity, -1.0);
    nh_private_.param("uwb/kAlpha", kAlpha, 0.7);
    max_velocity_ = static_cast<float>(max_velocity);
    kAlpha_ = static_cast<float>(kAlpha);
    nlink_parser_sub_ = nh_private_.subscribe("/nlink_linktrack_aoa_nodeframe0", 1000, &UWB_Filter::NLink_Parser_Callback, this);
}


std::string UWB_Filter::formatTimestamp(double timestamp) 
{
    std::time_t t = static_cast<std::time_t>(timestamp);
    std::tm tm = *std::localtime(&t);
    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}


bool UWB_Filter::handleMissingData(float& angle, float& distance) 
{
    if (local_times_.size() > 3) {
        float t0 = local_times_.back();
        float t1 = local_times_[local_times_.size() - 2];
        float t2 = local_times_[local_times_.size() - 3];
        float t3 = local_times_[local_times_.size() - 4];
        float dt1 = t0 - t1;
        float dt2 = t1 - t2;
        float dt3 = t2 - t3;

        if (std::isnan(angle)) {
            float v1 = (angles_.back() - angles_[angles_.size() - 2]) / dt2;
            float v2 = (angles_[angles_.size() - 2] - angles_[angles_.size() - 3]) / dt3;
            float a = (v1 - v2) / ((dt1 + dt2) / 2);
            angle = angles_.back() + v1 * dt1 + 0.5f * a * dt1 * dt1;
        }

        if (std::isnan(distance)) {
            float v1 = (distances_.back() - distances_[distances_.size() - 2]) / dt2;
            float v2 = (distances_[distances_.size() - 2] - distances_[distances_.size() - 3]) / dt3;
            float a = (v1 - v2) / ((dt1 + dt2) / 2);
            distance = distances_.back() + v1 * dt1 + 0.5f * a * dt1 * dt1;
        }
        return true;
    } else return false;
}


void UWB_Filter::updateThreshold( float &cur_dis )
{
    if( max_velocity_ < 0 ) {
        ROS_WARN("Inappropriate parameter initialization: Max velocity set to--%f.", max_velocity_ );
        return;
    }

    if( cur_dis <=  0 ) {
        ROS_WARN("Current Distance error detected: %f.", cur_dis );
        return;
    }

    yaw_thresh_ = max_velocity_ / cur_dis;
}


/// @brief 
/// @param angle 当前Tag相对于Anchor的角度 (Degree).
/// @return 
bool UWB_Filter::schwellenwertFilter( float &angle )
{
    if (angles_.empty() || local_times_.size() < 2 ) {
        return true; // If there are no previous angles or times, nothing to limit
    }

    float previous_angle = angles_.back();
    float previous_time = local_times_[local_times_.size() - 2];

    float current_time = local_times_.back();
    float time_interval = current_time - previous_time;

    // Calculate maximum allowed angle change based on time interval
    float max_angle_change_radius = yaw_thresh_ * time_interval;
    float max_angle_change_degree = radiansToDegrees(max_angle_change_radius);

    float angle_change = angle - previous_angle;

    // std::cout << "yaw_thresh_: " << yaw_thresh_ <<std::endl;
    // std::cout << "time_interval: " << time_interval <<std::endl;
    // std::cout << "max_angle_change_radius: " << max_angle_change_radius <<std::endl;
    // std::cout << "max_angle_change_degree: " << max_angle_change_degree <<std::endl;
    // std::cout << "previous_angle: " << previous_angle <<std::endl;
    // std::cout << "angle_change: " << angle_change <<std::endl;

    if( std::abs(angle_change) > 3.0f * max_angle_change_degree) {
        angle = -1.0f;
        return false;
    }

    if (std::abs(angle_change) > max_angle_change_degree) {
        angle = previous_angle + (angle_change > 0 ? max_angle_change_degree : -max_angle_change_degree);
        return false; // Angle was limited
    }

    return true; // Angle was within limits
}


bool UWB_Filter::inActiveArea(float &angle) 
{
    if (std::abs(angle) > 75.0f) {
        angle = sgn(angle) * 75.0f;
        return false;
    }
    return true;
}


// 指数移动平均（EMA）滤波器
void UWB_Filter::moving_average_filter( float &cur_data, float &pre_data )
{
    cur_data = (1 - kAlpha_) * pre_data + kAlpha_ * cur_data;
}


void UWB_Filter::NLink_Parser_Callback( const nlink_parser::LinktrackAoaNodeframe0 &msg )
{
    ros::Time t1 = ros::Time::now();

    if (std::isnan(msg.local_time) || msg.stamp.isZero()) {
        ROS_WARN("Received a NaN time.");
        return;
    }

    if (msg.nodes.empty()) {
        ROS_WARN("Received an empty nodes array, please check UWB Tag!");
        return;
    }

    float local_time = msg.local_time / 1000.0f; // 将时间转换为秒
    timestamp_ = msg.stamp.toSec();

    if( !local_times_.empty() && local_time < local_times_.back() ) {
        ROS_WARN("Received a local time that is earlier than the last received local time.");
        return;
    }

    local_times_.push_back(local_time);

    for (const auto& node : msg.nodes) {
        float angle = node.angle;
        float distance = node.dis;

        // std::cout << "Angle: " << angle <<std::endl;

        // 如果数据无效，根据之前的数据变化速度和加速度推测一个值
        if ( std::isnan(angle) || std::isnan(distance) ) 
        {
            if (local_times_.size() > 3) {
                float t0 = local_times_.back();
                float t1 = local_times_[local_times_.size() - 2];
                float t2 = local_times_[local_times_.size() - 3];
                float t3 = local_times_[local_times_.size() - 4];
                float dt1 = t0 - t1;
                float dt2 = t1 - t2;
                float dt3 = t2 - t3;

                if (std::isnan(angle)) {
                    float v1 = (angles_.back() - angles_[angles_.size() - 2]) / dt2;
                    float v2 = (angles_[angles_.size() - 2] - angles_[angles_.size() - 3]) / dt3;
                    float a = (v1 - v2) / ((dt1 + dt2) / 2);
                    angle = angles_.back() + v1 * dt1 + 0.5f * a * dt1 * dt1;
                }

                if (std::isnan(distance)) {
                    float v1 = (distances_.back() - distances_[distances_.size() - 2]) / dt2;
                    float v2 = (distances_[distances_.size() - 2] - distances_[distances_.size() - 3]) / dt3;
                    float a = (v1 - v2) / ((dt1 + dt2) / 2);
                    distance = distances_.back() + v1 * dt1 + 0.5f * a * dt1 * dt1;
                }
            } else {
                local_times_.pop_back();
                return;
            }
        }

        if( distance < 0.0f || distance > 40.0f ) {
            ROS_WARN("Received distance value is unstable, wait...");
            return;
        }

        updateThreshold( distance );
        std::cout << "Angle before: " << angle <<std::endl;
        if( !schwellenwertFilter(angle) && angle < 0.0f ) {
            local_times_.pop_back();
            return;
        }
        std::cout << "Angle after: " << angle <<std::endl;

        if( !angles_.empty() ) {
            moving_average_filter( angle, angles_.back() );
            moving_average_filter( distance, distances_.back() );
        }
        inActiveArea(angle);
        angles_.push_back(angle);
        distances_.push_back(distance);

        // 保持数组长度在 MAX_LENGTH 以内
        if( local_times_.size() > MAX_LENGTH_ ) {
            local_times_.pop_front();
            distances_.pop_front();
            angles_.pop_front();
        }
        // if (angles_.size() > PYWT_MAX_LENGTH) {
        //     angles_.pop_front();
        // }

        // if (local_times_.size() > DIST_MAX_LENGTH) {
        //     local_times_.pop_front();
        //     distances_.pop_front();
        // }
    }

    publish();

    // 打印当前的timestamp_，local_time，angle和distance
    std::cout << "Timestamp: " << formatTimestamp(timestamp_) 
                  << ", Local Time: " << local_time 
                  << ", Angle: " << angles_.back() 
                  << ", Distance: " << distances_.back() << std::endl;
    
    ROS_WARN( "Process UWB data: %lf", (ros::Time::now() - t1).toSec());
}


void UWB_Filter::publish()
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    if (angles_.empty() || distances_.empty()) {
        ROS_WARN("No data available to publish.");
        return;
    }

    // 假设我们取最后一个 angle 和 distance 进行转换
    float angle = angles_.back();
    float distance = distances_.back();

    // 将角度转换为弧度
    float angle_rad = angle * M_PI / 180.0;

    // 计算平面直角坐标
    float x = distance * cos(angle_rad);
    float y = distance * sin(angle_rad);  
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "uwb";
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, angle_rad);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

}  // namespace uwb_signal_processing