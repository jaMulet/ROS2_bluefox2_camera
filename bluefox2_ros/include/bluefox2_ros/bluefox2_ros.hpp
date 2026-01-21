#ifndef BLUEFOX2_ROS__BLUEFOX2_ROS_HPP_
#define BLUEFOX2_ROS__BLUEFOX2_ROS_HPP_

#include "rclcpp/rclcpp.hpp"

#include <bluefox2/bluefox2.hpp>
#include <bluefox2/bluefox2_setting.hpp>

// From camera_ros_base.h
#include <camera_info_manager/camera_info_manager.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace bluefox2 {

class Bluefox2Ros : public rclcpp::Node
{
  public:

  Bluefox2Ros(const std::string& prefix);

  virtual void Acquire();

  void AcquireOnce();

  // From camera_node_base.h
  void Run();
  void End();

  void Setup(bluefox2::Config config);

  std::unique_ptr<std::thread> acquire_thread_;

 private:

  bluefox2::Bluefox2 bluefox2_;
  bluefox2::Config config_;
  rclcpp::Node::SharedPtr node_handle_;

  // From camera_ros_base.h
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_mgr_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::HeaderlessTopicDiagnostic topic_diagnostic_;
  
  bool is_acquire_;
  void RequestSingle() const { bluefox2_.RequestSingle(); }
  Bluefox2& camera() { return bluefox2_; }
  bool Grab(sensor_msgs::msg::Image::SharedPtr& image_msg);
  
  // From camera_ros_base.h
  double fps() const { return fps_; };
  void set_fps(double fps) { fps_ = fps; };
  void SetHardwareId(const std::string& id);
  void PublishCamera(const rclcpp::Time& time);
  void Publish(sensor_msgs::msg::Image::SharedPtr& image_msg);

  // From camera_node_base.h
  bool is_acquire() const { return is_acquire_; };
  void Sleep() const { rate_->sleep(); }
  void ConfigCb(bluefox2::Config config, int level); // TO DEFINE
  void SetRate(double fps) { rate_.reset(new rclcpp::Rate(fps)); }
  void Start();
  void Stop();

  std::unique_ptr<rclcpp::Rate> rate_;

  // From camera_ros_base.h
  double fps_; 
  std::string frame_id_;
  std::string identifier_;

};

}  // namespace bluefox2

#endif  // BLUEFOX2_ROS_HPP_
