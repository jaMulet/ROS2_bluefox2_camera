#include "rclcpp/rclcpp.hpp"

#include "bluefox2_ros/bluefox2_ros.hpp"

#include <bluefox2/bluefox2.hpp>
#include <bluefox2/bluefox2_setting.hpp>

#include <memory>
#include <thread>

// From camera_ros_base.h
#include <camera_info_manager/camera_info_manager.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace bluefox2 {

  /**************************************************************************
  * Constructor
  **************************************************************************/
  Bluefox2Ros::Bluefox2Ros(const std::string& prefix = std::string()): Node("bluefox2_node"),
                                                                       bluefox2_(),
                                                                       config_(),
                                                                       node_handle_(std::shared_ptr<Bluefox2Ros>(this, [](auto *) {})),
                                                                       it_(node_handle_),
                                                                       image_pub_(it_.advertise("camera/image_raw", 10)),
                                                                       diagnostic_updater_(node_handle_),
                                                                       topic_diagnostic_(prefix.empty() ? "image_raw" : (prefix + "/image_raw"),
                                                                          diagnostic_updater_,
                                                                          diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10))
  {
    this->declare_parameter("serial", rclcpp::PARAMETER_STRING);
    this->declare_parameter("mode", rclcpp::PARAMETER_STRING);
    this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
    this->declare_parameter("mm", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("fps", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("width", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("height", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("idpf", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("cbm", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("aec", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("expose_us", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("agc", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("gain_db", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("acs", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("des_grey_value", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("hdr", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("dcfm", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("cpc", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("ctm", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("cts", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("request", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("wbp", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("r_gain", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("g_gain", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("b_gain", rclcpp::PARAMETER_INTEGER);

    //From camera_ros_base.h   
    cinfo_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

    bluefox2_.SetSerial(this->get_parameter("serial").as_string());
    SetHardwareId(bluefox2_.serial());

    std::string mode = this->get_parameter("mode").as_string();
    if (mode == "master") {
      bluefox2_.SetMaster();
    } else if (mode == "slave") {
      bluefox2_.SetSlave();
    }

    // Set mirror mode on construction
    bluefox2_.SetMM(this->get_parameter("mm").as_int());

    // Store other config values
    std::string frame_id = this->get_parameter("frame_id").as_string();
    config_.fps = this->get_parameter("fps").as_double();
    config_.width = this->get_parameter("width").as_int();
    config_.height = this->get_parameter("height").as_int();
    config_.idpf = this->get_parameter("idpf").as_int();
    config_.cbm = this->get_parameter("cbm").as_int();
    config_.aec = this->get_parameter("aec").as_bool();
    config_.expose_us = this->get_parameter("expose_us").as_int();
    config_.agc = this->get_parameter("agc").as_bool();
    config_.gain_db = this->get_parameter("gain_db").as_double();
    config_.acs = this->get_parameter("acs").as_int();
    config_.des_grey_value = this->get_parameter("des_grey_value").as_int();
    config_.hdr = this->get_parameter("hdr").as_bool();
    config_.dcfm = this->get_parameter("dcfm").as_int();
    config_.cpc = this->get_parameter("cpc").as_int();
    config_.ctm = this->get_parameter("ctm").as_int();
    config_.cts = this->get_parameter("cts").as_int();
    config_.request = this->get_parameter("request").as_int();
    config_.wbp = this->get_parameter("wbp").as_int();
    config_.r_gain = this->get_parameter("r_gain").as_int();
    config_.g_gain = this->get_parameter("g_gain").as_int();
    config_.b_gain = this->get_parameter("b_gain").as_int();

  }

  /**************************************************************************
  * Acquire function
  **************************************************************************/
  void Bluefox2Ros::Acquire()
  {
    while (is_acquire() && rclcpp::ok())
      {

      bluefox2_.RequestSingle();
      
      const auto expose_us = bluefox2_.GetExposeUs();
      const auto expose_duration = rclcpp::Duration::from_seconds(expose_us * 1e-6 / 2);
      const auto time = this->get_clock()->now() + expose_duration;

      PublishCamera(time);

      Sleep();
      }
  }

  /**************************************************************************
  * AcquireOnce function
  **************************************************************************/
  void Bluefox2Ros::AcquireOnce()
  {
    while (is_acquire() && rclcpp::ok())
      {

      bluefox2_.RequestSingle();
      
      const auto expose_us = bluefox2_.GetExposeUs();
      const auto expose_duration = rclcpp::Duration::from_seconds(expose_us * 1e-6 / 2);
      const auto time = this->get_clock()->now() + expose_duration;

      PublishCamera(time);
      }
  }

  /**************************************************************************
  * Run function
  **************************************************************************/
  void Bluefox2Ros::Run()
  {
    RCLCPP_INFO(rclcpp::get_logger("bluefox2_node"), "Running camera...");
    
    ConfigCb(config_, 0);
  }

  /**************************************************************************
  * End function
  **************************************************************************/
  void Bluefox2Ros::End()
  {
    Stop();
  }

  /**************************************************************************
  * Setup function
  **************************************************************************/
  void Bluefox2Ros::Setup(bluefox2::Config config)
  {
    set_fps(config.fps);
    bluefox2_.Configure(config);
  }


  /**************************************************************************
  * Grab function
  **************************************************************************/
  bool Bluefox2Ros::Grab(sensor_msgs::msg::Image::SharedPtr& image_msg) {
    // Add expose time to current time stamp
    return bluefox2_.GrabImage(*image_msg);
  }

  /**************************************************************************
  * SetHardwareID function (from camera_ros_base.h)
  **************************************************************************/
  void Bluefox2Ros::SetHardwareId(const std::string& id) {
    diagnostic_updater_.setHardwareID(id);
  }

  /**************************************************************************
  * PublishCamera function
  **************************************************************************/
  void Bluefox2Ros::PublishCamera(const rclcpp::Time& time) {
    
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    auto cinfo_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_mgr_->getCameraInfo());
    
    // image_msg.header.frame_id = frame_id_;
    // image_msg.header.stamp = time;
    
    if (Bluefox2Ros::Grab(image_msg)) {

      // Update camera info header
      Update_msg(image_msg, *cinfo_msg);

      image_pub_.publish(image_msg);

      topic_diagnostic_.tick();

    }
    else {
      RCLCPP_WARN(rclcpp::get_logger("bluefox2_node"), "Image lost");
      // TO DO: Lost images counter
    }
    
    diagnostic_updater_.force_update();
  }

  /**************************************************************************
  * Publish function
  **************************************************************************/
  void Bluefox2Ros::Publish(sensor_msgs::msg::Image::SharedPtr& image_msg) {
    
    auto cinfo_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_mgr_->getCameraInfo());

    // Update camera info header
    Update_msg(image_msg, *cinfo_msg);

    image_pub_.publish(image_msg);
    
    topic_diagnostic_.tick();

    diagnostic_updater_.force_update();
  }

  /**************************************************************************
  * ConfigCb function
  **************************************************************************/
  void Bluefox2Ros::ConfigCb(bluefox2::Config config, int level)
  {
    if (level < 0) {
      RCLCPP_INFO(rclcpp::get_logger("bluefox2_node"), "Initiating camera reconfiguration...");
    }
    if (is_acquire()) {
      Stop();
    }

    Setup(config);

    SetRate(config.fps);

    Start();
  }

  /**************************************************************************
  * Start function
  **************************************************************************/
  void Bluefox2Ros::Start()
  {
    is_acquire_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("bluefox2_node"), "STARTING acquisition...");

    acquire_thread_.reset(new std::thread(&Bluefox2Ros::Acquire, this));
    //AcquireOnce();
  }

  /**************************************************************************
  * Stop function
  **************************************************************************/
  void Bluefox2Ros::Stop()
  {
    if (!is_acquire_) return;
    is_acquire_ = false;
    acquire_thread_->join();
  }

  /**************************************************************************
  * UpdateMSG function
  **************************************************************************/
  void Bluefox2Ros::Update_msg(sensor_msgs::msg::Image::SharedPtr& msg, sensor_msgs::msg::CameraInfo & camera_info_msg)
  {
    rclcpp::Time timestamp = this->get_clock()->now();
    
    msg->header.frame_id = frame_id_;
    msg->header.stamp = timestamp;
    camera_info_msg.header.frame_id = frame_id_;
    camera_info_msg.header.stamp = timestamp;
  }

}  // namespace bluefox2


/**************************************************************************
* MAIN function
**************************************************************************/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<bluefox2::Bluefox2Ros>();

  node->Run();
  
  rclcpp::spin(node);
  
  node->End();

  rclcpp::shutdown();

  return 0;

}