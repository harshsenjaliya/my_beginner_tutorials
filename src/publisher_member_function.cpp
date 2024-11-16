/**
 * @file publisher_member_function.cpp
 * @author Harsh Senjaliya (hsenjali@umd.edu)
 * @brief This file contains a simple ROS2 cpp publisher node
 * @version 2.0
 * @date 2024-11-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>

#include "beginner_tutorials/srv/modify_string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

using StringMsg = std_msgs::msg::String;
using PublisherPtr = rclcpp::Publisher<StringMsg>::SharedPtr;
using TimerPtr = rclcpp::TimerBase::SharedPtr;
using ParamEventHandlerPtr = std::shared_ptr<rclcpp::ParameterEventHandler>;
using ParamCallbackHandlePtr = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

/**
 * @brief Class StringPublisher defines a ROS node that publishes a string message to a topic
 */
class StringPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new StringPublisher object
   * 
   * Sets up a parameter, publisher, timer, and service.
   */
  StringPublisher() : Node("string_publisher"), message_count_(0) {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    // Initialize "frequency" parameter with default value 2 Hz and description
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set callback frequency.";
    this->declare_parameter("frequency", 2.0, param_desc);

    // Retrieve the initial frequency and set up the parameter event handler
    auto frequency = this->get_parameter("frequency").as_double();
    parameter_event_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto param_callback = std::bind(&StringPublisher::handle_parameter_update, this, std::placeholders::_1);
    parameter_callback_handle_ = parameter_event_subscriber_->add_parameter_callback("frequency", param_callback);

    // Set up the publisher and timer with the initial frequency
    message_publisher_ = this->create_publisher<StringMsg>("topic", 10);
    auto timer_interval = std::chrono::milliseconds(static_cast<int>(1000 / frequency));
    auto timer_callback = std::bind(&StringPublisher::publish_message, this);
    message_timer_ = this->create_wall_timer(timer_interval, timer_callback);

    // Service to change string content dynamically
    string_service_ = this->create_service<beginner_tutorials::srv::ModifyString>(
        "modify_string",
        std::bind(&StringPublisher::update_string_content, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize transform broadcaster
    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

 private:
  /**
   * @brief Timer callback to publish messages at a specified frequency
   */
  void publish_message() {
    auto message = StringMsg();
    message.data = "Hello, from " + service_message_ + " : " + std::to_string(message_count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.data << "'");

    if (message_count_ % 10 == 0) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Current publisher rate: " << this->get_parameter("frequency").as_double());
    }

    message_publisher_->publish(message);

    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "world";
    transform.child_frame_id = "talker";
    transform.transform.translation.x = 1.0;
    transform.transform.translation.y = 2.0;
    transform.transform.translation.z = 3.0;
    transform.transform.rotation.x = 0.707;
    transform.transform.rotation.y = 0.707;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 0.0;

    transform_broadcaster_->sendTransform(transform);
  }

  /**
   * @brief Service callback to update the string content
   * 
   * @param request Request containing new string content
   * @param response Response with updated string content
   */
  void update_string_content(
      const std::shared_ptr<beginner_tutorials::srv::ModifyString::Request> request,
      const std::shared_ptr<beginner_tutorials::srv::ModifyString::Response> response) {
    service_message_ = request->input;
    response->output = request->input;
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request to change string to: [" << request->input << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "Response sent with content: [" << response->output << "]");
  }

  /**
   * @brief Callback to handle dynamic updates to the frequency parameter
   * 
   * @param param The updated frequency parameter
   */
  void handle_parameter_update(const rclcpp::Parameter& param) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Frequency updated to: " << param.as_double());

    if (param.as_double() > 1000.0) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Warning: High frequency may cause publishing delays.");
    }

    // Update the timer with the new frequency
    auto timer_interval = std::chrono::milliseconds(static_cast<int>(1000 / param.as_double()));
    message_timer_->cancel();  // Stop the current timer
    message_timer_ = this->create_wall_timer(timer_interval, std::bind(&StringPublisher::publish_message, this));
  }

  TimerPtr message_timer_;
  PublisherPtr message_publisher_;
  size_t message_count_;
  std::string service_message_ = "Harsh Senjaliya";
  rclcpp::Service<beginner_tutorials::srv::ModifyString>::SharedPtr string_service_;
  ParamEventHandlerPtr parameter_event_subscriber_;
  ParamCallbackHandlePtr parameter_callback_handle_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringPublisher>());
  rclcpp::shutdown();
  return 0;
}

