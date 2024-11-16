#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>

#include "beginner_tutorials/srv/modify_string.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::String;

////////////////////////////////////////////////////////
/// @brief Define Fixture for setting up the testing environment
////////////////////////////////////////////////////////
auto global_logger = rclcpp::get_logger("");  ///< Initial Logger instance

class TestFixture {
 public:
  TestFixture() {
    // Create the node for testing (Integration test node):
    test_node_ = rclcpp::Node::make_shared("integration_test_node");
    global_logger = test_node_->get_logger();  // Ensure messages appear in rqt_console

    // Declare and retrieve a parameter for the duration of the test:
    test_node_->declare_parameter<double>("test_duration");
    test_duration_ = test_node_->get_parameter("test_duration")
                         .get_parameter_value()
                         .get<double>();
    RCLCPP_INFO_STREAM(global_logger, "Received test_duration = " << test_duration_);
  }

  ~TestFixture() {}

 protected:
  double test_duration_;                       ///< Duration of the test
  rclcpp::Node::SharedPtr test_node_;          ///< Shared pointer to test node
};

////////////////////////////////////////////////////////
/// @brief Test Case 1: Verifies if the service "modify_string" is available
////////////////////////////////////////////////////////
TEST_CASE_METHOD(TestFixture, "test service server availability", "[service]") {
  // Create a client for the "modify_string" service:
  auto service_client = test_node_->create_client<beginner_tutorials::srv::ModifyString>("modify_string");
  RCLCPP_INFO_STREAM(global_logger, "Service client for 'modify_string' created");

  // Record the start time and initialize variables:
  rclcpp::Time start_time = rclcpp::Clock().now();
  bool is_service_available = false;
  rclcpp::Duration time_elapsed = 0s;
  RCLCPP_INFO_STREAM(global_logger, "Testing service availability...");

  // Wait for service availability with a timeout:
  auto wait_timeout = std::chrono::milliseconds(static_cast<int>(test_duration_ * 1000));
  if (service_client->wait_for_service(wait_timeout)) {  // blocking
    time_elapsed = (rclcpp::Clock().now() - start_time);
    is_service_available = true;
  }

  // Log the time taken and result:
  RCLCPP_INFO_STREAM(global_logger, "Elapsed time = " << time_elapsed.seconds() << "s, service_found = " << is_service_available);

  // Check that the service was found:
  CHECK(is_service_available);
}

////////////////////////////////////////////////////////
/// @brief Test Case 2: Verifies if a specific string topic is being published
////////////////////////////////////////////////////////
TEST_CASE_METHOD(TestFixture, "test topic publication", "[topic]") {
  bool topic_received = false;

  // Define a callback structure to capture the published string:
  struct TopicCallback {
    explicit TopicCallback(bool &topic_status) : topic_status_(topic_status) {}
    void operator()(const String msg) const {
      // Log the received message:
      RCLCPP_INFO_STREAM(global_logger, "Received message: " << msg.data.c_str());
      topic_status_ = true;
      // Assert the message content:
      CHECK(msg.data.find("Hello, from") != std::string::npos);
    }
    bool &topic_status_;
  };

  // Create a subscriber for the string topic:
  auto string_subscriber = test_node_->create_subscription<String>(
      "topic", 10, TopicCallback(topic_received));

  rclcpp::Rate check_rate(10.0);  // 10Hz check rate
  auto start_time = rclcpp::Clock().now();
  auto elapsed_time = rclcpp::Clock().now() - start_time;
  auto timeout_duration = rclcpp::Duration::from_seconds(test_duration_);

  // Log the start of the test:
  RCLCPP_INFO_STREAM(global_logger, "Elapsed time = " << elapsed_time.seconds() << "s, timeout = " << timeout_duration.seconds() << "s");

  // Loop until the topic is received or timeout occurs:
  while (!topic_received && (elapsed_time < timeout_duration)) {
    rclcpp::spin_some(test_node_);
    check_rate.sleep();
    elapsed_time = (rclcpp::Clock().now() - start_time);
  }

  // Log the result:
  RCLCPP_INFO_STREAM(global_logger, "Elapsed time = " << elapsed_time.seconds() << "s, topic_received = " << topic_received);

  // Assert that the topic was received:
  CHECK(topic_received);
}

