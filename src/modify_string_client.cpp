/**
 * @file modify_string_client.cpp
 * @author Harsh senjaliya (hsenjali@umd.edu)
 * @brief A cpp file to run a client to call modify_string service
 * @version 2.0
 * @date 2024-11-08
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <chrono>
#include <iostream>
#include <rclcpp/client.hpp>
#include <string>

#include "beginner_tutorials/srv/modify_string.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief A class that defines a service client for calling the ModifyString service.
 */
class StringModifierClient : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new StringModifierClient object
   * @param node_name Name of the service client node
   */
  StringModifierClient() : Node("string_modifier_client") {
    modify_string_client_ = this->create_client<beginner_tutorials::srv::ModifyString>(
        "modify_string");
  }

  rclcpp::Client<beginner_tutorials::srv::ModifyString>::SharedPtr modify_string_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Verify that an input argument is provided when running the node
  if (argc < 2) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Usage: please provide a string as an argument");
    return 1;
  }

  // Create an instance of the StringModifierClient
  auto string_modifier_client = std::make_shared<StringModifierClient>();

  // Create a request for the ModifyString service
  auto modify_request = std::make_shared<beginner_tutorials::srv::ModifyString::Request>();
  modify_request->input = argv[1];

  // Check if the service is available in a loop
  while (!string_modifier_client->modify_string_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("rclcpp"),
          "Interrupted while waiting for the service to become available. Exiting.");
      return 1;
    }
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),
                       "Service not available; waiting...");
  }

  // Send the request to the service
  auto response_future = string_modifier_client->modify_string_client_->async_send_request(modify_request);

  // Wait for the response
  if (rclcpp::spin_until_future_complete(string_modifier_client, response_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("rclcpp"),
        "Response received: modified string is: " << response_future.get()->output.c_str());
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}

