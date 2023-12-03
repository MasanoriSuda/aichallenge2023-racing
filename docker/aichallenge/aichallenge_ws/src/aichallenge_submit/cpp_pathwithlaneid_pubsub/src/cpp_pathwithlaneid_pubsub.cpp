#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cpp_pathwithlaneid_pubsub.hpp"
#include <memory>

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathSubscriber>());
  rclcpp::shutdown();
  return 0;
}
