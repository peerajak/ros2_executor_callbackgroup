/*
Executor Running Minimal Node with Callback Function
*/
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomSubscriber : public rclcpp::Node {
public:
  OdomSubscriber(std::string odom_topic_name) : Node("simple_subscriber") {

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name, 10,
        std::bind(&OdomSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Odometry = ['%f','%f','%f']",
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  std::shared_ptr<OdomSubscriber> node =
      std::make_shared<OdomSubscriber>("/box_bot_1/odom");

  RCLCPP_INFO(node->get_logger(), "Bacon pancakes, makin bacon pancakes");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
/*
Great. The Node is working as expected with all
 intended functionality!
 However, it is still a basic node.
 There is just one callback function that
 does not take much time to execute and
 will not block any other Callbacks since
 there are no other Callbacks in the code.
 Review the different types of Executors
 before moving on to more sophisticated
 examples.
*/