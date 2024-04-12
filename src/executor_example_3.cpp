/*
4.5 Single-Threaded Executor Running Node with Two Callback Functions
*/

/*

Types of Executors in ROS2
rclcpp provides three Executor types, derived from a shared parent class:
- The Single-threaded Executor uses one thread to execute all the Node
instructions.
- The Multi-T=threaded Executor creates a variable number of threads that allow
multiple messages/events to process in parallel.
- The Static Single-threaded Executor performs a Node scan only once when the
Node is added to the Executor. Therefore, use it only with Nodes that create all
related Callbacks during initialization.

In the first two implementations, the Executor adapts to changes such as adding,
removing or changing subscriptions, timers, service servers and action servers
during runtime. Use the last option in the list above if you do not need that
capability and you are fine with single-threaded processes set up at
initialization time.
*/
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

class OdomSubscriber : public rclcpp::Node {
public:
  OdomSubscriber(std::string odom_topic_name) : Node("odom_subscriber") {

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name, 10,
        std::bind(&OdomSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Odometry=['%f','%f','%f']",
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                msg->pose.pose.position.z);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

class SlowTimer : public rclcpp::Node {
public:
  SlowTimer(float sleep_timer) : Node("slow_timer_subscriber") {
    this->wait_time = sleep_timer;
    timer_ = this->create_wall_timer(
        500ms, std::bind(&SlowTimer::timer_callback, this));
  }

private:
  void timer_callback() {
    sleep(this->wait_time);
    RCLCPP_INFO(this->get_logger(), "TICK");
  }
  rclcpp::TimerBase::SharedPtr timer_;
  float wait_time;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Instantiate the odometry subscriber node
  std::shared_ptr<OdomSubscriber> odom_subs_node =
      std::make_shared<OdomSubscriber>("/box_bot_1/odom");

  // Instantiate the timer node node
  float sleep_time = 3.0;
  std::shared_ptr<SlowTimer> slow_timer_node =
      std::make_shared<SlowTimer>(sleep_time);

  RCLCPP_INFO(odom_subs_node->get_logger(), "odom_subs_node INFO...");
  RCLCPP_INFO(slow_timer_node->get_logger(), "slow_timer_node INFO...");

  rclcpp::executors::SingleThreadedExecutor executor;

  // Add the odometry subscriber node to the executor
  executor.add_node(odom_subs_node);
  // Add the timer node to the executor
  executor.add_node(slow_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

/*
The problem is that the odometry Callback function is not executed even when the
/box_bot_1/odom topic has published values. Its execution is blocked by the
timer callback function and has not finalized its work yet.

This is because you only have ONE THREAD in the Executor, as is the case with
the Single-Threaded Executor and Static Single-Threaded Executor types.

So the logical solution is to use the Multi-Threaded Executor. So, do it.
*/