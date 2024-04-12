/*
 Use of Callback Groups

 The above code declares a class Node with more than one Callback. In this case,
two Callbacks.

Inside the main function, you will see that it loads the Node into one
MultiThreadedExecutor.

Complete the remaining steps in this example and see what happens when you run
the executable.
*/

#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

class TwoTimer : public rclcpp::Node {
public:
  TwoTimer(float sleep_timer1, float sleep_timer2)
      : Node("slow_timer_subscriber") {

    this->wait_time1 = sleep_timer1;
    this->wait_time2 = sleep_timer2;

    timer1_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer1_callback, this));
    timer2_ = this->create_wall_timer(
        500ms, std::bind(&TwoTimer::timer2_callback, this));
  }

private:
  void timer1_callback() {
    sleep(this->wait_time1);
    RCLCPP_INFO(this->get_logger(), "TIMER 1 CALLBACK");
  }

  void timer2_callback() {
    sleep(this->wait_time2);
    RCLCPP_INFO(this->get_logger(), "TIMER 2 CALLBACK");
  }

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  float wait_time1;
  float wait_time2;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Instantiate the Node
  float sleep_time1 = 1.0;
  float sleep_time2 = 3.0;
  std::shared_ptr<TwoTimer> two_timer_node =
      std::make_shared<TwoTimer>(sleep_time1, sleep_time2);

  // Initialize one executor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(two_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
/*
As you can see, only the timer 1 callback is executed.

This is why only one thread gets created, and only one Callback can be
performed. This is because the Multi-Threaded Executor creates one thread per
Node if you do not specify otherwise. So you only have one thread for all
Callbacks in this Node; therefore, it cannot execute Callbacks in parallel.

The timer1_callback is executed because it has been instantiated first in the
constructor.
*/

/*
So how can you solve this issue? You have probably guessed it already. The
solution is to implement callback groups!
*/