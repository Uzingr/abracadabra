// pose_subscriber_pkg/src/pose_subscriber.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <serial/serial.h>

class PoseSubscriber : public rclcpp::Node
{
public:
  PoseSubscriber()
  : Node("pose_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/Hagrid/Pose", 10, std::bind(&PoseSubscriber::topic_callback, this, std::placeholders::_1));
    
    // Configura la connessione seriale al flight controller via USB
    try {
      ser_.setPort("/dev/ttyUSB0");
      ser_.setBaudrate(57600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser_.setTimeout(to);
      ser_.open();
    } catch (serial::IOException& e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open port ");
    }

    if (ser_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Serial Port not initialized");
    }
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Position: [%f, %f, %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // Prepara i dati da inviare
    std::stringstream ss;
    ss << msg->pose.position.x << "," << msg->pose.position.y << "," << msg->pose.position.z << "\n";
    std::string data = ss.str();

    // Invia i dati tramite USB
    if (ser_.isOpen()) {
      ser_.write(data);
      RCLCPP_INFO(this->get_logger(), "Sending data to Pixhawk: %s", data.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Serial port not open");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  mutable serial::Serial ser_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
