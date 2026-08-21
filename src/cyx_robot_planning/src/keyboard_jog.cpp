#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "cyx_robot_interfaces/msg/cartesian_jog.hpp"
#include "rclcpp/rclcpp.hpp"

class TerminalGuard
{
public:
  TerminalGuard()
  {
    tcgetattr(STDIN_FILENO, &original_);
    auto raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  ~TerminalGuard() {tcsetattr(STDIN_FILENO, TCSANOW, &original_);}

private:
  termios original_{};
};

class KeyboardJog : public rclcpp::Node
{
public:
  KeyboardJog() : Node("cyx_keyboard_jog")
  {
    step_ = declare_parameter("step_size", 0.010);
    frame_ = declare_parameter("reference_frame", "world");
    eef_ = declare_parameter("end_effector_link", "tool0");
    duration_ = declare_parameter("duration", 1.0);
    pub_ = create_publisher<cyx_robot_interfaces::msg::CartesianJog>(
      "/cyx_arm/cartesian_jog", 10);
  }

  void run()
  {
    TerminalGuard guard;
    std::cout << "CYX Cartesian jog: W/S=+/-X, A/D=+/-Y, I/K=+/-Z, Space=stop, Q=quit\n";
    std::cout << "step=" << step_ * 1000.0 << " mm frame=" << frame_ << " eef=" << eef_ << '\n';
    while (rclcpp::ok()) {
      char key{};
      if (::read(STDIN_FILENO, &key, 1) != 1) {
        continue;
      }
      key = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
      if (key == 'q') {
        break;
      }
      cyx_robot_interfaces::msg::CartesianJog msg;
      msg.header.stamp = now();
      msg.header.frame_id = frame_;
      msg.reference_frame = frame_;
      msg.end_effector_link = eef_;
      msg.duration = duration_;
      if (key == 'w') msg.translation.x = step_;
      else if (key == 's') msg.translation.x = -step_;
      else if (key == 'a') msg.translation.y = step_;
      else if (key == 'd') msg.translation.y = -step_;
      else if (key == 'i') msg.translation.z = step_;
      else if (key == 'k') msg.translation.z = -step_;
      else if (key != ' ') continue;
      pub_->publish(msg);
    }
  }

private:
  double step_{0.01};
  double duration_{1.0};
  std::string frame_;
  std::string eef_;
  rclcpp::Publisher<cyx_robot_interfaces::msg::CartesianJog>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardJog>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
