#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "cyx_robot_interfaces/msg/cartesian_jog.hpp"
#include "cyx_robot_interfaces/msg/joint_jog.hpp"
#include "rclcpp/rclcpp.hpp"

class TerminalGuard
{
public:
  TerminalGuard()
  {
    // ros2 launch does not forward its stdin to child processes. Open the
    // controlling terminal explicitly so this node remains interactive when
    // it is started from a launch file.
    fd_ = ::open("/dev/tty", O_RDONLY | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error(
              std::string("failed to open /dev/tty: ") + std::strerror(errno));
    }
    if (::tcgetattr(fd_, &original_) != 0) {
      const auto error = std::string("failed to read terminal settings: ") +
        std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error(error);
    }
    auto raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    if (::tcsetattr(fd_, TCSANOW, &raw) != 0) {
      const auto error = std::string("failed to enable raw terminal mode: ") +
        std::strerror(errno);
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error(error);
    }
  }
  ~TerminalGuard()
  {
    if (fd_ >= 0) {
      ::tcsetattr(fd_, TCSANOW, &original_);
      ::close(fd_);
    }
  }

  TerminalGuard(const TerminalGuard &) = delete;
  TerminalGuard & operator=(const TerminalGuard &) = delete;

  int fd() const {return fd_;}

private:
  int fd_{-1};
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
    joint_step_ = declare_parameter("joint_step", 10.0);
    cartesian_pub_ = create_publisher<cyx_robot_interfaces::msg::CartesianJog>(
      "/cyx_arm/cartesian_jog", 10);
    joint_pub_ = create_publisher<cyx_robot_interfaces::msg::JointJog>(
      "/cyx_arm/joint_jog", 10);
  }

  void run()
  {
    TerminalGuard guard;
    std::cout << "CYX keyboard jog: T=switch mode, Space=stop, Q=quit\n";
    print_mode();
    while (rclcpp::ok()) {
      char key{};
      const auto bytes_read = ::read(guard.fd(), &key, 1);
      if (bytes_read < 0 && errno == EINTR) {
        continue;
      }
      if (bytes_read != 1) {
        continue;
      }
      key = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
      if (key == 'q') {
        break;
      }
      if (key == 't') {
        joint_mode_ = !joint_mode_;
        print_mode();
        continue;
      }
      if (joint_mode_) {
        handle_joint_key(key);
        continue;
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
      if (cartesian_pub_->get_subscription_count() == 0) {
        RCLCPP_WARN(get_logger(), "No subscriber on /cyx_arm/cartesian_jog; is demo.launch.py running?");
      }
      cartesian_pub_->publish(msg);
      RCLCPP_INFO(
        get_logger(), "Published key '%c': dx=%.3f dy=%.3f dz=%.3f m",
        key == ' ' ? '_' : key, msg.translation.x, msg.translation.y, msg.translation.z);
    }
  }

private:
  void print_mode() const
  {
    if (joint_mode_) {
      std::cout << "[JOINT] A/D=select joint, W/S=+/-angle, selected=arm_joint" <<
        selected_joint_ + 1 << ", step=" << joint_step_ << " rad\n";
    } else {
      std::cout << "[CARTESIAN] W/S=+/-X, A/D=+/-Y, I/K=+/-Z, step=" <<
        step_ * 1000.0 << " mm, frame=" << frame_ << ", eef=" << eef_ << '\n';
    }
  }

  void handle_joint_key(char key)
  {
    if (key == 'a' || key == 'd') {
      selected_joint_ = (selected_joint_ + (key == 'd' ? 1 : 5)) % 6;
      print_mode();
      return;
    }
    if (key == ' ') {
      cyx_robot_interfaces::msg::CartesianJog stop;
      cartesian_pub_->publish(stop);
      RCLCPP_INFO(get_logger(), "Published stop request");
      return;
    }
    if (key != 'w' && key != 's') {
      return;
    }
    cyx_robot_interfaces::msg::JointJog msg;
    msg.joint_index = selected_joint_;
    msg.delta = key == 'w' ? joint_step_ : -joint_step_;
    if (joint_pub_->get_subscription_count() == 0) {
      RCLCPP_WARN(get_logger(), "No subscriber on /cyx_arm/joint_jog; is demo.launch.py running?");
    }
    joint_pub_->publish(msg);
    RCLCPP_INFO(
      get_logger(), "Published arm_joint%d jog: delta=%+.6f rad",
      selected_joint_ + 1, msg.delta);
  }

  double step_{0.01};
  double joint_step_{10.0};
  double duration_{1.0};
  bool joint_mode_{false};
  int selected_joint_{0};
  std::string frame_;
  std::string eef_;
  rclcpp::Publisher<cyx_robot_interfaces::msg::CartesianJog>::SharedPtr cartesian_pub_;
  rclcpp::Publisher<cyx_robot_interfaces::msg::JointJog>::SharedPtr joint_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardJog>();
  try {
    node->run();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Keyboard input unavailable: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
