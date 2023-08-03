#ifndef ARDUINO_INTERFACE_HPP
#define ARDUINO_INTERFACE_HPP

#include "arduino_comm.hpp"
#include "hardware_interface/system_interface.hpp"
#include "wheel.hpp"

namespace arduino_interface {

class ArduinoInterface : public hardware_interface::SystemInterface {
    struct Config {
        std::string fl_wheel_name = "";
        std::string fr_wheel_name = "";
        std::string rl_wheel_name = "";
        std::string rr_wheel_name = "";
        float loop_rate = 0.0;
        std::string device = "";
        int baud_rate = 0;
        int timeout_ms = 0;
        int enc_counts_per_rev = 0;
        int pid_p = 0;
        int pid_d = 0;
        int pid_i = 0;
        int pid_o = 0;
    };

  public:
    ArduinoInterface() {}

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    ArduinoComm comm_;
    Config cfg_;
    Wheel rl_wheel_, rr_wheel_, fl_wheel_, fr_wheel_;
};

} // namespace arduino_interface

#endif // ARDUINO_INTERFACE_HPP