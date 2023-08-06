#include "arduino_interface/arduino_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arduino_interface {

hardware_interface::CallbackReturn ArduinoInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.fl_wheel_name = info_.hardware_parameters["fl_wheel_name"];
    cfg_.fr_wheel_name = info_.hardware_parameters["fr_wheel_name"];
    cfg_.rl_wheel_name = info_.hardware_parameters["rl_wheel_name"];
    cfg_.rr_wheel_name = info_.hardware_parameters["rr_wheel_name"];
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    if (info_.hardware_parameters.count("pid_p") > 0) {
        cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
        cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
        cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "PID values not supplied, using defaults.");
    }

    rl_wheel_.setup(cfg_.fl_wheel_name, cfg_.enc_counts_per_rev);
    rr_wheel_.setup(cfg_.fr_wheel_name, cfg_.enc_counts_per_rev);
    fl_wheel_.setup(cfg_.rl_wheel_name, cfg_.enc_counts_per_rev);
    fr_wheel_.setup(cfg_.rr_wheel_name, cfg_.enc_counts_per_rev);

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        // System has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                rclcpp::get_logger("ArduinoInterface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("ArduinoInterface"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("ArduinoInterface"),
                "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("ArduinoInterface"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                rclcpp::get_logger("ArduinoInterface"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rl_wheel_.name, hardware_interface::HW_IF_POSITION, &rl_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rl_wheel_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rr_wheel_.name, hardware_interface::HW_IF_POSITION, &rr_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rr_wheel_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        fl_wheel_.name, hardware_interface::HW_IF_POSITION, &fl_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        fr_wheel_.name, hardware_interface::HW_IF_POSITION, &fr_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.vel));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        rl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rl_wheel_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        rr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rr_wheel_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd));

    return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Configuring...");

    if (comm_.connected()) {
        comm_.disconnect();
    }

    comm_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Successfully configured.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Cleaning up...");

    if (comm_.connected()) {
        comm_.disconnect();
    }

    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Successfully cleaned up.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Activating...");

    if (!comm_.connected()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (cfg_.pid_p > 0) {
        comm_.set_pid_values(cfg_.pid_p, cfg_.pid_i, cfg_.pid_d);
    }

    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Successfully activated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Deactivating...");

    // Deactivation code goes here if needed

    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Successfully deactivated.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
    if (!comm_.connected()) {
        return hardware_interface::return_type::ERROR;
    }

    comm_.read_encoder_values(fl_wheel_.enc, fr_wheel_.enc, rl_wheel_.enc, rr_wheel_.enc);
    comm_.read_velocity(fl_wheel_.vel, fr_wheel_.vel, rl_wheel_.vel, rr_wheel_.vel);

    fl_wheel_.pos = fl_wheel_.calc_enc_angle();
    fr_wheel_.pos = fr_wheel_.calc_enc_angle();
    rl_wheel_.pos = rl_wheel_.calc_enc_angle();
    rr_wheel_.pos = rr_wheel_.calc_enc_angle();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    if (!comm_.connected()) {
        return hardware_interface::return_type::ERROR;
    }

    // TODO: verify that cmd is actually in radians per second
    comm_.set_motor_values(fl_wheel_.cmd, fr_wheel_.cmd, rl_wheel_.cmd, rr_wheel_.cmd);

    return hardware_interface::return_type::OK;
}

} // namespace arduino_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arduino_interface::ArduinoInterface, hardware_interface::SystemInterface)