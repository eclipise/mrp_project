// From https://github.com/joshnewans/diffdrive_arduino/tree/humble, modified where marked

#ifndef ARDUINO_COMM_HPP
#define ARDUINO_COMM_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <iostream>
#include <libserial/SerialPort.h>

LibSerial::BaudRate convert_baud_rate(int baud_rate) {
    // Just handle some common baud rates
    switch (baud_rate) {
    case 1200:
        return LibSerial::BaudRate::BAUD_1200;
    case 1800:
        return LibSerial::BaudRate::BAUD_1800;
    case 2400:
        return LibSerial::BaudRate::BAUD_2400;
    case 4800:
        return LibSerial::BaudRate::BAUD_4800;
    case 9600:
        return LibSerial::BaudRate::BAUD_9600;
    case 19200:
        return LibSerial::BaudRate::BAUD_19200;
    case 38400:
        return LibSerial::BaudRate::BAUD_38400;
    case 57600:
        return LibSerial::BaudRate::BAUD_57600;
    case 115200:
        return LibSerial::BaudRate::BAUD_115200;
    case 230400:
        return LibSerial::BaudRate::BAUD_230400;
    default:
        std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
        return LibSerial::BaudRate::BAUD_57600;
    }
}

class ArduinoComm {

  public:
    ArduinoComm() = default;

    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms) {
        timeout_ms_ = timeout_ms;
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    }

    void disconnect() {
        serial_conn_.Close();
    }

    bool connected() const {
        return serial_conn_.IsOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = false) {
        serial_conn_.FlushIOBuffers(); // Just in case
        serial_conn_.Write(msg_to_send);

        std::string response = "";
        try {
            // Responses end with \r\n so we will read up to (and including) the \n.
            serial_conn_.ReadLine(response, '\n', timeout_ms_);
        } catch (const LibSerial::ReadTimeout &) {
            std::cerr << "The ReadByte() call has timed out." << std::endl;
        }

        if (print_output) {
            std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
        }

        return response;
    }

    // function modified
    void send_empty_msg() {
        std::string response = send_msg("\n");
    }

    // function modified
    void read_encoder_values(int &val_1, int &val_2, int &val_3, int &val_4) {
        std::string response = send_msg("e\n");

        std::stringstream ss(response);

        int i = 0;
        long value[4];

        // Reads 4 longs from the response
        while (ss >> value[i]) {
            i++;

            // This should be redundant, but will prevent unexpected values overflowing the array
            if (i >= 4) {
                break;
            }
        }

        val_1 = value[0];
        val_2 = value[1];
        val_3 = value[2];
        val_4 = value[3];
    }

    // function modified
    void set_motor_values(int val_1, int val_2, int val_3, int val_4) {
        std::stringstream ss;
        ss << "v " << val_1 << " " << val_2 << " " << val_3 << " " << val_4 << "\n";
        send_msg(ss.str());
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o) {
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\n";
        send_msg(ss.str());
    }

  private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // ARDUINO_COMM_HPP