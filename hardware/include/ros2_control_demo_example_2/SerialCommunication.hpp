#ifndef SERIAL_COMMUNICATION_HPP
#define SERIAL_COMMUNICATION_HPP

#include <libserialport.h>
#include <string>
#include <cstdlib> // For std::atoi
#include <sstream>

class SerialCommunication {
public:
    SerialCommunication() : port(nullptr) {}

    bool connect(const char* portName) {
        sp_get_port_by_name(portName, &port);
        if (sp_open(port, SP_MODE_READ_WRITE) == SP_OK) {
            if (!configure_serial_port()) {
                return false; // Configuration failed, return false
            }
            return true;
        }
        return false;
    }

    void disconnect() {
        if (port) {
            sp_close(port);
            sp_free_port(port);
            port = nullptr;
        }
    }

    bool connected() const {
        return port != nullptr;
    }

    std::string send_msg(const std::string &msg_to_send) {
        if (!port) {
            return "Error: Port not open";
        }

        size_t bytes_written = sp_nonblocking_write(port, msg_to_send.c_str(), msg_to_send.length());

        if (bytes_written == msg_to_send.length()) {
            return "Message sent successfully";
        } else {
            return "Error: Could not send the complete message";
        }
    }

    std::string receive_msg() {
        if (!port) {
            return "Error: Port not open";
        }

        char buffer[256];
        std::string result = "";
        int bytes_read = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0'; // Null-terminate the received data
            result = std::string(buffer);
        }
        return result;
    }


    void read_encoder_values(int &val_1, int &val_2) {
        //clear serial buffer first
        char buffer[128];
        int bytes_read = sp_nonblocking_read(port, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0'; // Null-terminate the received data
            std::string response = std::string(buffer);
        }
        
        std::string response = send_msg("e\r\n"); 

        std::string delimiter = " ";
        size_t del_pos = response.find(delimiter);
        if (del_pos == std::string::npos) {
            val_1 = 0;
            val_2 = 0;
            return;
        }
        std::string token_1 = response.substr(0, del_pos);
        std::string token_2 = response.substr(del_pos + delimiter.length());

        val_1 = std::atoi(token_1.c_str());
        val_2 = std::atoi(token_2.c_str());
    }

    void set_motor_values(int val_1, int val_2) {
        std::stringstream ss;
        ss << "m " << val_1 << " " << val_2 << "\r\n"; 
        send_msg(ss.str());
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o) {
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r\n";
        send_msg(ss.str());
    }

private:
    struct sp_port* port;

    bool configure_serial_port() {
        sp_set_baudrate(port, 57600);
        sp_set_bits(port, 8);
        sp_set_parity(port, SP_PARITY_NONE);
        sp_set_stopbits(port, 1);
        sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE);

        return true; // Add error checking as needed
    }
};

#endif // SERIAL_COMMUNICATION_HPP
