#include "can_handler.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <iomanip> // For std::hex and std::setw

CanHandler::CanHandler(const std::string& can_interface)
    : data_available_(false) {
    // Initialize socket and interface
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    std::strncpy(ifr_.ifr_name, can_interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr_) < 0) {
        perror("IOCTL failed");
        close(socket_fd_);
        exit(EXIT_FAILURE);
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0) {
        perror("Bind failed");
        close(socket_fd_);
        exit(EXIT_FAILURE);
    }
}

CanHandler::~CanHandler() {
    stopListening();
    close(socket_fd_);
}

void CanHandler::startListening() {
    listening_thread_ = std::thread(&CanHandler::listenForMessages, this);
}

void CanHandler::stopListening() {
    if (listening_thread_.joinable()) {
        listening_thread_.join();
    }
}

bool CanHandler::isDataAvailable() const {
    return data_available_;
}

std::string CanHandler::getReceivedData(uint32_t id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = cur_pulse_dict_.find(id);
    if (it != cur_pulse_dict_.end()) {
        return it->second;
    }
    return "";
}

void CanHandler::clearDataAvailable() {
    data_available_ = false;
}

void CanHandler::listenForMessages() {
    struct can_frame frame;
    while (true) {
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            uint32_t can_id = frame.can_id;
            std::ostringstream data_stream;
            data_stream << "0x" << std::hex << std::setw(2) << std::setfill('0') << (int)frame.data[0];
            for (int i = 1; i < 8; ++i) {
                data_stream << " " << std::hex << std::setw(2) << std::setfill('0') << (int)frame.data[i];
            }
            cur_pulse_dict_[can_id] = data_stream.str();
            data_available_ = true;
        }
    }
}

void CanHandler::sendCanMessage(uint32_t id, uint8_t command_word, uint16_t index, uint32_t data) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;  // Set the data length to 8 bytes (CAN standard)

    // Fill the frame data according to the format
    frame.data[0] = command_word;  // Set command word
    frame.data[1] = index & 0xFF;  // Index low byte
    frame.data[2] = (index >> 8) & 0xFF;  // Index high byte
    frame.data[3] = 0x00;  // Reserved byte
    frame.data[4] = data & 0xFF;  // Data low byte
    frame.data[5] = (data >> 8) & 0xFF;  // Data byte 2
    frame.data[6] = (data >> 16) & 0xFF;  // Data byte 3
    frame.data[7] = (data >> 24) & 0xFF;  // Data high byte

    // Send the CAN frame
    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        std::cerr << "Error sending CAN message" << std::endl;
    }
}
