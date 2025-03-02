// can_receiver.cpp

#include "can_receiver.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>    // For CAN-specific structures and constants
#include <linux/can/raw.h> // For CAN_RAW and socket operations
#include <sys/socket.h>   // For socket-related functions
#include <net/if.h>       // For ifreq and interface operations
#include <thread>    // For std::thread
#include <chrono>

CanReceiver::CanReceiver(const std::string& can_interface)
    : data_available_(false) {
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

CanReceiver::~CanReceiver() {
    stopListening();
    close(socket_fd_);
}

void CanReceiver::startListening() {
    listening_thread_ = std::thread(&CanReceiver::listenForMessages, this);
}

void CanReceiver::stopListening() {
    if (listening_thread_.joinable()) {
        listening_thread_.join();
    }
}

bool CanReceiver::isDataAvailable() const {
    return data_available_;
}

std::string CanReceiver::getReceivedData(uint32_t id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = cur_pulse_dict_.find(id);
    if (it != cur_pulse_dict_.end()) {
        return it->second;
    }
    return "";
}

void CanReceiver::clearDataAvailable() {
    data_available_ = false;
}

void CanReceiver::listenForMessages() {
    struct can_frame frame;
    while (true) {
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            uint32_t can_id = frame.can_id;
            std::string data_receive = std::to_string(can_id) + "#";
            for (int i = 0; i < 8; ++i) {
                data_receive += (frame.data[i] < 0x10 ? "0" : "") + std::to_string(frame.data[i]);
            }
            cur_pulse_dict_[can_id] = data_receive;
            data_available_ = true;
        }
    }
}
