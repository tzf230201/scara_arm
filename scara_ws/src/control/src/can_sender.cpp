#include "can_sender.h"
#include <iostream>
#include <cstring>  // For std::memcpy
#include <unistd.h> // For write
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>


CanSender::CanSender(const std::string& interface) {
    // Create a CAN socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "Error opening socket" << std::endl;
        return;
    }

    // Set up the CAN interface
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error in ioctl" << std::endl;
        return;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error in bind" << std::endl;
        return;
    }
}

void CanSender::sendCanMessage(uint32_t id, uint8_t command_word, uint16_t index, uint32_t data) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;  // Set the data length to 8 bytes (CAN standard)

    // Fill the frame data according to the format
    frame.data[0] = command_word;  // Set length
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
