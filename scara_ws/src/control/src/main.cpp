#include "ros/ros.h"
#include "can_handler.h"
#include <iostream>
#include <thread>
#include <chrono>

#define SET_1_BYTE 0x2F
#define SET_2_BYTE 0x2B
#define SET_3_BYTE 0x27
#define SET_4_BYTE 0x23
#define SET_OK 0x60
#define READ_1_BYTE 0x4F
#define READ_2_BYTE 0x4B
#define READ_3_BYTE 0x47
#define READ_4_BYTE 0x43
#define SET_ERROR 0x80

#define INDEX_POSITION_KP 0x204B
#define INDEX_ERROR_CODE 0x603F
#define INDEX_CONTROL_WORD 0x6040
#define INDEX_STATUS_WORD 0x6041
#define INDEX_MODE_OF_OPERATION 0x6060
#define INDEX_POSITION_ACTUAL_VALUE 0x6064
#define INDEX_VELOCITY_ACTUAL_VALUE 0x606C
#define INDEX_TARGET_POSITION 0x607A
#define INDEX_PROFILE_VELOCITY 0x6081
#define INDEX_END_VELOCITY 0x6082
#define INDEX_PROFILE_ACCELERATION 0x6083
#define INDEX_PROFILE_DECELERATION 0x6084
#define INDEX_TARGET_VELOCITY 0x60FF

#define ID2 0x602
#define ID3 0x603
#define ID4 0x604

int main() {
    // Initialize the CAN handler with the desired CAN interface
    CanHandler can_handler("can0");
    int id = ID2;
    
    // Start listening for CAN messages
    can_handler.startListening();
    auto start_time = std::chrono::high_resolution_clock::now();
    // Send a CAN message
    can_handler.sendCanMessage(id, SET_2_BYTE, INDEX_CONTROL_WORD, 0x00);

    // Capture start time
    

    // Wait until data is available
   // while (!can_handler.isDataAvailable()) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Prevent busy-waiting
    //}
	  // Clear the data available flag
    //can_handler.clearDataAvailable();
    // Capture end time
    

    // Retrieve and print the received data
    //std::string received_data = can_handler.getReceivedData(id - 0x80);
    //std::cout << "Received CAN ID: " << std::hex << id << std::endl;
    //std::cout << "Received data (hex): " << received_data << std::endl;
    //std::cerr << "Received data (debug): " << received_data << std::endl;
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    // Print the time taken to receive the data
    std::cerr << "Time taken to receive data: " << elapsed.count() << " seconds" << std::endl;

  

    // Stop listening and clean up
    can_handler.stopListening();

    return 0;
}
