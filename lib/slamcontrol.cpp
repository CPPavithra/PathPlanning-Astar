#include <chrono>
#include "../include/slamcontrol.h"
#include <boost/asio.hpp>
#include <iostream>
#include <stdlib.h>
#include <cstring>
#include <thread>
#include <chrono>
#include "cobs.h"
using namespace std;
float yaw();

//float rot=-2.5;
//float speed=-0.5;
float rot=2.5;
float speed=-0.3;

boost::asio::io_service io;
boost::asio::serial_port serial(io);// DEFINITION

void initSerial(const string& portname, unsigned int baud_rate) {
    try {
        serial.open(portname);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        cout << "Serial port configured successfully!" << endl;
    } catch (const std::exception &e) {
        cerr << "Error configuring serial port: " << e.what() << endl;
        exit(1);
    }
}

void serializeDrive(const drive &cmd, uint8_t *buffer, size_t buf_size) {
    if (buf_size < sizeof(drive)) return;
    memcpy(buffer, &cmd, sizeof(drive));
}

// Function to send command via serial port
void sendcommand(const drive &cmd) {
    uint8_t raw_data[sizeof(drive)];
    serializeDrive(cmd, raw_data, sizeof(raw_data));

    uint8_t encoded_data[sizeof(raw_data) + 2]; // COBS overhead
    cobs_encode_result result = cobs_encode(encoded_data, sizeof(encoded_data), raw_data, sizeof(raw_data));

    if (result.status == COBS_ENCODE_OK) {
        encoded_data[result.out_len] = 0x00; // Append COBS delimiter (0x00)
        
        boost::system::error_code ec;  // If using Boost
        boost::asio::write(serial, boost::asio::buffer(encoded_data, result.out_len + 1), ec); // Write to serial

        if (ec) {
            cerr<<"Error writing to serial: " << ec.message() << endl;
        } else {
            cout<<"Command sent successfully!" << endl;
        }
    } else {
        cerr<<"COBS encoding failed!" << endl;
    }
}


/*void move(float goal_x, float goal_y, float goal_theta) {
    float current_x, current_y, current_theta;
    getCurrentPose(&current_x, &current_y, &current_theta); //pose from stella

    float angle_diff=goal_theta-current_theta;
    while (angle_diff > 180) 
    {
      angle_diff -= 360;
    }
    while (angle_diff < -180)
    {
      angle_diff += 360;
    }

    drive d;
    if (angle_diff > 10) {
        d.linear_x = 0.0f;
        d.angular_z = 1.5f; // Rotate right
    } 
    else if (angle_diff < -10) {
        d.linear_x = 0.0f;
        d.angular_z = -1.5f; // Rotate left
    } 
    else {
        d.linear_x = 0.5f;  // Move forward
        d.angular_z = 0.0f;
    }
    
    sendcommand(d);
}*/

/***********************************************************************************************
 * DO NOT KNOW IF THIS IS CORRECT
 * **********************************************************************************************/

struct RealsenseHandle {
  rs2::frame_queue frame_q;
  rs2::pointcloud pc;
  rs2::pipeline pipe;
  rs2::align align;
  stella_vslam::system slam;
  stella_vslam::config slam_cfg;
};

void getCurrentPose(float* x, float* y, float* theta, RealsenseHandle* handle) {
    auto result = runLocalization(handle, nullptr);
    if (std::holds_alternative<Error>(result)) {
        std::cerr << "Localization Error!" << std::endl;
        return;
    }

    Eigen::Matrix<double, 4, 4> inverse_pose = std::get<Eigen::Matrix<double, 4, 4>>(result);

    *x = inverse_pose(0, 3);  // X position
    *y = inverse_pose(1, 3);  // Y position
    *theta = yawFromPose(inverse_pose);  // Extract theta (yaw)
}

void move(float goal_x, float goal_y, float goal_theta, RealsenseHandle* handle) {
    float current_x, current_y, current_theta;
    getCurrentPose(&current_x, &current_y, &current_theta, handle); // Get pose from Stella SLAM

    float angle_diff = goal_theta - current_theta;

    // Normalize angle (-180 to 180 degrees)
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    drive d;
    if (abs(angle_diff) > 0.1) {  // Turn if not aligned
        d.linear_x = 0.0f;
        d.angular_z = (angle_diff > 0) ? 1.0f : -1.0f;  // Turn left/right
    } 
    else {
        d.linear_x = 0.3f;  // Move forward
        d.angular_z = 0.0f;
    }

    sendcommand(d);  // Send motion command
}

