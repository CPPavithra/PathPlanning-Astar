#ifndef IMU_H
#define IMU_H
#include <iostream>
#include <stdlib.h>
#include <chrono>
#include <boost/asio.hpp>
#include <cstring>
#include <string.h>

// PID Controller Constants
#define Ki 0.0
#define Kp 1
#define Kd 0.0

boost::asio::io_service io;
boost::asio::serial_port serial(io);


// Motion and Gyro Structures
struct motion {
    float ax, ay, az;
    float vx, vy, vz;
    float sx, sy, sz;
};

struct gyro {
    float x, y, z;
};

struct drive {
    float linear_x, angular_z;
    int msg;
};

// Constants
const float g = 9.81;
const float alpha = 0.5;
const float alpha_gyro = 0.5;

// Function Declarations
float yaw();
void left();
void right();
void diagonal_forward();
void forward();
void Drive(int dir, float t, int prev_dir);
int PID(int target, int initial);
void initSerial(const std::string& portname, unsigned int baud_rate);
void serializeDrive(const drive &cmd, uint8_t *buffer, size_t buf_size);
void sendcommand(const drive &cmd);


#endif // IMU_H

