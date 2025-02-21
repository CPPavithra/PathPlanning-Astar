#ifndef IMU_H
#define IMU_H
#include <iostream>
#include <stdlib.h>
#include <chrono>

// PID Controller Constants
#define Ki 0.0
#define Kp 1
#define Kd 0.0

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

#endif // IMU_H

