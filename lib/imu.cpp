#include <chrono>
#include "../include/imu.h"
#include <boost/asio.hpp>
#include <iostream>
#include <stdlib.h>
#include <cstring>
#include "cobs.h"
using namespace std;
/*#define Ki 0.0
#define Kp 1
#define Kd 0.0
struct motion{
  float ax, ay, az;
  float vx, vy, vz;
  float sx, sy, sz;
};
struct gyro{
  float x, y, z;
};
struct drive{
  float linear_x, angular_z;
};
const float g = 9.81;
const float alpha = 0.5;
const float alpha_gyro = 0.5;
/*motion low_pass_filter(motion a, motion a_lpf){*/
/*    a_lpf.ax = alpha * a_lpf.ax + (1 - alpha) * a.ax;*/
/*    a_lpf.ay = alpha * a_lpf.ay + (1 - alpha) * a.ay;*/
/*    a_lpf.az = alpha * a_lpf.az + (1 - alpha) * a.az;*/
/*    return (a);*/
/*}*/
/**/
/*gyro low_pass_filter(gyro Gyro_new, gyro Gyro_old){*/
/*  Gyro_old.x = alpha * Gyro_old.x + (1-alpha);*/
/*}*/
/*motion distance(motion R, float dt) */
/*{*/
/*    R.vx += R.ax * dt;*/
/*    R.vy += R.ay * dt;*/
/*    R.vz += R.az * dt;*/
/**/
/*    R.sx += (R.vx * dt) + (0.5 * R.ax * dt * dt);*/
/*    R.sy += (R.vy * dt) + (0.5 * R.ay * dt * dt);*/
/*    R.sz += (R.vz * dt) + (0.5 * R.az * dt * dt);*/
/*    return (R);*/
/*}*/
float yaw();
float rot=0.5;
float speed=0.3;
/*boost::asio::io_service io;
boost::asio::serial_port serial(io);*/ //ALREADY INCLUDED IN HEADER

/*void serializeDrive(const drive &cmd, uint8_t *buffer, size_t buf_size) {
    if (buf_size < sizeof(drive)) return;
    memcpy(buffer, &cmd, sizeof(drive));
}*/

/*********************************************************************
 * TO SET UP THE SERIAL CONNECTION SERIAL PORT &
 * *******************************************************************/

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
/********************************************************************
 * ******************************************************************/

void left()
{
 drive d;
 d.linear_x=0.0f;
 d.angular_z=rot;
 d.msg=0;
 cout<<"Going LEFT with speed: "<<d.linear_x<<" and rotation: "<<d.angular_z<<endl;
 sendcommand(d);
}

void right()
{
 drive d;
 d.linear_x=0.0f;
 d.angular_z=-rot;
 d.msg=0;
  cout<<"Going RIGHT with speed: "<<d.linear_x<<" and rotation: "<<d.angular_z<<endl;
sendcommand(d);
}

void diagonal_forward()
{
drive d;
d.linear_x=speed;
d.angular_z=rot/2;
d.msg=0;
cout<<"Going DIAGONAL with speed: "<<d.linear_x<<" and rotation: "<<d.angular_z<<endl;
sendcommand(d);
} 

void forward()
{
drive d;
d.linear_x=speed;
d.angular_z=0.0f;
d.msg=0;
cout<<"Going DIAGONAL with speed: "<<d.linear_x<<" and rotation: "<<d.angular_z<<endl;
sendcommand(d);
}

void Drive(int dir, float t, int prev_dir)
{
  float current_angle = prev_dir * 45;
  float final_angle = dir*45;

  drive drive;
  if((final_angle - current_angle) < 0)
    for(int i=current_angle ; i!=final_angle ; i-=45)
      left();
  
  else if((final_angle - current_angle) > 0)
    for(int i=current_angle ; i!=final_angle ; i+=45)
      right();

 if(dir%2 == 1) //that is if dir is odd (diagonal)
  diagonal_forward();
 else 
   forward();
}

int PID(int target, int initial)
{
  return 0;
}
