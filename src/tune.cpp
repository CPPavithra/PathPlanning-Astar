#include <iostream> 
#include <imu.h> 
#include <chrono>
#include <thread>
int main()
{
   initSerial("/dev/ttyACM0", 9600);
   drive d; 
   d.linear_x = 0.0;
   d.angular_z = -2.5; 
   d.msg = 0; 
   sendcommand(d);
   std::this_thread::sleep_for(std::chrono::milliseconds(int(1890)));
   d.linear_x=0.0;
   d.angular_z=0.0;
   d.msg=0;
   sendcommand(d);

}
