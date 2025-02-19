#include <chrono>
#define Ki 0.0
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
  float liner_x, angular_z;
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
void left();
void right();
void diagonal_forward();
void forward();
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
 if(dir == 1) 
  diagonal_forward();
 else forward();
}
int PID(int target, int initial)
{
  return 0;
}
