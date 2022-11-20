#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 64

int main(int argc, char **argv) {

  wb_robot_init();
  
  int i;

     WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }

   
     WbDeviceTag motors[4];
  char motors_names[4][8] = {"motor_1", "motor_2", "motor_3", "motor_4"};
  for (int i = 0; i < 4; i++) {
    motors[i] = wb_robot_get_device(motors_names[i]);
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i],0);
  }
  
   double left_speed = 1.0;
   double right_speed = 1.0;

  while (wb_robot_step(TIME_STEP) != -1) {

    float  lf_wheel = wb_distance_sensor_get_value(ds[0]);
    float  rt_wheel = wb_distance_sensor_get_value(ds[1]);
    printf("left:%f,right:%f\n",lf_wheel,rt_wheel);
  
    if((lf_wheel <= 60)&&(rt_wheel <= 60))
    {
      left_speed = 1.0;
      right_speed = 1.0;
    }
    else if((lf_wheel <=60)&&(rt_wheel > 60))
    {
      left_speed = 1.0;
      right_speed = -0.4;
    }
    else if((lf_wheel > 60)&&(rt_wheel <= 60))
    {
      left_speed = -0.4;
      right_speed = 1.0;
    }

    wb_motor_set_velocity(motors[0],left_speed);
    wb_motor_set_velocity(motors[1],right_speed);
    wb_motor_set_velocity(motors[2],left_speed);
    wb_motor_set_velocity(motors[3],right_speed);

  };

  wb_robot_cleanup();

  return 0;
}
