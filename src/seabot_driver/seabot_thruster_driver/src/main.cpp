#include <iostream>
#include <unistd.h>

#include <ros/ros.h>

#include "thruster.h"
#include "seabot_thruster_driver/Engine.h"
#include "seabot_thruster_driver/Velocity.h"
#include "geometry_msgs/Twist.h"

#include "std_srvs/SetBool.h"

using namespace std;
float linear_velocity = 0.0;
float angular_velocity = 0.0;
bool state_enable = true;

float coeff_cmd_to_pwm = 9.0;

Thruster t;
ros::Time time_last_cmd;
bool stop_sent = false;
bool send_cmd = true;

float manual_linear_velocity = 0.0;
float manual_angular_velocity = 0.0;
ros::Time manual_time_last_cmd;

void velocity_callback(const seabot_thruster_driver::Velocity::ConstPtr& msg){
  linear_velocity = msg->linear;
  angular_velocity = msg->angular;
  time_last_cmd = ros::Time::now();
}

void manual_velocity_callback(const geometry_msgs::Twist::ConstPtr& msg){
  manual_linear_velocity = msg->linear.x;
  manual_angular_velocity = msg->angular.z;
  manual_time_last_cmd = ros::Time::now();
}

bool engine_enable(std_srvs::SetBool::Request  &req,
                   std_srvs::SetBool::Response &res){
  state_enable = req.data;
  res.success = true;
  t.write_cmd(MOTOR_PWM_STOP, MOTOR_PWM_STOP);
  stop_sent = true;

  return true;
}

uint8_t convert_u(const float &u){
  uint8_t cmd = round(u*coeff_cmd_to_pwm + MOTOR_PWM_STOP);

  if(cmd>MAX_PWM)
    cmd = MAX_PWM;
  if(cmd<MIN_PWM)
    cmd = MIN_PWM;

  return cmd;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "thruster_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  const double frequency = n_private.param<double>("frequency", 10.0);
  coeff_cmd_to_pwm = n_private.param<float>("coeff_cmd_to_pwm", 9.0);
  const double delay_stop = n_private.param<float>("delay_stop", 0.5);

  // Subscriber
  ros::Subscriber velocity_sub = n.subscribe("cmd_engine", 1, velocity_callback);
  ros::Subscriber manual_velocity_sub = n.subscribe("/cmd_vel", 1, manual_velocity_callback);

  // Publisher
  ros::Publisher cmd_pub = n.advertise<seabot_thruster_driver::Engine>("engine", 1);
  seabot_thruster_driver::Engine cmd_msg;

  // Service (ON/OFF)
  ros::ServiceServer service = n.advertiseService("engine_enable", engine_enable);

  // Sensor initialization
  t.i2c_open();

  if(t.get_version()!=0x01){
    ROS_WARN("[Thruster] Wrong PIC code version");
  }

  time_last_cmd = ros::Time::now();
  manual_time_last_cmd = ros::Time::now();
  ros::Duration(delay_stop*1.1).sleep();

  ROS_INFO("[Thruster] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok()){
    ros::spinOnce();

    if(state_enable){
      uint8_t cmd_left, cmd_right;

      if((ros::Time::now() - manual_time_last_cmd).toSec() < delay_stop){
        cmd_left = convert_u(manual_linear_velocity + manual_angular_velocity);
        cmd_right = convert_u(manual_linear_velocity - manual_angular_velocity);
      }
      else if((ros::Time::now() - time_last_cmd).toSec() < delay_stop){
        cmd_left = convert_u(linear_velocity + angular_velocity);
        cmd_right = convert_u(linear_velocity - angular_velocity);
      }
      else{
        cmd_right = MOTOR_PWM_STOP;
        cmd_left = MOTOR_PWM_STOP;
      }

      if(cmd_right != MOTOR_PWM_STOP && cmd_left != MOTOR_PWM_STOP){
        send_cmd = true;
        stop_sent = false;
      }
      else{
        if(!stop_sent){
          send_cmd = true;
          stop_sent = true;
        }
        else
          send_cmd = false;
      }

      if(send_cmd){
        t.write_cmd(cmd_left, cmd_right);

        // Publish cmd send for loggin
        cmd_msg.left = (float)cmd_left;
        cmd_msg.right = cmd_right;
        cmd_pub.publish(cmd_msg);
      }
    }

    loop_rate.sleep();
  }

  return 0;
}
