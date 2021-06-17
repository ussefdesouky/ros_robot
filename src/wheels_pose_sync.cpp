#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros_robot/WheelData.h>
#include <ros_robot/WheelCMD.h>

double sensed_output, control_signal;
double setpoint;
double Kp = 0.1; //proportional gain
double Ki = 0; //integral gain
double Kd = 0; //derivative gain
int T = 10; //sample time in milliseconds (ms)
double total_error, last_error;
int max_control = 250;
int min_control = -250; 
volatile long encoder_pos = 0;
double npr = 6000;
double wheel_radius = 0.335;
double pi = 3.14159265358979323846;
double encoder_pulses = 0.0;
double wheel_rps = 0.0;
double wheel_vel = 0.0;
double cmd_vel = 0;
ros::Time current_time;
ros::Time last_time;
double dt = 0.0;

void PID_Control(){
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    if (dt >= T){
    double error = setpoint - sensed_output;
    
    total_error += error; //accumalates the error - integral term
    if (total_error >= max_control){
        total_error = max_control;
    }
    else if (total_error <= min_control){
        total_error = min_control;
    }
    double delta_error = error - last_error; //difference of error for derivative term

    control_signal = Kp*error + (Ki*T)*total_error + (Kd/T)*delta_error; //PID control compute
    if (control_signal >= max_control){
        control_signal = max_control;
    } 
    else if (control_signal <= min_control){
        control_signal = min_control;
    } 
    last_error = error;
    last_time = current_time;
    }  
}

void encoder_Callback(const ros_robot::WheelData &wheel_data)
{
    encoder_pos = wheel_data.left_wheel_position;
    encoder_pulses = wheel_data.left_wheel_velocity;  	
    wheel_rps =  encoder_pulses/npr; 
	wheel_vel = 2 * pi * wheel_radius * wheel_rps; 
}

void velocity_Callback(const geometry_msgs::Twist &vel_msg)
{
    cmd_vel = vel_msg.linear.x;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "left_wheel_pid");
	ros::NodeHandle n;
	ros::Subscriber encoder_sub = n.subscribe("/encoder", 1000, encoder_Callback);
    ros::Subscriber velocity_sub = n.subscribe("/cmd_vel", 1000, velocity_Callback);
	ros::Publisher pwm_pub = n.advertise<ros_robot::WheelCMD>("lw_pwm", 1000);
    ros_robot::WheelCMD wheel_signal;
    ros::Rate loop_rate(1000);
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    sensed_output = encoder_pos;
    setpoint = 60000;
    while(ros::ok()){
        PID_Control(); 
        if (control_signal >= 0){
            wheel_signal.wheel_dir = 1;
            wheel_signal.wheel_pwm = abs(control_signal);
            pwm_pub.publish(wheel_signal);
        }
        else if (control_signal < 0){
            wheel_signal.wheel_dir = 0;
            wheel_signal.wheel_pwm = abs(control_signal);
            pwm_pub.publish(wheel_signal);
        }
        ros::spinOnce();
    }
}
