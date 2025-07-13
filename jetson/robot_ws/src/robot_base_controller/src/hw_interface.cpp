#include "hw_interface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>                         
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

RobotHWInterface::RobotHWInterface(ros::NodeHandle& nh)
: nh(nh),
  command_timeout_(nh.createTimer(ros::Duration(0.1),
                                  &RobotHWInterface::commandTimeoutCallback,
                                  this, true, false))
{
    cmd_vel_sub = nh.subscribe("cmd_vel", 10, &RobotHWInterface::cmdVelCallback, this);
    velocity_command_pub = nh.advertise<robot_base_controller::velocity_data>("velocity_command", 10);

    imu_orientation = tf::createQuaternionMsgFromYaw(0.0);

    encoder_sub = nh.subscribe("/encoder_data", 10, &RobotHWInterface::encoderCallback, this);
    imu_sub     = nh.subscribe("/imu/data_stable", 10, &RobotHWInterface::imuCallback, this);
    odom_pub    = nh.advertise<nav_msgs::Odometry>("odom", 50);

    nh.getParam("wheel_control/wheel_radius",               wheel_radius);
    nh.getParam("wheel_control/wheel_separation_width",     wheel_separation_width);
    nh.getParam("wheel_control/wheel_separation_length",    wheel_separation_length);
    nh.getParam("wheel_control/deceleration_rate",          deceleration_rate);
    nh.getParam("wheel_control/max_speed",                  max_speed);
    nh.getParam("wheel_control/min_speed",                  min_speed);

    base_geometry = (wheel_separation_width + wheel_separation_length) / 2;

    x = y = th = 0.0;
    vel_linearx = vel_lineary = vel_angular_z = 0.0;

    current_time = ros::Time::now();
}

/* ——————————————————— Callbacks ———————————————————————— */

void RobotHWInterface::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    float vx    = msg->linear.x;
    float vy    = msg->linear.y;
    float omega = msg->angular.z;

    front_left_wheel_speed  = mapSpeed((vx - vy - (omega * base_geometry)) / wheel_radius);
    front_right_wheel_speed = mapSpeed((vx + vy + (omega * base_geometry)) / wheel_radius);
    rear_left_wheel_speed   = mapSpeed((vx + vy - (omega * base_geometry)) / wheel_radius);
    rear_right_wheel_speed  = mapSpeed((vx - vy + (omega * base_geometry)) / wheel_radius);

    command_timeout_.stop();
    command_timeout_.setPeriod(ros::Duration(0.1), true);
    command_timeout_.start();
}

void RobotHWInterface::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_orientation = msg->orientation;
    vel_angular_z   = msg->angular_velocity.z; 
}

void RobotHWInterface::encoderCallback(const robot_base_controller::encoder_data::ConstPtr& msg)
{
    current_time = ros::Time::now();

    vel_linearx = (msg->front_right_encoder_data + msg->front_left_encoder_data +
                   msg->rear_right_encoder_data  + msg->rear_left_encoder_data) * (wheel_radius / 4.0);

    vel_lineary = (msg->front_right_encoder_data - msg->front_left_encoder_data -
                   msg->rear_right_encoder_data  + msg->rear_left_encoder_data) * (wheel_radius / 4.0);

    if (vel_linearx >= max_speed || vel_lineary >= max_speed || vel_linearx <= min_speed || vel_lineary <= min_speed) {
        vel_linearx = 0;
        vel_lineary = 0;
    }

    // ROS_INFO("Velocidades Calculadas -> Vy: %f", vel_lineary);
    // ROS_INFO("Encoder Data -> FR: %f, FL: %f, RR: %f, RL: %f", msg->front_right_encoder_data, msg->front_left_encoder_data, msg->rear_right_encoder_data, msg->rear_left_encoder_data);
}

/* ——————————————— Rotinas auxiliares —————————————————— */

float RobotHWInterface::mapSpeed(float v_input)
{
    return std::min(std::max(v_input, min_speed), max_speed);
}

void RobotHWInterface::publishWheelSpeeds()
{
    robot_base_controller::velocity_data msg;
    msg.front_left_wheel  = front_left_wheel_speed;
    msg.front_right_wheel = front_right_wheel_speed;
    msg.rear_left_wheel   = rear_left_wheel_speed;
    msg.rear_right_wheel  = rear_right_wheel_speed;
    velocity_command_pub.publish(msg);
}

void RobotHWInterface::commandTimeoutCallback(const ros::TimerEvent&)
{
    updateWheelSpeedForDeceleration();
}

void RobotHWInterface::updateWheelSpeedForDeceleration()
{
    auto decel = [this](float& v){
        if (std::abs(v) > deceleration_rate) v -= deceleration_rate * ((v > 0) ? 1 : -1);
        else v = 0;
    };

    decel(front_left_wheel_speed);
    decel(front_right_wheel_speed);
    decel(rear_left_wheel_speed);
    decel(rear_right_wheel_speed);

    if (front_left_wheel_speed || front_right_wheel_speed ||
        rear_left_wheel_speed  || rear_right_wheel_speed)
    {
        command_timeout_.stop();
        command_timeout_.setPeriod(ros::Duration(0.05), true);
        command_timeout_.start();
    }
}

/* ———————————————— Atualização de Odometria ——————————————— */

void RobotHWInterface::updateOdometry()
{
    current_time = ros::Time::now();

    double yaw = tf2::getYaw(imu_orientation);
    th = yaw; 

    /* deslocamento em mapa usando yaw da IMU */
    double delta_x = (vel_linearx * cos(yaw) - vel_lineary * sin(yaw)) * HW_IF_TICK_PERIOD;
    double delta_y = (vel_linearx * sin(yaw) + vel_lineary * cos(yaw)) * HW_IF_TICK_PERIOD;

    x += delta_x;
    y += delta_y;

    if (x > 1) {
        ROS_INFO("Deu erro aqui!");
        ROS_INFO("Encoder Data -> FR: %f, FL: %f, RR: %f, RL: %f", msg->front_right_encoder_data, msg->front_left_encoder_data, msg->rear_right_encoder_data, msg->rear_left_encoder_data);
    }

    /* quaternion somente com yaw */
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);

    /* TF odom → base_link */
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp    = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation      = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp    = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x  = vel_linearx;
    odom.twist.twist.linear.y  = vel_lineary;
    odom.twist.twist.angular.z = vel_angular_z; 
    odom_pub.publish(odom);
}

/* ———————————————————— main ———————————————————————— */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hw_interface");
    ros::NodeHandle nh;

    RobotHWInterface controller(nh);

    ros::Rate rate(HW_IF_UPDATE_FREQ);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok())
    {
        controller.publishWheelSpeeds();
        controller.updateOdometry();
        rate.sleep();
    }
    return 0;
}
