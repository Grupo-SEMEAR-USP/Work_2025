#ifndef ROBOT_HW_INTERFACE_HPP
#define ROBOT_HW_INTERFACE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "robot_base_controller/velocity_data.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include "robot_base_controller/encoder_data.h" 
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#define HW_IF_UPDATE_FREQ 10
#define HW_IF_TICK_PERIOD 1 / HW_IF_UPDATE_FREQ

class RobotHWInterface {
public:
    RobotHWInterface(ros::NodeHandle& nh); // Ajustado para receber NodeHandle por referência
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void publishWheelSpeeds(); // Publicando velocidades do cmd_vel
    void commandTimeoutCallback(const ros::TimerEvent&); // Callback para o timeout
    void updateWheelSpeedForDeceleration(); // Desaceleração
    float mapSpeed(float v_input); // Normalização da velocidade
    void encoderCallback(const robot_base_controller::encoder_data::ConstPtr& msg); // Callback para os dados do encoder
    void updateOdometry();

    int teste = 0;

private:
    ros::NodeHandle nh;

    // Hw Interface 
    ros::Publisher velocity_command_pub;
    ros::Subscriber cmd_vel_sub;

    // IMU Data
    ros::Subscriber imu_sub;
    geometry_msgs::Quaternion imu_orientation;

    // Odometria
    ros::Subscriber encoder_sub;
    ros::Publisher odom_pub;

    // Última orientação do robô (quaternion)
    geometry_msgs::Quaternion last_orientation;

    // Posição atual do robô
    geometry_msgs::Point current_position;

    // Temporizador para o timeout de comandos
    ros::Timer command_timeout_; 

    // Variáveis membro para armazenar as velocidades das rodas
    float front_left_wheel_speed = 0.0;
    float front_right_wheel_speed = 0.0;
    float rear_left_wheel_speed = 0.0;
    float rear_right_wheel_speed = 0.0;

    // Parâmetros carregados do arquivo .yaml
    float wheel_radius; // Raio das rodas
    float base_geometry; // Geometria da base
    float wheel_separation_width;
    float wheel_separation_length;
    float deceleration_rate; // Taxa de desaceleração
    float max_speed; // Velocidade máxima
    float min_speed; // Velocidade mínima

    // Parâmetros para a odometria 

    double x;
    double y;
    double th;

    double vel_linearx;
    double vel_lineary;
    double vel_angular_z;

    ros::Time current_time;
    tf::TransformBroadcaster odom_broadcaster;
};

#endif // ROBOT_HW_INTERFACE_HPP
