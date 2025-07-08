#include "hw_interface.hpp"

RobotHWInterface::RobotHWInterface(ros::NodeHandle& nh) : nh(nh), command_timeout_(nh.createTimer(ros::Duration(0.1), &RobotHWInterface::commandTimeoutCallback, this, true, false)) {
    // Subscreve ao tópico de comandos de movimento
    cmd_vel_sub = nh.subscribe("cmd_vel", 10, &RobotHWInterface::cmdVelCallback, this);

    // Publicador para o comando de velocidade das rodas
    velocity_command_pub = nh.advertise<robot_base_controller::velocity_data>("velocity_command", 10);

    encoder_sub = nh.subscribe("/encoder_data", 10, &RobotHWInterface::encoderCallback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    // Carregar parâmetros do arquivo .yaml
    nh.getParam("wheel_control/wheel_radius", wheel_radius);
    nh.getParam("wheel_control/wheel_separation_width", wheel_separation_width);
    nh.getParam("wheel_control/wheel_separation_length", wheel_separation_length);
    nh.getParam("wheel_control/deceleration_rate", deceleration_rate);
    nh.getParam("wheel_control/max_speed", max_speed);
    nh.getParam("wheel_control/min_speed", min_speed);

    base_geometry = (wheel_separation_width + wheel_separation_length) / 2;

    x = 0;
    y = 0;
    th = 0;

    vel_linearx = 0;
    vel_lineary = 0;
    vel_angular_z = 0;

    current_time = ros::Time::now();
}

void RobotHWInterface::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float omega = msg->angular.z;

    front_left_wheel_speed = mapSpeed((vx - vy - (omega * base_geometry)) * (1 / wheel_radius));
    front_right_wheel_speed = mapSpeed((vx + vy + (omega * base_geometry)) * (1 / wheel_radius));
    rear_left_wheel_speed = mapSpeed((vx + vy - (omega * base_geometry)) * (1 / wheel_radius));
    rear_right_wheel_speed = mapSpeed((vx - vy + (omega * base_geometry)) * (1 / wheel_radius));

    // Reinicia o temporizador cada vez que um comando é recebido
    command_timeout_.stop();
    command_timeout_.setPeriod(ros::Duration(0.1), true); // Reset com auto-restart
    command_timeout_.start();
}

float RobotHWInterface::mapSpeed(float v_input) {
    return std::min(std::max(v_input, min_speed), max_speed);
}

void RobotHWInterface::publishWheelSpeeds() {
    robot_base_controller::velocity_data msg;

    msg.front_left_wheel = front_left_wheel_speed;
    msg.front_right_wheel = front_right_wheel_speed;
    msg.rear_left_wheel = rear_left_wheel_speed;
    msg.rear_right_wheel = rear_right_wheel_speed;

    velocity_command_pub.publish(msg);
}

void RobotHWInterface::commandTimeoutCallback(const ros::TimerEvent&) {
    updateWheelSpeedForDeceleration();
}

void RobotHWInterface::updateWheelSpeedForDeceleration() {
    // Desacelera cada roda gradualmente até zero
    if (std::abs(front_left_wheel_speed) > deceleration_rate) front_left_wheel_speed -= deceleration_rate * (front_left_wheel_speed > 0 ? 1 : -1);
    else front_left_wheel_speed = 0;

    if (std::abs(front_right_wheel_speed) > deceleration_rate) front_right_wheel_speed -= deceleration_rate * (front_right_wheel_speed > 0 ? 1 : -1);
    else front_right_wheel_speed = 0;

    if (std::abs(rear_left_wheel_speed) > deceleration_rate) rear_left_wheel_speed -= deceleration_rate * (rear_left_wheel_speed > 0 ? 1 : -1);
    else rear_left_wheel_speed = 0;

    if (std::abs(rear_right_wheel_speed) > deceleration_rate) rear_right_wheel_speed -= deceleration_rate * (rear_right_wheel_speed > 0 ? 1 : -1);
    else rear_right_wheel_speed = 0;

    // Verifica se todas as velocidades chegaram a zero, se não, continua desacelerando
    if (front_left_wheel_speed != 0 || front_right_wheel_speed != 0 || rear_left_wheel_speed != 0 || rear_right_wheel_speed != 0) {
        command_timeout_.stop();
        command_timeout_.setPeriod(ros::Duration(0.05), true); // Use um intervalo mais curto para desaceleração suave
        command_timeout_.start();
    }
}

void RobotHWInterface::encoderCallback(const robot_base_controller::encoder_data::ConstPtr& msg) {

    current_time = ros::Time::now();

    vel_linearx = (msg->front_right_encoder_data + msg->front_left_encoder_data + msg->rear_right_encoder_data + msg->rear_left_encoder_data) * (wheel_radius / 4.0);
    vel_lineary = (msg->front_right_encoder_data - msg->front_left_encoder_data - msg->rear_right_encoder_data + msg->rear_left_encoder_data) * (wheel_radius / 4.0);
    vel_angular_z = (msg->front_right_encoder_data - msg->front_left_encoder_data + msg->rear_right_encoder_data - msg->rear_left_encoder_data) * (wheel_radius / (4.0 * base_geometry));

    //std::cout << "Velocidade Linear X: " << vel_linearx << ", Velocidade Linear Y: " << vel_lineary << ", Velocidade Angular Z: " << vel_angular_z << std::endl;

    //std::cout << "Teste Encoder: " << teste << "\n";

}

void RobotHWInterface::updateOdometry() {

    current_time = ros::Time::now();

    double delta_x = (vel_linearx * cos(th) - vel_lineary * sin(th)) * HW_IF_TICK_PERIOD;
    double delta_y = (vel_linearx * sin(th) + vel_lineary * cos(th)) * HW_IF_TICK_PERIOD;
    double delta_th = vel_angular_z * HW_IF_TICK_PERIOD;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vel_linearx;
    odom.twist.twist.linear.y = vel_lineary;
    odom.twist.twist.angular.z = vel_angular_z;

    odom_pub.publish(odom);

    //std::cout << "Teste Odometry: " << teste << "\n";

}

int main(int argc, char** argv) {
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
