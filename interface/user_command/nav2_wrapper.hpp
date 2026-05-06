#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "user_command_interface.h"
#include "robot_interface.h"
#include "custom_types.h"

#include <mutex>
#include <thread>
#include <termios.h>
#include <unistd.h>

namespace interface {

class Nav2Wrapper : public UserCommandInterface {
public:
    Nav2Wrapper(std::shared_ptr<RobotInterface> ri) : ri_(ri) {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = rclcpp::Node::make_shared("nav2_bridge");
        
        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&Nav2Wrapper::CmdVelCallback, this, std::placeholders::_1));
        
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

        std::memset(&cmd_, 0, sizeof(cmd_));
        std::cout << "[Nav2Wrapper] Initialized with Keyboard Support. Listening on /cmd_vel" << std::endl;
    }

    ~Nav2Wrapper() {
        Stop();
    }

    void Start() override {
        running_ = true;
        ros_thread_ = std::thread(&Nav2Wrapper::RosSpin, this);
        kb_thread_ = std::thread(&Nav2Wrapper::KeyboardLoop, this);
    }

    void Stop() override {
        running_ = false;
        if (ros_thread_.joinable()) ros_thread_.join();
        if (kb_thread_.joinable()) kb_thread_.join();
    }

    UserCommand GetUserCommand() override {
        std::lock_guard<std::mutex> lock(mtx_);
        return cmd_;
    }

    void SetMotionStateFeedback(const MotionStateFeedback& msfb) override {
        std::lock_guard<std::mutex> lock(mtx_);
        msfb_ = msfb;
        if (cmd_.target_mode == 0) {
             cmd_.target_mode = msfb.current_state;
        }
    }

    void PublishData() {
        if (!ri_) return;
        auto now = node_->get_clock()->now();

        // 1. Publish IMU
        auto rpy = ri_->GetImuRpy();
        auto omg = ri_->GetImuOmega();
        auto acc = ri_->GetImuAcc();

        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = "base_link";
        tf2::Quaternion q;
        q.setRPY(rpy.x(), rpy.y(), rpy.z());
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
        imu_msg.angular_velocity.x = omg.x();
        imu_msg.angular_velocity.y = omg.y();
        imu_msg.angular_velocity.z = omg.z();
        imu_msg.linear_acceleration.x = acc.x();
        imu_msg.linear_acceleration.y = acc.y();
        imu_msg.linear_acceleration.z = acc.z();
        imu_pub_->publish(imu_msg);

        // 2. Publish Odometry (Open Loop)
        static double last_ts = 0;
        double dt = (last_ts == 0) ? 0 : (now.seconds() - last_ts);
        last_ts = now.seconds();
        if (dt > 0.1) dt = 0;

        static double x = 0, y = 0, yaw = 0;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            double vx = cmd_.forward_vel_scale * 0.8;
            double vy = cmd_.side_vel_scale * 0.8;
            double wz = cmd_.turnning_vel_scale * 0.8;
            
            yaw += wz * dt;
            x += (vx * cos(yaw) - vy * sin(yaw)) * dt;
            y += (vx * sin(yaw) + vy * cos(yaw)) * dt;
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        
        tf2::Quaternion q_odom;
        q_odom.setRPY(rpy.x(), rpy.y(), yaw);
        odom_msg.pose.pose.orientation.x = q_odom.x();
        odom_msg.pose.pose.orientation.y = q_odom.y();
        odom_msg.pose.pose.orientation.z = q_odom.z();
        odom_msg.pose.pose.orientation.w = q_odom.w();
        odom_pub_->publish(odom_msg);

        // 3. Broadcast TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }

private:
    void RosSpin() {
        rclcpp::spin(node_);
    }

    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!nav2_enabled_) return;

        // Scale ROS2 velocity to policy range [-1, 1]
        cmd_.forward_vel_scale = msg->linear.x / 0.8f;
        cmd_.side_vel_scale = msg->linear.y / 0.8f;
        cmd_.turnning_vel_scale = msg->angular.z / 0.8f;
        
        if (msfb_.current_state == (uint8_t)types::RobotMotionState::StandingUp) {
            cmd_.target_mode = (uint8_t)types::RobotMotionState::RLControlMode;
        }
    }

    void KeyboardLoop() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        char input;
        while (running_) {
            // Set stdin to non-blocking
            struct timeval tv = {0L, 10000L}; // 10ms timeout
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            
            if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
                if (read(STDIN_FILENO, &input, 1) != -1) {
                    std::lock_guard<std::mutex> lock(mtx_);
                    if (input == 'r') cmd_.target_mode = (uint8_t)types::RobotMotionState::JointDamping;
                    if (input == 'z' && msfb_.current_state == (uint8_t)types::RobotMotionState::WaitingForStand) 
                        cmd_.target_mode = (uint8_t)types::RobotMotionState::StandingUp;
                    if (input == 'c' && msfb_.current_state == (uint8_t)types::RobotMotionState::StandingUp)
                        cmd_.target_mode = (uint8_t)types::RobotMotionState::RLControlMode;

                    if (input == 'n') {
                        nav2_enabled_ = !nav2_enabled_;
                        std::cout << "[Nav2Wrapper] Autonomous Navigation: " << (nav2_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
                        if (!nav2_enabled_) {
                            cmd_.forward_vel_scale = 0;
                            cmd_.side_vel_scale = 0;
                            cmd_.turnning_vel_scale = 0;
                        }
                    }

                    if (msfb_.current_state == (uint8_t)types::RobotMotionState::RLControlMode) {
                        // Manual overrides always work and disable Nav2 mode
                        if (input == 'w' || input == 'a' || input == 's' || input == 'd' || input == 'q' || input == 'e') {
                            if (nav2_enabled_) {
                                nav2_enabled_ = false;
                                std::cout << "[Nav2Wrapper] Manual Override: Nav2 DISABLED" << std::endl;
                            }
                        }

                        if (input == 'w') cmd_.forward_vel_scale += 0.1f;
                        else if (input == 's') cmd_.forward_vel_scale -= 0.1f;
                        else if (input == 'a') cmd_.side_vel_scale += 0.1f;
                        else if (input == 'd') cmd_.side_vel_scale -= 0.1f;
                        else if (input == 'q') cmd_.turnning_vel_scale += 0.1f;
                        else if (input == 'e') cmd_.turnning_vel_scale -= 0.1f;
                        else if (input == ' ') {
                            cmd_.forward_vel_scale = 0;
                            cmd_.side_vel_scale = 0;
                            cmd_.turnning_vel_scale = 0;
                        }
                    }
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    std::shared_ptr<RobotInterface> ri_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    UserCommand cmd_;
    MotionStateFeedback msfb_;
    std::mutex mtx_;
    std::thread ros_thread_;
    std::thread kb_thread_;
    bool running_ = false;
    bool nav2_enabled_ = false;
};

}
