#pragma once

#include "robot_interface.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <mujoco/mujoco.h>
#include <mutex>
#include <thread>

namespace interface {

class MujocoInterface : public RobotInterface {
public:
  MujocoInterface(const std::string &name, const std::string &mjcf_path)
      : RobotInterface(name, 12), mjcf_path_(mjcf_path) {

    char error[1000];
    model_ = mj_loadXML(mjcf_path_.c_str(), nullptr, error, 1000);
    if (!model_) {
      std::cerr << "Could not load MuJoCo model from " << mjcf_path_
                << ". Error: " << error << std::endl;
      return;
    }
    data_ = mj_makeData(model_);

    joint_pos_ = VecXf::Zero(dof_num_);
    joint_vel_ = VecXf::Zero(dof_num_);
    joint_tau_ = VecXf::Zero(dof_num_);
    joint_cmd_ = MatXf::Zero(dof_num_, 5);

    std::cout << "[MujocoInterface] Loaded model: " << mjcf_path_ << std::endl;
  }

  ~MujocoInterface() {
    Stop();
    if (data_)
      mj_deleteData(data_);
    if (model_)
      mj_deleteModel(model_);
  }

  void Start() override {
    running_ = true;
    sim_thread_ = std::thread(&MujocoInterface::SimLoop, this);
  }

  void Stop() override {
    running_ = false;
    if (sim_thread_.joinable())
      sim_thread_.join();
  }

  double GetInterfaceTimeStamp() override {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_ ? data_->time : 0.0;
  }

  VecXf GetJointPosition() override {
    std::lock_guard<std::mutex> lock(mtx_);
    for (int i = 0; i < dof_num_; ++i)
      joint_pos_(i) = data_->qpos[i + 7]; // Skip root 7-dof
    return joint_pos_;
  }

  VecXf GetJointVelocity() override {
    std::lock_guard<std::mutex> lock(mtx_);
    for (int i = 0; i < dof_num_; ++i)
      joint_vel_(i) = data_->qvel[i + 6]; // Skip root 6-dof
    return joint_vel_;
  }

  VecXf GetJointTorque() override {
    std::lock_guard<std::mutex> lock(mtx_);
    for (int i = 0; i < dof_num_; ++i)
      joint_tau_(i) = data_->qfrc_applied[i + 6];
    return joint_tau_;
  }

  Vec3f GetImuRpy() override {
    std::lock_guard<std::mutex> lock(mtx_);
    // Simple RPY from root quaternion for now
    mjtNum quat[4] = {data_->qpos[3], data_->qpos[4], data_->qpos[5],
                      data_->qpos[6]};
    Vec3f rpy;
    // Quat to RPY conversion (simplified)
    rpy.x() = atan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]),
                    1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]));
    rpy.y() = asin(2 * (quat[0] * quat[2] - quat[3] * quat[1]));
    rpy.z() = atan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]),
                    1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return rpy;
  }

  Vec3f GetImuAcc() override {
    return Vec3f::Zero(); // Add sensor reading if needed
  }

  Vec3f GetImuOmega() override {
    std::lock_guard<std::mutex> lock(mtx_);
    return Vec3f(data_->qvel[3], data_->qvel[4], data_->qvel[5]);
  }

  void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) override {
    std::lock_guard<std::mutex> lock(mtx_);
    joint_cmd_ = input;
  }

  VecXf GetContactForce() override { return VecXf::Zero(4); }

private:
  void SimLoop() {
    // 1. Initialize GLFW for visualization
    if (!glfwInit()) return;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Lite3 RL Simulation", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 2. Initialize MuJoCo visualization objects
    mjvScene scn;
    mjvCamera cam;
    mjvOption opt;
    mjrContext con;
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(model_, &scn, 2000);
    mjr_makeContext(model_, &con, mjFONTSCALE_150);

    // Set camera to follow robot
    cam.distance = 3.0;
    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 0.5;

    while (running_ && !glfwWindowShouldClose(window)) {
      {
        std::lock_guard<std::mutex> lock(mtx_);
        // Simple PD control implementation inside MuJoCo step
        for (int i = 0; i < dof_num_; ++i) {
          float kp = joint_cmd_(i, 0);
          float q_target = joint_cmd_(i, 1);
          float kd = joint_cmd_(i, 2);
          float dq_target = joint_cmd_(i, 3);
          float tau_ff = joint_cmd_(i, 4);

          float q = data_->qpos[i + 7];
          float dq = data_->qvel[i + 6];

          data_->qfrc_applied[i + 6] =
              kp * (q_target - q) + kd * (dq_target - dq) + tau_ff;
        }
        mj_step(model_, data_);
      }

      // 3. Render
      mjrRect viewport = {0, 0, 0, 0};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
      mjv_updateScene(model_, data_, &opt, NULL, &cam, mjCAT_ALL, &scn);
      mjr_render(viewport, &scn, &con);
      glfwSwapBuffers(window);
      glfwPollEvents();

      std::this_thread::sleep_for(
          std::chrono::microseconds(int(model_->opt.timestep * 1000000)));
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();
  }

  std::string mjcf_path_;
  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;
  std::thread sim_thread_;
  std::mutex mtx_;
  bool running_ = false;

  Vec3f rpy_, acc_, omega_body_;
  VecXf joint_pos_, joint_vel_, joint_tau_;
};

} // namespace interface
