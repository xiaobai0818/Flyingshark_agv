#include <ros/ros.h>
#include <serial/serial.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint8_t kHead1 = 0xAA;
constexpr uint8_t kHead2 = 0x55;
constexpr uint8_t kTypeV1Data = 0x06;
constexpr uint8_t kTypeV1Vel = 0x11;
constexpr uint8_t kTypeV1Pid = 0x12;
constexpr uint8_t kTypeV1Param = 0x13;
constexpr uint8_t kTypeXData = 0x10;
constexpr uint8_t kTypeXVel = 0x50;

int16_t readBe16(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

double wrapAngle(double yaw) {
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

std::string resolvePort(const std::string& requested) {
  if (!requested.empty() && requested != "auto") return requested;
  char lidar_buf[512];
  std::string lidar_real;
  if (realpath("/dev/ldlidar", lidar_buf) != nullptr) lidar_real = lidar_buf;
  const char* candidates[] = {"/dev/openctr", "/dev/ttyACM0", "/dev/ttyACM1",
                              "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};
  for (const char* candidate : candidates) {
    if (access(candidate, F_OK) != 0) continue;
    char real_buf[512];
    std::string real = candidate;
    if (realpath(candidate, real_buf) != nullptr) real = real_buf;
    if (!lidar_real.empty() && real == lidar_real) continue;
    return candidate;
  }
  return "";
}

}  // namespace

class OpenCtrDriver {
 public:
  OpenCtrDriver() : nh_(), pnh_("~"), tf_broadcaster_() {
    pnh_.param<std::string>("port", port_param_, std::string("auto"));
    pnh_.param("baud_rate", baud_rate_, 115200);
    pnh_.param<std::string>("protocol", protocol_, std::string("auto"));
    pnh_.param("publish_tf", publish_tf_, false);
    pnh_.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
    pnh_.param<std::string>("base_frame", base_frame_, std::string("base_footprint"));
    pnh_.param<std::string>("imu_frame", imu_frame_, std::string("base_imu_link"));
    pnh_.param("control_rate", control_rate_, 25.0);
    pnh_.param("cmd_timeout", cmd_timeout_, 0.5);
    pnh_.param("linear_scale", linear_scale_, 1.0);
    pnh_.param("angular_scale", angular_scale_, 1.0);
    pnh_.param("wheel_vx_variance", wheel_vx_var_, 0.05);
    pnh_.param("wheel_wz_variance", wheel_wz_var_, 0.25);
    pnh_.param("gyro_z_variance", gyro_z_var_, 0.01);
    pnh_.param("calibrate_gyro_bias", calibrate_gyro_, true);
    pnh_.param("gyro_bias_seconds", gyro_bias_seconds_, 2.0);
    pnh_.param("kp", kp_, 300);
    pnh_.param("ki", ki_, 0);
    pnh_.param("kd", kd_, 200);
    if (std::abs(linear_scale_) < 1e-6) linear_scale_ = 1.0;
    if (std::abs(angular_scale_) < 1e-6) angular_scale_ = 1.0;

    wheel_pub_ = nh_.advertise<nav_msgs::Odometry>("wheel/odom", 20);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 50);
    voltage_pub_ = nh_.advertise<std_msgs::Float32>("voltage", 5);
    cmd_sub_ = nh_.subscribe("cmd_vel", 10, &OpenCtrDriver::onCmd, this);
    last_cmd_time_ = ros::Time(0);
    last_rx_time_ = ros::Time(0);
    last_send_time_ = ros::Time(0);
    start_time_ = ros::Time::now();
  }

  void spin() {
    ros::Rate rate(200);
    while (ros::ok()) {
      ros::spinOnce();
      ensureOpen();
      if (serial_.isOpen()) {
        readAvailable();
        sendCommandIfDue();
      }
      rate.sleep();
    }
    sendVelocity(0.0, 0.0, 0.0);
  }

 private:
  enum class RxState { kHead1, kHead2, kLen, kBody, kCheck };
  enum class Proto { kUnknown, kV1, kX };

  void onCmd(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vx_ = msg->linear.x;
    cmd_vy_ = msg->linear.y;
    cmd_wz_ = msg->angular.z;
    last_cmd_time_ = ros::Time::now();
  }

  void ensureOpen() {
    if (serial_.isOpen()) return;
    const ros::Time now = ros::Time::now();
    if ((now - last_open_try_).toSec() < 2.0) return;
    last_open_try_ = now;
    const std::string port = resolvePort(port_param_);
    if (port.empty()) {
      ROS_WARN_THROTTLE(5.0, "还没有找到 OpenCTR 串口。请确认 B20 的 USB 已接到香橙派。");
      return;
    }
    try {
      serial_.setPort(port);
      serial_.setBaudrate(static_cast<uint32_t>(baud_rate_));
      serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
      serial_.setTimeout(timeout);
      serial_.open();
      serial_.flushInput();
      rx_state_ = RxState::kHead1;
      pose_ready_ = false;
      params_sent_ = false;
      ROS_INFO("OpenCTR 已打开 %s @ %d", port.c_str(), baud_rate_);
    } catch (const std::exception& ex) {
      ROS_ERROR_THROTTLE(5.0, "打开串口失败: %s", ex.what());
    }
  }

  void readAvailable() {
    try {
      const size_t waiting = serial_.available();
      if (waiting == 0) {
        if (last_rx_time_.isZero() || (ros::Time::now() - last_rx_time_).toSec() > 2.0) {
          ROS_WARN_THROTTLE(5.0, "串口已打开，但还没有收到底盘数据。可尝试把 baud_rate 改成 230400。");
        }
        return;
      }
      uint8_t buf[512];
      const size_t n = serial_.read(buf, std::min(waiting, sizeof(buf)));
      for (size_t i = 0; i < n; ++i) feed(buf[i]);
    } catch (const std::exception& ex) {
      ROS_ERROR("读串口失败，准备重连: %s", ex.what());
      closePort();
    }
  }

  void feed(uint8_t byte) {
    switch (rx_state_) {
      case RxState::kHead1:
        if (byte == kHead1) rx_state_ = RxState::kHead2;
        break;
      case RxState::kHead2:
        rx_state_ = (byte == kHead2) ? RxState::kLen : RxState::kHead1;
        if (byte == kHead1) rx_state_ = RxState::kHead2;
        break;
      case RxState::kLen:
        frame_len_ = byte;
        if (frame_len_ < 6 || frame_len_ > 80) {
          rx_state_ = RxState::kHead1;
          break;
        }
        frame_.assign(frame_len_, 0);
        frame_[0] = kHead1;
        frame_[1] = kHead2;
        frame_[2] = frame_len_;
        frame_index_ = 3;
        rx_state_ = RxState::kBody;
        break;
      case RxState::kBody:
        frame_[frame_index_++] = byte;
        if (frame_index_ == frame_len_ - 1) rx_state_ = RxState::kCheck;
        break;
      case RxState::kCheck: {
        uint8_t sum = 0;
        for (int i = 0; i < frame_len_ - 1; ++i) sum = static_cast<uint8_t>(sum + frame_[i]);
        if (sum == byte) handleFrame();
        rx_state_ = RxState::kHead1;
        break;
      }
    }
  }

  void handleFrame() {
    const uint8_t type = frame_[3];
    if (type == kTypeV1Data && frame_len_ >= 44) {
      lockProtocol(Proto::kV1);
      handleV1();
    } else if (type == kTypeXData && frame_len_ >= 24) {
      lockProtocol(Proto::kX);
      handleX();
    }
  }

  void lockProtocol(Proto proto) {
    if (active_proto_ == proto) return;
    active_proto_ = proto;
    params_sent_ = false;
    ROS_INFO("识别到底盘协议: %s", proto == Proto::kV1 ? "OpenCTR V1" : "X-Protocol");
  }

  Proto txProtocol() const {
    if (protocol_ == "v1") return Proto::kV1;
    if (protocol_ == "x") return Proto::kX;
    if (active_proto_ != Proto::kUnknown) return active_proto_;
    return Proto::kV1;
  }

  void handleV1() {
    const uint8_t* b = frame_.data();
    const double gyro_x = readBe16(b + 4) / 32768.0 * 2000.0 / 180.0 * M_PI;
    const double gyro_y = readBe16(b + 6) / 32768.0 * 2000.0 / 180.0 * M_PI;
    const double gyro_z = readBe16(b + 8) / 32768.0 * 2000.0 / 180.0 * M_PI;
    const double acc_x = readBe16(b + 10) / 32768.0 * 2.0 * 9.8;
    const double acc_y = readBe16(b + 12) / 32768.0 * 2.0 * 9.8;
    const double acc_z = readBe16(b + 14) / 32768.0 * 2.0 * 9.8;
    const double roll_deg = readBe16(b + 16) / 100.0;
    const double pitch_deg = readBe16(b + 18) / 100.0;
    const double yaw_deg = readBe16(b + 20) / 100.0;
    const double dx = readBe16(b + 28) / 1000.0 * linear_scale_;
    const double dy = readBe16(b + 30) / 1000.0 * linear_scale_;
    const double dyaw = readBe16(b + 32) / 1000.0 * angular_scale_;
    const ros::Time now = ros::Time::now();
    double dt = pose_ready_ ? (now - last_odom_stamp_).toSec() : 0.02;
    if (dt < 0.001 || dt > 0.2) dt = 0.02;
    integrateBody(dx, dy, dyaw);
    const double vx = dx / dt;
    const double vy = dy / dt;
    const double wz = dyaw / dt;
    publishSensors(now, vx, vy, wz, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, roll_deg, pitch_deg, yaw_deg);
    std_msgs::Float32 voltage;
    voltage.data = static_cast<uint16_t>((b[42] << 8) | b[43]) / 100.0f;
    voltage_pub_.publish(voltage);
    last_odom_stamp_ = now;
    last_rx_time_ = now;
    pose_ready_ = true;
    last_vx_ = vx;
    last_wz_ = wz;
  }

  void handleX() {
    const uint8_t* b = frame_.data();
    const double acc_x = readBe16(b + 4) * (4.0 * 9.8 / 32768.0);
    const double acc_y = readBe16(b + 6) * (4.0 * 9.8 / 32768.0);
    const double acc_z = readBe16(b + 8) * (4.0 * 9.8 / 32768.0);
    const double gyro_x = readBe16(b + 10) * (512.0 * M_PI / 180.0 / 32768.0);
    const double gyro_y = readBe16(b + 12) * (512.0 * M_PI / 180.0 / 32768.0);
    const double gyro_z = readBe16(b + 14) * (512.0 * M_PI / 180.0 / 32768.0);
    const double vx = readBe16(b + 16) / 1000.0 * linear_scale_;
    const double vy = readBe16(b + 18) / 1000.0 * linear_scale_;
    const double wz = readBe16(b + 20) / 1000.0 * angular_scale_;
    const ros::Time now = ros::Time::now();
    double dt = pose_ready_ ? (now - last_odom_stamp_).toSec() : 0.02;
    if (dt < 0.001 || dt > 0.2) dt = 0.02;
    integrateBody(vx * dt, vy * dt, wz * dt);
    publishSensors(now, vx, vy, wz, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, 0.0, 0.0, yaw_ * 180.0 / M_PI);
    std_msgs::Float32 voltage;
    voltage.data = static_cast<uint16_t>((b[22] << 8) | b[23]) / 100.0f;
    voltage_pub_.publish(voltage);
    last_odom_stamp_ = now;
    last_rx_time_ = now;
    pose_ready_ = true;
    last_vx_ = vx;
    last_wz_ = wz;
  }

  void integrateBody(double dx, double dy, double dyaw) {
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    x_ += c * dx - s * dy;
    y_ += s * dx + c * dy;
    yaw_ = wrapAngle(yaw_ + dyaw);
  }

  void publishSensors(const ros::Time& now, double vx, double vy, double wz, double ax, double ay, double az,
                      double gx, double gy, double gz, double roll_deg, double pitch_deg, double yaw_deg) {
    updateGyroBias(gz, vx, wz, now);
    const double gz_corr = gz - gyro_bias_z_;

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;
    odom.pose.covariance[0] = 0.2;
    odom.pose.covariance[7] = 0.2;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = 0.5;
    odom.twist.covariance[0] = wheel_vx_var_;
    odom.twist.covariance[7] = 1e3;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = wheel_wz_var_;
    wheel_pub_.publish(odom);

    if (publish_tf_) {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header = odom.header;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = x_;
      tf_msg.transform.translation.y = y_;
      tf_msg.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_.sendTransform(tf_msg);
    }

    sensor_msgs::Imu imu;
    imu.header.stamp = now;
    imu.header.frame_id = imu_frame_;
    tf2::Quaternion iq;
    iq.setRPY(roll_deg * M_PI / 180.0, pitch_deg * M_PI / 180.0, yaw_deg * M_PI / 180.0);
    imu.orientation.x = iq.x();
    imu.orientation.y = iq.y();
    imu.orientation.z = iq.z();
    imu.orientation.w = iq.w();
    imu.angular_velocity.x = gx;
    imu.angular_velocity.y = gy;
    imu.angular_velocity.z = gz_corr;
    imu.linear_acceleration.x = ax;
    imu.linear_acceleration.y = ay;
    imu.linear_acceleration.z = az;
    imu.orientation_covariance[0] = 1e6;
    imu.orientation_covariance[4] = 1e6;
    imu.orientation_covariance[8] = 1e6;
    imu.angular_velocity_covariance[0] = 1e3;
    imu.angular_velocity_covariance[4] = 1e3;
    imu.angular_velocity_covariance[8] = gyro_z_var_;
    imu.linear_acceleration_covariance[0] = 0.4;
    imu.linear_acceleration_covariance[4] = 0.4;
    imu.linear_acceleration_covariance[8] = 0.4;
    imu_pub_.publish(imu);
  }

  void updateGyroBias(double gz, double vx, double wz, const ros::Time& now) {
    if (!calibrate_gyro_ || bias_done_) return;
    const bool still = std::abs(vx) < 0.02 && std::abs(wz) < 0.05 && std::abs(cmd_vx_) < 0.01 &&
                       std::abs(cmd_wz_) < 0.01;
    if (still) {
      bias_sum_ += gz;
      ++bias_count_;
    }
    if ((now - start_time_).toSec() >= gyro_bias_seconds_ && bias_count_ > 20) {
      gyro_bias_z_ = bias_sum_ / static_cast<double>(bias_count_);
      bias_done_ = true;
      ROS_INFO("陀螺仪零偏已校准: %.5f rad/s（%d 个静止样本）", gyro_bias_z_, bias_count_);
    } else if ((now - start_time_).toSec() > gyro_bias_seconds_ + 1.0) {
      bias_done_ = true;
      ROS_WARN("启动时底盘不静止，跳过陀螺仪零偏校准。");
    }
  }

  void sendCommandIfDue() {
    const ros::Time now = ros::Time::now();
    if ((now - last_send_time_).toSec() < 1.0 / std::max(1.0, control_rate_)) return;
    last_send_time_ = now;
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    if (!last_cmd_time_.isZero() && (now - last_cmd_time_).toSec() <= cmd_timeout_) {
      vx = cmd_vx_;
      vy = cmd_vy_;
      wz = cmd_wz_;
    }
    if (txProtocol() == Proto::kV1 && !params_sent_) {
      sendV1Params();
      sendV1Pid();
      params_sent_ = true;
    }
    sendVelocity(vx / linear_scale_, vy / linear_scale_, wz / angular_scale_);
  }

  void sendVelocity(double vx, double vy, double wz) {
    uint8_t payload[6];
    writeBe16(payload + 0, static_cast<int16_t>(std::lround(vx * 1000.0)));
    writeBe16(payload + 2, static_cast<int16_t>(std::lround(vy * 1000.0)));
    writeBe16(payload + 4, static_cast<int16_t>(std::lround(wz * 1000.0)));
    const uint8_t type = txProtocol() == Proto::kX ? kTypeXVel : kTypeV1Vel;
    sendFrame(type, payload, 6);
  }

  void sendV1Pid() {
    uint8_t payload[6];
    writeBe16(payload + 0, static_cast<int16_t>(kp_));
    writeBe16(payload + 2, static_cast<int16_t>(ki_));
    writeBe16(payload + 4, static_cast<int16_t>(kd_));
    sendFrame(kTypeV1Pid, payload, 6);
  }

  void sendV1Params() {
    uint8_t payload[4];
    writeBe16(payload + 0, 1000);
    writeBe16(payload + 2, 0);
    sendFrame(kTypeV1Param, payload, 4);
  }

  void sendFrame(uint8_t type, const uint8_t* payload, size_t n) {
    if (!serial_.isOpen()) return;
    std::vector<uint8_t> frame(5 + n);
    frame[0] = kHead1;
    frame[1] = kHead2;
    frame[2] = static_cast<uint8_t>(frame.size());
    frame[3] = type;
    std::memcpy(frame.data() + 4, payload, n);
    uint8_t sum = 0;
    for (size_t i = 0; i + 1 < frame.size(); ++i) sum = static_cast<uint8_t>(sum + frame[i]);
    frame.back() = sum;
    try {
      serial_.write(frame);
    } catch (const std::exception& ex) {
      ROS_ERROR("写串口失败，准备重连: %s", ex.what());
      closePort();
    }
  }

  static void writeBe16(uint8_t* p, int16_t value) {
    const uint16_t u = static_cast<uint16_t>(value);
    p[0] = static_cast<uint8_t>((u >> 8) & 0xFF);
    p[1] = static_cast<uint8_t>(u & 0xFF);
  }

  void closePort() {
    try {
      if (serial_.isOpen()) serial_.close();
    } catch (...) {
    }
    params_sent_ = false;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher wheel_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher voltage_pub_;
  ros::Subscriber cmd_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  serial::Serial serial_;

  std::string port_param_;
  std::string protocol_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  int baud_rate_{115200};
  int kp_{300};
  int ki_{0};
  int kd_{200};
  bool publish_tf_{false};
  bool calibrate_gyro_{true};
  double control_rate_{25.0};
  double cmd_timeout_{0.5};
  double linear_scale_{1.0};
  double angular_scale_{1.0};
  double wheel_vx_var_{0.05};
  double wheel_wz_var_{0.25};
  double gyro_z_var_{0.01};
  double gyro_bias_seconds_{2.0};

  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_wz_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double last_vx_{0.0};
  double last_wz_{0.0};
  double gyro_bias_z_{0.0};
  double bias_sum_{0.0};
  int bias_count_{0};
  bool bias_done_{false};
  bool pose_ready_{false};
  bool params_sent_{false};

  ros::Time last_cmd_time_;
  ros::Time last_rx_time_;
  ros::Time last_send_time_;
  ros::Time last_open_try_;
  ros::Time last_odom_stamp_;
  ros::Time start_time_;

  RxState rx_state_{RxState::kHead1};
  Proto active_proto_{Proto::kUnknown};
  std::vector<uint8_t> frame_;
  int frame_len_{0};
  int frame_index_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "openctr_driver");
  OpenCtrDriver driver;
  driver.spin();
  return 0;
}
