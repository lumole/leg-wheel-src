#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "message/imu_controller.h"

// IMU 回调函数

class imu_transmit
{
public:
  imu_transmit()
  {
    // Topic you want to publish  这个位置就是要发布的话题以及发布者实现
    imu_pub = n.advertise<message::imu_controller>("/imu_topic", 10);

    // Topic you want to subscribe    这个位置是订阅之前IMU的话题，并且回调
    imu_sub = n.subscribe("/imu", 10, &imu_transmit::IMU_Callback, this); // 注意这里，和平时使用回调函数不一样了。

    imu_435_sub = n.subscribe("/camera/gyro/sample", 10, &imu_transmit::IMU_435_Callback, this);
  }

  void IMU_Callback(const sensor_msgs::Imu msg)
  {
    message::imu_controller imu_msg;
    if (msg.orientation_covariance[0] < 0)
      return;
    // 四元数转成欧拉角
    tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    double roll, pitch, yaw;
    double acc_x,acc_y,acc_z;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // 弧度换算成角度
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;
    // ROS_INFO("滚转= %.3f 俯仰= %.3f 朝向= %.3f", roll-0.17, pitch+0.92, yaw-14+3);
    ROS_INFO("~~~~~~~~~~~~~~| IMU |~~~~~~~~~~~~~~");
    ROS_INFO("滚转= %.3f 俯仰= %.3f 朝向= %.3f", roll, pitch, yaw);

    double roll_v, pitch_v, yaw_v;
    roll_v = msg.angular_velocity.x;
    pitch_v = msg.angular_velocity.y;
    yaw_v = msg.angular_velocity.z;
    acc_x = msg.linear_acceleration.x;
    acc_y = msg.linear_acceleration.y;
    acc_z = msg.linear_acceleration.z;
    ROS_INFO("滚转x角速度= %.3f 俯仰y角速度= %.3f 朝向z角速度= %.3f", roll_v, pitch_v, yaw_v);
    ROS_INFO("x方向加速度=%.3f", acc_x);

    // pitch
    imu_msg.angle_pitch = pitch;
    imu_msg.angle_vel_pitch = pitch_v;
    // roll
    imu_msg.angle_roll = roll;
    imu_msg.angle_vel_roll = roll_v;
    // yaw
    imu_msg.angle_yaw = yaw;
    imu_msg.angle_vel_yaw = yaw_v;
    // acc
    imu_msg.acc_x = acc_x;
    imu_msg.acc_y = acc_y;
    imu_msg.acc_z = acc_z;

    imu_msg.header.stamp = ros::Time::now();
    imu_pub.publish(imu_msg); // imu to controller
  }

  void IMU_435_Callback(const sensor_msgs::Imu msg)
  {
    message::imu_controller imu_msg;

    tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    double roll, pitch, yaw;
    double acc_x;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // 弧度换算成角度
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;
    // ROS_INFO("滚转= %.3f 俯仰= %.3f 朝向= %.3f", roll-0.17, pitch+0.92, yaw-14+3);
    ROS_INFO("~~~~~~~~~~~~~~| IMU |~~~~~~~~~~~~~~");
    ROS_INFO("滚转= %.3f 俯仰= %.3f 朝向= %.3f", roll, pitch, yaw);

    double roll_v, pitch_v, yaw_v;
    roll_v = msg.angular_velocity.x;
    pitch_v = msg.angular_velocity.y;
    yaw_v = msg.angular_velocity.z;
    ROS_INFO("滚转x角速度= %.3f 俯仰y角速度= %.3f 朝向z角速度= %.3f", roll_v, pitch_v, yaw_v);

    imu_msg.angle_vel_pitch=pitch_v;

    imu_msg.header.stamp = ros::Time::now();
    imu_pub.publish(imu_msg); // imu to controller
  }

private:
  ros::NodeHandle n;
  ros::Publisher imu_pub;
  ros::Subscriber imu_sub;
  ros::Subscriber imu_435_sub;
};

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "imu_transmit");
  imu_transmit APObject;
  ros::Rate loop_rate(10000);
  loop_rate.sleep();
  ros::spin();
  return 0;
}
