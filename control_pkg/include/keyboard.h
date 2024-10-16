#pragma once
#include<iostream>
#include<string>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/UInt16.h>
#include <termio.h>
#include "message/imu_controller.h"
#include "message/interface_controller.h" //
#include "message/pid_interface.h" //

#include "std_msgs/String.h" //���ݵ���Ϣ����
#include "sstream"

#define KEYCODE_0 0x30
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39
#define KEYCODE_SPACE 0x20
#define KEYCODE_DOUHAO 0x2C
#define KEYCODE_JIANHAO 0x2D
#define KEYCODE_JUHAO 0x2E
#define KEYCODE_XIEGANG 0x2F
#define KEYCODE_FENHAO 0x3B
#define KEYCODE_DENGHAO 0x3D

#define KEYCODE_A 0x61
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_H 0x68
#define KEYCODE_I 0x69
#define KEYCODE_J 0x6A
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C
#define KEYCODE_M 0x6D
#define KEYCODE_N 0x6E
#define KEYCODE_O 0x6F
#define KEYCODE_P 0x70
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_S 0x73
#define KEYCODE_T 0x74
#define KEYCODE_U 0x75
#define KEYCODE_V 0x76
#define KEYCODE_W 0x77
#define KEYCODE_X 0x78
#define KEYCODE_Y 0x79
#define KEYCODE_Z 0x7A


class keyboard {
public:
	void scanKeyboard();
	int key_command = 0x00;
    std_msgs::UInt16 key_c;

	void InitializePublishers();
	ros::Publisher key_publisher;
	ros::NodeHandle nh;

	void switchkeyboard();

};