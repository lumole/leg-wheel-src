#include "manipulator.h"

void arm_init(bool _side, device *M);

M_controller::M_controller()
{
	M_L = (struct device *)malloc(sizeof(struct device));
	M_R = (struct device *)malloc(sizeof(struct device));
	arm_init(right_side, M_R);
	arm_init(left_side, M_L);
	m_pub = nh.advertise<message::manipulator>("/manipulator_output", 1);
	m_sub = nh.subscribe("/manipulator_feedback", 10, &M_controller::Feedback, this); // 接受键盘输入
	key_sub = nh.subscribe("/key_command", 10, &M_controller::Keyboard_Input, this);  // 接受键盘输入
	cmd_sub = nh.subscribe("/CMD2ARM", 10, &M_controller::cmd, this);
}

void arm_init(bool _side, device *M)
{
	M->side = _side;
	M->angle[0] = 0;
	M->angle[1] = 0;
	M->angle[2] = 0;
	M->exp_angle[0] = 0;
	M->exp_angle[1] = 0;
	M->exp_angle[2] = 0;
	M->exp_pos[0] = init[0];
	M->exp_pos[1] = init[1];
	M->exp_pos[2] = init[2];
	M->pos[0] = 0;
	M->pos[1] = 0;
	M->pos[2] = 0;
	M->tou[0] = 0;
	M->tou[1] = 0;
	M->tou[2] = 0;
}

void M_controller::inverse_kine(device *M)
{
	double X = M->exp_pos[0];
	double Y = M->exp_pos[1];
	double Z = M->exp_pos[2];
	double T1;
	[[maybe_unused]] double T2_1, T3_1, T2_2, T3_2;
	double c_R_tmp[16];
	double temp[4];
	double R_tmp;
	double b_R_tmp;
	T1 = std::atan(Y / X);
	R_tmp = std::sin(T1);
	b_R_tmp = std::cos(T1);
	c_R_tmp[0] = b_R_tmp;
	c_R_tmp[4] = R_tmp;
	c_R_tmp[8] = 0.0;
	c_R_tmp[12] = 0.0;
	c_R_tmp[1] = -R_tmp;
	c_R_tmp[5] = b_R_tmp;
	c_R_tmp[9] = 0.0;
	c_R_tmp[13] = 0.0;
	c_R_tmp[2] = 0.0;
	c_R_tmp[6] = 0.0;
	c_R_tmp[10] = 1.0;
	c_R_tmp[14] = -M_L2;
	c_R_tmp[3] = 0.0;
	c_R_tmp[7] = 0.0;
	c_R_tmp[11] = 0.0;
	c_R_tmp[15] = 1.0;
	for (int i{0}; i < 4; i++)
	{
		temp[i] = ((c_R_tmp[i] * X + c_R_tmp[i + 4] * Y) + c_R_tmp[i + 8] * Z) +
				  c_R_tmp[i + 12];
	}
	double T2_1_tmp;
	double b_T2_1_tmp;
	double c_T2_1_tmp;
	double d_T2_1_tmp;
	double e_T2_1_tmp;
	double f_T2_1_tmp;
	double g_T2_1_tmp;
	double h_T2_1_tmp;
	double i_T2_1_tmp;
	double j_T2_1_tmp;
	T2_1_tmp = M_L0 * M_L0;
	b_T2_1_tmp = 2.0 * M_L0 * M_L1;
	R_tmp = M_L1 * M_L1;
	b_R_tmp = temp[0] * temp[0];
	c_T2_1_tmp = temp[2] * temp[2];
	d_T2_1_tmp = (((-T2_1_tmp + b_T2_1_tmp) - R_tmp) + b_R_tmp) + c_T2_1_tmp;
	e_T2_1_tmp =
		std::sqrt(d_T2_1_tmp *
				  ((((T2_1_tmp + b_T2_1_tmp) + R_tmp) - b_R_tmp) - c_T2_1_tmp));
	f_T2_1_tmp = 2.0 * M_L1 * temp[2];
	g_T2_1_tmp = T2_1_tmp * e_T2_1_tmp / d_T2_1_tmp;
	h_T2_1_tmp = R_tmp * e_T2_1_tmp / d_T2_1_tmp;
	i_T2_1_tmp = b_R_tmp * e_T2_1_tmp / d_T2_1_tmp;
	j_T2_1_tmp = c_T2_1_tmp * e_T2_1_tmp / d_T2_1_tmp;
	b_T2_1_tmp = b_T2_1_tmp * e_T2_1_tmp / d_T2_1_tmp;
	T2_1_tmp =
		(((-T2_1_tmp + R_tmp) + 2.0 * M_L1 * temp[0]) + b_R_tmp) + c_T2_1_tmp;
	T2_1 = 2.0 *
		   std::atan((((((f_T2_1_tmp + g_T2_1_tmp) + h_T2_1_tmp) - i_T2_1_tmp) -
					   j_T2_1_tmp) -
					  b_T2_1_tmp) /
					 T2_1_tmp); // atan((F + E) / G)
	T2_2 = 2.0 *
		   std::atan((((((f_T2_1_tmp - g_T2_1_tmp) - h_T2_1_tmp) + i_T2_1_tmp) +
					   j_T2_1_tmp) +
					  b_T2_1_tmp) /
					 T2_1_tmp); // atan((F - E) / G)
	R_tmp = std::atan(e_T2_1_tmp / d_T2_1_tmp);
	T3_1 = -2.0 * R_tmp;
	T3_2 = 2.0 * R_tmp;
	M->exp_angle[0] = T1;
	M->exp_angle[1] = T2_1;
	M->exp_angle[2] = T3_1;
}

void M_controller::Feedback(const message::interface_controller &msg)
{
	M_R->angle[0] = anglenormalize(-msg.encoder1 / (65535 / 2 / pi));
	M_R->angle[1] = anglenormalize(-msg.encoder3 / (65535 / 2 / pi));
	M_R->angle[2] = anglenormalize(-msg.encoder5 / (65535 / 2 / pi));
	M_L->angle[0] = anglenormalize(msg.encoder2 / (65535 / 2 / pi));
	M_L->angle[1] = anglenormalize(msg.encoder4 / (65535 / 2 / pi));
	M_L->angle[2] = anglenormalize(msg.encoder6 / (65535 / 2 / pi));
	cout << "实际角度：" << endl
		 << "R:" << M_R->angle[0] << " " << M_R->angle[1] << " " << M_R->angle[2] << endl
		 << "L:" << M_L->angle[0] << " " << M_L->angle[1] << " " << M_L->angle[2] << endl;
}

//void M_controller::cmd_handler()

void M_controller::Keyboard_Input(const std_msgs::UInt16 &msg) // 键盘回调函数
{
	switch (msg.data)
	{
	case 'U':
		// M_L->exp_pos[0] += 0.001;
		// M_R->exp_pos[0] += 0.001;
		mani_joint_l = 1;
		angle_base_l += 2;
		if (angle_base_l > 180)
		{
			angle_base_l = 180;
		}
		if (angle_base_l < -180)
		{
			angle_base_l = -180;
		}
		break;
	case 'J':
		mani_joint_l = 2;
		angle_shoulder_l += 2;
		if (angle_shoulder_l > 90)
		{
			angle_shoulder_l = 90;
		}
		if (angle_shoulder_l < -90)
		{
			angle_shoulder_l = -90;
		}
		break;
	case 'I':
		// M_L->exp_pos[1] += 0.001;
		// M_R->exp_pos[1] += 0.001;
		mani_joint_l = 3;
		angle_elbow_l += 2;
		if (angle_elbow_l > 180)
		{
			angle_elbow_l = 180;
		}
		if (angle_elbow_l < -45)
		{
			angle_elbow_l = -45;
		}
		break;
	case 'K':
		// M_L->exp_pos[1] -= 0.001;
		// M_R->exp_pos[1] -= 0.001;
		mani_joint_l = 4;
		angle_eoat_l -= 2;
		if (angle_eoat_l > 180)
		{
			angle_eoat_l = 180;
		}
		if (angle_eoat_l < 45)
		{
			angle_eoat_l = 45;
		}
		break;
	case 'O':
		// M_L->exp_pos[2] += 0.001;
		// M_R->exp_pos[2] += 0.001;
		mani_joint_r = 1;
		angle_base_r += 2;
		if (angle_base_r > 180)
		{
			angle_base_r = 180;
		}
		if (angle_base_r < -180)
		{
			angle_base_r = -180;
		}
		break;
	case 'L':
		// M_L->exp_pos[2] -= 0.001;
		// M_R->exp_pos[2] -= 0.001;
		mani_joint_r = 2;
		angle_shoulder_r += 2;
		if (angle_shoulder_r > 90)
		{
			angle_shoulder_r = 90;
		}
		if (angle_shoulder_r < -90)
		{
			angle_shoulder_r = -90;
		}
		break;
	case 'N':
		// M_L->p = new playset(TIME_STEP);
		// M_L->p->setFrame(2, down_loose);
		// M_L->p->setFrame(1, down_seize);
		// M_L->p->setFrame(2, up_seize);
		// M_L->p->setFrame(1, up_loose);
		// M_R->p = new playset(TIME_STEP);
		// M_R->p->setFrame(2, down_loose);
		// M_R->p->setFrame(1, down_seize);
		// M_R->p->setFrame(2, up_seize);
		// M_R->p->setFrame(1, up_loose);
		mani_joint_r = 3;
		angle_elbow_r += 2;
		if (angle_elbow_r > 180)
		{
			angle_elbow_r = 180;
		}
		if (angle_elbow_r < -45)
		{
			angle_elbow_r = -45;
		}
		break;
	case 'M':
		// M_L->p = new playset(TIME_STEP);
		// M_L->p->setFrame(2 * wave_time, wave1);
		// M_L->p->setFrame(wave_time, wave2);
		// M_L->p->setFrame(wave_time, wave1);
		// M_L->p->setFrame(wave_time, wave2);
		// M_L->p->setFrame(wave_time, wave0);
		// M_R->p = new playset(TIME_STEP);
		// M_R->p->setFrame(2 * wave_time, wave2);
		// M_R->p->setFrame(wave_time, wave1);
		// M_R->p->setFrame(wave_time, wave2);
		// M_R->p->setFrame(wave_time, wave1);
		// M_R->p->setFrame(wave_time, wave0);
		mani_joint_r = 4;
		angle_eoat_r -= 2;
		if (angle_eoat_r > 180)
		{
			angle_eoat_r = 180;
		}
		if (angle_eoat_r < 45)
		{
			angle_eoat_r = 45;
		}
		break;
	case 'R':
		mani_joint_l = 1;
		angle_base_l -= 2;
		if (angle_base_l > 180)
		{
			angle_base_l = 180;
		}
		if (angle_base_l < -180)
		{
			angle_base_l = -180;
		}
		break;
	case 'T':
		mani_joint_l = 2;
		angle_shoulder_l -= 2;
		if (angle_shoulder_l > 90)
		{
			angle_shoulder_l = 90;
		}
		if (angle_shoulder_l < -90)
		{
			angle_shoulder_l = -90;
		}
		break;
	case 'V':
		mani_joint_l = 3;
		angle_elbow_l -= 2;
		if (angle_elbow_l > 180)
		{
			angle_elbow_l = 180;
		}
		if (angle_elbow_l < -45)
		{
			angle_elbow_l = -45;
		}
		break;
	case 'P':
		mani_joint_l = 4;
		angle_eoat_l += 2;
		if (angle_eoat_l > 180)
		{
			angle_eoat_l = 180;
		}
		if (angle_eoat_l < 45)
		{
			angle_eoat_l = 45;
		}
		break;
	case 'B': // 卧倒
		pump_flag = 0;
		break;
	case 'F':
		pump_flag = 1;
		break;
	case 'G':
		pump_flag = 2;
		break;
	case 'H':
		pump_flag = 3;
		break;
	case 'X':
		pump_flag = 5;
		break;

	case '1':
		x += 2;
		axis_flag = 1;
		break;
	case '2':
		x -= 2;
		axis_flag = 1;
		break;
	case '3':
		y += 2;
		axis_flag = 2;
		break;
	case '4':
		y -= 2;
		axis_flag = 2;
		break;
	case '5':
		z += 2;
		axis_flag = 3;
		break;
	case '6':
		z -= 2;
		axis_flag = 3;
		break;
	case '7':
		t -= 2 * 3.14 / 180;
		axis_flag = 4;
		break;
	case '8':
		t += 2 * 3.14 / 180;
		axis_flag = 4;
		break;
	}
}

void M_controller::cmd(const std_msgs::String &msg)
{
	// if(msg.data=="reset")
	// {
	// 	mani_joint_l = 1;
	// 	angle_base_l = 0;
	// 	mani_joint_r = 1;
	// 	angle_base_r = 0;
	// 	mani_joint_l = 2;
	// 	angle_shoulder_l = 0;
	// 	mani_joint_r = 2;
	// 	angle_shoulder_r = 0;
	// 	mani_joint_l = 3;
	// 	angle_elbow_l = 90;
	// 	mani_joint_r = 3;
	// 	angle_elbow_r = 90;
	// 	mani_joint_l = 4;
	// 	angle_eoat_l = 175;
	// 	mani_joint_r = 4;
	// 	angle_eoat_r = 175;
	// }
	if(msg.data[0]=='l')
	{
		mani_joint_l = msg.data[1]-'0';
		std::string subStr = msg.data.substr(2);
		switch (msg.data[1])
		{
			case '1':
				std::stringstream(subStr) >> angle_base_l;
				if (angle_base_l > 180)
				{
					angle_base_l = 180;
				}
				if (angle_base_l < -180)
				{
					angle_base_l = -180;
				}
				break;
			case '2':
				std::stringstream(subStr) >> angle_shoulder_l;
				if (angle_shoulder_l > 90)
				{
					angle_shoulder_l = 90;
				}
				if (angle_shoulder_l < -90)
				{
					angle_shoulder_l = -90;
				}
				break;
			case '3':
				std::stringstream(subStr) >> angle_elbow_l;
				if (angle_elbow_l > 180)
				{
					angle_elbow_l = 180;
				}
				if (angle_elbow_l < -45)
				{
					angle_elbow_l = -45;
				}
				break;
			case '4':
				std::stringstream(subStr) >> angle_eoat_l;
				if (angle_eoat_l > 180)
				{
					angle_eoat_l = 180;
				}
				if (angle_eoat_l < -45)
				{
					angle_eoat_l = -45;
				}
				break;				
		}
	}
	if(msg.data[0]=='r')
	{
		mani_joint_r = msg.data[1]-'0';
		std::string subStr = msg.data.substr(2);
		switch (msg.data[1])
		{
			case '1':
				std::stringstream(subStr) >> angle_base_r;
				if (angle_base_r > 180)
				{
					angle_base_r = 180;
				}
				if (angle_base_r < -180)
				{
					angle_base_r = -180;
				}
				break;
			case '2':
				std::stringstream(subStr) >> angle_shoulder_r;
				if (angle_shoulder_r > 90)
				{
					angle_shoulder_r = 90;
				}
				if (angle_shoulder_r < -90)
				{
					angle_shoulder_r = -90;
				}
				break;
			case '3':
				std::stringstream(subStr) >> angle_elbow_r;
				if (angle_elbow_r > 180)
				{
					angle_elbow_r = 180;
				}
				if (angle_elbow_r < -45)
				{
					angle_elbow_r = -45;
				}
				break;
			case '4':
				std::stringstream(subStr) >> angle_eoat_r;
				if (angle_eoat_r > 180)
				{
					angle_eoat_r = 180;
				}
				if (angle_eoat_r < -45)
				{
					angle_eoat_r = -45;
				}
				break;				
		}		
	}

	//test
	// if(msg.data=="111")
	// {
	// 	mani_joint_l = 1;
	// 	angle_base_l += 2;
	// 	if (angle_base_l > 180)
	// 	{
	// 	angle_base_l = 180;
	// 	}
	// 	if (angle_base_l < -180)
	// 	{
	// 		angle_base_l = -180;
	// 	}
	// }
}

void M_controller::Print(device *M)
{
	if (M->side == right_side)
	{
		cout << "右侧机械臂：" << endl;
	}
	else
	{
		cout << "左侧机械臂：" << endl;
	}
	// cout << "期望位置:x=" << M->exp_pos[0] << " y=" << M->exp_pos[1] << " z=" << M->exp_pos[2] << endl;
	// cout << "期望角度:零=" << M->exp_angle[0] << " 壹=" << M->exp_angle[1] << " 贰=" << M->exp_angle[2] << endl
	// 	 << endl;
	cout << "实际角度:零=" << M->angle[0] << " 壹=" << M->angle[1] << " 贰=" << M->angle[2] << endl
		 << endl;
}

void M_controller::Pos_Control()
{
	double tl0_e, tl1_e, tl2_e;
	double tr0_e, tr1_e, tr2_e;
	// double tl0, tl1, tl2;
	// double tr0, tr1, tr2;
	tl0_e = M_L->exp_angle[0];
	tl1_e = M_L->exp_angle[1];
	tl2_e = M_L->exp_angle[2];
	tr0_e = M_R->exp_angle[0];
	tr1_e = M_R->exp_angle[1];
	tr2_e = M_R->exp_angle[2];
	if (tl1_e < 0)
	{
		tl1_e += 2 * pi;
	}
	if (tr1_e < 0)
	{
		tr1_e += 2 * pi;
	}
	manipulator_msg.l0 = (tl0_e) * 360000 / 2 / pi;
	manipulator_msg.l1 = (tl1_e) * 360000 / 2 / pi;
	manipulator_msg.l2 = (tl2_e + pi) * 360000 / 2 / pi;
	manipulator_msg.r0 = -(tr0_e) * 360000 / 2 / pi;
	manipulator_msg.r1 = -(tr1_e) * 360000 / 2 / pi;
	manipulator_msg.r2 = -(tr2_e + pi) * 360000 / 2 / pi;
	manipulator_msg.header.stamp = ros::Time::now();
	cout << "test_l " << manipulator_msg.l0 << " " << manipulator_msg.l1 << " " << manipulator_msg.l2 << endl;
	cout << "test_r " << manipulator_msg.r0 << " " << manipulator_msg.r1 << " " << manipulator_msg.r2 << endl;
	// m_pub.publish(manipulator_msg);
}

void M_controller::play(device *M)
{
	if (M->p != NULL)
	{
		frame f = M->p->update();
		M->exp_pos[0] = f.x;
		M->exp_pos[1] = f.y;
		M->exp_pos[2] = f.z;
		if (!M->p->state)
		{
			M->p = NULL;
		}
	}
	if (M->p != NULL)
	{
		cout << "playing..." << endl;
	}
	else
	{
		cout << "p=null" << endl;
	}
}

// waveshare manipulator json command function
std::string CMD_TORQUE_CTRL(int flag)
{
	// 扭矩锁
	// 0：接通电源时手动可以转动关节
	// 1：~不可以~
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":210,";
	oss << "\"cmd\":";
	string str = to_string(flag);
	oss << str;
	oss << "}\n";
	return oss.str();
}
std::string CMD_DYNAMIC_ADAPTATION(int flag, int b, int s, int e, int h)
{
	// 动态外力自适应
	// 0：接通电源时手动不可以转动关节
	// 1：使用外力转动后机械臂会回弹至转动前的位置
	// b:BASE关节最大输出扭矩
	// s:SHOULDER~
	// e:ELBOW~
	// h:wrist~
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":112,";

	oss << "\"mode\":";
	oss << to_string(flag);
	oss << ",";

	oss << "\"b\":";
	oss << to_string(b);
	oss << ",";

	oss << "\"s\":";
	oss << to_string(s);
	oss << ",";

	oss << "\"e\":";
	oss << to_string(e);
	oss << ",";

	oss << "\"h\":";
	oss << to_string(h);

	oss << "}\n";
	return oss.str();
}
std::string CMD_MOVE_INIT()
{
	// 所有关节回到初始位置
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":100";
	oss << "}\n";
	return oss.str();
}
std::string CMD_SINGLE_JOINT_CTRL(int joint, double rad, double spd, double acc)
{
	// 101：表示这条指令为CMD_SINGLE_JOINT_CTRL，以【弧度制】形式来控制机械臂某个关节的转动。
	// joint：关节序号。
	//     1：BASE_JOINT基础关节。
	//     2：SHOULDER_JOINT肩关节。
	//     3：ELBOW_JOINT肘关节。
	//     4：EOAT_JOINT手腕/夹爪关节。
	// rad：为需要转动到的角度（以弧度制形式来显示），以各个关节的初始位置为准，各个关节的默认角度和转动方向如下：
	//     BASE_JOINT的初始位置默认角度为 0，角度转动范围为 3.14 至 -3.14 之间。角度增加时，基础关节向左转动；角度减少时，基础关节向右转动。
	//     SHOULDER_JOINT的初始位置默认角度为 0，角度转动范围为 1.57 至 -1.57 之间。角度增加时，肩关节向前转动；角度减少时，肩关节会向后转动。
	//     ELBOW_JOINT的初始位置默认角度为 1.570796，角度转动范围为 3.14 至 -1.11 之间。角度增加时，肘关节向下转动；角度减少时，肘关节会往反方向转动。
	//     EOAT_JOINT的初始位置默认角度为 3.141593。产品默认的是夹爪关节，角度转动范围为 1.08 至 3.14 之间，角度减少时，夹爪关节会张开。若更换为手腕关节，角度转动范围为 1.08 至 5.20 之间，角度增加时，手腕关节向下转动；角度减少时，手腕关节向上转动。
	// spd：转动的速度，速度单位为步/秒，舵机的一圈为 4096 步，数值越大速度越快，当速度值为 0 时，以最大速度转动。
	// acc：转动开始和结束时的加速度，数值越小启停越平滑，数值可以为 0-254 ，单位为 100 步/秒^2。如设置为 10 时，则按照 1000 步每秒的平方加速度变速。当加速度值为 0 时，则按照最大的加速度运行。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":101,";

	oss << "\"joint\":";
	oss << to_string(joint);
	oss << ",";

	oss << "\"rad\":";
	oss << to_string(rad);
	oss << ",";

	oss << "\"spd\":";
	oss << to_string(spd);
	oss << ",";

	oss << "\"acc\":";
	oss << to_string(acc);

	oss << "}\n";
	return oss.str();
}
std::string CMD_SINGLE_JOINT_ANGLE(int joint, double angle, double spd, double acc)
{
	// 121：这条指令为CMD_SINGLE_JOINT_ANGLE，以角度制形式来控制机械臂某个关节的转动。
	// joint：关节序号。
	//     1：BASE_JOINT基础关节。
	//     2：SHOULDER_JOINT肩关节。
	//     3：ELBOW_JOINT肘关节。
	//     4：EOAT_JOINT手腕/夹爪关节。
	// angle：为需要转动到的角度，以各个关节的初始位置为准，各个关节的默认角度和转动方向如下：
	//     BASE_JOINT的初始位置默认角度为0°，角度转动范围为180°至-180°之间。角度增加时，基础关节向左转动；角度减少时，基础关节向右转动。
	//     SHOULDER_JOINT的初始位置默认角度为0°，角度转动范围为90°至-90°之间。角度增加时，肩关节向前转动；角度减少时，肩关节会向后转动。
	//     ELBOW_JOINT的初始位置默认角度为90°，角度转动范围为180°至-45°之间。角度增加时，肘关节向下转动；角度减少时，肘关节会往反方向转动。
	//     EOAT_JOINT的初始位置默认角度为180°。产品默认的是夹爪关节，角度转动范围为45°至180°之间，角度减少时，夹爪关节会张开。若更换为手腕关节，角度转动范围为45°至315°之间，角度增加时，手腕关节向下转动；角度减少时，手腕关节向上转动。
	// spd：转动的速度，速度单位为°/s，数值越大速度越快，当速度值为0时，以最大速度转动。
	// acc：转动开始和结束时的加速度，数值越小启停越平滑，单位为°/s^2。当加速度值为0时，则按照最大的加速度运行。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":121,";

	oss << "\"joint\":";
	oss << to_string(joint);
	oss << ",";

	oss << "\"angle\":";
	oss << to_string(angle);
	oss << ",";

	oss << "\"spd\":";
	oss << to_string(spd);
	oss << ",";

	oss << "\"acc\":";
	oss << to_string(acc);

	oss << "}\n";
	return oss.str();
}
std::string CMD_JOINTS_RAD_CTRL(double base, double shoulder, double elbow, double hand, double spd, double acc)
{
	// 102：表示这条指令为CMD_JOINTS_RAD_CTRL，以弧度制形式来控制机械臂全部关节的转动。
	// base：基础关节的角度，角度转动范围见上方“CMD_SINGLE_JOINT_CTRL”指令中rad键的说明。
	// shoulder：肩关节的角度。
	// elbow：肘关节的角度。
	// hand：夹爪/手腕关节的角度。
	// spd：转动的速度，速度单位为步/秒，舵机的一圈为4096步，数值越大速度越快，当速度值为0时，以最大速度转动。
	// acc：转动开始和结束时的加速度，数值越小启停越平滑，数值可以为0-254，单位为100步/秒^2。如设置为10时，则按照1000步每秒的平方加减速度变速。当加速度值为0时，则按照最大的加速度运行。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":102,";

	oss << "\"base\":";
	oss << to_string(base);
	oss << ",";

	oss << "\"shoulder\":";
	oss << to_string(shoulder);
	oss << ",";

	oss << "\"elbow\":";
	oss << to_string(elbow);
	oss << ",";

	oss << "\"hand\":";
	oss << to_string(hand);
	oss << ",";

	oss << "\"spd\":";
	oss << to_string(spd);
	oss << ",";

	oss << "\"acc\":";
	oss << to_string(acc);

	oss << "}\n";
	cout << oss.str() << "\n";
	return oss.str();
}
std::string CMD_JOINTS_ANGLE_CTRL(double base, double shoulder, double elbow, double hand, double spd, double acc)
{
	// 122：这条指令为CMD_JOINTS_ANGLE_CTRL，以角度制形式来控制机械臂所有关节的转动。
	// b：基础关节的角度，角度转动范围见“CMD_SINGLE_JOINT_ANGLE”指令中angle键的说明。
	// s：肩关节的角度。
	// e：肘关节的角度。
	// h：夹爪/手腕关节的角度。
	// spd：转动的速度，速度单位为°/s，数值越大速度越快，当速度值为0时，以最大速度转动。
	// acc：转动开始和结束时的加速度，数值越小启停越平滑，数值可以为0-254，单位为°/s^2。当加速度值为0时，则按照最大的加速度运行。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":122,";

	oss << "\"b\":";
	oss << to_string(base);
	oss << ",";

	oss << "\"s\":";
	oss << to_string(shoulder);
	oss << ",";

	oss << "\"e\":";
	oss << to_string(elbow);
	oss << ",";

	oss << "\"h\":";
	oss << to_string(hand);
	oss << ",";

	oss << "\"spd\":";
	oss << to_string(spd);
	oss << ",";

	oss << "\"acc\":";
	oss << to_string(acc);

	oss << "}\n";
	return oss.str();
}
std::string CMD_EOAT_HAND_CTRL(double cmd, double spd, double acc)
{
	// 106：这条指令为CMD_EOAT_HAND_CTRL，用来设置夹爪/手腕关节的转动角度。
	// cmd：为需要转动到的角度（以弧度制来显示）。EOAT_JOINT的初始位置默认角度为3.141593。
	//     产品默认的是夹爪关节，角度转动范围为1.08至3.14之间，角度减少时，夹爪关节会张开。
	// spd：转动的速度，速度单位为步/秒，舵机的一圈为4096步，数值越大速度越快，当速度值为0时，以最大速度转动。
	// acc：转动开始和结束时的加速度，数值越小启停越平滑，数值可以为0-254，单位为100步/秒^2。如设置为10时，则按照1000步每秒的平方加减速度变速。当加速度值为0时，则按照最大的加速度运行。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":106,";

	oss << "\"cmd\":";
	oss << to_string(cmd);
	oss << ",";

	oss << "\"spd\":";
	oss << to_string(spd);
	oss << ",";

	oss << "\"acc\":";
	oss << to_string(acc);

	oss << "}\n";
	return oss.str();
}
std::string CMD_SINGLE_AXIS_CRTL(double axis, double pos, double spd)
{
	// 机械臂的坐标轴定义基于右手定则，机械臂的正前方为X轴正方向，机械臂的前方的左侧为Y轴正方向，机械臂的竖直正上方为Z轴正方向。
	// 103：表示这条指令为CMD_SIGNLE_AXIS_CRTL，以给机械臂末端点单独轴的坐标位置来控制机械臂运动。
	// axis：表示为轴的序号。1-X轴；2-Y轴；3-Z轴；4-T轴，夹爪/手腕的角度，弧度制。
	// pos：某个轴的具体位置，单位为mm。例如上方的例子就是让机械臂末端点运动至Y轴的0位置，也就是机械臂的正前方。
	// spd：运动的速度，该数值越大速度越快，本移动命令底层包含了曲线速度控制函数，所以速度并不是恒定的。
	// 该指令会引起进程阻塞。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":103,";

	oss << "\"axis\":";
	oss << to_string(axis);
	oss << ",";

	oss << "\"pos\":";
	oss << to_string(pos);
	oss << ",";

	oss << "\"spd\":";
	oss << to_string(spd);

	oss << "}\n";
	return oss.str();
}
std::string CMD_XYZT_GOAL_CTRL(double x, double y, double z, double t, double spd)
{
	// 104：表示这条指令为CMD_XYZT_GOAL_CTRL，可以控制机械臂末端点位置的运动。
	// x，y，z，t：分别为四个轴的具体位置，单位为mm。具体可参考上方CMD_SINGLE_AXIS_CTRL指令中的介绍。
	// spd ：运动的速度，该数值越大速度越快，本移动命令底层包含了曲线速度控制函数，所以速度并不是恒定的。
	// 该指令会引起进程阻塞。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":104,";

	oss << "\"x\":";
	oss << to_string(x);
	oss << ",";

	oss << "\"y\":";
	oss << to_string(y);
	oss << ",";

	oss << "\"z\":";
	oss << to_string(z);
	oss << ",";

	oss << "\"t\":";
	oss << to_string(t);
	oss << ",";

	oss << "\"spd\":";
	oss << to_string(spd);

	oss << "}\n";
	return oss.str();
}
std::string CMD_XYZT_DIRECT_CTRL(double x, double y, double z, double t)
{
	// 1041：表示这条指令为CMD_XYZT_DIRECT_CTRL，可以控制机械臂末端点位置的运动。
	// x, y, z, t ：分别为四个轴的具体位置，单位为mm。具体可参考上方CMD_SINGLE_AXIS_CTRL指令中的介绍。
	// 注意：该命令与上一个命令的区别在于，该命令不会引起进程阻塞。因为底层没有插值计算，调用该指令后机械臂会以最快的速度运动到目标点。适合通过该命令连续给新的目标点的情况，且每个命令之间的目标点位置相差不要太大。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":1041,";

	oss << "\"x\":";
	oss << to_string(x);
	oss << ",";

	oss << "\"y\":";
	oss << to_string(y);
	oss << ",";

	oss << "\"z\":";
	oss << to_string(z);
	oss << ",";

	oss << "\"t\":";
	oss << to_string(t);

	oss << "}\n";
	return oss.str();
}
std::string CMD_SERVO_RAD_FEEDBACK()
{
	// x、y、z：分别代表末端点X轴、Y轴、Z轴的坐标。
	// b、s、e、t：分别代表基础关节、肩关节、肘关节、末端关节角度，以弧度制形式显示。
	// torB、torS、torE、torH：分别代表基础关节、肩关节、肘关节、末端关节的负载。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":105";
	oss << "}\n";
	return oss.str();
}
std::string CMD_CONSTANT_CTRL(int m, int axis, int cmd, double spd)
{
	// 123：这条指令为CMD_CONSTANT_CTRL，使机械臂各关节或机械臂末端点在输入指令后可以连续运动。
	// m：表示连续运动控制模式。
	//     0：角度控制模式。
	//     1：坐标控制模式。
	// axis：不同模式下控制转动的部位不一样。
	//     角度控制模式下：m值为0时，控制的是机械臂各关节角度的转动。1 - BASE基础关节；2 - SHOULDER肩关节；3 - ELBOW肘关节；4 - HAND夹爪/手腕关节。
	//     坐标控制模式下：m的值为1时，控制的是机械臂末端点坐标的转动。1 - X轴；2 - Y轴；3 - Z轴；4 - HAND夹爪/手腕关节。
	// cmd：运动的状态。
	//     0 - STOP停止运动。
	//     1 - INCREASE角度控制模式，增加角度；逆运动学控制模式，增加坐标轴值。
	//     2 - DECREASE角度控制模式，减少角度；逆运动学控制模式，减少坐标轴值。
	// spd：速度系数，数值越大速度越快，由于各个关节转速有上限，因此建议在0-20之间取值。
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":123,";

	oss << "\"m\":";
	oss << to_string(m);
	oss << ",";

	oss << "\"axis\":";
	oss << to_string(axis);
	oss << ",";

	oss << "\"cmd\":";
	oss << to_string(cmd);
	oss << ",";

	oss << "\"spd\":";
	oss << to_string(spd);

	oss << "}\n";
	return oss.str();
}
// 测试使用：
std::string build_json_string()
{
	std::ostringstream oss;
	oss << "{";
	oss << "\"T\":102,";
	oss << "\"base\":0.5,";
	oss << "\"shoulder\":0,";
	oss << "\"elbow\":1,";
	oss << "\"hand\":3.1415926,";
	oss << "\"spd\":0,";
	oss << "\"acc\":10";
	oss << "}\n";

	return oss.str();
}

void M_controller::run()
{
	ros::Duration(1).sleep();
	ros::Rate control_rate(CONTROL_RATE);

	serial::Serial sp_left, sp_right;
	serial::Timeout to = serial::Timeout::simpleTimeout(100);
	sp_left.setPort("/dev/ttyUSB1");
	sp_left.setBaudrate(115200);
	sp_left.setTimeout(to);
	sp_right.setPort("/dev/ttyUSB2");
	sp_right.setBaudrate(115200);
	sp_right.setTimeout(to);
	try
	{
		sp_left.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR("Unable to open port.");
	}
	if (sp_left.isOpen())
	{
		ROS_INFO("/dev/ttyUSB1 is opened.");
	}
	else
	{
	}

	try
	{
		sp_right.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR("Unable to open port.");
	}
	if (sp_right.isOpen())
	{
		ROS_INFO("/dev/ttyUSB2 is opened.");
	}
	else
	{
	}

	if (sp_right.isOpen() && sp_left.isOpen())
	{
		ROS_INFO("Both Open");
	}

	// 气泵程序//
	//  serial::Serial sp;
	//  serial::Timeout to = serial::Timeout::simpleTimeout(100);
	//  // 设置要打开的串口名称
	//  sp.setPort("/dev/ttyCH9344USB0");
	//  // 设置串口通信的波特率
	//  sp.setBaudrate(115200);
	//  // 串口设置timeout
	//  sp.setTimeout(to);
	//  try
	//  {
	//  	// 打开串口
	//  	sp.open();
	//  }
	//  catch (serial::IOException &e)
	//  {
	//  	ROS_ERROR("Unable to open port.");
	//  }
	//  // 判断串口是否打开成功
	//  if (sp.isOpen())
	//  {
	//  	//	ROS_INFO("/dev/ttyUSB0 is opened.");
	//  }
	//  else
	//  {
	//  }
	std::string output_l, output_r;
	std::string output = build_json_string();

	size_t feedback_left = sp_left.available();
	size_t feedback_right = sp_right.available();
	while (ros::ok())
	{
		startTime_control = ros::Time::now().toNSec();
		ros::spinOnce();
		// sp_right.write(output);
		// play(M_L);
		// play(M_R);
		// inverse_kine(M_L);
		// inverse_kine(M_R);
		// Print(M_L);
		// Print(M_R);
		// Pos_Control();
		if (mani_joint_l == 1)
		{
			sp_left.write(CMD_SINGLE_JOINT_ANGLE(mani_joint_l, angle_base_l, 10, 1));
			mani_joint_l = 5;
		}
		else if (mani_joint_l == 2)
		{
			sp_left.write(CMD_SINGLE_JOINT_ANGLE(mani_joint_l, angle_shoulder_l, 10, 1));
			mani_joint_l = 5;
		}
		else if (mani_joint_l == 3)
		{
			sp_left.write(CMD_SINGLE_JOINT_ANGLE(mani_joint_l, angle_elbow_l, 10, 1));
			mani_joint_l = 5;
		}
		else if (mani_joint_l == 4)
		{
			sp_left.write(CMD_SINGLE_JOINT_ANGLE(mani_joint_l, angle_eoat_l, 10, 1));
			mani_joint_l = 5;
		}

		if (mani_joint_r == 1)
		{
			sp_right.write(CMD_SINGLE_JOINT_ANGLE(mani_joint_r, angle_base_r, 10, 1));
			mani_joint_r = 5;
		}
		else if (mani_joint_r == 2)
		{
			sp_right.write(CMD_SINGLE_JOINT_ANGLE(mani_joint_r, angle_shoulder_r, 10, 1));
			mani_joint_r = 5;
		}
		else if (mani_joint_r == 3)
		{
			sp_right.write(CMD_SINGLE_JOINT_ANGLE(mani_joint_r, angle_elbow_r, 10, 1));
			mani_joint_r = 5;
		}
		else if (mani_joint_r == 4)
		{
			sp_right.write(CMD_SINGLE_JOINT_ANGLE(mani_joint_r, angle_eoat_r, 10, 1));
			mani_joint_r = 5;
		}

		if (axis_flag != 5)
		{
			sp_right.write(CMD_XYZT_DIRECT_CTRL(x, y, z, t));
			sp_left.write(CMD_XYZT_DIRECT_CTRL(x, y-70, z, t));
			axis_flag = 5;
		}

		if (pump_flag == 0)
		{
			printf("抓取->对接!\n");
			sp_right.write(CMD_MOVE_INIT());
			sp_left.write(CMD_MOVE_INIT());
			ros::Duration(5.0).sleep();
			sp_left.write(CMD_SINGLE_JOINT_ANGLE(3, 180, 20, 10));
			sp_right.write(CMD_SINGLE_JOINT_ANGLE(3, 180, 20, 10));
			sp_left.write(CMD_SINGLE_JOINT_ANGLE(1, 180, 20, 10));
			sp_right.write(CMD_SINGLE_JOINT_ANGLE(1, -180, 20, 10));
			ros::Duration(10.0).sleep();
			sp_left.write(CMD_SINGLE_JOINT_ANGLE(2, -90, 20, 10));
			sp_right.write(CMD_SINGLE_JOINT_ANGLE(2, -90, 20, 10));
			pump_flag = 6;
			ros::Duration(10.0).sleep();
			sp_left.write(CMD_TORQUE_CTRL(0));
			sp_right.write(CMD_TORQUE_CTRL(0));
		}
		else if (pump_flag == 1)
		{
			printf("对接->抓取!\n");
			sp_left.write(CMD_TORQUE_CTRL(1));
			sp_right.write(CMD_TORQUE_CTRL(1));
			sp_left.write(CMD_SINGLE_JOINT_ANGLE(2, 0, 20, 10));
			sp_right.write(CMD_SINGLE_JOINT_ANGLE(2, 0, 20, 10));
			ros::Duration(5.0).sleep();
			sp_left.write(CMD_SINGLE_JOINT_ANGLE(1, 0, 20, 10));
			sp_right.write(CMD_SINGLE_JOINT_ANGLE(1, 0, 20, 10));
			ros::Duration(10.0).sleep();
			sp_right.write(CMD_MOVE_INIT());
			sp_left.write(CMD_MOVE_INIT());

			pump_flag = 6;
		}
		else if (pump_flag == 2)
		{
			printf("抓取示例!\n");
			sp_left.write(CMD_XYZT_DIRECT_CTRL(200, 70, 80, 1.57));
			sp_right.write(CMD_XYZT_DIRECT_CTRL(350, -90.5, 120, 3.14));
			pump_flag = 6;
		}
		else if (pump_flag == 3)
		{
			// sp.write("#001P2500T0000!");
			// sp_right.write(output);
			// ROS_INFO("%s", output.c_str());
			sp_right.write(CMD_XYZT_DIRECT_CTRL(235, 0, 234, 3.14));
			pump_flag = 6;
		}
		else if (pump_flag == 4)
		{
			// printf("B!\n");
			output_l = CMD_SINGLE_JOINT_ANGLE(3, 180, 10, 0);
			sp_left.write(output_l);
			ROS_INFO("%s", output_l.c_str());

			output_r = CMD_SINGLE_JOINT_ANGLE(3, 180, 10, 0);
			sp_right.write(output_r);
			ROS_INFO("%s", output_r.c_str());
			pump_flag = 6;
		}
		else if (pump_flag == 5)
		{
			// sp.write("#001P1500T0000!");
			// sp_right.write(CMD_TORQUE_CTRL(1));
			// sp_right.write(CMD_DYNAMIC_ADAPTATION(0, 1000, 1000, 1000, 1000));
			sp_right.write(CMD_MOVE_INIT());
			sp_left.write(CMD_MOVE_INIT());
			pump_flag = 6;
		}
		else
		{
			// sp.write("#000P1500T0000!");
			// sp.write("#001P1500T0000!");
		}
		control_rate.sleep();
		endTime_T = ros::Time::now().toNSec();
		// std::cout << "机械臂控制周期:" << (endTime_T - startTime_control) / 1e6 << "ms" << std::endl
		// 		  << std::endl
		// 		  << std::endl;
	}
	sp_left.close();
	sp_right.close();
}