#include "udp_master.h"

std::mutex _reconfigure_mtx;
std::vector<double> x_diff={0.0,0.0,0.0};
std::vector<double> y_diff={0.0,1.0,-1.0};
std::vector<double> yaw_diff={0.0,0.0,0.0};

UDPMaster::UDPMaster(int _i){
    std::string cfg_path;
    ros::param::get("cfg_path", cfg_path);
    YAML::Node cfg = YAML::LoadFile(cfg_path);
    auto self_cfg = cfg["self"];
    auto remote_cfg = cfg["remote"][_i];

    self_id = 0;
    if (self_cfg["id"]) {
        self_id = self_cfg["id"].as<int>();
    }
    else {
        ROS_ERROR("id undefined");
    }
    remote_id = remote_cfg["id"].as<int>();
    int LOCAL_PORT = remote_cfg["recv_port"].as<int>();
    std::string REMOTE_IP = remote_cfg["ip"].as<std::string>();
    int REMOTE_PORT = remote_cfg["send_port"].as<int>();


    recv_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_sock < 0) {
        ROS_ERROR("Socket creation failed");
        exit(1);
    }
    send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_sock < 0) {
        ROS_ERROR("Socket creation failed");
        exit(1);
    }

    // Local address setup
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;//监听所有来源
    local_addr.sin_port = htons(LOCAL_PORT);
    int buflen = 1024;
    // 设置读缓存大小
    if(0!=setsockopt(recv_sock,SOL_SOCKET,SO_RCVBUF,&buflen,sizeof(buflen)))
    {
        exit(1);
    }
    if (bind(recv_sock, (struct sockaddr*) &local_addr, sizeof(local_addr)) < 0) {
        ROS_ERROR("Socket bind failed");
        exit(1);
    }

    int buffer_size = 1024; // 设置为1MB
    if (setsockopt(recv_sock, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0) {
        ROS_ERROR("Setting receive buffer size failed");
    }

    // Remote address setup
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(REMOTE_IP.c_str());//向主机发送
    remote_addr.sin_port = htons(REMOTE_PORT);

    // if (bind(send_sock, (struct sockaddr*) &remote_addr, sizeof(remote_addr)) < 0) {
    //     ROS_ERROR("Socket bind failed");
    //     exit(1);
    // }

    // ROS subscribers and publishers
    nh = ros::NodeHandle();
    auto send_topics = remote_cfg["send_topics"];
    auto recv_topics = remote_cfg["recv_topics"];

    assert(send_topics.size() == 4);
    assert(recv_topics.size() == 4);
    assert(send_topics[0].as<std::string>().find(std::to_string(remote_id)) != send_topics[0].as<std::string>().npos);

    subs.push_back(nh.subscribe(send_topics[0].as<std::string>(), 10, &UDPMaster::twistCallback, this));
    subs.push_back(nh.subscribe(send_topics[1].as<std::string>(), 10, &UDPMaster::catCmdCallback, this));
    subs.push_back(nh.subscribe(send_topics[2].as<std::string>(), 10, &UDPMaster::slave_motor_control_Callback, this));//从机的电机调整信息
    subs.push_back(nh.subscribe(send_topics[3].as<std::string>(), 10, &UDPMaster::slave_mag_Control_Callback, this));//从机的电磁铁控制命令

    pubs.push_back(nh.advertise<nav_msgs::Odometry>(recv_topics[0].as<std::string>(), 10));
    pubs.push_back(nh.advertise<std_msgs::Bool>(recv_topics[1].as<std::string>(), 10));
    pubs.push_back(nh.advertise<message::imu_controller>(recv_topics[2].as<std::string>(), 10));
    pubs.push_back(nh.advertise<message::rs232_elec_mag_state>(recv_topics[3].as<std::string>(), 10));

    ROS_INFO("UDP Connection Between #%d Initialized as Index %d", remote_id, _i);
}

UDPMaster::~UDPMaster() {
    close(send_sock);
    close(recv_sock);
}

void UDPMaster::twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    twist_buf = *msg;
    ROS_INFO("relay slave %d twist", remote_id);
    return;
    
    
}

void UDPMaster::catCmdCallback(const std_msgs::Bool::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    bool_buf = *msg;
    ROS_INFO("relay slave %d command", remote_id);
    return;
    
}


void UDPMaster::slave_motor_control_Callback(const message::cmb_slave::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    motor_buf = *msg;
    ROS_INFO("left wheel: %f command", msg->T_l);
    ROS_INFO("right wheel: %f command", msg->T_r);
    return;
}


void UDPMaster::slave_mag_Control_Callback(const message::rs232_elec_mag_ctrl::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    mag_buf = *msg;
    // ROS_INFO("relay slave %d command", remote_id);
    return;
}

void UDPMaster::transferData() {
    // Send
    sendData();

    // Recv
    recvData();
    recvData();//receive twice to avoid unmatched frequency
}

void UDPMaster::sendData() {
    std::unique_lock<std::mutex> ulck(_mtx);
    uint32_t serial_size = 16;
    serial_size += ros::serialization::serializationLength(twist_buf);
    serial_size += ros::serialization::serializationLength(bool_buf);
    serial_size += ros::serialization::serializationLength(motor_buf);   //modified ar 2024/08/26 
    serial_size += ros::serialization::serializationLength(mag_buf);

    boost::shared_array<uint8_t> send_buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(send_buffer.get(), serial_size);
    ros::serialization::serialize(stream, twist_buf);
    ros::serialization::serialize(stream, bool_buf);
    ros::serialization::serialize(stream, motor_buf);
    ros::serialization::serialize(stream, mag_buf);
    ulck.unlock();
    sendto(send_sock, send_buffer.get(), serial_size, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));
}

void UDPMaster::recvData() {
    nav_msgs::Odometry odom_msg;
    std_msgs::Bool bool_msg;
    message::imu_controller imu_msg;
    uint32_t msg_len = 32;
    msg_len += ros::serialization::serializationLength(odom_msg);
    msg_len += ros::serialization::serializationLength(bool_msg);
    msg_len += ros::serialization::serializationLength(imu_msg);
    boost::shared_array<uint8_t> recv_buffer(new uint8_t[msg_len]);
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    int len = recvfrom(recv_sock, recv_buffer.get(), msg_len, MSG_DONTWAIT, (struct sockaddr*) &sender_addr, &sender_len);
    // ROS_INFO("len = %d : %d, sender = %d:%d",len,msg_len,sender_addr.sin_addr.s_addr,sender_addr.sin_port);
    if (len > 0) {
        ros::serialization::IStream stream((uint8_t*) recv_buffer.get(), msg_len);
        ros::serialization::deserialize(stream, odom_msg);
        std::unique_lock<std::mutex> ulck(_reconfigure_mtx);
        odom_msg.pose.pose.position.x += x_diff[remote_id - 1];
        odom_msg.pose.pose.position.y += y_diff[remote_id - 1];
        //
        // auto quat = odom_msg.pose.pose.orientation;
        ulck.unlock();
        // ROS_INFO("relay slave %d odom", remote_id);
        pubs[0].publish(odom_msg);
        ros::serialization::deserialize(stream, bool_msg);
        pubs[1].publish(bool_msg);
        ros::serialization::deserialize(stream, imu_msg);
        pubs[2].publish(imu_msg);
    }
}

void reconfigureCallback(udpmaster_pkg::initialPoseConfig& config, uint32_t level) {
    std::unique_lock<std::mutex> ulck(_reconfigure_mtx);
    x_diff[0] = config.robot1_x;
    x_diff[1] = config.robot2_x;
    x_diff[2] = config.robot3_x;
    y_diff[0] = config.robot1_y;
    y_diff[1] = config.robot2_y;
    y_diff[2] = config.robot3_y;
    yaw_diff[0] = config.robot1_yaw;
    yaw_diff[1] = config.robot2_yaw;
    yaw_diff[2] = config.robot3_yaw;
}
