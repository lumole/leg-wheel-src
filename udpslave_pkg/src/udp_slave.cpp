#include "udp_slave.h"


UDPSlave::UDPSlave() {
    std::string cfg_path;
    ros::param::get("cfg_path", cfg_path);
    YAML::Node cfg = YAML::LoadFile(cfg_path);
    auto self_cfg = cfg["self"];
    auto remote_cfg = cfg["remote"][0];

    int self_id = 0;
    if (self_cfg["id"]) {
        self_id = self_cfg["id"].as<int>();
    }
    else {
        ROS_ERROR("id undefined");
    }
    int LOCAL_PORT = self_cfg["port"].as<int>();
    std::string REMOTE_IP = remote_cfg["ip"].as<std::string>();
    int REMOTE_PORT = remote_cfg["port"].as<int>();

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

    if (bind(recv_sock, (struct sockaddr*) &local_addr, sizeof(local_addr)) < 0) {
        ROS_ERROR("Socket bind failed");
        exit(1);
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

    subs.push_back(nh.subscribe(send_topics[0].as<std::string>(), 1000, &UDPSlave::odomCallback, this));
    subs.push_back(nh.subscribe(send_topics[1].as<std::string>(), 1000, &UDPSlave::catCmdCallback, this));
    subs.push_back(nh.subscribe(send_topics[2].as<std::string>(), 1000, &UDPSlave::Imu_Callback, this));
    subs.push_back(nh.subscribe(send_topics[3].as<std::string>(), 1000, &UDPSlave::Mag_Callback, this));

    pubs.push_back(nh.advertise<geometry_msgs::Twist>(recv_topics[0].as<std::string>(), 1000));
    pubs.push_back(nh.advertise<std_msgs::Bool>(recv_topics[1].as<std::string>(), 1000));
    pubs.push_back(nh.advertise<message::cmb_interface>(recv_topics[2].as<std::string>(), 1000));
    pubs.push_back(nh.advertise<message::rs232_elec_mag_ctrl>(recv_topics[3].as<std::string>(), 1000));

    ROS_INFO("UDP Communication Initialized.");
    std::cout<<send_topics[2].as<std::string>()<<std::endl;
}

UDPSlave::~UDPSlave() {
    close(send_sock);
    close(recv_sock);
}

void UDPSlave::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    odom_buf = *msg;
    return;
    // std::string data = serializeOdom(*msg);
    // data += generateChecksum(data);
    uint32_t serial_size = ros::serialization::serializationLength(*msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, *msg);
    sendto(send_sock, buffer.get(), serial_size, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));
}


void UDPSlave::catCmdCallback(const std_msgs::Bool::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    bool_buf = *msg;
    ROS_INFO("relay slave command");
    return;
}

void UDPSlave::Imu_Callback(const message::imu_controller::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    imu_buf = *msg;
    // ROS_INFO("SlAVE IMU");
    return;
}

void UDPSlave::Mag_Callback(const message::rs232_elec_mag_state::ConstPtr& msg) {
    std::unique_lock<std::mutex> ulck(_mtx);
    mag_buf = *msg;
    // ROS_INFO("SlAVE IMU");
    return;
}

void UDPSlave::transferData() {
    // Send
    std::unique_lock<std::mutex> ulck(_mtx);
    uint32_t serial_size = 16;
    serial_size += ros::serialization::serializationLength(odom_buf);
    serial_size += ros::serialization::serializationLength(bool_buf);
    serial_size += ros::serialization::serializationLength(imu_buf);   //从机收到imu程序
    serial_size += ros::serialization::serializationLength(mag_buf);   //从机收到mag
    boost::shared_array<uint8_t> send_buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(send_buffer.get(), serial_size);
    ros::serialization::serialize(stream, odom_buf);
    ros::serialization::serialize(stream, bool_buf);
    ros::serialization::serialize(stream, imu_buf);
    ros::serialization::serialize(stream, mag_buf);

    ulck.unlock();
    sendto(send_sock, send_buffer.get(), serial_size, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));

    // Recv
    geometry_msgs::Twist twist_msg;
    std_msgs::Bool bool_msg;
    message::cmb_interface motor_msg;
    message::rs232_elec_mag_ctrl mag_msg;
    uint32_t msg_len = 16;
    msg_len += ros::serialization::serializationLength(twist_msg);
    msg_len += ros::serialization::serializationLength(bool_msg);
    msg_len += ros::serialization::serializationLength(motor_msg);
    msg_len += ros::serialization::serializationLength(mag_msg);
    boost::shared_array<uint8_t> recv_buffer(new uint8_t[msg_len]);
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    int len = recvfrom(recv_sock, recv_buffer.get(), msg_len, MSG_DONTWAIT, (struct sockaddr*) &sender_addr, &sender_len);

    if (len > 0) {
        ros::serialization::IStream stream((uint8_t*) recv_buffer.get(), msg_len);
        ros::serialization::deserialize(stream, twist_msg);
        pubs[0].publish(twist_msg);
        ros::serialization::deserialize(stream, bool_msg);
        pubs[1].publish(bool_msg);
        //ROS_INFO("master!!!!");
        ros::serialization::deserialize(stream, motor_msg);//电机需要调整位置的信息
        pubs[2].publish(bool_msg);
        ros::serialization::deserialize(stream, mag_msg);//发布电磁铁的控制命令的信息
        pubs[3].publish(bool_msg);

    }
}
