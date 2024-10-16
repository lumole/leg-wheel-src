#include "manipulator.h"
#include <kalman_filter/ukf.hpp>
#include <cmath>
#include "message/pose.h"
// WorldFrameArmController::WorldFrameArmController(std::vector<double> base_position, std::vector<double> arm_offset) {
//     // 初始化机械臂控制器
//     // base_position: 底盘在绝对坐标系中的初始位置，格式为 (x, y, z)
//     // arm_offset: 机械臂基座相对于底盘的偏移，格式为 (x_offset, y_offset, z_offset)
//     this->base_position = base_position;
//     this->arm_offset = arm_offset;
//     arm_joint_positions = {0.0, 0.0, 0.0}; // 假设机械臂的关节初始状态
    
//     InitBaseAbsolutePosition();
// }

// void WorldFrameArmController::InitBaseAbsolutePosition(){
//     // Set up a new KF that has 5 states, 2 inputs, and 4 observers.
//     // States: [x, y, theta, v, omega]
//     // Inputs: [v_L, v_R]
//     // Observers: [v_m, omega_m]
    
//     kf_base = new kalman_filter::kf_t(5,2,2);
//     // Populate the model matrices accordingly.
//     // State Transition Matrix (A)
//     double delta_t = 1/250; // Time step
//     kf.A(0, 0) = 1.0;                   // x -> x
//     kf.A(0, 3) = delta_t * cos(0);      // v -> x (initially cos(theta) = 1)
//     kf.A(1, 1) = 1.0;                   // y -> y
//     kf.A(1, 3) = delta_t * sin(0);      // v -> y (initially sin(theta) = 0)
//     kf.A(2, 2) = 1.0;                   // theta -> theta
//     kf.A(2, 4) = delta_t;               // omega -> theta
//     kf.A(3, 3) = 1.0;                   // v -> v
//     kf.A(4, 4) = 1.0;                   // omega -> omega

//     // Control Input Matrix (B)
//     double b = 0.5; // Distance between wheels
//     kf.B(3, 0) = 0.5;                   // v_L -> v
//     kf.B(3, 1) = 0.5;                   // v_R -> v
//     kf.B(4, 0) = -1.0 / b;              // v_L -> omega
//     kf.B(4, 1) = 1.0 / b;               // v_R -> omega

//     // Observation Matrix (H)
//     kf.H(0, 3) = 1.0;   // v -> v_m
//     kf.H(1, 4) = 1.0;   // omega -> omega_m

//     // Set up process and observation covariance matrices.
//     kf.Q = Eigen::MatrixXd::Identity(5, 5) * 0.01; // Process Covariance
//     kf.R = Eigen::MatrixXd::Identity(2, 2) * 0.05; // Observation Covariance

//     // OPTIONAL: Set the initial state and covariance of the model.
//     Eigen::VectorXd x_o(5);
//     x_o << 0, 0, 0, 0, 0; // Initial state [x, y, theta, v, omega]
//     Eigen::MatrixXd P_o = Eigen::MatrixXd::Identity(5, 5) * 0.5;
//     kf.initialize_state(x_o, P_o);
// }


// std::vector<double> WorldFrameArmController::getBaseAbsolutePosition() {


//     // Pass inputs into the filter.
//     kf.new_input(0, v_L);
//     kf.new_input(1, v_R);

//     // Take some measurement as an observation:
//     double_t v_m = (v_L + v_R) / 2.0;  // Measured velocity (encoder)
//     double_t omega_m = (v_R - v_L) / b; // Measured angular velocity (IMU)

//     // Pass observations into the filter.
//     kf.new_observation(0, v_m);
//     kf.new_observation(1, omega_m);

//     // Run the filter predict/update iteration.
//     kf.iterate();

//     // You may grab the current state and covariance estimates from the filter at any time:
//     const Eigen::VectorXd& estimated_state = kf.state();

//     // 获取底盘的绝对坐标系位置
//     return estimated_state;
// }

RobotDynamicsUKF::RobotDynamicsUKF() : ukf_t(5, 4)
{
    pose_pub = n.advertise<message::pose>("base_world", 1000);
    Motor_sub = n.subscribe<message::interface_controller>("/motor_feedback", 1, &RobotDynamicsUKF::Motor_Input, this);
    imu_sub = n.subscribe("/imu_topic", 1, &RobotDynamicsUKF::IMU_Input, this);

    // Initialize process covariance matrix Q
    Q = Eigen::MatrixXd::Identity(5, 5) * 0.1;
    Q(4, 4) = 1;

    // Initialize measurement covariance matrix R
    R = Eigen::MatrixXd::Identity(4, 4) * 0.1;
    R(0, 0) = 10;
    // R(1, 1) = 0.5;
    R(2, 2) = 1.0;
    R(3, 3) = 1.0;

    // Set the initial state and covariance of the model
    Eigen::VectorXd x_o(5);
    x_o << 0, 0, 0, 0, 0; // Initial state [x, y, theta, v, omega]
    Eigen::MatrixXd P_o = Eigen::MatrixXd::Identity(5, 5)*0.5;
    this->initialize_state(x_o, P_o);
}

void RobotDynamicsUKF::state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const
{
    // State transition model equations
    double v = (v_L + v_R) / 2.0;                // Linear velocity
    double omega = imu_msg.angle_vel_yaw;         // Angular velocity

    // Update state
    x(0) = xp(0) + v * std::cos(xp(2)) * delta_t; // x_{k+1} = x_k + v * cos(theta) * delta_t
    x(1) = xp(1) + v * std::sin(xp(2)) * delta_t; // y_{k+1} = y_k + v * sin(theta) * delta_t
    x(2) = xp(2) + omega * delta_t;               // theta_{k+1} = theta_k + omega * delta_t
    x(3) = v;                                     // v remains constant (predicted as current velocity)
    x(4) = omega;                                 // omega remains constant (predicted as current angular velocity)
}

void RobotDynamicsUKF::observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const
{
    // Observation model equations
    z(0) = x(3);                                 // v_m: Measured velocity from encoder
    z(1) = x(4);                                 // omega_m: Measured angular velocity from IMU
    z(2) = x(3) * std::cos(x(2));                // a_x: Acceleration in x-direction
    z(3) = x(3) * std::sin(x(2));                // a_y: Acceleration in y-direction
}


void RobotDynamicsUKF::IMU_Input(const message::imu_controller::ConstPtr &msg)
{
    imu_msg = *msg;
}

void RobotDynamicsUKF::Motor_Input(const message::interface_controller::ConstPtr &msg) 
{
    motor_msg = *msg;
}

void RobotDynamicsUKF::update()
{
// The following can be run in a loop:
// while (ros::ok()) {
    // Calculate some new control input and store within the model:
    v_L = motor_msg.speed2*radius; 
    v_R = motor_msg.speed1*radius; 

    // Take some measurement as an observation:
    double_t v_m = (v_L + v_R) / 2.0;  // Measured velocity (encoder)
    double_t omega_m = imu_msg.angle_vel_yaw; // Measured angular velocity (IMU)

    double acc_x = imu_msg.acc_x*std::cos(imu_msg.angle_pitch);
    double_t a_x = acc_x * std::cos(state(2));    // Simplified acceleration x-direction
    double_t a_y = acc_x * std::sin(state(2));    // Simplified acceleration y-direction

    // ROS_INFO("v_m: %f, omega_m: %f, a_x: %f, a_y: %f", v_m, omega_m, a_x, a_y); // Debug info

    // Pass observations into the filter.
    new_observation(0, v_m);
    new_observation(1, omega_m);
    new_observation(2, a_x);
    new_observation(3, a_y);

    // Run the filter predict/update iteration.
    iterate();

    // You may grab the current state and covariance estimates from the filter at any time:
    message::pose msg;
    msg.x =  state(0);
    msg.y = state(1);
    msg.theta = state(2);
    pose_pub.publish(msg);

    // Output all the states for debugging
    // ROS_INFO("State: [x: %f, y: %f, theta: %f, velocity: %f, angular_velocity: %f]", 
    //      state(0), state(1), state(2), state(3), state(4));
    // Add a sleep here if running in real-time, to match sensor frequency.
    // ros::spinOnce();
    // r.sleep();
// }
}



WorldFrameArmController::WorldFrameArmController()
{
    ukf_base = new RobotDynamicsUKF();
}

std::vector<double> WorldFrameArmController::getBaseAbsolutePosition()
{
    return {ukf_base->state(0), ukf_base->state(1)};//x y
}

extern std::string CMD_XYZT_DIRECT_CTRL(double x, double y, double z, double t);
void WorldFrameArmController::run()
{
    M_controller *controller=new M_controller();
    controller->initializePorts();
    ros::Rate r(250);
    while(ros::ok())
    {
        ukf_base->update();

        base_world_position = getBaseAbsolutePosition();
        arm_world_position = {235, 0, 234, 3.14};//x y z t
        arm_base_position.assign(arm_world_position.begin(), arm_world_position.end());
        arm_base_position[0] = arm_world_position[0]-base_world_position[0]*1000;
        arm_base_position[1] = arm_world_position[1]-base_world_position[1]*1000;


        controller->sp_right.write(CMD_XYZT_DIRECT_CTRL(arm_base_position[0], 
                    arm_base_position[1], arm_base_position[2], arm_base_position[3]));
        ros::spinOnce();
        r.sleep();
    }
}

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "WorldFrameArmController");
    ros::NodeHandle n;


    WorldFrameArmController wfac;
    wfac.run();
    
    return 0;
} 