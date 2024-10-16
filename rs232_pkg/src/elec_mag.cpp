#include <vector>
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <iomanip>
#include "message/rs232_elec_mag_state.h"
#include "message/rs232_elec_mag_ctrl.h"

class ICommunicator
{
public:
    virtual ~ICommunicator() = default;
    virtual bool send(const std::vector<uint8_t>& data) = 0;
    virtual bool receive(std::vector<uint8_t>& data, size_t size) = 0;
};

class SerialCommunicator : public ICommunicator
{
public:
    SerialCommunicator(const std::string& port, uint32_t baud_rate)
        : serial_port(port, baud_rate, serial::Timeout::simpleTimeout(100000))
    {
        if (!serial_port.isOpen())
        {
            serial_port.open();
        }
    }

    ~SerialCommunicator()
    {
        if (serial_port.isOpen())
        {
            serial_port.close();
        }
    }

    bool send(const std::vector<uint8_t>& data) override
    {
        size_t bytes_written = serial_port.write(data);
        return bytes_written == data.size();
    }

    bool receive(std::vector<uint8_t>& data, size_t size) override
    {
        size_t bytes_read = serial_port.read(data, size);
        if (bytes_read != size)
        {
            std::cerr << "Failed to read " << size << " bytes, actually read " << bytes_read << " bytes." << std::endl;
            return false;
        }

        return true;
    }

private:
    serial::Serial serial_port;
};


class Electromagnet
{
public:
    Electromagnet(ICommunicator& communicator)
        : communicator(communicator), state(false) {}

    bool turnOn(uint8_t id)
    {
        return sendCommand(COMMAND_TURN_ON, id);
    }

    bool turnOff(uint8_t id)
    {
        return sendCommand(COMMAND_TURN_OFF, id);
    }

    bool queryState()
    {
        bool flag = sendCommand(COMMAND_QUERY_STATE, 0);
        flag = flag & receiveState();
        return flag;
    }

    uint8_t getState() const { return state; }

private:
    uint8_t state;
    ICommunicator& communicator;

    static const uint8_t FRAME_START = 0xAA;
    static const uint8_t COMMAND_TURN_ON = 0x01;
    static const uint8_t COMMAND_TURN_OFF = 0x02;
    static const uint8_t COMMAND_QUERY_STATE = 0x03;

    bool sendCommand(uint8_t command, uint8_t id)
    {
        uint8_t command_byte = (id << 4) | command;
        std::vector<uint8_t> frame = {FRAME_START, command_byte, 0x00};
        frame[2] = frame[0] ^ frame[1];

        if (!communicator.send(frame))
        {
            std::cerr << "Failed to send command to electromagnet " << id << std::endl;
            return false;
        }
        return true;
    }

    bool receiveState()
    {
        std::vector<uint8_t> response;
        if (!communicator.receive(response, 3))
        {
            std::cerr << "Failed to receive state from electromagnet " << std::endl;
            return false;
        }

        uint8_t checksum = response[0] ^ response[1];

        if (checksum != response[2])
        {
            std::cerr << "Failed checksum" << std::endl;
            return false;
        }

        if (response[0] == 0xAA)
        {
            state = response[1] & 0x0F;
            return true;
        }

        std::cerr << "Invalid response received for electromagnet " << std::endl;
        return false;
    }
};

void rs232_elec_mag_ctrl_Callback(const message::rs232_elec_mag_ctrl::ConstPtr& ctrl, Electromagnet& em)
{
    uint8_t ctrl_in_4bits = 0;
    ctrl_in_4bits |= ctrl->ctrl4;
    ctrl_in_4bits <<= 1;
    ctrl_in_4bits |= ctrl->ctrl3;
    ctrl_in_4bits <<= 1;
    ctrl_in_4bits |= ctrl->ctrl2;
    ctrl_in_4bits <<= 1;
    ctrl_in_4bits |= ctrl->ctrl1;

    em.turnOn(ctrl_in_4bits);
    em.turnOff(~ctrl_in_4bits);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rs232_elec_mag");
    ros::NodeHandle nh;

    SerialCommunicator communicator("/dev/ttyS0", 115200);
    Electromagnet em(communicator);

    ros::Rate r(20);
    ros::Subscriber sub = nh.subscribe<message::rs232_elec_mag_ctrl>("/mag_cmd", 100, boost::bind(rs232_elec_mag_ctrl_Callback, _1, boost::ref(em)));//订阅udpslave收到的电磁铁控制命令
    ros::Publisher pub = nh.advertise<message::rs232_elec_mag_state>("/mag_state", 1000);//向udp_pkg发送消息，实时反馈状态
    message::rs232_elec_mag_state state;

    while (ros::ok())
    {
        if (em.queryState())
        {
        	state.header.stamp = ros::Time::now();
            uint8_t state_in_4bits = em.getState();
            state.state1 = state_in_4bits & 1;
            state_in_4bits >>= 1;
            state.state2 = state_in_4bits & 1;
            state_in_4bits >>= 1;
            state.state3 = state_in_4bits & 1;
            state_in_4bits >>= 1;
            state.state4 = state_in_4bits & 1;
            pub.publish(state);
        }
        else
        {
            std::cerr << "Failed to get elec_mag states from rs232" << std::endl;
        }

        r.sleep();
        ros::spinOnce(); // Process ROS callbacks
    }

    return 0;
}
