#include <ros/ros.h>
#include <modbus/modbus.h>
#include <control_msgs/GripperCommand.h>
#include "robotiq_epick_vaccum_gripper_msgs/GripperStatus.h"

class RobotiQVaccumGripperNode
{
    public:
        RobotiQVaccumGripperNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        {
            std::string port_name;
            int baudrate;
            double rate;
            double timeout;

            pnh.param<std::string>("port_name", port_name, "/dev/ttyUSB0");
            pnh.param<int>("baudrate", baudrate, 115200);
            pnh.param<double>("rate", rate, 10.0);
            pnh.param<double>("gripper_timeout", timeout, 0.0);


            ROS_INFO("Get parameters port_name [%s], baudrate: %d, rate: %4.1f...", port_name.c_str(), baudrate, rate);

            modbus_ = modbus_new_rtu(port_name.c_str(), baudrate, 'N', 8, 1);
            assert(modbus_ != NULL);

            modbus_set_slave(modbus_, 9);
            if(modbus_connect(modbus_) == -1)
            {
                modbus_free(modbus_);
                assert(false);
            }

            req_gripper_pos_ = 255;
            req_gripper_min_force_ = 90;
            req_gripper_timeout_ = (uint8_t)(timeout * 10);

            ROS_INFO("Gripper timeout [%d]... ", req_gripper_timeout_);

            sub_gripper_command_ = nh_.subscribe("gripper_command", 1, &RobotiQVaccumGripperNode::callback_gripper_command, this);
            pub_gripper_status_ = nh_.advertise<robotiq_epick_vaccum_gripper_msgs::GripperStatus>("gripper_status", 10);
            timer_pub_state_ = nh_.createTimer(ros::Duration(0.01), &RobotiQVaccumGripperNode::callback_timer_pub_state, this);

            ROS_INFO("Initialized...");
        }
        ~RobotiQVaccumGripperNode() {}

    private:
        void callback_timer_pub_state(const ros::TimerEvent& event)
        {
            // Send command to Gripper
            uint16_t send_registers[3] = {0, 0, 0};

            send_registers[0] = 0x0900;  // gACT and gGTO is always on
            send_registers[1] = req_gripper_pos_ & 0xFF;
            send_registers[2] = (req_gripper_timeout_ << 8) + (req_gripper_min_force_);

            modbus_write_registers(modbus_, 0x3E8, 3, send_registers);
            ros::Duration(0.01).sleep();

            // Receive status from Gripper
            uint16_t recv_registers[3] = {0, };
            modbus_read_registers(modbus_, 0x7D0, 3, recv_registers);

            robotiq_epick_vaccum_gripper_msgs::GripperStatus msg = robotiq_epick_vaccum_gripper_msgs::GripperStatus();
            msg.g_act = (uint8_t)(recv_registers[0] >> 8) & 0x01;
            msg.g_mod = (uint8_t)(recv_registers[0] >> 9) & 0x03;
            msg.g_gto = (uint8_t)(recv_registers[0] >> 11) & 0x01;
            msg.g_sta = (uint8_t)(recv_registers[0] >> 12) & 0x03;
            msg.g_obj = (uint8_t)(recv_registers[0] >> 14) & 0x03;
            msg.g_vas = (uint8_t)(recv_registers[0]) & 0x03;
            msg.g_flt = (uint8_t)(recv_registers[1] >> 8) & 0x0F;
            msg.k_flt = (uint8_t)(recv_registers[1] >> 12) & 0x0F;
            msg.g_pr = (uint8_t)(recv_registers[1]) & 0xFF;
            msg.g_po = (uint8_t)(recv_registers[2] >> 8) & 0xFF;

            pub_gripper_status_.publish(msg);
        }

        void callback_gripper_command(const control_msgs::GripperCommandConstPtr &msg)
        {
            req_gripper_pos_ = (uint8_t)((1.0 - msg->position) * 100.0);
            // req_gripper_min_force_ = (uint8_t)(msg->max_effort * 255.0);
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_gripper_status_;
        ros::Subscriber sub_gripper_command_;
        ros::Timer timer_pub_state_;
        modbus_t *modbus_;

        uint8_t req_gripper_pos_;
        uint8_t req_gripper_min_force_;
        uint8_t req_gripper_timeout_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotiq_vaccum_gripper_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    RobotiQVaccumGripperNode m(nh, pnh);
    ros::spin();

    return 0;
}