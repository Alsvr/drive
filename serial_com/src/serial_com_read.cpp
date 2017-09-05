#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "dev_mavlink.h"
#include "stdio.h"

serial::Serial ser;
_MAVLINK mavlink;

void write_callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher  read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);

        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    //发送频率是1ms
    ros::Rate loop_rate(1000000); //hz
    while (ros::ok())
    {

        ros::spinOnce();
        if (ser.available())
        {

            std_msgs::String result;
            uint8_t get;

            ser.read(&get, 1);
            mavlink.read_data(get);
            result.data = get;
            read_pub.publish(result);
        }
        loop_rate.sleep();
    }
}
