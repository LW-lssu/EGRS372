#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>

class BarcodeConfirm
{
private:
    ros::NodeHandle nh;
    ros::Subscriber barcode_sub;
    ros::Publisher barcode_pub;

    std::string last_barcode;
    int count;

public:

    BarcodeConfirm()
    {
        count = 0;

        barcode_sub = nh.subscribe("/barcode", 10, &BarcodeConfirm::barcodeCallback, this);
        barcode_pub = nh.advertise<std_msgs::String>("/barcode_confirmed", 10);
    }

    void barcodeCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string current_barcode = msg->data;

        if(current_barcode == last_barcode)
        {
            count++;
        }
        else
        {
            last_barcode = current_barcode;
            count = 1;
        }

        ROS_INFO("Barcode: %s  Count: %d", current_barcode.c_str(), count);

        if(count >= 5)
        {
            std_msgs::String confirmed_msg;
            confirmed_msg.data = current_barcode;

            barcode_pub.publish(confirmed_msg);

            ROS_INFO("Confirmed barcode: %s", current_barcode.c_str());

            // Reset so it doesn't continuously republish
            count = 0;
            last_barcode = "";
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "barcode_confirm_node");

    BarcodeConfirm bc;

    ros::spin();

    return 0;
}
