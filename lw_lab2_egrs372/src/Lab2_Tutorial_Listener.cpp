//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"

void print_uint(const std_msgs::UInt64 squared_value)
{

  unsigned output_value;
  
  output_value = squared_value.data;

  std::cout << "the number " << output_value << "was published" <<std::endl;
}


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_Tutorial_Listener");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  ros::Subscriber square = n.subscribe("squarednum", 1, print_uint);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {

    //sends out any data necessary then waits based on the loop rate
    ros::spin();

  }

  return 0;
}

