//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include "math.h"
#include <iostream>

//Uncomment this and replace {type} with the type of message when needed
#include "std_msgs/UInt64.h"


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_Tutorial_Talker");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  int input_value;
  unsigned output_value;
  std_msgs::UInt64 publish_int;

  ros::Publisher square = n.advertise<std_msgs::UInt64>("squarednum", 1);




  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {

  std::fflush;
  std::cout << "Enter the number to be squared: ";
  std::cin >> input_value;



  output_value = (unsigned)pow(input_value,2);
  std::cout << "Sending the number " << output_value << std::endl;

  publish_int.data=output_value;

  square.publish(publish_int);

    //sends out any data necessary then waits based on the loop rate
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}

