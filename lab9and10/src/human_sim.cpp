#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <cstdlib>
#include <ctime>
#include <unistd.h>

// CHANGE THESE TO MATCH YOUR LAB 9 POINTS
double coords[5][3] = {
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {1.0, 1.0, 90.0},
    {0.0, 1.0, 180.0},
    {0.0, 0.0, 0.0}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "human_sim");
    ros::NodeHandle n;

    ros::Publisher human_pub = n.advertise<geometry_msgs::Point>("human", 10);

    geometry_msgs::Point position;

    srand((unsigned)time(0));

    while (ros::ok())
    {
        int randint = rand() % 5;

        position.x = coords[randint][0];
        position.y = coords[randint][1];
        position.z = 0.0;

        human_pub.publish(position);

        ROS_INFO("Published human position: x=%.2f y=%.2f",
                 position.x, position.y);

        ros::spinOnce();

        sleep(2);
    }

    return 0;
}




