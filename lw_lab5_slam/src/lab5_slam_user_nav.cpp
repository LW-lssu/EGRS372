#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/tfMessage.h>
#include <cmath>
#include <string>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265359


class SlamMapMove
{
    public:
    SlamMapMove()
    {
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);

        ROS_INFO("SlamMapMove Started. Publishing to /move_base_simple/goal");
    }

    void run()
    {
        get_user_input();
        rot_to_q();
        send_goal();
    }

    private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;

    std::string reference_frame;
    double x_goal;
    double y_goal;
    double psi_goal;
    double qw_goal;
    double qz_goal;
    geometry_msgs::PoseStamped goal_pose;


    void get_user_input()
    {
        int frame;
        std::cout << "Enter Desired Reference Frame [1:Map, 2:MR]: ";
        std::cin >> frame;
        if((frame < 1) || (frame > 2))
        {
            while ((frame < 1) || (frame > 2))
            {
            std::cout << "Invalid Reference Frame\nEnter Desired Reference Frame [1:Map, 2:MR]: ";
            std::cin >> frame;
            }
        }
        std::cout << "Enter Desired X Position: ";
        std::cin >> x_goal;
        std::cout << "Enter Desired Y Position: ";
        std::cin >> y_goal;
        std::cout << "Enter Desired Rotation (deg): ";
        std::cin >> psi_goal;

        if(frame == 1)
            reference_frame = "map";
        else
            reference_frame = "base_link";
    }

    void rot_to_q()
    {
        double rad_goal = psi_goal * PI / 180.0;

        while(rad_goal > 2 * PI)
            rad_goal -= 2 * PI;

        while(psi_goal >= 360)
            rad_goal += 2 * PI;

        qw_goal = std::cos(rad_goal / 2.0);
        qz_goal = std::sin(rad_goal / 2.0);
    }

    void send_goal()
    {
        // Setting Pose Variables
        goal_pose.pose.position.x = x_goal;
        goal_pose.pose.position.y = y_goal;
        goal_pose.pose.position.z = 0.0;

        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = qz_goal;
        goal_pose.pose.orientation.w = qw_goal;
        // Setting header varaibels
        goal_pose.header.seq = 0;
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.header.frame_id = reference_frame;

        pose_pub.publish(goal_pose);

        ROS_INFO("Published goal: frame=%s x=%.2f y=%.2f yaw=%.2f deg",
                 reference_frame.c_str(), x_goal, y_goal, psi_goal);
    }
};


    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "lab5_slam_user_nav");
        SlamMapMove node;
	ros::Duration(2.0).sleep();
        node.run();
	ros::Duration(2.0).sleep();
        return 0;
    }
