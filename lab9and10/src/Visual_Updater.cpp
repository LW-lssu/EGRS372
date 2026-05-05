#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cmath>

ros::Publisher marker_pub;

geometry_msgs::Point human_position;
bool human_received = false;

// CHANGE THESE TO YOUR CROSSWALK CORNERS
double X1 = 0.5;
double X2 = 1.5;
double Y1 = -0.25;
double Y2 = 0.25;

// CHANGE THESE TO YOUR LAB 9 POINTS
double points[5][3] = {
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {1.0, 1.0, 90.0},
    {0.0, 1.0, 180.0},
    {0.0, 0.0, 0.0}
};

void humanCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    human_position = *msg;
    human_received = true;
}

void publishPointMarkers()
{
    for (int i = 0; i < 5; i++)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "lab10_points";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = points[i][0];
        marker.pose.position.y = points[i][1];
        marker.pose.position.z = 0.05;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_pub.publish(marker);
    }
}

void publishCrosswalkMarker()
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "crosswalk";
    marker.id = 100;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (X1 + X2) / 2.0;
    marker.pose.position.y = (Y1 + Y2) / 2.0;
    marker.pose.position.z = 0.02;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = fabs(X2 - X1);
    marker.scale.y = fabs(Y2 - Y1);
    marker.scale.z = 0.02;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.45;

    marker_pub.publish(marker);
}

void publishHumanMarker()
{
    if (!human_received)
    {
        return;
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "human_zone";
    marker.id = 200;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = human_position.x;
    marker.pose.position.y = human_position.y;
    marker.pose.position.z = 0.03;
    marker.pose.orientation.w = 1.0;

    // Diameter 1.0 m = radius 0.5 m
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.02;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.45;

    marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Visual_Updater");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber human_sub = nh.subscribe("human", 10, humanCallback);

    ros::Rate rate(10);

    while (ros::ok())
    {
        publishPointMarkers();
        publishCrosswalkMarker();
        publishHumanMarker();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



