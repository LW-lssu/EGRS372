#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "lab9/behavior_update.h"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#define PI 3.14159265359

class Lab9
{
public:
    Lab9()
    {
        // Publishers
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        behavior_pub = nh.advertise<lab9::behavior_update>("/behavior_update", 10);

        // Subscriber for move_base result
        result_sub = nh.subscribe("/move_base/result", 10, &Lab9::resultCallback, this);

        // Load parameter file path from launch file.
        // If no launch file is used, this default path is used.
        ros::param::param<std::string>(
            "/lab9/parameter_file",
            parameter_file,
            "/home/egrs372/catkin_ws/src/lab9/parameters.txt"
        );

        // Load five navigation points from ROS parameters.
        loadPoints();

        // Load behavior settings from parameters.txt.
        loadBehaviorFile();

        current_point = 0;
        goal_sent = false;
        goal_reached = false;
        finished = false;

        ROS_INFO("Lab 9 node started.");
        ROS_INFO("Loaded parameter file: %s", parameter_file.c_str());
        ROS_INFO("Number of points loaded: %lu", points.size());
        ROS_INFO("Number of behavior lines loaded: %lu", behaviors.size());
    }

    void run()
    {
        ros::Rate loop_rate(10);

        while (ros::ok() && !finished)
        {
            ros::spinOnce();

            runStateMachine();

            loop_rate.sleep();
        }

        ROS_INFO("Lab 9 complete. All 5 points have been reached.");
    }

private:
    ros::NodeHandle nh;

    ros::Publisher goal_pub;
    ros::Publisher behavior_pub;
    ros::Subscriber result_sub;

    std::string parameter_file;

    bool goal_sent;
    bool goal_reached;
    bool finished;

    int current_point;

    struct Point
    {
        double x;
        double y;
        double theta_deg;
    };

    struct Behavior
    {
        double max_vel_x;
        bool allow_backward;
        double max_rot_vel;
        bool adjust_orientation;

        double min_vel_x;
        double min_rot_vel;
        double yaw_goal_tolerance;
    };

    std::vector<Point> points;
    std::vector<Behavior> behaviors;

    // ================================================================
    // move_base result callback
    //
    // Status 3 means successful goal completion.
    // ================================================================
    void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
    {
        if (msg->status.status == 3)
        {
            goal_reached = true;
            ROS_INFO("Navigation goal reached.");
        }
        else
        {
            ROS_INFO("move_base result status: %d", msg->status.status);
        }
    }

    // ================================================================
    // Load five points from launch-file parameters.
    //
    // If launch parameters are missing, default values are used.
    // ================================================================
    void loadPoints()
    {
        points.clear();

        for (int i = 1; i <= 5; i++)
        {
            Point p;

            std::stringstream base;
            base << "/lab9/point" << i;

            ros::param::param<double>(base.str() + "/x", p.x, defaultX(i));
            ros::param::param<double>(base.str() + "/y", p.y, defaultY(i));
            ros::param::param<double>(base.str() + "/theta", p.theta_deg, defaultTheta(i));

            points.push_back(p);

            ROS_INFO("Point %d: x=%.2f y=%.2f theta=%.2f",
                     i, p.x, p.y, p.theta_deg);
        }
    }

    double defaultX(int point_number)
    {
        if (point_number == 1) return 0.0;
        if (point_number == 2) return 1.0;
        if (point_number == 3) return 1.0;
        if (point_number == 4) return 0.0;
        return 0.0;
    }

    double defaultY(int point_number)
    {
        if (point_number == 1) return 0.0;
        if (point_number == 2) return 0.0;
        if (point_number == 3) return 1.0;
        if (point_number == 4) return 1.0;
        return 0.0;
    }

    double defaultTheta(int point_number)
    {
        if (point_number == 1) return 0.0;
        if (point_number == 2) return 0.0;
        if (point_number == 3) return 90.0;
        if (point_number == 4) return 180.0;
        return 0.0;
    }

    // ================================================================
    // Convert string to bool.
    //
    // Accepts:
    // t, true, 1, y, yes
    // Everything else is treated as false.
    // ================================================================
    bool stringToBool(std::string value)
    {
        value.erase(remove(value.begin(), value.end(), ' '), value.end());

        std::transform(value.begin(), value.end(), value.begin(), ::tolower);

        if (value == "t") return true;
        if (value == "true") return true;
        if (value == "1") return true;
        if (value == "y") return true;
        if (value == "yes") return true;

        return false;
    }

    // ================================================================
    // Load behavior settings from parameters.txt.
    //
    // File format:
    // max_vel_x,allow_backward,max_rot_vel,adjust_orientation
    //
    // Example:
    // 0.13,f,0.50,t
    // ================================================================
    void loadBehaviorFile()
    {
        behaviors.clear();

        std::ifstream file(parameter_file.c_str());

        if (!file.is_open())
        {
            ROS_ERROR("Could not open parameters.txt file: %s", parameter_file.c_str());
            ROS_WARN("Using default behavior settings.");

            for (int i = 0; i < 5; i++)
            {
                behaviors.push_back(defaultBehavior());
            }

            return;
        }

        std::string line;

        while (std::getline(file, line))
        {
            if (line.length() == 0)
            {
                continue;
            }

            Behavior b;
            std::string max_vel_x_text;
            std::string allow_backward_text;
            std::string max_rot_vel_text;
            std::string adjust_orientation_text;

            std::stringstream ss(line);

            std::getline(ss, max_vel_x_text, ',');
            std::getline(ss, allow_backward_text, ',');
            std::getline(ss, max_rot_vel_text, ',');
            std::getline(ss, adjust_orientation_text, ',');

            b.max_vel_x = atof(max_vel_x_text.c_str());
            b.allow_backward = stringToBool(allow_backward_text);
            b.max_rot_vel = atof(max_rot_vel_text.c_str());
            b.adjust_orientation = stringToBool(adjust_orientation_text);

            // If backward motion is allowed, min_vel_x is negative.
            // If not allowed, min_vel_x is zero.
            if (b.allow_backward)
            {
                b.min_vel_x = -b.max_vel_x;
            }
            else
            {
                b.min_vel_x = 0.0;
            }

            // Use a small minimum rotational speed.
            // Make sure it is not larger than max_rot_vel.
            b.min_rot_vel = std::min(0.10, b.max_rot_vel);

            // If orientation should be adjusted, use normal tolerance.
            // If not, use a very large tolerance so the robot accepts
            // almost any final heading.
            if (b.adjust_orientation)
            {
                b.yaw_goal_tolerance = 0.17;
            }
            else
            {
                b.yaw_goal_tolerance = 10.0;
            }

            behaviors.push_back(b);
        }

        file.close();

        // Ensure there are at least 5 behavior entries.
        while (behaviors.size() < 5)
        {
            ROS_WARN("parameters.txt has fewer than 5 lines. Adding default behavior.");
            behaviors.push_back(defaultBehavior());
        }
    }

    Behavior defaultBehavior()
    {
        Behavior b;

        b.max_vel_x = 0.13;
        b.allow_backward = false;
        b.max_rot_vel = 0.50;
        b.adjust_orientation = true;

        b.min_vel_x = 0.0;
        b.min_rot_vel = 0.10;
        b.yaw_goal_tolerance = 0.17;

        return b;
    }

    // ================================================================
    // Dynamically reconfigure DWAPlannerROS parameters.
    //
    // The lab starter code shows setting DoubleParameter values and
    // calling /move_base/DWAPlannerROS/set_parameters. This expands it
    // to multiple parameters. :contentReference[oaicite:1]{index=1}
    // ================================================================
    bool applyBehavior(const Behavior& b, int point_number)
    {
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::Config conf;

        addDoubleParameter(conf, "max_vel_x", b.max_vel_x);
        addDoubleParameter(conf, "min_vel_x", b.min_vel_x);
        addDoubleParameter(conf, "max_rot_vel", b.max_rot_vel);
        addDoubleParameter(conf, "min_rot_vel", b.min_rot_vel);
        addDoubleParameter(conf, "yaw_goal_tolerance", b.yaw_goal_tolerance);

        srv_req.config = conf;

        bool success = ros::service::call(
            "/move_base/DWAPlannerROS/set_parameters",
            srv_req,
            srv_resp
        );

        if (!success)
        {
            ROS_ERROR("Failed to call /move_base/DWAPlannerROS/set_parameters.");
            return false;
        }

        ROS_INFO("Updated behavior for point %d:", point_number);
        ROS_INFO("  max_vel_x = %.2f", b.max_vel_x);
        ROS_INFO("  min_vel_x = %.2f", b.min_vel_x);
        ROS_INFO("  max_rot_vel = %.2f", b.max_rot_vel);
        ROS_INFO("  min_rot_vel = %.2f", b.min_rot_vel);
        ROS_INFO("  yaw_goal_tolerance = %.2f", b.yaw_goal_tolerance);
        ROS_INFO("  allow_backward = %s", b.allow_backward ? "true" : "false");
        ROS_INFO("  adjust_orientation = %s", b.adjust_orientation ? "true" : "false");

        publishBehaviorMessage(b, point_number);

        return true;
    }

    void addDoubleParameter(dynamic_reconfigure::Config& conf,
                            const std::string& name,
                            double value)
    {
        dynamic_reconfigure::DoubleParameter double_param;

        double_param.name = name;
        double_param.value = value;

        conf.doubles.push_back(double_param);
    }

    // ================================================================
    // Publish custom behavior update message.
    // ================================================================
    void publishBehaviorMessage(const Behavior& b, int point_number)
    {
        lab9::behavior_update msg;

        msg.point_number = point_number;
        msg.max_vel_x = b.max_vel_x;
        msg.min_vel_x = b.min_vel_x;
        msg.max_rot_vel = b.max_rot_vel;
        msg.min_rot_vel = b.min_rot_vel;
        msg.yaw_goal_tolerance = b.yaw_goal_tolerance;
        msg.allow_backward = b.allow_backward;
        msg.adjust_orientation = b.adjust_orientation;

        behavior_pub.publish(msg);
    }

    // ================================================================
    // Send navigation goal to move_base_simple.
    // ================================================================
    void sendGoal(const Point& p)
    {
        double yaw_rad = p.theta_deg * PI / 180.0;

        geometry_msgs::PoseStamped goal;

        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";

        goal.pose.position.x = p.x;
        goal.pose.position.y = p.y;
        goal.pose.position.z = 0.0;

        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = sin(yaw_rad / 2.0);
        goal.pose.orientation.w = cos(yaw_rad / 2.0);

        goal_pub.publish(goal);

        ROS_INFO("Sent goal: x=%.2f y=%.2f theta=%.2f",
                 p.x, p.y, p.theta_deg);
    }

    // ================================================================
    // Main state machine.
    //
    // For each point:
    // 1. Apply behavior
    // 2. Send goal
    // 3. Wait for result status 3
    // 4. Move to next point
    // ================================================================
    void runStateMachine()
    {
        if (current_point >= 5)
        {
            finished = true;
            return;
        }

        if (!goal_sent)
        {
            int point_number = current_point + 1;

            Behavior b = behaviors[current_point];
            Point p = points[current_point];

            ROS_INFO("Preparing point %d.", point_number);

            applyBehavior(b, point_number);

            goal_reached = false;

            // Small delay so the behavior update reaches move_base
            // before the new navigation goal is sent.
            ros::Duration(0.5).sleep();

            sendGoal(p);

            goal_sent = true;
        }

        if (goal_reached)
        {
            ROS_INFO("Point %d reached.", current_point + 1);

            current_point++;
            goal_sent = false;
            goal_reached = false;

            if (current_point >= 5)
            {
                finished = true;
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab9");

    Lab9 node;

    // Give ROS time to connect publishers/subscribers.
    ros::Duration(2.0).sleep();

    node.run();

    return 0;
}


