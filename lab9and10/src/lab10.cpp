#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>

#include <std_msgs/Bool.h>

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
#include <cstdlib>
#include <cctype>

#define PI 3.14159265359

class Lab10
{
public:
    Lab10()
    {
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        behavior_pub = nh.advertise<lab9::behavior_update>("/behavior_update", 10);
        motor_power_pub = nh.advertise<std_msgs::Bool>("/motor_power", 10);

        result_sub = nh.subscribe("/move_base/result", 10, &Lab10::resultCallback, this);
        feedback_sub = nh.subscribe("/move_base/feedback", 10, &Lab10::feedbackCallback, this);
        human_sub = nh.subscribe("/human", 10, &Lab10::humanCallback, this);

        ros::param::param<std::string>(
            "/lab9/parameter_file",
            parameter_file,
            "/home/egrs372/catkin_ws/src/lab9/parameters.txt"
        );

        // Crosswalk rectangular zone.
        // Change these values in code or add them to the launch file.
        ros::param::param<double>("/lab10/crosswalk/x1", crosswalk_x1, 0.5);
        ros::param::param<double>("/lab10/crosswalk/x2", crosswalk_x2, 1.5);
        ros::param::param<double>("/lab10/crosswalk/y1", crosswalk_y1, -0.25);
        ros::param::param<double>("/lab10/crosswalk/y2", crosswalk_y2, 0.25);

        human_stop_distance = 0.5;

        // Helps reduce move_base giving up while stopped near the human.
        ros::param::set("/move_base/oscillation_distance", 0.0);

        loadPoints();
        loadBehaviorFile();

        current_point = 0;
        goal_sent = false;
        goal_reached = false;
        finished = false;

        robot_x = 0.0;
        robot_y = 0.0;
        robot_position_received = false;

        human_received = false;
        human_stop_active = false;
        crosswalk_active = false;

        setMotorPower(true);

        ROS_INFO("Lab 10 node started.");
        ROS_INFO("Parameter file: %s", parameter_file.c_str());
        ROS_INFO("Crosswalk: x1=%.2f x2=%.2f y1=%.2f y2=%.2f",
                 crosswalk_x1, crosswalk_x2, crosswalk_y1, crosswalk_y2);
    }

    void run()
    {
        ros::Rate loop_rate(10);

        while (ros::ok() && !finished)
        {
            ros::spinOnce();

            checkSafetyZones();
            runStateMachine();

            loop_rate.sleep();
        }

        setMotorPower(true);
        ROS_INFO("Lab 10 complete. All 5 points have been reached.");
    }

private:
    ros::NodeHandle nh;

    ros::Publisher goal_pub;
    ros::Publisher behavior_pub;
    ros::Publisher motor_power_pub;

    ros::Subscriber result_sub;
    ros::Subscriber feedback_sub;
    ros::Subscriber human_sub;

    std::string parameter_file;

    bool goal_sent;
    bool goal_reached;
    bool finished;

    int current_point;

    double robot_x;
    double robot_y;
    bool robot_position_received;

    geometry_msgs::Point human_position;
    bool human_received;

    bool human_stop_active;
    bool crosswalk_active;

    double human_stop_distance;

    double crosswalk_x1;
    double crosswalk_x2;
    double crosswalk_y1;
    double crosswalk_y2;

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

    void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
    {
        robot_x = msg->feedback.base_position.pose.position.x;
        robot_y = msg->feedback.base_position.pose.position.y;
        robot_position_received = true;
    }

    void humanCallback(const geometry_msgs::Point::ConstPtr& msg)
    {
        human_position = *msg;
        human_received = true;
    }

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

    void loadBehaviorFile()
    {
        behaviors.clear();

        std::ifstream file(parameter_file.c_str());

        if (!file.is_open())
        {
            ROS_ERROR("Could not open parameter file: %s", parameter_file.c_str());
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

            if (b.allow_backward)
            {
                b.min_vel_x = -b.max_vel_x;
            }
            else
            {
                b.min_vel_x = 0.0;
            }

            b.min_rot_vel = std::min(0.10, b.max_rot_vel);

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

    void addDoubleParameter(dynamic_reconfigure::Config& conf,
                            const std::string& name,
                            double value)
    {
        dynamic_reconfigure::DoubleParameter double_param;

        double_param.name = name;
        double_param.value = value;

        conf.doubles.push_back(double_param);
    }

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

        ROS_INFO("Applied behavior for point %d:", point_number);
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

    void setMotorPower(bool enabled)
    {
        std_msgs::Bool msg;
        msg.data = enabled;

        motor_power_pub.publish(msg);

        if (enabled)
        {
            ROS_WARN("Motor power ON.");
        }
        else
        {
            ROS_WARN("Motor power OFF. Human safety stop active.");
        }
    }

    bool isInCrosswalk()
    {
        if (!robot_position_received)
        {
            return false;
        }

        double min_x = std::min(crosswalk_x1, crosswalk_x2);
        double max_x = std::max(crosswalk_x1, crosswalk_x2);
        double min_y = std::min(crosswalk_y1, crosswalk_y2);
        double max_y = std::max(crosswalk_y1, crosswalk_y2);

        return robot_x >= min_x &&
               robot_x <= max_x &&
               robot_y >= min_y &&
               robot_y <= max_y;
    }

    double distanceToHuman()
    {
        if (!human_received)
        {
            return 999.0;
        }

        double dx = robot_x - human_position.x;
        double dy = robot_y - human_position.y;

        return sqrt(dx * dx + dy * dy);
    }

    Behavior makeCrosswalkBehavior(const Behavior& original)
    {
        Behavior b = original;

        double crosswalk_max_vel_x = std::min(original.max_vel_x, 0.13);
        double crosswalk_max_rot_vel = std::min(original.max_rot_vel, 1.0);

        b.max_vel_x = crosswalk_max_vel_x;
        b.max_rot_vel = crosswalk_max_rot_vel;

        if (original.allow_backward)
        {
            b.min_vel_x = -crosswalk_max_vel_x;
        }
        else
        {
            b.min_vel_x = 0.0;
        }

        b.min_rot_vel = std::min(original.min_rot_vel, b.max_rot_vel);

        return b;
    }

    void checkSafetyZones()
    {
        if (!robot_position_received)
        {
            return;
        }

        double human_distance = distanceToHuman();

        // Human safety has highest priority.
        if (human_distance <= human_stop_distance)
        {
            if (!human_stop_active)
            {
                human_stop_active = true;
                setMotorPower(false);

                ROS_WARN("Human detected within %.2f m. Distance = %.2f m.",
                         human_stop_distance, human_distance);
            }

            return;
        }

        if (human_stop_active && human_distance > human_stop_distance)
        {
            human_stop_active = false;
            setMotorPower(true);

            ROS_WARN("Human cleared. Distance = %.2f m.", human_distance);

            // Re-send current goal after motor power returns.
            if (current_point < 5)
            {
                sendGoal(points[current_point]);
                goal_sent = true;
                goal_reached = false;
            }
        }

        if (current_point >= 5)
        {
            return;
        }

        bool now_in_crosswalk = isInCrosswalk();

        if (now_in_crosswalk && !crosswalk_active)
        {
            crosswalk_active = true;

            ROS_WARN("Entered crosswalk. Slowing down.");

            Behavior crosswalk_behavior = makeCrosswalkBehavior(behaviors[current_point]);

            applyBehavior(crosswalk_behavior, current_point + 1);
        }

        if (!now_in_crosswalk && crosswalk_active)
        {
            crosswalk_active = false;

            ROS_WARN("Exited crosswalk. Restoring original behavior.");

            applyBehavior(behaviors[current_point], current_point + 1);
        }
    }

    void runStateMachine()
    {
        if (human_stop_active)
        {
            return;
        }

        if (current_point >= 5)
        {
            finished = true;
            return;
        }

        if (!goal_sent)
        {
            int point_number = current_point + 1;

            ROS_INFO("Preparing point %d.", point_number);

            applyBehavior(behaviors[current_point], point_number);

            crosswalk_active = false;
            goal_reached = false;

            ros::Duration(0.5).sleep();

            sendGoal(points[current_point]);

            goal_sent = true;
        }

        if (goal_reached)
        {
            ROS_INFO("Point %d reached.", current_point + 1);

            current_point++;
            goal_sent = false;
            goal_reached = false;
            crosswalk_active = false;

            if (current_point >= 5)
            {
                finished = true;
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab10");

    Lab10 node;

    ros::Duration(2.0).sleep();

    node.run();

    return 0;
}



