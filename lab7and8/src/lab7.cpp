#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int32.h>

#include "lab7/turtlebot_status.h"
#include "lab7/go_home.h"
#include "lab7/return_to_work.h"
#include "lab7/update_count.h"

#include <cmath>
#include <string>

#define PI 3.14159265359

class Lab7
{
public:
    Lab7()
    {
        // ============================================================
        // Publishers
        // ============================================================
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        led_pub = nh.advertise<std_msgs::Int32>("/led_hex", 10);
        status_pub = nh.advertise<lab7::turtlebot_status>("/Turtlebot_status", 10);

        // ============================================================
        // Subscribers
        // ============================================================
        result_sub = nh.subscribe("/move_base/result", 10, &Lab7::resultCallback, this);
        battery_sub = nh.subscribe("/battery_state", 10, &Lab7::batteryCallback, this);
        button_sub = nh.subscribe("/button_state", 10, &Lab7::buttonCallback, this);

        // ============================================================
        // Services for Lab 8
        // ============================================================
        go_home_service = nh.advertiseService("go_home", &Lab7::goHomeService, this);
        return_to_work_service = nh.advertiseService("return_to_work", &Lab7::returnToWorkService, this);
        update_count_service = nh.advertiseService("update_count", &Lab7::updateCountService, this);

        // ============================================================
        // Default values
        //
        // These are used when running with:
        // rosrun lab7 lab7
        //
        // If using:
        // roslaunch lab7 lab8_launch.launch
        //
        // then these values are overwritten by launch-file parameters.
        // ============================================================

        home_x = 0.0;
        home_y = 0.0;
        home_yaw_deg = 0.0;

        pick_x = 1.0;
        pick_y = 0.0;
        pick_yaw_deg = 0.0;

        place_x = 1.0;
        place_y = 1.0;
        place_yaw_deg = 90.0;

        low_battery_voltage = 10.0;
        charged_battery_voltage = 11.0;

        // Read parameters once at startup.
        updateParameters();

        // ============================================================
        // Initial state values
        // ============================================================
        current_state = GO_HOME_START;
        saved_state_before_charging = GO_PICK;
        saved_state_before_service_home = GO_PICK;

        goal_sent = false;
        goal_reached = false;

        battery_voltage = 12.0;
        battery_received = false;

        button_pressed = false;
        previous_button_value = 0;

        place_count = 0;

        ROS_INFO("Lab 7 / Lab 8 node started.");
        ROS_INFO("Goal publisher: /move_base_simple/goal");
        ROS_INFO("Result subscriber: /move_base/result");
        ROS_INFO("Battery subscriber: /battery_state");
        ROS_INFO("Button subscriber: /button_state as std_msgs/Byte");
        ROS_INFO("LED publisher: /led_hex as std_msgs/Int32");
        ROS_INFO("Status publisher: /Turtlebot_status");
        ROS_INFO("Services: /go_home, /return_to_work, /update_count");
    }

    void run()
    {
        ros::Rate loop_rate(10);

        while (ros::ok())
        {
            ros::spinOnce();

            // Read parameters repeatedly so rosparam set can change locations
            // while the program is running.
            updateParameters();

            checkBatteryOverride();
            runStateMachine();
            publishStatus();

            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;

    ros::Publisher goal_pub;
    ros::Publisher led_pub;
    ros::Publisher status_pub;

    ros::Subscriber result_sub;
    ros::Subscriber battery_sub;
    ros::Subscriber button_sub;

    ros::ServiceServer go_home_service;
    ros::ServiceServer return_to_work_service;
    ros::ServiceServer update_count_service;

    enum RobotState
    {
        GO_HOME_START = 0,
        WAIT_AT_HOME_START = 1,
        GO_PICK = 2,
        WAIT_AT_PICK = 3,
        GO_PLACE = 4,
        WAIT_AT_PLACE = 5,
        GO_HOME_FOR_CHARGE = 6,
        WAIT_FOR_CHARGE = 7,
        GO_HOME_SERVICE = 8,
        WAIT_HOME_SERVICE = 9
    };

    RobotState current_state;
    RobotState saved_state_before_charging;
    RobotState saved_state_before_service_home;

    bool goal_sent;
    bool goal_reached;

    bool battery_received;
    double battery_voltage;

    bool button_pressed;
    int previous_button_value;

    int place_count;

    double home_x;
    double home_y;
    double home_yaw_deg;

    double pick_x;
    double pick_y;
    double pick_yaw_deg;

    double place_x;
    double place_y;
    double place_yaw_deg;

    double low_battery_voltage;
    double charged_battery_voltage;

    // ================================================================
    // Parameter update function
    //
    // This allows Lab 8 to change values using rosparam set.
    // Example:
    // rosparam set /pick_location/x 2.0
    // ================================================================
    void updateParameters()
    {
        ros::param::param<double>("/home_location/x", home_x, home_x);
        ros::param::param<double>("/home_location/y", home_y, home_y);
        ros::param::param<double>("/home_location/theta", home_yaw_deg, home_yaw_deg);

        ros::param::param<double>("/pick_location/x", pick_x, pick_x);
        ros::param::param<double>("/pick_location/y", pick_y, pick_y);
        ros::param::param<double>("/pick_location/theta", pick_yaw_deg, pick_yaw_deg);

        ros::param::param<double>("/place_location/x", place_x, place_x);
        ros::param::param<double>("/place_location/y", place_y, place_y);
        ros::param::param<double>("/place_location/theta", place_yaw_deg, place_yaw_deg);

        ros::param::param<double>("/battery_low_level", low_battery_voltage, low_battery_voltage);
        ros::param::param<double>("/battery_high_level", charged_battery_voltage, charged_battery_voltage);
    }

    // ================================================================
    // move_base result callback
    //
    // Status 3 means the robot successfully reached the goal.
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
    // Battery callback
    // ================================================================
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
    {
        battery_voltage = msg->voltage;
        battery_received = true;
    }

    // ================================================================
    // Button callback
    //
    // /button_state is std_msgs/Byte.
    // 0 = not pressed
    // 1 = pressed
    //
    // Rising-edge detection prevents one button hold from counting many times.
    // ================================================================
    void buttonCallback(const std_msgs::Byte::ConstPtr& msg)
    {
        int current_button_value = static_cast<int>(msg->data);

        if (current_button_value != 0 && previous_button_value == 0)
        {
            button_pressed = true;
            ROS_INFO("Button press detected. Button value = %d", current_button_value);
        }

        previous_button_value = current_button_value;
    }

    // ================================================================
    // Send navigation goal
    // ================================================================
    void sendGoal(double x_goal, double y_goal, double yaw_goal_deg)
    {
        double yaw_rad = yaw_goal_deg * PI / 180.0;

        double qw_goal = cos(yaw_rad / 2.0);
        double qz_goal = sin(yaw_rad / 2.0);

        geometry_msgs::PoseStamped goal_pose;

        goal_pose.header.stamp = ros::Time::now();
        goal_pose.header.frame_id = "map";

        goal_pose.pose.position.x = x_goal;
        goal_pose.pose.position.y = y_goal;
        goal_pose.pose.position.z = 0.0;

        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = qz_goal;
        goal_pose.pose.orientation.w = qw_goal;

        goal_pub.publish(goal_pose);

        ROS_INFO("Published goal: frame=map x=%.2f y=%.2f yaw=%.2f deg",
                 x_goal, y_goal, yaw_goal_deg);
    }

    // ================================================================
    // LED update
    //
    // /led_hex expects std_msgs/Int32.
    // Values above 9 may display as hexadecimal.
    // Example: 10 = A, 11 = B, 17 = 11.
    // ================================================================
    void updateLed()
    {
        std_msgs::Int32 led_msg;

        led_msg.data = place_count;

        led_pub.publish(led_msg);

        ROS_INFO("Updated /led_hex with value: %d", led_msg.data);
    }

    // ================================================================
    // State name for Turtlebot_status message
    // ================================================================
    std::string getStateName(RobotState state)
    {
        switch (state)
        {
            case GO_HOME_START:
                return "Going Home at Start";

            case WAIT_AT_HOME_START:
                return "Waiting at Home for Button";

            case GO_PICK:
                return "Going to Pick";

            case WAIT_AT_PICK:
                return "Waiting at Pick for Button";

            case GO_PLACE:
                return "Going to Place";

            case WAIT_AT_PLACE:
                return "Waiting at Place for Button";

            case GO_HOME_FOR_CHARGE:
                return "Going Home for Charging";

            case WAIT_FOR_CHARGE:
                return "Waiting for Battery Charge";

            case GO_HOME_SERVICE:
                return "Going Home by Service";

            case WAIT_HOME_SERVICE:
                return "Waiting at Home by Service";

            default:
                return "Unknown";
        }
    }

    // ================================================================
    // Publish TurtleBot status for Lab 8
    // ================================================================
    void publishStatus()
    {
        lab7::turtlebot_status status_msg;

        status_msg.current_job = getStateName(current_state);
        status_msg.place_count = place_count;
        status_msg.battery = battery_voltage;

        status_pub.publish(status_msg);
    }

    // ================================================================
    // Change state and reset one-time flags
    // ================================================================
    void changeState(RobotState new_state)
    {
        current_state = new_state;

        goal_sent = false;
        goal_reached = false;
        button_pressed = false;

        ROS_INFO("Changed to state: %d", current_state);
    }

    // ================================================================
    // Decide what job should resume after charging or service-home
    // ================================================================
    RobotState getResumeState()
    {
        if (current_state == GO_HOME_START || current_state == WAIT_AT_HOME_START)
        {
            return WAIT_AT_HOME_START;
        }

        if (current_state == GO_PICK || current_state == WAIT_AT_PICK)
        {
            return GO_PICK;
        }

        if (current_state == GO_PLACE || current_state == WAIT_AT_PLACE)
        {
            return GO_PLACE;
        }

        return GO_PICK;
    }

    // ================================================================
    // Battery override
    //
    // If voltage falls below the low level, save the current job and
    // return home. Wait until voltage rises above the high level.
    // ================================================================
    void checkBatteryOverride()
    {
        if (!battery_received)
        {
            return;
        }

        bool already_charging =
            current_state == GO_HOME_FOR_CHARGE ||
            current_state == WAIT_FOR_CHARGE;

        bool already_service_home =
            current_state == GO_HOME_SERVICE ||
            current_state == WAIT_HOME_SERVICE;

        if (battery_voltage < low_battery_voltage &&
            !already_charging &&
            !already_service_home)
        {
            saved_state_before_charging = getResumeState();

            ROS_WARN("Battery low: %.2f V. Returning Home.", battery_voltage);

            changeState(GO_HOME_FOR_CHARGE);
        }
    }

    // ================================================================
    // Lab 8 service: go_home
    //
    // Sends robot home and keeps it there until return_to_work is called.
    // ================================================================
    bool goHomeService(lab7::go_home::Request &req,
                       lab7::go_home::Response &res)
    {
        (void)req;

        saved_state_before_service_home = getResumeState();

        res.old_job = static_cast<int>(current_state);

        ROS_WARN("go_home service called. Sending robot Home.");

        changeState(GO_HOME_SERVICE);

        return true;
    }

    // ================================================================
    // Lab 8 service: return_to_work
    //
    // Returns the robot to the state saved before go_home was called.
    // ================================================================
    bool returnToWorkService(lab7::return_to_work::Request &req,
                             lab7::return_to_work::Response &res)
    {
        (void)req;

        res.old_job = static_cast<int>(current_state);

        ROS_WARN("return_to_work service called. Returning to saved work state.");

        changeState(saved_state_before_service_home);

        return true;
    }

    // ================================================================
    // Lab 8 service: update_count
    //
    // Updates the stored place count and sends it to the LED.
    // ================================================================
    bool updateCountService(lab7::update_count::Request &req,
                            lab7::update_count::Response &res)
    {
        res.old_count = place_count;

        place_count = static_cast<int>(req.new_count);

        updateLed();

        ROS_WARN("update_count service called. Count updated.");

        return true;
    }

    // ================================================================
    // Main state machine
    // ================================================================
    void runStateMachine()
    {
        switch (current_state)
        {
            case GO_HOME_START:
            {
                if (!goal_sent)
                {
                    ROS_INFO("Going to Home.");
                    sendGoal(home_x, home_y, home_yaw_deg);
                    goal_sent = true;
                }

                if (goal_reached)
                {
                    ROS_INFO("Reached Home. Waiting for button before going to Pick.");
                    changeState(WAIT_AT_HOME_START);
                }

                break;
            }

            case WAIT_AT_HOME_START:
            {
                if (button_pressed)
                {
                    ROS_INFO("Home button confirmed. Going to Pick.");
                    changeState(GO_PICK);
                }

                break;
            }

            case GO_PICK:
            {
                if (!goal_sent)
                {
                    ROS_INFO("Going to Pick.");
                    sendGoal(pick_x, pick_y, pick_yaw_deg);
                    goal_sent = true;
                }

                if (goal_reached)
                {
                    ROS_INFO("Reached Pick. Waiting for button press.");
                    changeState(WAIT_AT_PICK);
                }

                break;
            }

            case WAIT_AT_PICK:
            {
                if (button_pressed)
                {
                    place_count++;

                    ROS_INFO("Pick confirmed by button. Count = %d", place_count);

                    updateLed();

                    ROS_INFO("Going to Place.");
                    changeState(GO_PLACE);
                }

                break;
            }

            case GO_PLACE:
            {
                if (!goal_sent)
                {
                    ROS_INFO("Going to Place.");
                    sendGoal(place_x, place_y, place_yaw_deg);
                    goal_sent = true;
                }

                if (goal_reached)
                {
                    ROS_INFO("Reached Place. Waiting for button press.");
                    changeState(WAIT_AT_PLACE);
                }

                break;
            }

            case WAIT_AT_PLACE:
            {
                if (button_pressed)
                {
                    ROS_INFO("Place confirmed by button. Returning to Pick.");
                    changeState(GO_PICK);
                }

                break;
            }

            case GO_HOME_FOR_CHARGE:
            {
                if (!goal_sent)
                {
                    ROS_WARN("Going Home for charging.");
                    sendGoal(home_x, home_y, home_yaw_deg);
                    goal_sent = true;
                }

                if (goal_reached)
                {
                    ROS_WARN("At Home. Waiting for battery voltage to recover.");
                    changeState(WAIT_FOR_CHARGE);
                }

                break;
            }

            case WAIT_FOR_CHARGE:
            {
                if (battery_voltage > charged_battery_voltage)
                {
                    ROS_WARN("Battery recovered: %.2f V. Resuming saved job.", battery_voltage);
                    changeState(saved_state_before_charging);
                }

                break;
            }

            case GO_HOME_SERVICE:
            {
                if (!goal_sent)
                {
                    ROS_WARN("Going Home because /go_home service was called.");
                    sendGoal(home_x, home_y, home_yaw_deg);
                    goal_sent = true;
                }

                if (goal_reached)
                {
                    ROS_WARN("At Home. Waiting for /return_to_work service.");
                    changeState(WAIT_HOME_SERVICE);
                }

                break;
            }

            case WAIT_HOME_SERVICE:
            {
                // Stay here until /return_to_work is called.
                break;
            }

            default:
            {
                ROS_ERROR("Unknown state. Resetting to Home.");
                changeState(GO_HOME_START);
                break;
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab7");

    Lab7 node;

    ros::Duration(2.0).sleep();

    node.run();

    return 0;
}



