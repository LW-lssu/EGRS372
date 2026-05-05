#include <cmath>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class Lab6WallBump
{
public:
  Lab6WallBump()
  {
    // -----------------------------
    // Tunable settings
    // -----------------------------
    forward_speed_ = 0.08;        // m/s
    backup_speed_ = -0.10;        // m/s
    backup_distance_ = 0.35;      // meters
    turn_speed_ = 0.35;           // rad/s
    turn_angle_ = M_PI / 2.0;     // 90 degrees
    yaw_tolerance_ = 4.0 * M_PI / 180.0;

    pause_time_ = 0.5;            // seconds

    button_pressed_ = false;
    odom_received_ = false;

    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;

    start_x_ = 0.0;
    start_y_ = 0.0;
    target_yaw_ = 0.0;

    hit_count_ = 0;

    state_ = WAIT_FOR_ODOM;

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    led_pub_ = nh_.advertise<std_msgs::Int32>("/led_hex", 10, true);

    button_sub_ = nh_.subscribe("/button_state", 10, &Lab6WallBump::buttonCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &Lab6WallBump::odomCallback, this);

    // Reset LED count to zero when the node starts
    publishLedCount();

    ROS_INFO("Lab 6 C++ wall-bump node started.");
  }

  void run()
  {
    ros::Rate rate(20);

    while (ros::ok())
    {
      ros::spinOnce();
      updateStateMachine();
      rate.sleep();
    }

    stopRobot();
  }

private:
  enum RobotState
  {
    WAIT_FOR_ODOM,
    DRIVE_FORWARD,
    BACKUP,
    TURN,
    PAUSE
  };

  ros::NodeHandle nh_;

  ros::Publisher cmd_pub_;
  ros::Publisher led_pub_;

  ros::Subscriber button_sub_;
  ros::Subscriber odom_sub_;

  RobotState state_;

  double forward_speed_;
  double backup_speed_;
  double backup_distance_;
  double turn_speed_;
  double turn_angle_;
  double yaw_tolerance_;
  double pause_time_;

  bool button_pressed_;
  bool odom_received_;

  double x_;
  double y_;
  double yaw_;

  double start_x_;
  double start_y_;
  double target_yaw_;

  int hit_count_;

  ros::Time pause_start_time_;

  void buttonCallback(const std_msgs::Byte::ConstPtr& msg)
  {
    button_pressed_ = ((msg->data & 0x01) == 1);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    yaw_ = tf::getYaw(q);

    odom_received_ = true;
  }

  void updateStateMachine()
  {
    switch (state_)
    {
      case WAIT_FOR_ODOM:
        handleWaitForOdom();
        break;

      case DRIVE_FORWARD:
        handleDriveForward();
        break;

      case BACKUP:
        handleBackup();
        break;

      case TURN:
        handleTurn();
        break;

      case PAUSE:
        handlePause();
        break;
    }
  }

  void handleWaitForOdom()
  {
    stopRobot();

    if (odom_received_)
    {
      ROS_INFO("Received /odom. Starting wall-bump behavior.");
      state_ = DRIVE_FORWARD;
    }
  }

  void handleDriveForward()
  {
    if (button_pressed_)
    {
      stopRobot();

      hit_count_++;
      publishLedCount();

      ROS_INFO("Wall hit detected. Count = %d, hex = 0x%04X",
               hit_count_,
               hit_count_ & 0xFFFF);

      start_x_ = x_;
      start_y_ = y_;

      state_ = BACKUP;
      return;
    }

    publishVelocity(forward_speed_, 0.0);
  }

  void handleBackup()
  {
    double distance_backed = distance(start_x_, start_y_, x_, y_);

    if (distance_backed >= backup_distance_)
    {
      stopRobot();

      target_yaw_ = normalizeAngle(yaw_ + turn_angle_);

      ROS_INFO("Backup complete. Starting 90 degree turn.");

      state_ = TURN;
      return;
    }

    publishVelocity(backup_speed_, 0.0);
  }

  void handleTurn()
  {
    double error = angleDifference(target_yaw_, yaw_);

    if (std::fabs(error) <= yaw_tolerance_)
    {
      stopRobot();

      ROS_INFO("Turn complete.");

      pause_start_time_ = ros::Time::now();
      state_ = PAUSE;
      return;
    }

    double turn_direction = (error >= 0.0) ? 1.0 : -1.0;
    publishVelocity(0.0, turn_direction * turn_speed_);
  }

  void handlePause()
  {
    stopRobot();

    double elapsed = (ros::Time::now() - pause_start_time_).toSec();

    if (elapsed >= pause_time_)
    {
      state_ = DRIVE_FORWARD;
    }
  }

  void publishVelocity(double linear_x, double angular_z)
  {
    geometry_msgs::Twist cmd;

    cmd.linear.x = linear_x;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = angular_z;

    cmd_pub_.publish(cmd);
  }

  void stopRobot()
  {
    publishVelocity(0.0, 0.0);
  }

  void publishLedCount()
  {
    std_msgs::Int32 led_msg;

    // The OpenCR LED callback displays this value in HEX.
    led_msg.data = hit_count_ & 0xFFFF;

    led_pub_.publish(led_msg);
  }

  double distance(double x1, double y1, double x2, double y2)
  {
    double dx = x2 - x1;
    double dy = y2 - y1;

    return std::sqrt(dx * dx + dy * dy);
  }

  double normalizeAngle(double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }

    return angle;
  }

  double angleDifference(double target, double current)
  {
    return normalizeAngle(target - current);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lab6_wall_bump_cpp");

  Lab6WallBump controller;
  controller.run();

  return 0;
}



