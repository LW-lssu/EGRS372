#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/tfMessage.h>
#include <cmath>
#include <string>

#define INITIALIZE_VALUE -9999.0
#define PI 3.14159265359

class BarcodeAction
{
public:
  BarcodeAction()
  {

  current_x = INITIALIZE_VALUE;
  current_y = INITIALIZE_VALUE;
  current_yaw = 0.0;
  tf_ready = false;
  busy = false;

    // REAL BARCODE VALUES
    barcode_1 = "705632441947";
    barcode_2 = "051111407592";
    barcode_3 = "123456789012";

    // SETUP SUBS AND PUBS
    barcode_sub = nh.subscribe("/barcode_confirmed", 10, &BarcodeAction::barcodeCallback, this);
    tf_sub = nh.subscribe("/tf", 10, &BarcodeAction::tfCallback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ROS_INFO("barcode_action started. Listening on /barcode_confirmed");
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber barcode_sub;
  ros::Subscriber tf_sub;
  ros::Publisher vel_pub;

  // INITIALIZE KNOWN BARCODES
  std::string barcode_1;
  std::string barcode_2;
  std::string barcode_3;

  // INITIALIZE POSE
  double current_x;
  double current_y;
  double current_yaw;

  // MAKE SURE TF HAS BEEN READ ONCE
  bool tf_ready;

  // TO PREVENT RUNNING ANOTHER ACTION WHILE DOING ONE
  bool busy;


  // THIS FUNCTION TAKES THE CONFIRMED BARCODE AND DOES MOTION DEPENDING ON MATCHING BARCODE
  void barcodeCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::string barcode = msg->data;

    ROS_INFO("Received confirmed barcode: %s", barcode.c_str());

    // IF IT READS A BARCODE WHILE DOING AN ACTION
    if (busy)
    {
      ROS_WARN("Robot is already executing an action. Ignoring new barcode.");
      return;
    }

    busy = true;

    // IF BARCODE 1 THEN DO CLOCKWISE SQUARE
    if (barcode == barcode_1)
    {
      ROS_INFO("Valid barcode #1");
      SquareClockwise(1.0);
    }
    else if (barcode == barcode_2)
    {
      ROS_INFO("Valid barcode #2 -> square counterclockwise");
      SquareCounterclockwise(1.0);
    }
    else if (barcode == barcode_3)
    {
      ROS_INFO("Valid barcode #3 -> turn 180, forward 2m, turn 180, forward 2m");
      Barcode3Path();
    }
    else
    {
      ROS_WARN("invalid barcode");
    }
    //STOPS ROBOT
    stopRobot();
    //FLAG FALSE FOR DOING MOTION
    busy = false;
  }

  // THIS FUNCTION READS FROM TF TO GET MR POSE INFO
  void tfCallback(const tf::tfMessage& msg)
  {
    if (msg.transforms.empty())
    {
      return;
    }

    current_x = msg.transforms[0].transform.translation.x;
    current_y = msg.transforms[0].transform.translation.y;

    double z = msg.transforms[0].transform.rotation.z;
    double w = msg.transforms[0].transform.rotation.w;

    current_yaw = 2.0 * atan2(z, w);
    current_yaw = normalizeAngle(current_yaw);

    tf_ready = true;
  }

  // THIS FUNCTION PUTS ANGLE BETWEEN 0 AND 2 PI
  double normalizeAngle(double angle)
  {
    while (angle > PI)
      angle -= 2.0 * PI;
    while (angle < -PI)
      angle += 2.0 * PI;
    return angle;
  }

  // THIS FUNCTION CALCULATES THE DISTANCE BETWEEN TWO POINTS
  double distance(double x1, double y1, double x2, double y2)
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }

  // THIS FUNCTION MAKES THE MR STOP
  void stopRobot()
  {
    // SET VELOCITIES TO 0
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    ros::Rate rate(20);
    // PUBLISH ZEROED VELOCITIES MULTIPLE TIMES TO MAKE SURE
    for (int i = 0; i < 5; i++)
    {
      vel_pub.publish(cmd);
      ros::spinOnce();
      rate.sleep();
    }
  }

  // THIS FUNCTION MAKES THE MR GO FORWARD UNTIL TARGET DISTANCE IS REACHED
  void moveForward(double target_distance)
  {
    // KEEP UPDATING UNTIL TF HAS BEEN PROCESSED AT LEAST ONCE
    while (ros::ok() && !tf_ready)
    {
      ros::spinOnce();
    }

    double start_x = current_x;
    double start_y = current_y;

    geometry_msgs::Twist cmd;
    ros::Rate rate(50);

    while (ros::ok())
    {
      ros::spinOnce();

      double moved = distance(start_x, start_y, current_x, current_y);

      // IF MR HAS REACHED TARGET DISTANCE BREAK OUT OF LOOP
      if (moved >= target_distance)
      {
        break;
      }

      double remaining = target_distance - moved;
      double speed = remaining / 2.0 + 0.1;
      // MAKES MAX SPEED 0.25 M/S
      if (speed > 0.25)
      {
        speed = 0.25;
      }

      cmd.linear.x = speed;
      cmd.angular.z = 0.0;
      vel_pub.publish(cmd);

      rate.sleep();
    }

    stopRobot();
  }

  // THIS FUNCTION MAKES THE MR TURN UNTIL TAGET ANGLE IS ACHIEVED
  void turnRelative(double relative_angle)
  {
    // KEEP UPDATING UNTIL TF HAS BEEN PROCESSED AT LEAST ONCE
    while (ros::ok() && !tf_ready)
    {
      ros::spinOnce();
    }

    double start_yaw = current_yaw;
    double target_yaw = normalizeAngle(start_yaw + relative_angle);

    geometry_msgs::Twist cmd;
    ros::Rate rate(50);

    while (ros::ok())
    {
      ros::spinOnce();

      //ERROR IS THE TURNING DISTACNE BETWEEN GOAL ANGLE AND CURRENT ANGLE
      double error = normalizeAngle(target_yaw - current_yaw);

      // IF IT IS WITHING 0.05 RAD OF GOAL ANGLE
      if (std::abs(error) < 0.05)
      {
        break;
      }
      // SET ANGULAR SPEED PROPORTIAONL TO ERROR
      double speed = std::abs(error) / 2.0 + 0.2;

      // SET MAX ANGULAR SPEED OF MR TO 0.8
      if (speed > 0.8)
        speed = 0.8;

      // IF MR OVERSHOOTS TURN BACK SLIGHTLY
      if (error > 0)
        cmd.angular.z = speed;
      else
        cmd.angular.z = -speed;

      cmd.linear.x = 0.0;
      vel_pub.publish(cmd);

      rate.sleep();
    }

    stopRobot();
  }

  // THIS FUNCTION MAKES THE MR DRIVE A CW SQUARE
  void SquareClockwise(double side_length)
  {
    // LOOP 4 TIMES FOR EACH CORNER
    for (int i = 0; i < 4 && ros::ok(); i++)
    {
      turnRelative(-PI / 2.0);
      moveForward(side_length);
    }
  }

  // THIS FUNCTION MAKES THE MR DRIVE A CCW SQUARE
  void SquareCounterclockwise(double side_length)
  {
    // LOOPS 4 TIMES FOR EACH CORNER
    for (int i = 0; i < 4 && ros::ok(); i++)
    {
      turnRelative(PI / 2.0);
      moveForward(side_length);
    }
  }

  // THIS FUNCTION MAKE THE MR DRIVE A LINE THEN TURN AROUND AND DRIVE BACK ALONG THE LINE
  void Barcode3Path()
  {
    turnRelative(PI);
    moveForward(2.0);
    turnRelative(PI);
    moveForward(2.0);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barcode_action");
  BarcodeAction node;
  ros::spin();
  return 0;
}
