/*!
 * \nav_safety.h
 * \brief Prevents CARL from (manually) driving past a linear boundary on the map.
 *
 * nav_safety creates a ROS node that prevents CARL from crossing a line on the map
 * during manual navigation.  The node also adds estop functionality that prevents
 * only online manual navigation commands.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date August 6, 2014
 */

#ifndef NAV_SAFETY_H_
#define NAV_SAFETY_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>
#include <wpi_jaco_msgs/HomeArmAction.h>

//controller types
#define ANALOG 0 //analog triggers
#define DIGITAL 1 //digital triggers

//Boundary
#define BOUNDARY_X 4.4
#define BOUNDARY_Y 1.0
#define PI 3.14159

/*!
 * \class NavSafety
 * \brief Prevents CARL from (manually) driving past a linear boundary on the map.
 *
 * nav_safety creates a ROS node that prevents CARL from crossing a line on the map
 * during manual navigation.  The node also adds estop functionality that prevents
 * only online manual navigation commands.
 */
class NavSafety
{
public:
  /**
   * \brief Constructor
   */
  NavSafety();

  /**
   * \brief cancels all nav goals
   */
  void cancelNavGoals();

private:
  /**
   * \brief Joystick input callback.
   * @param joy joystick input data
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /**
   * \brief Callback for safe base velocity commands
   * @param msg velocity base command
   */
  void safeBaseCommandCallback(const geometry_msgs::Twist::ConstPtr& msg);

  /**
   * \brief Callback for robot base pose
   * @param msg pose message
   */
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);

  void safeMoveCallback(const move_base_msgs::MoveBaseGoalConstPtr &goal);

  bool isArmRetracted();

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher baseCommandPublisher; /*!< actual base command publisher */
  ros::Subscriber safeBaseCommandSubscriber; /*!< subscriber for base commands coming from the web */
  ros::Subscriber joySubscriber; /*!< subscriber for joystick input */
  ros::Subscriber robotPoseSubscriber; /*!< subscriber for the robot base pose */

  ros::ServiceClient jacoPosClient;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> acMoveBase;
  actionlib::SimpleActionClient<wpi_jaco_msgs::HomeArmAction> acHome;

  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> asSafeMove;

  int controllerType;
  bool stopped; /*!< true if safe nav commands should be stopped */
  float x;
  float y;
  float theta;
  std::vector<float> retractPos; //jaco arm retracted joint positions

  bool use_teleop_safety; /*!< launch param to determine which node to publish to */
};

#endif
