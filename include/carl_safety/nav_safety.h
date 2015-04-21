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
#include <carl_safety/Error.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
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

  /**
  * \brief Layer for safely controlling base movement commands
  *
  * This node will prevent the robot from moving while the arm is extended beyond the navigation footprint of the base;
  * input to this node can also be cut off independently from stopping lower level control, so that external base move
  * commands can be suspended
  */
  void safeMoveCallback(const move_base_msgs::MoveBaseGoalConstPtr &goal);

  /**
  * \brief Callback for stopping manual and autonomous navigation from the safe base movement layer
  *
  * @param req empty service request
  * @param res empty service response
  * @return true on success
  */
  bool navStopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool isArmRetracted();

  bool isArmContained();

  void publishArmNotContainedError();

  void publishStoppedError();

  void publishClearError();

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher baseCommandPublisher; /*!< actual base command publisher */
  ros::Publisher safetyErrorPublisher; /*!< safety error message publisher */
  ros::Subscriber safeBaseCommandSubscriber; /*!< subscriber for base commands coming from the web */
  ros::Subscriber joySubscriber; /*!< subscriber for joystick input */
  ros::Subscriber robotPoseSubscriber; /*!< subscriber for the robot base pose */

  ros::ServiceClient jacoPosClient;
  ros::ServiceClient jacoCartesianClient;
  ros::ServiceServer stopBaseNavServer;

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
