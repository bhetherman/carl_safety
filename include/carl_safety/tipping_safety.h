/*!
 * \tipping_safety.h
 * \brief Prevents CARL from tipping over.
 *
 * tipping_safety listens to the joint state information of the base and sends an estop command to the arm and to
 * the nav_safety layer, preventing the robot from tipping over.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date February 6, 2015
 */

#ifndef TIPPING_SAFETY_H_
#define TIPPING_SAFETY_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <wpi_jaco_msgs/EStop.h>

//Tipping thresholds
#define BASE_PITCH_THRESHOLD 0.08
#define BASE_ROLL_THRESHOLD 0.035

class TippingSafety
{
public:
  /**
   * \brief Constructor
   */
  TippingSafety();

private:
  /**
   * \brief Robot frame joint states callback.
   * @param msg joint state data
   */
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Subscriber jointStateSubscriber; /*!< subscriber for robot frame joint states */

  ros::ServiceClient jacoEStopClient;
  ros::ServiceClient safeNavStopClient;

  bool enableAudibleWarnings;
};

#endif
