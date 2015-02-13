/*!
 * \arm_safety.h
 * \brief notifies users of exceeding torque limit on joints
 *
 * arm_safety creates a ROS node that causes an audible notification and ROS error
 * when the torque one one of CARL's arm joints exceeds a threshold.
 *
 * \author Brian Hetherman, WPI - bhetherman@wpi.edu
 * \date October 27, 2014
 */

#ifndef ARM_SAFETY_H_
#define ARM_SAFETY_H_

#include <ros/ros.h>
#include <time.h> 
#include <stdio.h>
#include <sensor_msgs/JointState.h>

#define J1_THRESHOLD  7.0
#define J2_THRESHOLD  25.5
#define J3_THRESHOLD 14.0
#define J4_THRESHOLD  5.0
#define J5_THRESHOLD  5.0
#define J6_THRESHOLD 3.5
#define F1_THRESHOLD  1.5
#define F2_THRESHOLD  1.5
#define F3_THRESHOLD 1.5

/*!
 * \class arm_safety
 * \brief notifies users of exceeding torque limit on joints
 *
 * arm_safety creates a ROS node that causes an audible notification and ROS error
 * when the torque one one of CARL's arm joints exceeds a threshold.
 */
class arm_safety
{
public:
  /**
   * \brief Constructor
   */
  arm_safety();

private:
  /*!
   * Joint_states topic callback function.
   *
   * \param scan - the message for the scan topic
   */
  void joints_cback(const sensor_msgs::JointState::ConstPtr& scan);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Subscriber joint_sub; /*!< the JointState topic */

  bool enable_audible_warnings; /*!< launch param to determine if this node should produce audible warnings when the
   arm exceeds a safety threshold*/
};

/*!
 * Creates and runs the arm_safety node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
