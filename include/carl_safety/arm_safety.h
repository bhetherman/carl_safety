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


/*!
 * \def START_FORWARD_SAFETY_THROTTLE_DIST
 *
 * The dist to start throttling forward linear vel for safety.
 */
#define LARGE_ACTUATOR_THRESHOLD  26.0

/*!
 * \def MIN_SAFE_DIST
 *
 * The dist to start throttling forward linear vel for safety.
 */
#define SMALL_ACTUATOR_THRESHOLD  7.0

/*!
 * \def START_SAFETY_THROTTLE_DIST
 *
 * The dist to start throttling linear vel for safety.
 */
#define FINGER_ACTUATOR_THRESHOLD 1.5



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
  int make_noise; /*!< launch param to determine if this node should make noise */
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
