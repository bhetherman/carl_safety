/*!
 * \teleop_safety.h
 * \brief Prevents CARL from (manually) driving past a linear boundary on the map.
 *
 * nav_safety creates a ROS node that prevents CARL from crossing a line on the map
 * during manual navigation.  The node also adds estop functionality that prevents
 * only online manual navigation commands.
 *
 * \author Brian Hetherman, WPI - bhetherman@wpi.edu
 * \date October 1, 2014
 */

#ifndef TELEOP_SAFETY_H_
#define TELEOP_SAFETY_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <unistd.h>

/*!
 * \def START_FORWARD_SAFETY_THROTTLE_DIST
 *
 * The dist to start throttling forward linear vel for safety.
 */
#define START_FORWARD_SAFETY_THROTTLE_DIST 1.25

/*!
 * \def MIN_SAFE_DIST
 *
 * The dist to start throttling forward linear vel for safety.
 */
#define MIN_FORWARD_SAFE_DIST 0.55

/*!
 * \def START_SAFETY_THROTTLE_DIST
 *
 * The dist to start throttling linear vel for safety.
 */
#define START_REVERSE_SAFETY_THROTTLE_DIST 1.4

/*!
 * \def MIN_SAFE_DIST
 *
 * The dist to start throttling linear vel for safety.
 */
#define MIN_REVERSE_SAFE_DIST 0.65


/*!
 * \class TeleopSafety
 * \brief Prevents CARL from (manually) driving past a linear boundary on the map.
 *
 * teleop_safety creates a ROS node that prevents CARL from driving into obsticals on the
 * map or seen by the laser scanner during manual navigation.
 */
class teleop_safety
{
public:
  /**
   * \brief Constructor
   */
  teleop_safety();

  /**
  * \brief Destructor
  */
  ~teleop_safety();

private:
  /*!
   * Scan topic callback function.
   *
   * \param scan - the message for the scan topic
   */
  void scan_cback(const sensor_msgs::LaserScan::ConstPtr& scan);

  /*!
   * map topic callback function.
   *
   * \param map - the message for the map topic
   */
  void map_cback(const nav_msgs::OccupancyGrid::ConstPtr& map);

  /*!
   * amcl_pose topic callback function.
   *
   * \param pose - the message for the amcl_pose topic
   */
  void amcl_pose_cback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);



  void cmd_vel_safety_check_cback(const geometry_msgs::Twist::ConstPtr& cmd);

  ros::NodeHandle node; /*!< a handle for this ROS node */
  tf::TransformListener* pListener;

  ros::Publisher cmd_vel_pub; /*!< cmd_vel publisher for the base */
  ros::Subscriber cmd_vel_safety_check; /*!< subscriber to the cmd_vel topic that should be checked for safety */
  ros::Subscriber scan_sub; /*!< the scan topic */
  ros::Subscriber map_sub; /*!< the map topic */
  ros::Subscriber amcl_pose_sub; /*!< the amcl_pose topic */

  geometry_msgs::Twist twist; /*!< base movement command */
  nav_msgs::OccupancyGrid savedMap; /*!< map of allowed speeds */

  double forward_throttle_safety_factor_base; /*!< factor for reducing the base maximum positive linear speed from laser scan*/
  double reverse_throttle_safety_factor_base; /*!< factor for reducing the base maximum negetive linear speed from map*/
};

/*!
 * Creates and runs the teleop_safety node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
