#include <carl_safety/teleop_safety.h>

using namespace std;

teleop_safety::teleop_safety()
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");

  // create the ROS topics
  safe_cmd_vel = node.advertise<geometry_msgs::Twist>("safe_cmd_vel", 10);
  cmd_vel = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &teleop_safety::cmd_vel_cback, this);
  scan_sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 10, &teleop_safety::scan_cback, this);
  map_sub = node.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &teleop_safety::map_cback, this);
  amcl_pose_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &teleop_safety::amcl_pose_cback, this);

  // initialize everything
  pListener = new (tf::TransformListener);

  forward_throttle_safety_factor_base = 0.0;
  reverse_throttle_safety_factor_base = 0.0;
}

void teleop_safety::cmd_vel_cback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  twist = *cmd;
  if(twist.linear.x > 0){
    twist.linear.x *= forward_throttle_safety_factor_base;
    safe_cmd_vel.publish(cmd);
  }
  else if(twist.linear.x < 0){
    twist.linear.x *= reverse_throttle_safety_factor_base;
    safe_cmd_vel.publish(twist);
  }
}

void teleop_safety::scan_cback(const sensor_msgs::LaserScan::ConstPtr& ptr)
{
  static int index = 0;
  static double closestDists[2];
  geometry_msgs::PointStamped laser_point;
  geometry_msgs::PointStamped base_point;

  sensor_msgs::LaserScan scan = *ptr;

  laser_point.header.frame_id = "hokuyo_link";
  laser_point.header.stamp = scan.header.stamp;
  laser_point.point.z = 0.0;

  int i = 0;
  double currentAngle;
  double closestDist = scan.range_max;

  for(currentAngle = scan.angle_min; currentAngle < scan.angle_max; currentAngle+=scan.angle_increment){
    //if the point is not within range
    if(scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max || isnan(scan.ranges[i])){
       i++;
       continue;
    }

    //get the x and y components of the vector to the point
    laser_point.point.x = cos(currentAngle) * scan.ranges[i];
    laser_point.point.y = sin(currentAngle) * scan.ranges[i];
    try{
      //transform that point to the base_link frame
      pListener->transformPoint("base_link", laser_point, base_point);
    }
    catch(tf::TransformException& ex){
      i++;
      continue;
    }
    
    //calculate the distance to point
    double dist = sqrt(pow(base_point.point.x,2.0)+pow(base_point.point.y,2.0));
    //if it is the closest point in the scan record it
    if(dist < closestDist){
      closestDist = dist;
    }
    i++;
  }
  
  //record the new closest point in the buffer of last 3 closest distances
  closestDists[index] = closestDist;
  if(index+1==2) index=0;
  else index++;

  //take the closest distance of the last 3 scans to determine speed with
  closestDist = scan.range_max;
  for(i = 0; i < 2; i++)
    if(closestDists[i] < closestDist)
      closestDist = closestDists[i];

  //set the throttle value based on the distance
  if(closestDist < MIN_FORWARD_SAFE_DIST)
    forward_throttle_safety_factor_base = 0.0;
  else if(closestDist > START_FORWARD_SAFETY_THROTTLE_DIST)
    forward_throttle_safety_factor_base = 1.0;
  else{
    forward_throttle_safety_factor_base = (closestDist-MIN_FORWARD_SAFE_DIST)/(START_FORWARD_SAFETY_THROTTLE_DIST-MIN_FORWARD_SAFE_DIST);
    if(forward_throttle_safety_factor_base < 0.05) forward_throttle_safety_factor_base = 0.05;
  }
  
}

void teleop_safety::map_cback(const nav_msgs::OccupancyGrid::ConstPtr& map){
  //captures the map for later use
  savedMap = *map;
}

void teleop_safety::amcl_pose_cback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
  //get distances in occupancy grid units 
  static double startTrottleDistMap = START_REVERSE_SAFETY_THROTTLE_DIST/savedMap.info.resolution;
  static double minSafeDistMap = MIN_REVERSE_SAFE_DIST/savedMap.info.resolution;

  //get the yaw angle of robot in radians 
  double angle = tf::getYaw(pose->pose.pose.orientation);
  double closestDist;

  //round estimated position to nearest occupancy grid cell
  int i = round(pose->pose.pose.position.y/savedMap.info.resolution);
  int j = round(pose->pose.pose.position.x/savedMap.info.resolution);

  closestDist = startTrottleDistMap;
  reverse_throttle_safety_factor_base = 0.8;
  //for each grid cell that is within startTrottleDistMap of (x,y)
  for(int x = i - (int)round(startTrottleDistMap); x < i + (int)round(startTrottleDistMap); x++){
    for(int y = j - (int)round(startTrottleDistMap); y < j + (int)round(startTrottleDistMap); y++){

      //if current point is within the map and is an obstical 
      if(x >= 0 && x < savedMap.info.height && y >= 0 && y < savedMap.info.width 
         && savedMap.data[(x*savedMap.info.width) + y] != 0){

        //get the angle and distance of the point in relation to the robot in the grid
        double ptAngle = atan2(x-i,y-j);
        double dist = sqrt(pow(x-i,2)+pow(y-j,2));

        double angleDiff = angle-ptAngle;
        while (angleDiff < -M_PI) angleDiff += (2*M_PI);
        while (angleDiff > M_PI) angleDiff -= (2*M_PI);

        //if the point is the clostes point thus far and it is behind the robot
        if(dist<=closestDist && angleDiff>(3*M_PI/4)){//(ptAngle<angle-(3*M_PI/4) || ptAngle>angle+(3*M_PI/4))){
          //set new closest distance
          closestDist = dist;
          //if the distance is below the min allowed stop the robot
          if(closestDist < minSafeDistMap)
            reverse_throttle_safety_factor_base = 0.0; 
          //else if it is in the trottle zone, linearly decearse the allowed speed 
          //in relation to the distance of the point
          else if(closestDist <= startTrottleDistMap){
            reverse_throttle_safety_factor_base = (closestDist-minSafeDistMap)/(startTrottleDistMap-minSafeDistMap);
            if(reverse_throttle_safety_factor_base < 0.05) reverse_throttle_safety_factor_base = 0.05;
          }
        } 

      }
    }
  }
  
  if(reverse_throttle_safety_factor_base > 0.8) reverse_throttle_safety_factor_base = 0.8;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "teleop_safety");
  teleop_safety t;
  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
