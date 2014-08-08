#include <carl_safety/nav_safety.h>

navSafety::navSafety() :
    acMoveBase("/move_base", true)
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");
  
  // read in parameters
  std::string str;
  private_nh.param<std::string>("controller_type", str, "digital");
  if (str.compare("digital") == 0)
    controllerType = DIGITAL;
  else
    controllerType = ANALOG;

  // ROS topics
  baseCommandPublisher = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  safeBaseCommandSubscriber = node.subscribe("cmd_vel_safe", 1, &navSafety::safeBaseCommandCallback, this);
  joySubscriber = node.subscribe("joy", 1, &navSafety::joyCallback, this);
  robotPoseSubscriber = node.subscribe("robot_pose", 1, &navSafety::poseCallback, this);
  
  //initialization
  stopped = false;
  x = 0.0;
  y = 0.0;
  theta = 0.0;
}

void navSafety::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (controllerType == DIGITAL)
  {
    if (joy->buttons.at(8) == 1)
      stopped = true;
    else if (joy->buttons.at(9) == 1)
      stopped = false;
  }
  else
  {
    if (joy->buttons.at(6) == 1)
      stopped = true;
    else if (joy->buttons.at(7) == 1)
      stopped = false;
  }
}

void navSafety::safeBaseCommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (!stopped)
  {
    if (x < BOUNDARY_X && y > BOUNDARY_Y)
    {
      //pass command through
      baseCommandPublisher.publish(*msg);
    }
    else
    {
      if (x >= BOUNDARY_X)
      {
        if (theta > -PI/2.0 && theta < PI/2.0)
        {
          //only publish if going backwards (left on map)
          if (msg->linear.x <= 0.0)
            baseCommandPublisher.publish(*msg);
        }
        else
        {
          //only publish if going forwards (left on map)
          if (msg->linear.x >= 0.0)
            baseCommandPublisher.publish(*msg);
        }
      }
      
      if (y <= BOUNDARY_Y)
      {
        if (theta > 0.0)
        {
          //only publish if going forwards (up on map)
          if (msg->linear.x <= 0.0)
            baseCommandPublisher.publish(*msg);
        }
        else
        {
          //only publish if going backwards (up on map)
          if (msg->linear.x >= 0.0)
            baseCommandPublisher.publish(*msg);
        }
      }
    }
  }
}

bool navSafety::isStopped()
{
  return stopped;
}

void navSafety::cancelNavGoals()
{
  acMoveBase.cancelAllGoals();
}

void navSafety::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  x = msg->position.x;
  y = msg->position.y;
  
  //convert quaternion to yaw
  float q0 = msg->orientation.w;
  float q1 = msg->orientation.x;
  float q2 = msg->orientation.y;
  float q3 = msg->orientation.z;
  theta = -atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "nav_safety");

  navSafety n;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    if (n.isStopped())
      n.cancelNavGoals();
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
