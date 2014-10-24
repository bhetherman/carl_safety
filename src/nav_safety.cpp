#include <carl_safety/nav_safety.h>

NavSafety::NavSafety() :
    acMoveBase("/move_base", true),
    acHome("jaco_arm/home_arm", true),
    asSafeMove(node, "/move_base_safe", boost::bind(&NavSafety::safeMoveCallback, this, _1), false)
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
  safeBaseCommandSubscriber = node.subscribe("cmd_vel_safe", 1, &NavSafety::safeBaseCommandCallback, this);
  joySubscriber = node.subscribe("joy", 1, &NavSafety::joyCallback, this);
  robotPoseSubscriber = node.subscribe("robot_pose", 1, &NavSafety::poseCallback, this);
  jacoJointsSubscriber = node.subscribe("jaco_arm/joint_states", 1, &NavSafety::jacoJointsCallback, this);

  //initialization
  stopped = false;
  x = 0.0;
  y = 0.0;
  theta = 0.0;
  joints.resize(6);
  retractPos.resize(6);
  retractPos[0] = -2.57;
  retractPos[1] = 1.39;
  retractPos[2] = .527;
  retractPos[3] = -.084;
  retractPos[4] = .515;
  retractPos[5] = -1.745;

  asSafeMove.start();
}

void NavSafety::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
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

void NavSafety::safeBaseCommandCallback(const geometry_msgs::Twist::ConstPtr& msg)
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
        if (theta > -PI / 2.0 && theta < PI / 2.0)
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

bool NavSafety::isStopped()
{
  return stopped;
}

void NavSafety::cancelNavGoals()
{
  acMoveBase.cancelAllGoals();
}

void NavSafety::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  x = msg->position.x;
  y = msg->position.y;

  //convert quaternion to yaw
  float q0 = msg->orientation.w;
  float q1 = msg->orientation.x;
  float q2 = msg->orientation.y;
  float q3 = msg->orientation.z;
  theta = -atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
}

void NavSafety::jacoJointsCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  for (unsigned int i = 0; i < 6; i ++)
  {
    joints[i] = msg->position[i];
  }
}

void NavSafety::safeMoveCallback(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
  float dstFromRetract = 0;
  for (unsigned int i = 0; i < 6; i ++)
  {
    dstFromRetract += fabs(retractPos[i] - joints[i]);
  }
  ROS_INFO("Distance from retract position: %f", dstFromRetract);
  if (dstFromRetract > 0.175)
  {
    ROS_INFO("Retracting arm for safe navigation...");
    wpi_jaco_msgs::HomeArmGoal retractGoal;
    retractGoal.retract = true;
    retractGoal.retractPosition.position = true;
    retractGoal.retractPosition.armCommand = true;
    retractGoal.retractPosition.fingerCommand = false;
    retractGoal.retractPosition.repeat = false;
    retractGoal.retractPosition.joints.resize(6);
    retractGoal.retractPosition.joints = retractPos;
    acHome.sendGoal(retractGoal);
    acHome.waitForResult(ros::Duration(15.0));
    ros::Duration(3.0).sleep();
  }
  ROS_INFO("Sending nav goal to move_base action server.");
  acMoveBase.sendGoal(*goal);
  acMoveBase.waitForResult();
  asSafeMove.setSucceeded(*acMoveBase.getResult());
  ROS_INFO("Finished");
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "nav_safety");

  NavSafety n;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    if (n.isStopped())
      n.cancelNavGoals();
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
