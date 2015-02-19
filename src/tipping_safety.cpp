#include <carl_safety/tipping_safety.h>

TippingSafety::TippingSafety()
{
  // a private handle for this ROS node (allows retrieval of relative parameters)
  ros::NodeHandle private_nh("~");
  private_nh.param<bool>("enable_audible_warnings", enableAudibleWarnings, false);

  jointStateSubscriber = node.subscribe("frame_joint_states", 1, &TippingSafety::jointStateCallback, this);

  // ROS services
  jacoEStopClient = node.serviceClient<wpi_jaco_msgs::EStop>("jaco_arm/software_estop");
  safeNavStopClient = node.serviceClient<std_srvs::Empty>("carl_safety/stop_base_nav");
}

void TippingSafety::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  int pitchIndex = 0;
  int rollIndex = 1;
  for (unsigned int i = 0; i < msg->name.size(); i ++)
  {
    if (msg->name[i].compare("base_footprint_base_link_pitch_joint") == 0)
    {
      pitchIndex = i;
    }
    else if (msg->name[i].compare("base_footprint_base_link_roll_joint") == 0)
    {
      rollIndex = i;
    }
  }

  if (fabs(msg->position[pitchIndex]) > BASE_PITCH_THRESHOLD || fabs(msg->position[rollIndex]) > BASE_ROLL_THRESHOLD)
  {
    ROS_INFO("Tipping detected, sending stop commands!");
    wpi_jaco_msgs::EStop armStopSrv;
    armStopSrv.request.enableEStop = true;
    std_srvs::Empty baseStopSrv;
    if (!jacoEStopClient.call(armStopSrv))
    {
      ROS_INFO("Could not call jaco estop service.");
    }
    if (!safeNavStopClient.call(baseStopSrv))
    {
      ROS_INFO("Could not call safe nav stop service.");
    }
    if (enableAudibleWarnings)
    {
      system("mpg123 /etc/carl/mario.mp3");
    }
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "tipping_safety");

  TippingSafety ts;

  ros::spin();

  return EXIT_SUCCESS;
}
