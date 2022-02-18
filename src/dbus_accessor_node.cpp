#include <atemr_dbus/dbus_accessor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atemr_dbus_NODE");
  ros::NodeHandle g_nh("");
  ros::NodeHandle p_nh("~");
  dbus::DBUSAccessor accessor(p_nh, g_nh);
  ros::Publisher ssid_pub = g_nh.advertise<std_msgs::String>("/connection_status", 1, true);
  std_msgs::String msg;

  ros::Rate r(8);
  std::string curSSID;
  while(ros::ok() && !ros::isShuttingDown())
  {
    ROS_INFO_ONCE("DBUS node running ...");

    //! SSID publishing
    if(accessor.canCheckSSID())
    {
      curSSID = accessor.currentSSID();
      if(!curSSID.empty())
      {
        msg.data = curSSID;
        ssid_pub.publish(msg);
        accessor.resetSSIDCheck();
      }
    }

    //! SHUTDOWN request processing
    if(accessor.isShutDown() || accessor.isReboot())
    {
      ROS_INFO_ONCE("DBUS node - preparing to shutdown (restart) system");
      //! 1- wait a few seconds (8 s)
      std::this_thread::sleep_for(8s);
      //! 2- break loop
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
  if(accessor.isReboot())//! 3- initiate shutdown
    accessor.reboot();
  if(accessor.isShutDown())//! 3- initiate shutdown
    accessor.shutDown();

  return 0;
}
