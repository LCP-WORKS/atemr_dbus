#include <atemr_dbus/dbus_accessor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atemr_dbus_NODE");
  ros::NodeHandle g_nh("");
  dbus::DBUSAccessor accessor(g_nh);

  ros::Rate r(15);
  while(ros::ok() && !ros::isShuttingDown())
  {
    ROS_INFO_ONCE("DBUS node running ...");
    if(accessor.isShutDown())
    {
      ROS_INFO_ONCE("DBUS node - preparing to shutdown system");
      //! 1- wait a few seconds (3 s)
      std::this_thread::sleep_for(3s);
      //! 2- break loop
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
  if(accessor.isShutDown())//! 3- initiate shutdown
    accessor.shutDown();

  return 0;
}
