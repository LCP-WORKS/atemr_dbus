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
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
