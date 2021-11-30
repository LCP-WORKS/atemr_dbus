#ifndef DBUS_ACCESSOR_H
#define DBUS_ACCESSOR_H

#include <atemr_dbus/network_access.h>
#include <atemr_dbus/system_access.h>
#include <ros/ros.h>

namespace dbus
{
  class DBUSAccessor
  {
  public:
    DBUSAccessor(const ros::NodeHandle &nh);
    ~DBUSAccessor(){}

  private:
    std::string snm_serviceName_, snm_objectPath_, slgn_serviceName_, slgn_objectPath_,
                swlan_iface_;
    ros::NodeHandle nh_;
  };
}

#endif // DBUS_ACCESSOR_H
