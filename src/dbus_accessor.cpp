#include <atemr_dbus/dbus_accessor.h>

namespace dbus
{
  DBUSAccessor::DBUSAccessor(const ros::NodeHandle &nh): nh_(nh)
  {
    snm_serviceName_ = "org.freedesktop.NetworkManager";
    snm_objectPath_ = "/org/freedesktop/NetworkManager";
    slgn_serviceName_ = "org.freedesktop.login1";
    slgn_objectPath_ = "/org/freedesktop/login1";
  }
}
