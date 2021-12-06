#ifndef DBUS_ACCESSOR_H
#define DBUS_ACCESSOR_H

#include <atemr_dbus/network_access.h>
#include <atemr_dbus/system_access.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <atemr_msgs/DBUSService.h>
#include <boost/atomic.hpp>

namespace dbus
{
  class DBUSAccessor
  {
  public:
    DBUSAccessor(const ros::NodeHandle &nh);
    ~DBUSAccessor(){}

    bool dbusServe(atemr_msgs::DBUSServiceRequest &, atemr_msgs::DBUSServiceResponse &);
    boost::atomic_bool &isShutDown(){ return bshutdown_triggered_; }
    void shutDown(){ lgnProxy_->PowerOff(true); }

  private:
    std::string snm_serviceName_, snm_objectPath_, slgn_serviceName_, slgn_objectPath_,
                swlan_iface_, sip_address_;
    boost::atomic_bool bshutdown_triggered_{false};
    ros::NodeHandle nh_;
    ros::ServiceServer dbus_srvr_;
    boost::shared_ptr<NetworkManagerProxy> nmProxy_;
    boost::shared_ptr<LoginProxy> lgnProxy_;
  };
}

#endif // DBUS_ACCESSOR_H
