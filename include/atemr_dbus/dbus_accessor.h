#ifndef DBUS_ACCESSOR_H
#define DBUS_ACCESSOR_H

#include <atemr_dbus/network_access.h>
#include <atemr_dbus/system_access.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <atemr_msgs/DBUSService.h>
#include <std_msgs/String.h>
#include <boost/atomic.hpp>

namespace dbus
{
  class DBUSAccessor
  {
  public:
    DBUSAccessor(const ros::NodeHandle &, const ros::NodeHandle &);
    ~DBUSAccessor(){}

    bool dbusServe(atemr_msgs::DBUSServiceRequest &, atemr_msgs::DBUSServiceResponse &);
    boost::atomic_bool &isShutDown(){ return bshutdown_triggered_; }
    boost::atomic_bool &isReboot(){ return breboot_triggered_; }
    void shutDown(){ lgnProxy_->PowerOff(true); }
    void reboot(){ lgnProxy_->Reboot(true); }
    boost::atomic_bool &canCheckSSID(){ return bcheck_ssid_; }
    void resetSSIDCheck(){ bcheck_ssid_ = false; }
    std::string currentSSID(){ return nmProxy_->getCurrentSSID(); }

  private:
    std::string snm_serviceName_, snm_objectPath_, slgn_serviceName_, slgn_objectPath_,
                swlan_iface_, sip_address_;
    boost::atomic_bool bshutdown_triggered_{false}, bcheck_ssid_{true}, breboot_triggered_{false};
    ros::NodeHandle p_nh_, nh_;
    ros::ServiceServer dbus_srvr_;
    boost::shared_ptr<NetworkManagerProxy> nmProxy_;
    boost::shared_ptr<LoginProxy> lgnProxy_;
  };
}

#endif // DBUS_ACCESSOR_H
