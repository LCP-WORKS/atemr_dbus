#include <atemr_dbus/dbus_accessor.h>

namespace dbus
{
  DBUSAccessor::DBUSAccessor(const ros::NodeHandle &pnh, const ros::NodeHandle &nh): p_nh_(pnh), nh_(nh)
  {
    snm_serviceName_ = "org.freedesktop.NetworkManager";
    snm_objectPath_ = "/org/freedesktop/NetworkManager";
    slgn_serviceName_ = "org.freedesktop.login1";
    slgn_objectPath_ = "/org/freedesktop/login1";

    p_nh_.param<std::string>("wlan_interface", swlan_iface_, "wlan0");
    dbus_srvr_ = nh_.advertiseService<atemr_msgs::DBUSServiceRequest, atemr_msgs::DBUSServiceResponse>
        ("DBUSServer", boost::bind(&DBUSAccessor::dbusServe, this, _1, _2));
    try
    {
      nmProxy_.reset(new NetworkManagerProxy(swlan_iface_, snm_serviceName_, snm_objectPath_));
      lgnProxy_.reset(new LoginProxy(slgn_serviceName_, slgn_objectPath_));
    } catch (ATEMRException &e) {
      if(e.code() == DBUS_ERROR)
      {
        ROS_ERROR_STREAM("Unable to instantiate DBUS node | " << e.code() << " | " << e.what());
        exit(-1);
      }
    }
  }

  bool DBUSAccessor::dbusServe(atemr_msgs::DBUSServiceRequest &req, atemr_msgs::DBUSServiceResponse &res)
  {
    if(req.queryIP.data)
      res.IP_Address.data = nmProxy_->getIPAddress();

    if(req.setNetwork.data)
    {
      bool res = nmProxy_->device_FindAndConnectAP(req.WIFI_SSID.data, req.WIFI_PASS.data, sip_address_);
      bcheck_ssid_ = true;
      return res;
    }

    if(req.queryPower.data)
      res.canPowerOFF.data = (lgnProxy_->CanPowerOff() == "yes");

    if(req.queryConnection.data)
       res.isConnected.data = nmProxy_->isConnected();

    if(req.triggerPowerOFF.data)//! return true, wait 3s and initiate shutdown procedures.
      bshutdown_triggered_ = true;

    return true;
  }
}
