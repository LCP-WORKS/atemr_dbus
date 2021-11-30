#ifndef NETWORK_ACCESS_H
#define NETWORK_ACCESS_H

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <thread>
#include <chrono>
#include <boost/bind.hpp>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <string.h>
#include <atemr_dbus/nm-glue.h>

#define NM_ACTIVE_CONNECTION_STATE_ACTIVATED 2
#define NM_STATE_CONNECTED_GLOBAL 70

using namespace std::chrono_literals;


class NetworkManagerProxy: public sdbus::ProxyInterfaces<org::freedesktop::NetworkManager_proxy,
    org::freedesktop::DBus::Properties_proxy>
{
public:
  NetworkManagerProxy(const std::string &iface, std::string destination, std::string objectPath):
    ProxyInterfaces(destination, std::move(objectPath))
  {
    service_name_ = destination;
    registerProxy();

    device_path_ = GetDeviceByIpIface(iface);
    device_proxy_ = sdbus::createProxy(service_name_, device_path_);
    //! enable wireless if not enabled
    if(!WirelessEnabled())
    {
      WirelessEnabled(true);
      //! sleep 10 secs
      std::this_thread::sleep_for(10s);
    }
  }

  ~NetworkManagerProxy()
  {
    unregisterProxy();
  }

  bool device_FindAndConnectAP(const std::string &ssid,
                               const std::string &passwd, std::string &ip)
  {
    if(!checkExistingConnection(device_proxy_, ssid))
    {//! establish new connection
      //! trigger AP scan
      requestScan();
      //! get available APs and find the object path of our desired AP
      sdbus::ObjectPath ap_obj_path = device_getAccessPoint(ssid);
      //!check if AP found
      if ((ap_obj_path.data() != NULL) && (ap_obj_path.data()[0] == '\0')) {
         std::cerr << "AP not found!!";
         return false;
      }

      //! begin connection to AP
      std::map<std::string, std::map<std::string, sdbus::Variant>> connection_params{
        {"802-11-wireless", std::map<std::string, sdbus::Variant>{{"security", sdbus::Variant("802-11-wireless-security")}}},
        {"802-11-wireless-security", std::map<std::string, sdbus::Variant>{{"psk", sdbus::Variant(passwd)},
                                                                           {"key-mgmt", sdbus::Variant("wpa-psk")}}},
      };
      std::tuple<sdbus::ObjectPath, sdbus::ObjectPath> conn_res = AddAndActivateConnection(connection_params,
                                                                                           device_path_, ap_obj_path);
      //! wait for connection to become active
      conn_proxy_ = sdbus::createProxy(service_name_, std::get<1>(conn_res));
      settings_proxy_ = sdbus::createProxy(service_name_, std::get<0>(conn_res));
    }

    int cnt = 30;
    try
    {
      while(true)
      {
        std::cout << "Connecting ... " << std::endl;
        uint32_t conn_state = conn_proxy_->getProperty("State")
            .onInterface("org.freedesktop.NetworkManager.Connection.Active");
        if(conn_state == NM_ACTIVE_CONNECTION_STATE_ACTIVATED)
          break;
        if(cnt <= 0)
          break;
        std::this_thread::sleep_for(0.2s);
        cnt --;
      }
    } catch (sdbus::Error &e) {
      std::cerr << "Got concatenate error " << e.getName() <<
                   " with message " << e.getMessage() << std::endl;
    }

    //! check if connnected successfully
    uint32_t conn_state = conn_proxy_->getProperty("State")
        .onInterface("org.freedesktop.NetworkManager.Connection.Active");
    if(conn_state != NM_ACTIVE_CONNECTION_STATE_ACTIVATED)
      return false;

    //! get IP address
    try {
      ip = getIPAddress();
    } catch (...) {
      return false;
    }
    return true;
  }

  void device_Disconnect()
  {
    if(settings_proxy_ != nullptr)
      settings_proxy_->callMethod("Delete").onInterface("org.freedesktop.NetworkManager.Settings.Connection");
  }
protected:
  sdbus::ObjectPath device_getAccessPoint(const std::string &target_ssid)
  {
    std::vector<sdbus::ObjectPath> access_points_list;
    sdbus::ObjectPath ap_obj_path;
    device_proxy_->callMethod("GetAccessPoints")
        .onInterface("org.freedesktop.NetworkManager.Device.Wireless").storeResultsTo(access_points_list);
    for (std::vector<sdbus::ObjectPath>::const_iterator iter =  access_points_list.cbegin();
         iter != access_points_list.cend(); ++iter)
    {
      std::cout << "Found -> "<<iter->data() << std::endl;
      //! AP obj to access its properties
      std::unique_ptr<sdbus::IProxy> tmp_ap_proxy_ = sdbus::createProxy(service_name_, iter->data());
      std::vector<uint8_t> Ssid = tmp_ap_proxy_->getProperty("Ssid")
          .onInterface("org.freedesktop.NetworkManager.AccessPoint");
      std::string tmp_ssid;
      for (std::size_t t = 0; t < Ssid.size(); t++)
      { tmp_ssid.push_back(char(Ssid[t])); }
      if(tmp_ssid == target_ssid)
      {
        ap_obj_path = *iter;
        break;
      }
    }
    return ap_obj_path;
  }

  void requestScan()
  {
    /*bool scan_completed{false};
    device_proxy_->uponSignal("PropertiesChanged").onInterface(service_name_)
        .call([&](const std::map<std::string, sdbus::Variant> &){ scan_completed = true; });
    device_proxy_->finishRegistration();*/
    try {
      device_proxy_->callMethod("RequestScan")
          .onInterface("org.freedesktop.NetworkManager.Device.Wireless")
          .withArguments(std::map<std::string, sdbus::Variant>({}));
      std::cout << "Waiting for AP scan to complete ..." << std::endl;
      std::this_thread::sleep_for(3s);
    } catch (sdbus::Error &e) {
      std::cout << e.getMessage() << std::endl;
      std::cout << "Waiting 5 seconds ..." << std::endl;
      std::this_thread::sleep_for(5s);
    }
  }

  bool checkExistingConnection(std::shared_ptr<sdbus::IProxy> device_proxy, const std::string &target_ssid)
  {
    sdbus::ObjectPath active_conn_path = PrimaryConnection();
    CheckConnectivity();
    if(State() == NM_STATE_CONNECTED_GLOBAL)
    {//! instantiate proxies
      std::cout << "Confirming SSID" << std::endl;
      conn_proxy_ = sdbus::createProxy(service_name_, active_conn_path);
      std::string cur_ssid = conn_proxy_->getProperty("Id")
          .onInterface("org.freedesktop.NetworkManager.Connection.Active");
      if(cur_ssid == target_ssid)
      {
        sdbus::ObjectPath settings_path = conn_proxy_->getProperty("Connection")
            .onInterface("org.freedesktop.NetworkManager.Connection.Active");
        settings_proxy_ = sdbus::createProxy(service_name_, settings_path);
        return true;
      }
    }

    std::cout << "Checking for existing connection ..." << std::endl;
    //! loop through all active connections, compare Ssid and connect to if exists
    std::vector<sdbus::ObjectPath> active_conns = ActiveConnections();
    bool found = false;
    for (std::vector<sdbus::ObjectPath>::const_iterator c_iter = active_conns.cbegin();
         c_iter != active_conns.cend(); ++c_iter)
    {
      std::unique_ptr<sdbus::IProxy> tmp_conn_proxy = sdbus::createProxy(service_name_, *c_iter);
      std::string cur_ssid = tmp_conn_proxy->getProperty("Id")
          .onInterface("org.freedesktop.NetworkManager.Connection.Active");
      if(cur_ssid == target_ssid)
      {//!make connection
        std::cout << "Existing connection found! Connecting ..." << std::endl;
        sdbus::ObjectPath settings_path = tmp_conn_proxy->getProperty("Connection")
            .onInterface("org.freedesktop.NetworkManager.Connection.Active");
        sdbus::ObjectPath ap_obj_path = device_getAccessPoint(target_ssid);
        ActivateConnection(settings_path, device_path_, ap_obj_path);

        conn_proxy_ = sdbus::createProxy(service_name_, *c_iter);
        settings_proxy_ = sdbus::createProxy(service_name_, settings_path);
        found = true;
        break;
      }
    }
    return found;
  }

  std::string getIPAddress()
  {
    std::string ipAddress="No IP";
    struct ifaddrs *interfaces = nullptr;
    struct ifaddrs *temp_addr = nullptr;
    int success = 0;
    // retrieve the current interfaces - returns 0 on success
    success = getifaddrs(&interfaces);
    if (success == 0)
    {
      // Loop through linked list of interfaces
      temp_addr = interfaces;
      while(temp_addr != nullptr) {
        if(temp_addr->ifa_addr->sa_family == AF_INET) {
          // Check if interface is en0 which is the wifi connection on the iPhone
          if((strcmp(temp_addr->ifa_name, "en0") == 0) || (strcmp(temp_addr->ifa_name, "wlp0s20f3") == 0)){
            ipAddress=inet_ntoa(((struct sockaddr_in*)temp_addr->ifa_addr)->sin_addr);
          }
        }
        temp_addr = temp_addr->ifa_next;
      }
    }
    // Free memory
    freeifaddrs(interfaces);
    return ipAddress;
  }

private:
  std::unique_ptr<sdbus::IProxy> ap_proxy_, conn_proxy_, settings_proxy_;
  std::shared_ptr<sdbus::IProxy> device_proxy_;
  sdbus::ObjectPath device_path_;
  std::string service_name_;
};

#endif // NETWORK_ACCESS_H
