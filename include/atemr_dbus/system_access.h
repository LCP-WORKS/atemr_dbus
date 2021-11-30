#ifndef SYSTEM_ACCESS_H
#define SYSTEM_ACCESS_H

#include <atemr_dbus/login1-glue.h>

class LoginProxy: public sdbus::ProxyInterfaces<org::freedesktop::login1::Manager_proxy>
{
public:
  LoginProxy(std::string destination, std::string objectPath):
    ProxyInterfaces(destination, std::move(objectPath))
  {
    service_name_ = destination;
  }

  ~LoginProxy()
  {
    unregisterProxy();
  }

private:
  std::string service_name_;
};

#endif // SYSTEM_ACCESS_H
