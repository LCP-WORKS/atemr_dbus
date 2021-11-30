## RUNNING SDBUS SERVER ON THE SYSTEM
To do this, the create a conf file for the service you want to register and
allow root the necessary permissions 

Example service net.poettering.Calculator
<!DOCTYPE busconfig PUBLIC
 "-//freedesktop//DTD D-BUS Bus Configuration 1.0//EN"
 "http://www.freedesktop.org/standards/dbus/1.0/busconfig.dtd">
<busconfig>
  <policy user="root">
    <allow own="net.poettering.Calculator"/>
  </policy>
</busconfig>
The file should be placed in:
/etc/dbus-1/system.d/net.poettering.Calculator.conf

## CONF FILE FOR THIS PROJECT
<!DOCTYPE busconfig PUBLIC
 "-//freedesktop//DTD D-BUS Bus Configuration 1.0//EN"
 "http://www.freedesktop.org/standards/dbus/1.0/busconfig.dtd">
<busconfig>
  <policy user="ephson">
    <allow own="org.sdbuscpp.concatenator"/>
    <allow send_destination="org.sdbuscpp.concatenator"/>
    <allow send_interface="org.sdbuscpp.Concatenator" send_member="concatenate"/>
  </policy>
</busconfig>

The file should named as follows and be placed at:
/etc/dbus-1/system.d/org.sdbuscpp.concatenator.conf


##DBUS UNABLE TO START ??
Set in .bashrc directory
export XDG_RUNTIME_DIR=/run/user/$(id -u)


## NETWORKMANAGER CONFIGURATION
.AddAndActivateConnection <- a{sa{sv}}oo -> oo

{'802-11-wireless': {'ssid': [87, 76, 65, 78, 45, 83, 77, 75, 54, 54, 66],
                     'mode': 'infrastructure',
                     'security': '802-11-wireless-security'},
 '802-11-wireless-security': {'auth-alg': 'open', 
                              'key-mgmt': 'wpa-psk'},
 '802-1x': {'password-raw': [21, 32, 54, 56, 45, 56]},
 'connection': {'id': 'WLAN-SMK66B',
                'type': '802-11-wireless',
                'autoconnect': 'TRUE'}}

  - NOTE: SSID & PASSWORD uses DEC - ASCII conversions
  E.g SSID: LCPSync -> [76, 67, 80, 83, 121, 110, 99]
      PASSWORD: geniusWIP6 -> [103, 101, 110, 105, 117, 115, 87, 73, 80, 54]
  Generate the proxy header file with: "sdbus-c++-xml2cpp dbus/NetworkManagerClient.xml --proxy=nm-client-glue.h"
  
#D-FEET GUI INTERFACE

{"802-11-wireless": {"ssid": GLib.Variant("ay", b'LCPSync'),
                     'mode': GLib.Variant("s", 'infrastructure'),
                     'security': GLib.Variant("s", '802-11-wireless-security')},
 '802-11-wireless-security': {'auth-alg': GLib.Variant("s", 'open'), 
                              'key-mgmt': GLib.Variant("s", 'wpa-psk')},
 "802-1x": {"eap": GLib.Variant("as", ['md5']),
            "identity": GLib.Variant("s", 'ephson'),
            "password-raw": GLib.Variant("ay", b'103, 101, 110, 105, 117, 115, 87, 73, 80, 54')},
 "connection": {"id": GLib.Variant("s", "LCPSync"),
                "type": GLib.Variant("s", "802-11-wireless")}
}, 
__import__('dbus').ObjectPath("/org/freedesktop/NetworkManager/Devices/3"), 
__import__('dbus').ObjectPath("/")

interface-name: wlp0s20f3
new-connection:
'/org/freedesktop/NetworkManager/Settings/32',
'/org/freedesktop/NetworkManager/ActiveConnection/24'

## GENERATE THE GLUE USING
sdbus-c++-xml2cpp dbus/NetworkManagerClient.xml --proxy=nm-client-glue.h
sdbus-c++-xml2cpp dbus/Login1.xml --proxy=login1-glue.h



















