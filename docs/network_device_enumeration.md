# Network device enumeration

Currently, the network device enumeration function is supported only by the Femto Mega device. When accessing this
device over the network, if `enumerate_net_device` is set to `true`, the device will be automatically enumerated,
eliminating the need to configure the IP address in advance or set the enable switch to true. The specific configuration
methods are as follows:

- `enumerate_net_device`: enumeration network device automatically, only supported by Femto Mega.
  if `enumerate_net_device` set to `true`, the device will be enumerated automatically,No need to set
  the `net_device_ip`
  and `net_device_port` parameters.
- `net_device_ip`: The IP address of the device.
- `net_device_port`: The port number of the device.
