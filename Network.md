# BumpyBot Networking Scheme

## Networks
#### `192.168.50.0/24`
- Wi-Fi Network for HCRL's router, SSID: `hcrlab2`
- All *wireless* devices should be on this network and direct their default traffic through it (unless Bumpybot is used in AP mode).
- Gateway IP is `192.168.50.1`

#### `10.42.0.0/24`
- Ethernet connection between Bumpybot-Upboard and Bumpybot-Orin
- There is no router or switch within this network, just a direct connection between the two onboard computers

#### `10.42.99.0/24`
- Wi-Fi Hotspot Network, only used when Bumpybot-Upboard is in AP mode
- SSID: `bumpybot`, pass: `hcrlRobot$`
- No Internet to Bumpybot in this mode unless you decide to route internet traffic through a remote computer (not currently done as of writing)


## Testing 

On the remote desktop computer (Franck's desktop) with bumpybot in normal Wifi mode and both connected to `hcrlab2`, you should be able to do the following:
 -  ping/ssh to bumpybot-upboard on `192.168.50.35` ***AND*** on `10.42.0.1`
 -  ping/ssh to bumpybot-**orin** on `10.42.0.71` ***DIRECTLY*** (ie, not by first ssh'ing to bumpybot-upboard)

This ensures that as long as your ROS_MASTER_URI is setup correctly on all computers, ROS will be able to make *"direct"* TCP connects between the computers.

## Bumpybot-Upboard
#### Network Devices 
- `eno1`: Ethernet port used to connect to Bumpybot-Orin.
    - IP: `10.42.0.1` on the `10.42.0.0/24` network.
- `enp1s0`: Ethernet port used to connect to the onboard EtherCAT router, connects to the Everest Servo Drivers.
    - IP: `192.168.2.1` on the `192.168.2.0/24` network.
- `wlx00e04c4b307f`: Wi-Fi Adapter (USB) used to either connect to a shared wireless network (like `hcrlab2`) or project a wireless access point (AP).
    - IP: `192.168.50.35` on the `192.168.50.0/24` `hcrlab2` Wi-Fi network.

#### IP Routing

For `bumpybot-upboard`, the routing configuration ensures that traffic between the Wi-Fi network (`192.168.50.0/24`) and Ethernet (`10.42.0.0/24`) is correctly forwarded. Here's the routing setup:

```bash
# Default route through Wi-Fi (hcrlab2 network)
default via 192.168.50.1 dev wlx00e04c4b307f
192.168.50.1 dev eno1 proto static scope link 

# Route to the Ethernet network (direct connection between Bumpybot-Upboard and Bumpybot-Orin)
10.42.0.0/24 dev eno1 proto kernel scope link src 10.42.0.1

# Local Wi-Fi traffic on the hcrlab2 network
192.168.50.0/24 dev wlx00e04c4b307f proto kernel scope link src 192.168.50.35
```


You may sometimes see these entries:
```bash 
default via 192.168.50.1 dev eno1 # or similiar
192.168.50.1 dev eno1 proto static scope link # or similiar
```
Be sure to *remove* that entry as it will block connection to the internet on Bumpybot-Upboard!
```bash
sudo ip route del default via 192.168.50.1 dev eno1 # or similiar, be sure to paste in entire entry. 

sudo ip route del 192.168.50.1 dev eno1 proto static scope link # or similiar
```


#### IP Table Settings

To forward traffic between the Wi-Fi and Ethernet networks, the following commands are needed:



```bash
# Enable IP forwarding
sudo sysctl -w net.ipv4.ip_forward=1

# Allow forwarding from Wi-Fi to Ethernet
sudo iptables -A FORWARD -i wlx00e04c4b307f -o eno1 -j ACCEPT

# Allow forwarding from Ethernet to Wi-Fi
sudo iptables -A FORWARD -i eno1 -o wlx00e04c4b307f -j ACCEPT

# Remove reject rules if they exist (to prevent blocking)
sudo iptables -D FORWARD -o eno1 -j REJECT --reject-with icmp-port-unreachable
sudo iptables -D FORWARD -i eno1 -j REJECT --reject-with icmp-port-unreachable
```

Usually running these commands will correctly set the `iptables` rules.

To make these rules persistent across reboots, save them using:

```bash
sudo iptables-save > /etc/iptables/rules.v4
```

## Bumpybot-Orin
#### Network Devices
- `eth0`: Ethernet interface connecting to Bumpybot-Upboard via the `10.42.0.0/24` network.
    - IP: `10.42.0.71` (assigned via DHCP from Bumpybot-Upboard)
    - Gateway: `10.42.0.1` (Bumpybot-Upboard)

#### IP Routing

```bash
# Default route through Bumpybot-Upboard (gateway)
default via 10.42.0.1 dev eth0 proto dhcp src 10.42.0.71 

# Direct route for local Ethernet traffic
10.42.0.0/24 dev eth0 proto kernel scope link src 10.42.0.71
10.42.0.1 dev eth0 proto dhcp scope link src 10.42.0.71

# Routing for reaching wireless devices on hcrlab2 through Bumpybot-Upboard
192.168.50.0/24 via 10.42.0.1 dev eth0 proto static onlink 
```

No special firewall or NAT rules are required on Bumpybot-Orin since Bumpybot-Upboard handles the forwarding and NAT for internet access.


## Ground Station / Remote Computer (Franck-Desktop)
#### Network Devices
- `wlx1cbfceef0aaa`: Wi-Fi interface connected to the `hcrlab2` network.
    - IP: `192.168.50.172`
    - Gateway: `192.168.50.1`
- `eno1`: Ethernet Port, connects to the port on the wall. `10.149.11.137` on the `10.149.11.0/24` network, used mainly for internet. If you've lost internet connection on the ground station computer after changing settings, ensure this connection has defauly traffic routed to it. 

#### IP Routing

```bash
# Default Routing for internet traffic through ethernet connection 
default via 10.149.11.1 dev eno1 proto dhcp metric 100
10.149.11.0/24 dev eno1 proto kernel scope link src 10.149.11.137 metric 100 

#  Routing for reaching wireless devices on hcrlab2 (192.168.50.0/24) 
192.168.50.0/24 dev wlx1cbfceef0aaa proto kernel scope link src 192.168.50.172 metric 600 

# Route for local Wi-Fi network traffic
192.168.50.0/24 dev wlx1cbfceef0aaa proto kernel scope link src 192.168.50.172

# Route to reach the 10.42.0.0/24 network via Bumpybot-Upboard routing.
10.42.0.0/24 via 192.168.50.35 dev wlx1cbfceef0aaa
# This should allow `ping 10.42.0.1` from remote computer
```

This setup ensures that the Ground Station can communicate with both Bumpybot-Upboard and Bumpybot-Orin, with internet access routed through the `192.168.50.0/24` network.
