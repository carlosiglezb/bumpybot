#!/bin/bash
# Run this script with sudo privileges
# Place script in /usr/local/bin

# Switch to AP mode with SSID 'bumpybot'
echo "Switching to AP mode with SSID 'bumpybot'..."

# Bring up 'bumpybot-AP' connection
sudo nmcli conn up bumpybot-AP

# Enable IP forwarding
echo "Enabling IP forwarding..."
sudo sysctl -w net.ipv4.ip_forward=1

# Flush existing iptables rules
echo "Flushing existing iptables rules..."
sudo iptables -F
sudo iptables -t nat -F

# Configure iptables for NAT and forwarding
echo "Configuring iptables rules..."
# Adjust interface names as per your system
WIFI_INTERFACE=$(nmcli device status | grep -i wifi | awk '{print $1}')
LAN_INTERFACE=$(nmcli device status | grep -E "ethernet|wired" | awk '{print $1}')

sudo iptables -A FORWARD -i "$WIFI_INTERFACE" -o "$LAN_INTERFACE" -j ACCEPT
sudo iptables -A FORWARD -i "$LAN_INTERFACE" -o "$WIFI_INTERFACE" -j ACCEPT
sudo iptables -t nat -A POSTROUTING -o "$WIFI_INTERFACE" -j MASQUERADE

# Save iptables rules
echo "Saving iptables rules..."
sudo iptables-save | sudo tee /etc/iptables/rules.v4

# Delete incorrect routes with the 10.42.99.99 gateway
echo "Removing incorrect routes with gateway 10.42.99.99..."
sudo ip route del default via 10.42.99.99 dev "$LAN_INTERFACE" 2>/dev/null || true
sudo ip route del 10.42.99.99 dev "$LAN_INTERFACE" 2>/dev/null || true

echo "AP mode setup completed successfully."

