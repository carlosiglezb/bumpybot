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
sudo iptables-save | sudo tee /etc/iptables/rules.v4 >/dev/null

# Removing incorrect routes logic from Script 1
echo "> Getting the correct gateway IP for the Wi-Fi interface ($WIFI_INTERFACE)"
GATEWAY_IP=$(ip route show default dev "$WIFI_INTERFACE" | awk '{print $3}')
echo "Detected Wi-Fi gateway IP: $GATEWAY_IP"

# Validate that the necessary info is found
if [ -z "$WIFI_INTERFACE" ] || [ -z "$GATEWAY_IP" ]; then
    echo "Error: Wi-Fi interface or gateway IP not found. Exiting..."
    exit 1
fi

echo "> Looking for and deleting all default routes except 'default via $GATEWAY_IP dev $WIFI_INTERFACE'..."

# Get all current default routes across all interfaces
DEFAULT_ROUTES=$(ip route show default)

# Iterate through each default route and check if it should be removed
echo "$DEFAULT_ROUTES" | while read -r route; do
    # Extract the route's gateway and interface
    CURRENT_GATEWAY=$(echo "$route" | awk '{print $3}')
    CURRENT_INTERFACE=$(echo "$route" | awk '{print $5}')
    
    # Check if this route matches the correct Wi-Fi route
    if [[ "$CURRENT_GATEWAY" == "$GATEWAY_IP" && "$CURRENT_INTERFACE" == "$WIFI_INTERFACE" ]]; then
        echo "> Preserving route: $route"
    else
        echo "> Deleting route: $route"
        sudo ip route del $route 2>/dev/null || true
    fi
done

# Restore specific route handling for AP mode (if necessary)
echo "Removing incorrect routes with gateway 10.42.99.99 (specific to AP mode)..."
sudo ip route del default via 10.42.99.99 dev "$LAN_INTERFACE" 2>/dev/null || true
sudo ip route del 10.42.99.99 dev "$LAN_INTERFACE" 2>/dev/null || true

echo "AP mode setup completed successfully."
