#!/bin/bash
# Place this file in /usr/local/bin/
# Run with sudo

# Exit immediately if a command exits with a non-zero status
set -e

# Function to prompt the user for confirmation
confirm() {
    while true; do
        read -rp "$1 [y/n]: " yn
        case $yn in
            [Yy]* ) return 0;;  # Proceed
            [Nn]* ) return 1;;  # Do not proceed
            * ) echo "Please answer yes (y) or no (n).";;
        esac
    done
}

# Function to retrieve the current hostname
get_hostname() {
    hostname
}

# Function to check if saved Wi-Fi connections exist
check_saved_connections() {
    connections=()
    while IFS=: read -r name type; do
        if [[ "$type" == *"wireless"* ]]; then
            connections+=("$name")
        fi
    done < <(nmcli -t -f NAME,TYPE connection show)
    if [ ${#connections[@]} -eq 0 ]; then
        return 1
    else
        return 0
    fi
}

# Function to list existing nmcli connections
list_nmcli_connections() {
    connections=()
    while IFS=: read -r name type; do
        if [[ "$type" == *"wireless"* ]]; then
            connections+=("$name")
        fi
    done < <(nmcli -t -f NAME,TYPE connection show)

    if [ ${#connections[@]} -eq 0 ]; then
        echo "No saved Wi-Fi connections to select from."
        return 1
    else
        echo "Available saved Wi-Fi connections:"
        for i in "${!connections[@]}"; do
            printf "%d) %s\n" "$((i + 1))" "${connections[$i]}"
        done
        TOTAL_CONNECTIONS=${#connections[@]}
        return 0
    fi
}

# Function to prompt user to select an existing nmcli connection
select_existing_connection() {
    if list_nmcli_connections; then
        echo "Enter the number corresponding to the connection you want to use:"
        while true; do
            read -rp "Selection: " selection
            if [[ ! "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -gt "$TOTAL_CONNECTIONS" ]]; then
                echo "Invalid selection. Please enter a number between 1 and $TOTAL_CONNECTIONS."
            else
                SELECTED_NETWORK="${connections[$((selection - 1))]}"
                CONNECTION_TYPE="saved"
                return 0
            fi
        done
    else
        echo "No saved Wi-Fi connections to select from."
        return 1
    fi
}

# Function to collect new connection details
create_new_connection() {
    read -rp "Enter the SSID of the Wi-Fi network you want to connect to: " NEW_SSID
    if [ -z "$NEW_SSID" ]; then
        echo "SSID cannot be empty. Please try again."
        create_new_connection
        return
    fi
    read -rsp "Enter the password for '$NEW_SSID': " NEW_PASSWORD
    echo
    CONNECTION_TYPE="new"
}

# Function to display the network selection menu
network_selection_menu() {
    while true; do
        echo "Please choose the Wi-Fi network you want to connect to:"
        options=()
        option_number=1

        if check_saved_connections; then
            echo "$option_number) Connect to a saved Wi-Fi network"
            options+=("saved")
            option_number=$((option_number + 1))
        fi

        echo "$option_number) Connect to a new Wi-Fi network by entering SSID and password"
        options+=("new")
        option_number=$((option_number + 1))

        echo "$option_number) Exit"
        options+=("exit")

        echo "Press Enter to select option 1."
        read -rp "Enter your choice [1-${#options[@]}]: " choice
        choice=${choice:-1}

        if [[ ! "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#options[@]}" ]]; then
            echo "Invalid choice. Please enter a number between 1 and ${#options[@]}."
            continue
        fi

        selected_option="${options[$((choice - 1))]}"

        case "$selected_option" in
            saved)
                if select_existing_connection; then
                    break
                else
                    echo "Failed to select a saved Wi-Fi connection. Please choose another option."
                fi
                ;;
            new)
                create_new_connection
                SELECTED_NETWORK="$NEW_SSID"
                break
                ;;
            exit)
                echo "Exiting the script."
                exit 0
                ;;
            *)
                echo "Invalid selection. Please try again."
                ;;
        esac
    done
}

# Main script execution starts here

echo "Starting Wi-Fi Network Switch Script..."

# Initial confirmation
if confirm "Do you want to proceed with switching the Wi-Fi network?"; then
    CURRENT_HOSTNAME=$(get_hostname)
    echo "Current hostname: '$CURRENT_HOSTNAME'"
    # Check if hostname is 'bumpybot' or 'bumpybot-upboard'
    if [[ "$CURRENT_HOSTNAME" == "bumpybot" || "$CURRENT_HOSTNAME" == "bumpybot-upboard" ]]; then
        echo "Hostname is valid. Proceeding..."
    else
        echo "Warning: This script is intended for 'bumpybot' or 'bumpybot-upboard'."
        echo "Proceed only if you know what you are doing, as this can break network settings."
        # Second confirmation
        if confirm "Do you still want to proceed?"; then
            echo "Proceeding despite hostname mismatch..."
        else
            echo "Operation cancelled by the user."
            exit 1
        fi
    fi
else
    echo "Operation cancelled by the user."
    exit 1
fi

# Network selection
network_selection_menu

echo "You have chosen to connect to '$SELECTED_NETWORK'."

# Begin main functionality
echo "Switching from the 'bumpybot' AP to the '$SELECTED_NETWORK' Wi-Fi network."
echo "This may take a few moments..."

# Disable AP mode services
echo "Disabling AP mode services..."
sudo systemctl stop hostapd || echo "hostapd service not running or not found."

# Bring down AP connection
echo "Bringing down 'bumpybot-AP' connection..."
sudo nmcli conn down "bumpybot-AP" || echo "'bumpybot-AP' connection not active or not found."

# Attempt to connect based on CONNECTION_TYPE
case "$CONNECTION_TYPE" in
    saved)
        echo "Connecting to saved Wi-Fi network '$SELECTED_NETWORK'..."
        if sudo nmcli conn up "$SELECTED_NETWORK"; then
            echo "Successfully connected to '$SELECTED_NETWORK'."
        else
            echo "Failed to connect to '$SELECTED_NETWORK' Wi-Fi network. Restoring 'bumpybot-AP'."
            sudo nmcli conn up "bumpybot-AP" || echo "Failed to restore 'bumpybot-AP' connection."
            exit 1
        fi
        ;;
    new)
        echo "Attempting to create and connect to '$NEW_SSID'..."
        if sudo nmcli dev wifi connect "$NEW_SSID" password "$NEW_PASSWORD" hidden yes; then
            echo "Successfully connected to '$NEW_SSID'."
        else
            echo "Failed to create and connect to the Wi-Fi network '$NEW_SSID'. Restoring 'bumpybot-AP'."
            sudo nmcli conn up "bumpybot-AP" || echo "Failed to restore 'bumpybot-AP' connection."
            exit 1
        fi
        ;;
    *)
        echo "Unknown connection type. Exiting."
        exit 1
        ;;
esac

# Wait for connection
echo "Waiting for the connection to establish..."
sleep 7

# Verify connection
if nmcli -t -f active,ssid dev wifi | grep -q "^yes:$SELECTED_NETWORK$"; then
    echo "Connection to '$SELECTED_NETWORK' confirmed."
else
    echo "Connection to '$SELECTED_NETWORK' not confirmed. Restoring 'bumpybot-AP'."
    sudo nmcli conn up "bumpybot-AP" || echo "Failed to restore 'bumpybot-AP' connection."
    exit 1
fi

# Ensure IP forwarding, iptables, and routing are correctly set up

echo "Setting up IP forwarding and NAT..."
# Set up IP forwarding and NAT
sudo sysctl -w net.ipv4.ip_forward=1

echo "Configuring iptables rules..."
# Adjust interface names as per your system
WIFI_INTERFACE=$(nmcli device status | grep -i wifi | awk '{print $1}')
LAN_INTERFACE=$(nmcli device status | grep -E "ethernet|wired" | awk '{print $1}')

# Set up iptables rules
sudo iptables -F
sudo iptables -t nat -F
sudo iptables -A FORWARD -i "$WIFI_INTERFACE" -o "$LAN_INTERFACE" -j ACCEPT
sudo iptables -A FORWARD -i "$LAN_INTERFACE" -o "$WIFI_INTERFACE" -j ACCEPT
sudo iptables -t nat -A POSTROUTING -o "$WIFI_INTERFACE" -j MASQUERADE

# Save iptables rules
echo "Saving iptables rules..."
sudo iptables-save | sudo tee /etc/iptables/rules.v4 >/dev/null

echo "Removing incorrect routes if any..."
# Remove any incorrect routes
sudo ip route del default via 192.168.50.1 dev "$LAN_INTERFACE" 2>/dev/null || true
sudo ip route del 192.168.50.1 dev "$LAN_INTERFACE" 2>/dev/null || true

echo "Wi-Fi network switch completed successfully."

