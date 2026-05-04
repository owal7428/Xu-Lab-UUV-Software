#!/usr/bin/env bash
set -e

# ---- DHCP Server Setup ----

read -rp "Interface [default: eth0]: " IFACE
IFACE=${IFACE:-eth0}

IP="192.168.50.1/24"
RANGE_START="192.168.50.10"
RANGE_END="192.168.50.100"

echo "--- Using interface: $IFACE"

# Find connection for ethernet interface
CON=$(nmcli -t -f NAME,DEVICE connection show | awk -F: -v d="$IFACE" '$2==d{print $1; exit}')
if [ -z "$CON" ]; then
    echo "--- Connection not found for interface $IFACE"
    exit 1
fi

echo "--- Configuring static IP for $CON..."
nmcli connection modify "$CON" ipv4.addresses "$IP" ipv4.method manual
nmcli connection up "$CON"

# Assuming Ubuntu/Debian for dnsmasq installation
echo "--- Installing dnsmasq..."
apt-get install -y dnsmasq

CONF="/etc/dnsmasq.d/${IFACE}.conf"

echo "--- Writing $CONF..."
cat > "$CONF" <<EOF
interface=$IFACE
bind-dynamic
dhcp-range=$RANGE_START,$RANGE_END,255.255.255.0,24h
dhcp-option=3
dhcp-option=6
EOF

# Set dnsmasq to start on boot and start now
echo "--- Enabling dnsmasq..."
systemctl enable dnsmasq
systemctl restart dnsmasq

# ---- Python Virtual Environment Setup ----

# Get absolute script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
REQ_FILE="$SCRIPT_DIR/requirements.txt"

# Create venv if missing
if [ ! -d "$VENV_DIR" ]; then
    echo "--- Creating python virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

PIP="$VENV_DIR/bin/pip"

echo "--- Installing python dependencies..."
if ! "$PIP" install -r "$REQ_FILE"; then
    echo "--- Dependency installation failed; check internet connection."
    exit 1
fi

echo "--- Done."
