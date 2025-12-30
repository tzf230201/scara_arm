#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-vcan0}"

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "This script must be run as root (use: sudo $0 [iface])" >&2
  exit 1
fi

if ! command -v ip >/dev/null 2>&1; then
  echo "Missing 'ip' command. Install: sudo apt install -y iproute2" >&2
  exit 1
fi

# Load vcan module if available.
if modprobe vcan 2>/dev/null; then
  :
else
  echo "modprobe vcan failed. Kernel might not include vcan support." >&2
  echo "WSL fallback: use userspace fake CAN:" >&2
  echo "  ./setup_fake_can.sh" >&2
  exit 1
fi

# Create the interface if it doesn't exist.
if ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "${IFACE} already exists"
else
  ip link add dev "${IFACE}" type vcan
  echo "Created ${IFACE}"
fi

ip link set up "${IFACE}"

echo "${IFACE} is UP"
echo "Test (in another terminal): candump ${IFACE}"
echo "Send: cansend ${IFACE} 123#DEADBEEF"
