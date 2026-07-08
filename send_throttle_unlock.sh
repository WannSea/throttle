#!/bin/sh
set -eu

IFACE="${1:-can0}"

cansend "$IFACE" 00000F55#A5
