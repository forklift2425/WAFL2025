#!/bin/bash
INTERFACE=$1
BITRATE=$2
sudo ip link set $INTERFACE type can bitrate $BITRATE
sudo ip link set $INTERFACE up

