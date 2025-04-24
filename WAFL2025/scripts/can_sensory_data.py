#!/usr/bin/env python3

import can
import time

def send_dummy_encoder_data():
    """
    Sends dummy encoder data over the vcan0 interface.
    """
    # Initialize the CAN bus interface on vcan0
    try:
        bus = can.interface.Bus(channel='vcan0', interface='socketcan')
        print("Connected to vcan0.")
    except Exception as e:
        print(f"Failed to connect to vcan0: {e}")
        return

    data=[0x0A, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]
    msg = can.Message(arbitration_id=0x123,
                          data=data,
                          is_extended_id=False)



    # Send the message
    try:
        bus.send(msg)
        print(f"Sent message: {msg}")
    except can.CanError as e:
        print(f"Failed to send message: {e}")

if __name__ == '__main__':
    while True:
        send_dummy_encoder_data()
        time.sleep(1)  # Send data every second
