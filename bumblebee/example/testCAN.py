import can

def send_can_message(channel, msg_id, data):
    """
    Send a CAN message.
    
    Parameters:
    - channel: The channel on which to send the message (usually the interface name, e.g., 'can0').
    - msg_id: The ID of the CAN message.
    - data: A list of bytes (up to 8) to send in the message.
    """
    bus = can.interface.Bus(channel=channel, bustype='socketcan')
    message = can.Message(arbitration_id=msg_id, data=data, extended_id=False)
    bus.send(message)
    print(f"Message sent on {channel}: {message}")

def receive_can_message(channel):
    """
    Receive a CAN message.
    
    Parameters:
    - channel: The channel on which to listen for messages.
    """
    bus = can.interface.Bus(channel=channel, bustype='socketcan')
    message = bus.recv(10.0)  # timeout after 10 seconds
    
    if message:
        print(f"Message received on {channel}: {message}")
    else:
        print(f"No message received on {channel}")

if __name__ == "__main__":
    # Example usage:
    #CAN_CHANNEL = '/dev/ttyUSB0'  # Replace 'can0' with the appropriate channel for your setup
    CAN_CHANNEL = 'can0'
    #send_can_message(CAN_CHANNEL, 0x123, [0x18, 0x06, 0xE5, 0xF4, 0x55, 0x66, 0x77, 0x88])
    send_can_message(CAN_CHANNEL, 0x123, [0x18, 0x06, 0xE5, 0xF4, 0, 0, 0, 0])
    receive_can_message(CAN_CHANNEL)
