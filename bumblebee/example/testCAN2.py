import can

bustype_define = 'slcan'
port = 'can0'

def send_can_message(channel, msg_id, data):
    """
    Send a CAN message.
    
    Parameters:
    - channel: The channel on which to send the message.
    - msg_id: The ID of the CAN message.
    - data: A list of bytes (up to 8) to send in the message.
    """
    #bus = can.interface.Bus(channel=channel, bustype='socketcan')
    #slcan
    bus = can.interface.Bus(channel=channel, bustype=bustype_define)
    message = can.Message(arbitration_id=msg_id, data=data, extended_id=False)
    bus.send(message)
    print(f"Message sent on {channel}: {message}")

def receive_can_message(channel):
    """
    Receive a CAN message.
    
    Parameters:
    - channel: The channel on which to listen for messages.
    """
    bus = can.interface.Bus(channel=channel, bustype=bustype_define)
    message = bus.recv(10.0)  # timeout after 10 seconds
    
    if message:
        print(f"Message received on {channel}: {message}")
    else:
        print(f"No message received on {channel}")

if __name__ == "__main__":
    CAN_CHANNEL = port  # You may need to adjust this depending on your setup
    send_can_message(CAN_CHANNEL, 0x123, [0x18, 0x06, 0xE5 , 0xF4])
    receive_can_message(CAN_CHANNEL)
