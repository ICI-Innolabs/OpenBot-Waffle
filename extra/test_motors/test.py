from dynamixel_sdk import *

# Set port and baud rate (update if necessary)
PORT_NAME = "/dev/ttyUSB0"
BAUD_RATE = 57600  # Use your actual baud rate

# Create a PortHandler instance
portHandler = PortHandler(PORT_NAME)

# Open the port
if portHandler.openPort():
    print(f"Successfully opened port: {PORT_NAME}")
else:
    print(f"Failed to open port: {PORT_NAME}")
    exit()

# Set baud rate
if portHandler.setBaudRate(BAUD_RATE):
    print(f"Baud rate set to: {BAUD_RATE}")
else:
    print("Failed to set baud rate")
    exit()

# Create a PacketHandler instance
packetHandler = PacketHandler(2.0)  # Use protocol 2.0 for XL430

# Scan for IDs (1 to 5)
for dxl_id in range(1, 6):
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)

    if dxl_comm_result == COMM_SUCCESS:
        print(f"Found Dynamixel motor with ID: {dxl_id}")
    else:
        print(f"No motor found at ID {dxl_id}")

# Close the port
portHandler.closePort()
