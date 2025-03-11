from dynamixel_sdk import *

# Port and motor settings
PORT_NAME = "/dev/ttyUSB0"
BAUD_RATE = 57600
DXL_ID_1 = 1  # First motor ID
DXL_ID_2 = 2  # Second motor ID
ADDR_TORQUE_ENABLE = 64  # Address for enabling torque
ADDR_GOAL_VELOCITY = 104  # Address for setting velocity
LEN_GOAL_VELOCITY = 4
PROTOCOL_VERSION = 2.0  # Dynamixel XL430 uses Protocol 2.0

# Initialize port handler and packet handler
portHandler = PortHandler(PORT_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
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

# Enable torque on both motors
for DXL_ID in [DXL_ID_1, DXL_ID_2]:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result == COMM_SUCCESS:
        print(f"Motor {DXL_ID}: Torque enabled")
    else:
        print(f"Failed to enable torque for motor {DXL_ID}")

# Set velocity (positive for forward, negative for reverse)
goal_velocity = 2000  # Change this value to increase/decrease speed

for DXL_ID in [DXL_ID_1, DXL_ID_2]:
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, goal_velocity)
    if dxl_comm_result == COMM_SUCCESS:
        print(f"Motor {DXL_ID} set to velocity {goal_velocity}")
    else:
        print(f"Failed to set velocity for motor {DXL_ID}")

# Keep running for 5 seconds, then stop motors
import time
time.sleep(5)

# Stop motors
for DXL_ID in [DXL_ID_1, DXL_ID_2]:
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, 0)
    print(f"Motor {DXL_ID} stopped.")

# Disable torque before exiting
for DXL_ID in [DXL_ID_1, DXL_ID_2]:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)

# Close port
portHandler.closePort()
