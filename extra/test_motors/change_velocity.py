from dynamixel_sdk import *

PORT_NAME = "/dev/ttyUSB0"
BAUD_RATE = 57600
DXL_ID_1 = 1
DXL_ID_2 = 2
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
PROTOCOL_VERSION = 2.0

portHandler = PortHandler(PORT_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

portHandler.openPort()
portHandler.setBaudRate(BAUD_RATE)

# Disable torque before changing mode
for DXL_ID in [DXL_ID_1, DXL_ID_2]:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)

# Set Velocity Mode
for DXL_ID in [DXL_ID_1, DXL_ID_2]:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 1)
    print(f"Motor {DXL_ID} set to Velocity Mode.")

# Enable torque
for DXL_ID in [DXL_ID_1, DXL_ID_2]:
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

portHandler.closePort()