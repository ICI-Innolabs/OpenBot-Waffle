from dynamixel_sdk import *

# Port and motor settings
PORT_NAME = "/dev/ttyUSB0"
BAUD_RATE = 57600
DXL_ID_1 = 1
DXL_ID_2 = 2
ADDR_OPERATING_MODE = 11  # Address for operating mode (Protocol 2.0)
PROTOCOL_VERSION = 2.0

portHandler = PortHandler(PORT_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

portHandler.openPort()
portHandler.setBaudRate(BAUD_RATE)

for DXL_ID in [DXL_ID_1, DXL_ID_2]:
    dxl_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE)
    print(f"Motor {DXL_ID} operating mode: {dxl_mode}")

portHandler.closePort()
