import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

from dynamixel_sdk import * 
# Define constants
MY_DXL = 'X_SERIES'
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0
DXL1_ID                     = 1 
DXL2_ID                     = 2  
DEVICENAME                  = '/dev/ttyACM0'
TORQUE_ENABLE               = 1 
TORQUE_DISABLE              = 0 
DXL_MOVING_STATUS_THRESHOLD = 20 

# Initialize port handler and packet handler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
# Initialize GroupSyncRead instance for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

def initialize_dynamixels():
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable Dynamixel#1 Torque
    enable_dynamixel_torque(DXL1_ID)
    # Move Dynamixel#1 to position 2000
    move_dynamixel_to_position(DXL1_ID, 2000)
    # Enable Dynamixel#2 Torque
    enable_dynamixel_torque(DXL2_ID)
    # Move Dynamixel#2 to position 2000
    move_dynamixel_to_position(DXL2_ID, 2000)

def enable_dynamixel_torque(dxl_id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % dxl_id)

    # Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(dxl_id)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % dxl_id)
        quit()

def disable_dynamixel_torque(dxl_id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully disable" % dxl_id)

def move_dynamixel_to_position(dxl_id, position):
    # Add Dynamixel goal position value to the Syncwrite parameter storage
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(position)),
                           DXL_HIBYTE(DXL_LOWORD(position)),
                           DXL_LOBYTE(DXL_HIWORD(position)),
                           DXL_HIBYTE(DXL_HIWORD(position))]
    
    dxl_addparam_result = groupSyncWrite.addParam(dxl_id, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
        quit()
    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

def read_dynamixel_position(dxl_id):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    return dxl_present_position

def oper(id, index):
    dxl_present_position = read_dynamixel_position(id)

    DXL_MINIMUM_POSITION_VALUE = dxl_present_position - 20
    DXL_MAXIMUM_POSITION_VALUE = dxl_present_position + 20

    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]

    move_dynamixel_to_position(id, dxl_goal_position[index])

    while True:
        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel#id is available
        dxl_getdata_result = groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % id)
            quit()

        dxl_present_position = read_dynamixel_position(id)
        print("[ID:%03d] GoalVel:%03d  PresVel:%03d" % (id, dxl_goal_position[index], dxl_present_position))
        if not (abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
            break

class DynamixelNode(Node):
    def __init__(self):
        super().__init__('dynamixel_node')
        self.subscription = self.create_subscription(Int32MultiArray, 'object_center', self.listener_callback, 10)

    def listener_callback(self, msg):
        if len(msg.data) >= 2:
            global_center_x = msg.data[0]
            global_center_y = msg.data[1]
            print("Received data - x:", global_center_x, "y:", global_center_y)

            dx1 = dx2 = dy1 = dy2 = 0
            if global_center_x < 620:
                dx1 = 620 - global_center_x
            elif global_center_x > 660:
                dx2 = global_center_x - 660

            if global_center_y < 340:
                dy1 = 340 - global_center_y
            elif global_center_y > 380:
                dy2 = global_center_y - 380

            if (dx1 < dy1 and dx2 <= dy1) or (dx1 < dy2 and dx2 <= dy2):
                if global_center_y > 380:
                    oper(DXL1_ID, 1)  # y시계방향
                elif global_center_y < 340:
                    oper(DXL1_ID, 0)  # y반시계방향
            elif (dy1 < dx1 and dy2 <= dx1) or (dy1 < dx2 and dy2 <= dx2):
                if global_center_x > 660:
                    oper(DXL2_ID, 1)  # x시계방향
                elif global_center_x < 620:
                    oper(DXL2_ID, 0)  # x반시계방향


def main(args=None):
    rclpy.init(args=args)
    dynamixel_node = DynamixelNode()
    rclpy.spin(dynamixel_node)
    dynamixel_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    # Initialize Dynamixel motors
    initialize_dynamixels()

    # Run ROS 2 node
    main()

    # Clear syncread parameter storage
    groupSyncRead.clearParam()

    # Disable Dynamixel#1 Torque
    disable_dynamixel_torque(DXL1_ID)
    # Disable Dynamixel#2 Torque
    disable_dynamixel_torque(DXL2_ID)

    # Close port
    portHandler.closePort()