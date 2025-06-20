import time
import macro
import scservo_sdk as scs

def convert_to_bytes(value, bytes):
    if bytes == 1:
        data = [
            scs.SCS_LOBYTE(scs.SCS_LOWORD(value)),
        ]
    elif bytes == 2:
        data = [
            scs.SCS_LOBYTE(scs.SCS_LOWORD(value)),
            scs.SCS_HIBYTE(scs.SCS_LOWORD(value)),
        ]
    elif bytes == 4:
        data = [
            scs.SCS_LOBYTE(scs.SCS_LOWORD(value)),
            scs.SCS_HIBYTE(scs.SCS_LOWORD(value)),
            scs.SCS_LOBYTE(scs.SCS_HIWORD(value)),
            scs.SCS_HIBYTE(scs.SCS_HIWORD(value)),
        ]
    else:
        raise NotImplementedError(
            f"Value of the number of bytes to be sent is expected to be in [1, 2, 4], but "
            f"{bytes} is provided instead."
        )
    return data

class FeetechMotor:
    def __init__(self, motor_id, port="/dev/ttyACM0"):
        self.motor_id = motor_id
        self.port = port
        
    def connect(self):
        self.port_handler = scs.PortHandler(self.port)
        self.packet_handler = scs.PacketHandler(macro.PROTOCOL_VERSION)
        try:
            if not self.port_handler.openPort():
                raise OSError(f"Failed to open port '{self.port}'.")
        except Exception:
            print("choose right port!")
            raise

        self.is_connected = True
        self.port_handler.setPacketTimeoutMillis(macro.TIMEOUT_MS)
    
    def disconnect(self):
        if self.port_handler is not None:
            self.port_handler.closePort()
            self.port_handler = None

        self.packet_handler = None
        self.is_connected = False
    
    def set_bus_baudrate(self, baudrate):
        present_bus_baudrate = self.port_handler.getBaudRate()
        if present_bus_baudrate != baudrate:
            print(f"Setting bus baud rate to {baudrate}. Previously {present_bus_baudrate}.")
            self.port_handler.setBaudRate(baudrate)

            if self.port_handler.getBaudRate() != baudrate:
                raise OSError("Failed to write bus baud rate.")

    def read_with_motor_ids(self, motor_ids, data_name, num_retry=macro.NUM_READ_RETRY):
        return_list = True
        if not isinstance(motor_ids, list):
            return_list = False
            motor_ids = [motor_ids]

        addr, bytes = macro.SCS_SERIES_CONTROL_TABLE[data_name]
        group = scs.GroupSyncRead(self.port_handler, self.packet_handler, addr, bytes)
        for idx in motor_ids:
            group.addParam(idx)

        for _ in range(num_retry):
            comm = group.txRxPacket()
            if comm == scs.COMM_SUCCESS:
                break

        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(
                f"Read failed due to communication error on port {self.port_handler.port_name} for indices {motor_ids}: "
                f"{self.packet_handler.getTxRxResult(comm)}"
            )

        values = []
        for idx in motor_ids:
            value = group.getData(idx, addr, bytes)
            values.append(value)

        if return_list:
            return values
        else:
            return values[0]
    
    def write_with_motor_ids(self, motor_ids, data_name, values, num_retry=macro.NUM_WRITE_RETRY):
        if not isinstance(motor_ids, list):
            motor_ids = [motor_ids]
        if not isinstance(values, list):
            values = [values]

        addr, bytes = macro.SCS_SERIES_CONTROL_TABLE[data_name]
        group = scs.GroupSyncWrite(self.port_handler, self.packet_handler, addr, bytes)
        for idx, value in zip(motor_ids, values, strict=True):
            data = convert_to_bytes(value, bytes)
            group.addParam(idx, data)

        for _ in range(num_retry):
            comm = group.txPacket()
            if comm == scs.COMM_SUCCESS:
                break

        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(
                f"Write failed due to communication error on port {self.port_handler.port_name} for indices {motor_ids}: "
                f"{self.packet_handler.getTxRxResult(comm)}"
            )

    def setPosition(self, position):
        self.write_with_motor_ids(self.motor_id, "Goal_Position", position)

    def getPosition(self):
        return self.read_with_motor_ids(self.motor_id, "Present_Position")

    def get_multiple_positions(self, motor_ids):
        """
        通过快速连续的单次读取来获取多个舵机的位置。
        这种方法比 GroupSyncRead 更稳定，但不是严格意义上的同步。
        
        参数:
            motor_ids (list): 一个包含舵机ID的列表，例如 [1, 2]。
            
        返回:
            一个包含对应位置的列表，如果某个舵机读取失败，对应位置可能为None。
        """
        positions = []
        for motor_id in motor_ids:
            try:
                pos = self.read_with_motor_ids(motor_id, "Present_Position")
                positions.append(pos)
            except ConnectionError as e:
                print(f"Warning: Read motor ID {motor_id} failed: {e}")
                positions.append(None) # 读取失败，用 None 占位
        return positions

# ==================== main program ====================
if __name__ == "__main__":
    motor_communicator = None
    try:
        motor_communicator = FeetechMotor(1, "/dev/ttyACM0")
        motor_communicator.connect()
        motor_ids_to_read = [1, 2]  # Modify Motor ID list

        while True:
            positions = motor_communicator.get_multiple_positions(motor_ids_to_read)
            
            if positions and None not in positions:
                print(f"Motor ID {motor_ids_to_read[0]} position: {positions[0]}, Motor ID {motor_ids_to_read[1]} position: {positions[1]}")
            else:
                print(f"Part of positions are None, received data: {positions}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Error occurred while running the program: {e}")
    finally:
        if motor_communicator and motor_communicator.is_connected:
            motor_communicator.disconnect()