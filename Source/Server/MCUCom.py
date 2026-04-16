import spidev
from ConnectionTypes import *

class MCUComServer:
    def __init__(self, spi_bus=0, spi_device=0, max_speed_hz=10000):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = max_speed_hz
        self.spi.mode = 0

        self.out_packet = OutPacket()
        self.in_bytes = bytearray() # this buffer will accumulate incoming bytes until a full packet can be parsed

        self.out_packet.SoP[0] = 0xAA
        self.out_packet.SoP[1] = 0x55

    def set_servo(self, forward=0, pitch=0, roll=0):
        self.out_packet.ServoCmd.forward = max(-1, min(1, forward))  # clamp to [-1, 1]
        self.out_packet.ServoCmd.pitch = max(-1, min(1, pitch))  # clamp to [-1, 1]
        self.out_packet.ServoCmd.roll = max(-1, min(1, roll))  # clamp to [-1, 1]

    def set_thruster(self, thrust=0):
        self.out_packet.ThrusterCmd.thrust = max(0, min(1, thrust))  # clamp to [0, 1]

    def send_receive(self):
        size = ctypes.sizeof(self.out_packet)

        if size > PACKETSIZE:
            raise ValueError("OutPacket too large!")

        # Get checksum for outgoing packet
        self.out_packet.CheckSum = CalculateChecksum(
            bytearray(ctypes.string_at(ctypes.byref(self.out_packet), size - 1))
        )

        out_bytes = bytearray(PACKETSIZE)
        out_bytes[:size] = ctypes.string_at(ctypes.byref(self.out_packet), size)

        # Send packet and receive response
        self.in_bytes.extend(self.spi.xfer2(out_bytes))

        in_packet_len = ctypes.sizeof(InPacket)

        # Find and parse oldest available packet in buffer
        while len(self.in_bytes) >= PACKETSIZE:
            # Look for start of packet
            index = self.in_bytes.find(b'\xAA\x55')

            # If start of packet not found, return null and clear buffer to avoid overflow
            if index == -1:
                self.in_bytes.clear()
                return None

            # If packet start found but full packet not yet received, wait for more data
            if len(self.in_bytes) - index < PACKETSIZE:
                self.in_bytes = self.in_bytes[index:]
                return None

            # Remove any bytes before the start of the packet to ensure we are aligned for parsing
            self.in_bytes = self.in_bytes[index:]

            raw = self.in_bytes[:PACKETSIZE]
            packet = InPacket.from_buffer_copy(raw)

            checksum = CalculateChecksum(
                ctypes.string_at(ctypes.byref(packet), in_packet_len - 1)
            )

            self.in_bytes = self.in_bytes[PACKETSIZE:]

            if checksum != packet.CheckSum:
                return None

            return packet

    def close(self):
        self.spi.close()

if __name__ == "__main__":
    """
    Example usage of MCUComServer. This will set fixed values and print continuous sensor data.
    """
    ComServer = MCUComServer()

    ComServer.set_servo(forward=1, pitch=0, roll=0)
    ComServer.set_thruster(thrust=1)

    try:
        while True:
            packet = ComServer.send_receive()
            if packet is not None:
                print(f"CheckSum OK: {packet.CheckSum}")
                print(f"    IMU:")
                print(f"        acc_x: {packet.IMUCom.acc_x}")
                print(f"        acc_y: {packet.IMUCom.acc_y}")
                print(f"        acc_z: {packet.IMUCom.acc_z}")
                print(f"        gyro_x: {packet.IMUCom.gyro_x}")
                print(f"        gyro_y: {packet.IMUCom.gyro_y}")
                print(f"        gyro_z: {packet.IMUCom.gyro_z}")
                print(f"    Magnetometer:")
                print(f"        head_x: {packet.MagCom.head_x}")
                print(f"        head_y: {packet.MagCom.head_y}")
                print(f"        head_z: {packet.MagCom.head_z}")
                print(f"    Bar30:")
                print(f"        pressure: {packet.Bar30Com.pressure}")
                print(f"        water_temp: {packet.Bar30Com.water_temp}")
                print(f"    Humidity:")
                print(f"        humidity: {packet.HumidCom.humidity}")
                print(f"        air_temp: {packet.HumidCom.air_temp}")
                print(f"    Board Temp:")
                print(f"        board_temp: {packet.BoardCom.board_temp}")
            else:
                print("No valid packet received")
    finally:
        ComServer.close()
