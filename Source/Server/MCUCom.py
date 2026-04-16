import spidev
import time
from ConnectionTypes import *

class MCUComServer:
    def __init__(self, spi_bus=0, spi_device=0, max_speed_hz=10000):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = max_speed_hz
        self.spi.mode = 0

        self.out_packet = OutPacket()

        self.out_packet.SoP[0] = 0xAA
        self.out_packet.SoP[1] = 0x55

    def set_servo(self, forward=0, pitch=0, roll=0):
        self.out_packet.ServoCmd.forward = max(-1, min(1, forward))  # clamp to [-1, 1]
        self.out_packet.ServoCmd.pitch = max(-1, min(1, pitch))  # clamp to [-1, 1]
        self.out_packet.ServoCmd.roll = max(-1, min(1, roll))  # clamp to [-1, 1]

    def set_thruster(self, thrust=0):
        self.out_packet.ThrusterCmd.thrust = max(0, min(1, thrust))  # clamp to [0, 1]

    def send_receive(self):
        # Serialize struct into bytes
        out_bytes = bytearray(ctypes.string_at(ctypes.byref(self.out_packet), out_size))

        # Compute and set checksum in byte array
        out_bytes[-1] = CalculateChecksum(out_bytes[:-1])

        # Copy byte array to tx which has fixed size agreed by Pi and controller
        tx = bytearray(PACKETSIZE)
        tx[:out_size] = out_bytes

        # Send packet and receive response
        rx = self.spi.xfer2(tx)

        # Decode response bytes
        in_packet_raw = bytes(rx[:in_size])
        in_packet = InPacket.from_buffer_copy(in_packet_raw)

        checksum = CalculateChecksum(in_packet_raw[:-1])

        if checksum != in_packet.CheckSum:
            return None

        if in_packet.SoP[0] != 0xAA or in_packet.SoP[1] != 0x55:
            return None

        return in_packet

    def close(self):
        self.spi.close()

if __name__ == "__main__":
    """
    Example usage of MCUComServer. This will set fixed values and print continuous sensor data.
    """
    ComServer = MCUComServer()
    
    ComServer.set_servo(forward=1)
    ComServer.send_receive()
    time.sleep(5)

    ComServer.set_servo(forward=0.25)
    ComServer.send_receive()
    time.sleep(5)

    ComServer.set_servo(forward=1)
    ComServer.send_receive()
    time.sleep(5)

    ComServer.set_servo(forward=0.25)
    ComServer.send_receive()
    time.sleep(5)

    ComServer.set_servo(forward=0)

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
