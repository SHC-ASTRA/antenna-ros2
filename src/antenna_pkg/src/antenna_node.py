import rclpy
from rclpy.node import Node
from rclpy import qos

import signal
import time
import struct

import serial
import sys
import glob

from sensor_msgs.msg import NavSatFix,NavSatStatus

serial_pub = None

NUM_SERIAL_PORTS = 3
BAUD_RATE = 115200

class SerialRelay(Node):
    def __init__(self):
        super().__init__("antenna_node")

        self.gps_sub_ = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, qos_profile=qos.qos_profile_sensor_data
        )      


        ports = SerialRelay.list_serial_ports()
        for i in range(NUM_SERIAL_PORTS):
            for port in ports:
                try:
                    ser = serial.Serial(port, BAUD_RATE, timeout=1)
                    ser.write(b"ping\n")
                    response = ser.read_until(bytes("\n", "utf8"))

                    if b"pong" in response:
                        self.port = port
                        self.get_logger().info(f"MCU found at {self.port}")
                        break
                except:
                    pass

        if self.port is None:
            self.get_logger().info("Unable to find MCU...")
            time.sleep(1)
            sys.exit(1)

        self.ser = serial.Serial(self.port,BAUD_RATE)


    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    
    
    def cleanup(self):
        self.get_logger().info("Cleaning up")
        if self.ser.is_open:
            self.ser.close()


    def gps_callback(self, msg: NavSatFix):
        latitude = msg.latitude
        longitude = msg.longitude

        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            self.get_logger().warn("Waiting for GPS Fix")
            return
        
        try:
            msg_bytes = struct.pack('<ff', latitude, longitude) # 8 bytes

            checksum_value = self.sum_checksum(msg_bytes)
            checksum_byte = struct.pack('<B', checksum_value) # 1 byte

            packet_data = msg_bytes + checksum_byte # 9 bytes

            encoded_packet = self.cobs_endode(packet_data) # 10 bytes

        except Exception as e:
            print(f"Couldn't encode message: {e}")


        try:
            self.ser.write(encoded_packet + 0x00) # 11 bytes, packet delimiter
            self.get_logger().info(f"Sent to MCU: {latitude}, {longitude} as {encoded_packet}")

        except serial.SerialException as e:
            self.get_logger().warn(f"Couldn't write to serial {e}")


    # Consistent Overhead Byte Stuffing
    def cobs_endode(data: bytes) -> bytes:
        # mutable bytes array
        out = bytearray()

        # first byte starts as pointing 1 ahead
        out.append(0x01)
        pointer_index = 0
        pointer_distance = 1

        for b in data:
            if b == 0x00:
                # set pointer byte to proper distance
                out[pointer_index] = pointer_distance
                # start pointer byte from current byte position
                out.append(0x01)
                pointer_index = len(out)
                pointer_distance = 1
            else:
                out.append(b)
                pointer_distance += 1
        # set last pointer byte to proper distance (to future delimiter byte)
        out[pointer_index] = pointer_distance

        return out
    
    def sum_checksum(data: bytes) -> int:
            return sum(data) & 0xFF

def myexcepthook(type, value, traceback):
    print("Uncaught exception", type, value)
    if serial_pub:
        serial_pub.cleanup()

def main(args=None):
    rclpy.init(args=args)

    global serial_pub
    serial_pub = SerialRelay()

    try:
        rclpy.spin(serial_pub)
    except KeyboardInterrupt:
        pass
    finally:
        sys.exit(0)

if __name__ == "__main__":
    signal.signal(
        signal.SIGTERM, lambda signum, frame: sys.exit(0)
    )
    main()