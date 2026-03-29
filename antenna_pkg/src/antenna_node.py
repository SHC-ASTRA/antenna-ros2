import signal
import sys
import threading
import time
from glob import glob
from serial import Serial
from os import getenv
from time import sleep
import atexit

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from astra_msgs.msg import AntennaControl, AntennaFeedback
from sensor_msgs.msg import NavSatFix

serial_pub = None
thread = None

class AntennaNode(Node):

    serial: Serial

    def __init__(self):
        # Initialize node
        super().__init__("tracking_antenna")

        # Topics

        self.rover_gps_sub = self.create_subscription(
            NavSatFix, "/core/gps", self.send_gps_callback, 10
        )

        self.to_base = self.create_publisher(
            AntennaFeedback,
            "/antenna/to_base",
            10
        )
        self.from_base = self.create_subscription(
            AntennaControl,
            "/antenna/from_base",
            self.from_base_callback,
            10,
        )

        # setting up mcu
        self.port: str | None = getenv("PORT_OVERRIDE")

        for _ in range(4):
            if self.port is not None:
                break

            for port in glob("/dev/ttyUSB*") + glob("/dev/ttyACM*"):
                try:
                    ser = Serial(port, 115200, timeout=1)
                    ser.write(b"ping\n")
                    result = ser.read_until(bytes("\n", "utf8"))

                    # if we didn't get a pong back, we aren't talking to the right thing
                    if b"pong" not in result:
                        continue

                    self.port = port
                    self.get_logger().info(f"Found MCU at {self.port}")
                except:
                    pass

        if self.port is None:
            self.get_logger().info("Unable to find MCU...")
            sleep(1)
            exit(1)

        self.serial = Serial(self.port, 115200)

        # atexit.register(self.cleanup)


    def run(self):
        global thread
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        try:
            while rclpy.ok():
                self.process_mcu()
                # time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    def send_gps_callback(self, msg: String):
        print("todo")

    def from_base_callback(self, msg: String):
        print("todo")

    def process_mcu(self):
        message = self.serial.read_until(bytes("\n", "utf8")).decode("utf-8")
        self.get_logger().info(message)
        # message_lst = message.split(",")
        #
        # feedback = AntennaFeedback(
        #     gps_latitude=float(message_lst[0]),
        #     gps_longitude=float(message_lst[1]),
        #     gps_satellites=int(message_lst[2]),
        #     gps_altitude=float(message_lst[3]),
        #     gyro=[
        #         float(message_lst[4]),
        #         float(message_lst[5]),
        #         float(message_lst[6])
        #     ],
        #     degrees_from_north=float(message_lst[7]),
        #     calibration=[
        #         int(1),
        #         int(2),
        #         int(3),
        #         int(4),
        #     ]
        # )

    def base_feedback(self):
        pass


def clamp_short(x: int) -> int:
    return max(-32768, min(32767, x))


def myexcepthook(type, value, tb):
    print("Uncaught exception:", type, value)


def main(args=None):
    rclpy.init(args=args)
    sys.excepthook = myexcepthook

    global serial_pub
    serial_pub = AntennaNode()
    serial_pub.run()


if __name__ == "__main__":
    signal.signal(
        signal.SIGTERM, lambda signum, frame: sys.exit(0)
    )  # Catch termination signals and exit cleanly
    main()
