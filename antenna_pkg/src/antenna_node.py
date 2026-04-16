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
    # Command constants from AntennaControl message
    FOLLOW_CORE = 0
    REQUEST_ANGLE = 1
    RECENTER = 2
    STOP = 3

    serial: Serial

    def __init__(self):
        # Initialize node
        super().__init__("tracking_antenna")

        self.is_following_coregps = True
        self.antenna_lat = None
        self.antenna_long = None

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

    def send_gps_callback(self, msg: NavSatFix):
        if self.is_following_corefeedback:
            # todo maddy
            # you can use self.antenna_lat and self.antenna_long
            # as well as msg.latitude and msg.longitude for the current gps of the rover
            # this function gets called every time there is a new NavSatFix message published
            # i set it to None by default so maybe include error handling
            requested_angle = 0
            self.serial.write(f"angle,{requested_angle}\n".encode("utf8"))


    def from_base_callback(self, msg: AntennaControl):
        if msg.command == self.FOLLOW_CORE:
            self.is_following_corefeedback = True
            self.get_logger().info("Command: FOLLOW_CORE")
        elif msg.command == self.REQUEST_ANGLE:
            self.is_following_corefeedback = False
            self.serial.write(f"angle,{msg.angle}\n".encode("utf8"))
            self.get_logger().info(f"Command: REQUEST_ANGLE -> {msg.angle}")
        elif msg.command == self.RECENTER:
            self.is_following_corefeedback = False
            self.serial.write(b"recenter\n")
            self.get_logger().info("Command: RECENTER")
        elif msg.command == self.STOP:
            self.is_following_corefeedback = False
            self.serial.write(b"stop\n")
            self.get_logger().info("Command: STOP")
        else:
            self.get_logger().warn(f"Unknown command: {msg.command}")

    def process_mcu(self):
        message = self.serial.read_until(bytes("\n", "utf8")).decode("utf-8")
        self.get_logger().info(message)
        message_lst = message.split(",")

        self.antenna_lat = message_lst[0]
        self.antenna_long = message_lst[1]

        feedback = AntennaFeedback(
            # gps_latitude=float(message_lst[0]),
            # gps_longitude=float(message_lst[1]),
            gps_satellites=int(message_lst[2]),
            # gps_altitude=float(message_lst[3]),
            gyro=[
                float(message_lst[4]),
                float(message_lst[5]),
                float(message_lst[6])
            ],
            degrees_from_north=float(message_lst[7]),
            calibration=[
                int(1),
                int(2),
                int(3),
                int(4),
            ]
        )

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
