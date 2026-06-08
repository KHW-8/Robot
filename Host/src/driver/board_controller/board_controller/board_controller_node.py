import rclpy
from rclpy.node import Node
import serial

class BoardController(Node):
    def __init__(self, device="", baudrate=1000000, timeout=5):
        # self.port = serial.Serial(None, baudrate, timeout)

    def 