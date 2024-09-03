# motor_kit_singleton.py
from adafruit_motorkit import MotorKit
import board

class MotorKitSingleton:
    _instance = None

    @staticmethod
    def get_instance():
        if MotorKitSingleton._instance is None:
            MotorKitSingleton._instance = MotorKit(i2c=board.I2C())

        return MotorKitSingleton._instance