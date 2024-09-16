import rclpy
from rclpy.node import Node
from re_rassor_sensors.lib.adafruit_ina260 import INA260
from std_msgs.msg import Bool, Float32
import board
from statistics import mean

class CurrentandPowerNode(Node):
    def __init__(self):

        super().__init__('current_and_power_node')

        # initialise chip
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.ina260 = INA260(i2c) # default address 0x40

        # 0x1770 x 1.25 mA = 7500 mA as alert limit
        self.ina260.alert_limit = 0x1770

        # assert alert pin
        self.ina260.overcurrent_limit = True

        # keep the flag high until MASK_ENABLE register will be read
        self.ina260.alert_latch_enable = True        

        # list to hold 4x voltage values
        self.voltage_array = []

        self.motor_shutdown_publisher_ = self.create_publisher(Bool, 'shutdown_cmd', 10)

        self.voltage_publisher_ = self.create_publisher(Float32, 'battery_voltage', 10)
        self.current_publisher_ = self.create_publisher(Float32, 'system_current', 10)

        self.timer_1 = self.create_timer(0.05, self.get_current_alert) # run every 0.05 s
        self.timer_2 = self.create_timer(1, self.get_voltage_and_current) # run every 1s

    def get_voltage_and_current(self):

        # create battery voltage msg
        battery_voltage_msg = Float32()

        # get the current voltage
        current_battery_voltage = self.ina260.voltage

        # add voltages to the array to check for low battery
        if len(self.voltage_array) < 4:
            self.voltage_array.append(current_battery_voltage)
        if len(self.voltage_array) == 4:
            # average and check voltage for low battery
            average_voltage = mean(self.voltage_array)
            self.voltage_array.clear()

            # shutdown the Raspberry Pi if the voltage over the given time period drops below 12.8V
            if average_voltage <= 12.8: # change to what this needs to be
                print("low battery: shutting down!!")
                # os.system("sudo shutdown -h now")
                return

        # publish the battery voltage
        battery_voltage_msg.data = current_battery_voltage
        self.voltage_publisher_.publish(battery_voltage_msg)

        # system current
        system_current = Float32()
        system_current.data = self.ina260.current
        self.current_publisher_.publish(system_current)

    def get_current_alert(self):

        shutdown_flag = Bool()

        if self.ina260.alert_function_flag:
            shutdown_flag = True
        else:
            shutdown_flag = False

        self.motor_shutdown_publisher_.publish(shutdown_flag)

def main(args=None):

    rclpy.init(args=args)
    
    node = CurrentandPowerNode()
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
