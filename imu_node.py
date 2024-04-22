import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import time
import smbus2

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.i2c_bus = smbus2.SMBus(1)  # Use 0 for older Raspberry Pi models
        self.address = 0x68  # MPU6050 I2C address
        self.configure_mpu6050()

    def configure_mpu6050(self):
        # Configure MPU6050 here if needed
        pass

    def read_acceleration(self):
        data = self.i2c_bus.read_i2c_block_data(self.address, 0x3B, 6)
        accel_x = self.convert_accel_data(data[0], data[1])
        accel_y = self.convert_accel_data(data[2], data[3])
        accel_z = self.convert_accel_data(data[4], data[5])
        return {'x': accel_x, 'y': accel_y, 'z': accel_z}

    def convert_accel_data(self, high_byte, low_byte):
        value = (high_byte << 8) | low_byte
        if value > 32767:
            value -= 65536
        return value / 16384.0  # MPU6050 sensitivity scale factor

    def read_gyro(self):
        data = self.i2c_bus.read_i2c_block_data(self.address, 0x43, 6)
        gyro_x = self.convert_gyro_data(data[0], data[1])
        gyro_y = self.convert_gyro_data(data[2], data[3])
        gyro_z = self.convert_gyro_data(data[4], data[5])
        return {'x': gyro_x, 'y': gyro_y, 'z': gyro_z}

    def convert_gyro_data(self, high_byte, low_byte):
        value = (high_byte << 8) | low_byte
        if value > 32767:
            value -= 65536
        return value / 131.0  # MPU6050 sensitivity scale factor

    def publish_imu_data(self):
        imu_msg = Imu()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "imu_link"  # Adjust frame_id according to your setup
        imu_msg.header = header

        # Read raw data from MPU6050
        accel_data = self.read_acceleration()
        gyro_data = self.read_gyro()

        # Fill in IMU message
        imu_msg.linear_acceleration.x = accel_data['x']
        imu_msg.linear_acceleration.y = accel_data['y']
        imu_msg.linear_acceleration.z = accel_data['z']
        imu_msg.angular_velocity.x = gyro_data['x']
        imu_msg.angular_velocity.y = gyro_data['y']
        imu_msg.angular_velocity.z = gyro_data['z']

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    while rclpy.ok():
        imu_node.publish_imu_data()
        time.sleep(0.1)  # Publish at 10 Hz
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#inside my_imu_package, run with ros2 run my_imu_package imu_node.py
