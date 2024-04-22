import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)  # Change the topic name here
        self.imu_sensor = mpu6050(0x68)  # Address of MPU6050, usually 0x68
        self.imu_sensor.setup()

    def publish_imu_data(self):
        imu_data = self.imu_sensor.get_all_data()  # Read raw data from MPU6050
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = imu_data['x_accel']
        imu_msg.linear_acceleration.y = imu_data['y_accel']
        imu_msg.linear_acceleration.z = imu_data['z_accel']
        imu_msg.angular_velocity.x = imu_data['x_gyro']
        imu_msg.angular_velocity.y = imu_data['y_gyro']
        imu_msg.angular_velocity.z = imu_data['z_gyro']

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
