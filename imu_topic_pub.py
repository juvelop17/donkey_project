__author__ = 'Geir Istad'
"""
MPU6050 Python I2C Class - MPU6050 example usage
Copyright (c) 2015 Geir Istad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from MPU6050.MPU6050 import MPU6050
import rospy
from sensor_msgs.msg import Imu




class ImuTopicPublisher:
    def __init__(self):
        self.setOffsets()

        self.mpu = MPU6050(self.i2c_bus, self.device_address, self.x_accel_offset, self.y_accel_offset,
                      self.z_accel_offset, self.x_gyro_offset, self.y_gyro_offset, self.z_gyro_offset,
                      self.enable_debug_output)

        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        self.mpu_int_status = self.mpu.get_int_status()
        print(hex(self.mpu_int_status))

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        print(self.packet_size)
        self.FIFO_count = self.mpu.get_FIFO_count()
        print(self.FIFO_count)

        self.seq = 0
        self.count = 0
        self.FIFO_buffer = [0] * 64

        self.FIFO_count_list = list()

        self.frame_name = 'imu_publisher'
        self.pub1 = rospy.Publisher('raw_imu', Imu, queue_size=10)
        self.pub2 = rospy.Publisher('ahrs', Imu, queue_size=10)
        self.pub3 = rospy.Publisher('rollpitch', Imu, queue_size=10)
        rospy.init_node(self.frame_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10hz

    def setOffsets(self):
        self.i2c_bus = 1
        self.device_address = 0x68
        # The offsets are different for each device and should be changed
        # accordingly using a calibration procedure
        self.x_accel_offset = -5489
        self.y_accel_offset = -1441
        self.z_accel_offset = 1305
        self.x_gyro_offset = -2
        self.y_gyro_offset = -72
        self.z_gyro_offset = -5
        self.enable_debug_output = True

    def createImuMsg(self, quat, accel, gyro):
        imuMsg = Imu()

        imuMsg.orientation.x = quat.x
        imuMsg.orientation.y = quat.y
        imuMsg.orientation.z = quat.z
        imuMsg.orientation.w = quat.w

        imuMsg.linear_acceleration.x = accel.x
        imuMsg.linear_acceleration.y = accel.y
        imuMsg.linear_acceleration.z = accel.z

        imuMsg.angular_velocity.x = gyro[0]
        imuMsg.angular_velocity.y = gyro[1]
        imuMsg.angular_velocity.z = gyro[2]

        imuMsg.header.stamp = rospy.Time.now()
        imuMsg.header.frame_id = self.frame_name
        imuMsg.header.seq = self.seq

        self.seq += 1

        return imuMsg

    def publishImuMsg(self, pub, imuMsg):
        pub.publish(imuMsg)
        # rospy.loginfo(imuMsg)
        self.rate.sleep()

    def run(self):
        while True:
            FIFO_count = self.mpu.get_FIFO_count()
            mpu_int_status = self.mpu.get_int_status()

            # If overflow is detected by status or fifo count we want to reset
            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                self.mpu.reset_FIFO()
                # print('overflow!')
            # Check if fifo data is ready
            elif (mpu_int_status & 0x02):
                # Wait until packet_size number of bytes are ready for reading, default
                # is 42 bytes
                while FIFO_count < self.packet_size:
                    FIFO_count = self.mpu.get_FIFO_count()
                FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                accel = self.mpu.DMP_get_acceleration_int16(FIFO_buffer)
                quat = self.mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav = self.mpu.DMP_get_gravity(quat)
                gyro = self.mpu.get_rotation()

                roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
                if self.count % 100 == 0:
                    print('quat: ', str(quat.x), str(quat.y), str(quat.z), str(quat.w))
                    print('accel: ', str(accel.x), str(accel.y), str(accel.z))
                    print('gyro: ', str(gyro[0]), str(gyro[1]), str(gyro[2]))
                    print('roll: ' + str(roll_pitch_yaw.x))
                    print('pitch: ' + str(roll_pitch_yaw.y))
                    print('yaw: ' + str(roll_pitch_yaw.z))

                imuMsg = self.createImuMsg(quat, accel, gyro)
                self.publishImuMsg(self.pub1,imuMsg)
                self.publishImuMsg(self.pub2,imuMsg)
                self.publishImuMsg(self.pub3,imuMsg)
                self.count += 1

if __name__ == '__main__':
    imuTopicPublisher = ImuTopicPublisher()
    imuTopicPublisher.run()