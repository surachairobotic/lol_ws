import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion

import serial, time, pygame, sys, threading, math
import numpy as np

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
                
        self.wheel = Controller('/dev/ttyUSB0')
        self.L = 0.7
        self.current_cmd = Twist()
        self.cmdCalOdom = [0,0]
        self.cmdL = self.cmdR = 0
        self.currentOdom = [0,0]
        
        self.prev_t = time.time()
        self.prev_pose = Pose()
        self.prev_pose.position.x = 0.0
        self.prev_pose.position.y = 0.0
        self.prev_pose.position.z = 0.0
        self.prev_pose.orientation.x = 0.0
        self.prev_pose.orientation.y = 0.0
        self.prev_pose.orientation.z = 0.0
        self.prev_pose.orientation.w = 1.0

        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.control_callback, 10)
        self.subscription  # prevent unused variable warning

        self.odom_pub = self.create_publisher(Odometry, '/agv/odom', 10)
        #self.vel_pub = self.create_publisher(Twist, '/cmd_vel', self.control_callback, 10)
        self.timer = self.create_timer(1/100, self.publish_odom)

        self.timer_readWheel = self.create_timer(1/5, self.wheel.readInfo)
        self.timer_controlWheel = self.create_timer(1/20, self.controlWheel)
        self.timer_printInfo = self.create_timer(1/10, self.printInfo)

    def control_callback(self, msg):
        self.current_cmd = msg

    def readWheel(self):
        self.wheel.readInfo()
    def linear_velocity_to_RPM(self, linear_velocity, wheel_diameter):
        circumference = wheel_diameter * math.pi
        rotations_per_second = linear_velocity / circumference
        RPM = rotations_per_second * 60
        return RPM
    def RPM_to_velocity(self, RPM_left, RPM_right, wheel_base, wheel_diameter):
        circumference = wheel_diameter * math.pi
        linear_velocity_left = RPM_left * circumference / 60
        linear_velocity_right = RPM_right * circumference / 60
        linear_velocity = (linear_velocity_left + linear_velocity_right) / 2
        angular_velocity = (linear_velocity_right - linear_velocity_left) / wheel_base
        return [linear_velocity, angular_velocity]

    def controlWheel(self):
        self.wheel.move(-self.cmdL, self.cmdR)
    def printInfo(self):
        x = self.current_cmd.linear.x        
        w = self.current_cmd.angular.z
        vL = (self.current_cmd.linear.x - (self.L/2)*self.current_cmd.angular.z)
        vR = (self.current_cmd.linear.x + (self.L/2)*self.current_cmd.angular.z)

        self.cmdL = int(self.linear_velocity_to_RPM(vL, self.wheel.radius*2))
        self.cmdR = int(self.linear_velocity_to_RPM(vR, self.wheel.radius*2))
        
        self.cmdCalOdom = self.RPM_to_velocity(self.cmdL, self.cmdR, self.L, self.wheel.radius*2)
        self.currentOdom = self.RPM_to_velocity(self.wheel.left_rpm, self.wheel.right_rpm, self.L, self.wheel.radius*2)
        print(self.currentOdom, " : ", [self.wheel.left_rpm, self.wheel.right_rpm], " : ", self.cmdCalOdom)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = float(self.currentOdom[0])
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = float(self.currentOdom[1])
        
        x = self.prev_pose.position.x
        y = self.prev_pose.position.y
        yaw = self.get_yaw_from_quaternion(self.prev_pose.orientation)
        vx = msg.twist.twist.linear.x
        #vy = msg.twist.twist.linear.y
        v_yaw = msg.twist.twist.angular.z

        dt = time.time()-self.prev_t
        self.prev_t = time.time()
        #print("dt : ", dt)

        new_x = x + (vx * math.cos(yaw)) * dt
        new_y = y + (vx * math.sin(yaw)) * dt
        new_yaw = yaw + v_yaw * dt

        msg.pose.pose.position.x = new_x
        msg.pose.pose.position.y = new_y
        msg.pose.pose.position.z = 0.0        
        msg.pose.pose.orientation = self.yaw_to_quaternion(new_yaw)
        self.prev_pose = msg.pose.pose
        self.odom_pub.publish(msg)

    def yaw_to_quaternion(self, yaw):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

    def get_position_and_orientation(self, pose, dt):
        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = get_yaw_from_quaternion(pose.pose.orientation)
        vx = twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        v_yaw = odom_msg.twist.twist.angular.z
        
        dt = 0.1  # Time interval in seconds
        new_x = x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        new_y = y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        new_yaw = yaw + v_yaw * dt
        
        return (new_x, new_y, new_yaw)
    
    def get_yaw_from_quaternion(self, quaternion):
        #print(type(quaternion))
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw

class Controller():
    def __init__(self, port='/dev/ttyUSB0'):
        self.left_id = 2
        self.right_id = 1
        
        self.radius = 0.1 # 10 cm.
        
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.left_radps = 0.0
        self.right_radps = 0.0
        self.left_meterps = 0.0
        self.right_meterps = 0.0
        
        self.raw_data = "".encode('ascii')
        self.run = True
        self.msg = ['Driver Address', 'Function Code', 'Number of byte read', 'High 8 bits of data', 'Low 8 bits of data', 'High CRC', 'Low CRC']
        self.modes = {'AbsolutePosition': 1, 'RelativePosition': 2, 'Velocity': 3, 'Torque': 4}

        # Open serial port at "COM3" with a baud rate of 115200
        self.ser = serial.Serial(port, 
                        baudrate=115200, 
                        bytesize=serial.EIGHTBITS, 
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE
                        )
        
        #self.setMode('Torque')
        self.setMode('Velocity')
        self.setMotorEnable(True)
        
        self.tRead = threading.Thread(target=self.readThread)
        self.tRead.start()

    def convert_to_int16(self, high_8_bits, low_8_bits):
        return np.int16((high_8_bits << 8) + low_8_bits)
    def convert_to_uint16(self,high_8_bits, low_8_bits):
        return np.uint16(self.convert_to_int16(high_8_bits, low_8_bits))
    def convert_to_int32(self, b1, b2, b3, b4):
        return np.int32((b1 << 24) + (b2 << 16) + (b3 << 8) + b4)

    def readThread(self):
        while (self.run):
            cNum = self.ser.inWaiting()
            if (cNum > 0):
                data_str = self.ser.read(cNum)
                if isinstance(self.raw_data, str):
                    self.raw_data = (self.raw_data).encode('ascii')
                #print(type(self.raw_data))
                #print(type(data_str))
                self.raw_data = (self.raw_data+data_str)
            if len(self.raw_data) > 128:
                #print(self.raw_data)
                cmd = self.extract_commands(self.raw_data)
                self.raw_data = ""
                #print(cmd)
                #print('-------')
                cmd_left = self.get_cmd(cmd, b'\x02\x03')
                cmd_right = self.get_cmd(cmd, b'\x01\x03')
                #print(cmd_right)
                if len(cmd_left) > 0:
                    self.extractData(cmd_left[-1])
                if len(cmd_right) > 0:
                    self.extractData(cmd_right[-1])
            time.sleep(0.01) 
    def extract_commands(self, data):
        commands = []
        start_indices = [i for i in range(len(data)) if data[i:i+2] in [b'\x01\x06', b'\x02\x06', b'\x01\x03', b'\x02\x03']]
        for i in range(len(start_indices) - 1):
            commands.append(data[start_indices[i]:start_indices[i + 1]])
        commands.append(data[start_indices[-1]:])
        return commands
    def get_cmd(self, commands, key):
        res = []
        for cmd in commands:
            #print(cmd)
            #if cmd[1] == b'\x03':
            #    print(cmd)
            if cmd[:2] == key:
                #print('OK')
                res.append(cmd)
        return res

    def extractData(self, data):
        '''
        01 Driver Address
        03 Function Code
        02 Number of bytes read
        00 High 8 bits of data
        64 Low 8 bits of data
        B9 High 8 bits of CRC check
        AF Low 8 bits of CRC check
        '''
        if len(data) > 6:
            #print('extractData: ', data)
            #print('left_id = ', self.left_id)
            if ((data[0]==self.left_id) or (data[0]==self.right_id)) and data[1]==3:
                numByte = data[2]
                #print('numByte: ', numByte)
                #print('extractData: ', data)
                if len(data) < (numByte+5):
                    return None
                for i in range(3, 3+numByte, 2):
                    if i==15:
                        continue
                    value = self.convert_to_uint16(data[i], data[i+1])
                    if i==3 or i==19 or i==21:
                        value = self.convert_to_int16(data[i], data[i+1])
                        if i==19:
                            #print('spd: ', value)
                            if data[0]==self.left_id:
                                self.left_rpm = -value*0.1
                                self.left_radps = (self.left_rpm/60.0)*(math.pi*2)
                                self.left_meterps = self.left_radps*self.radius
                            elif data[0]==self.right_id:
                                self.right_rpm = value*0.1
                                self.right_radps = (self.right_rpm/60.0)*(math.pi*2)
                                self.right_meterps = self.right_radps*self.radius
                    elif i==17:
                        value = self.convert_to_int32(data[i], data[i+1], data[i+2], data[i+3])
                    '''
                    if i==3:
                        print('Driver temperature: ', value*0.1)
                    elif i==5:
                        print('Software version: ', value)
                    elif i==7:
                        print('Motor temperature (C): ', value*0.1)
                    elif i==9:
                        print('Motor status register (0: stationary, 1: running): ', value)
                    elif i==11:
                        print('Hall input status (0-7 ;If 0 or 7 Hall error): ', value)
                    elif i==13:
                        print('Bus voltage (V): ', value*0.01)
                    elif i==17:
                        print('Actual position: ', value)
                    elif i==19:
                        print(data[i], ', ', data[i+1])
                        print('Actual speed (r/min): ', value*0.1)
                    elif i==21:
                        print('Actual torque (A): ', value*0.1)
                    elif i==23:
                        print('Last error code: ', value)
                    else:
                        print('step: ', i, ' = ', value)
                    '''
    
    def readThreadOld(self):
        while (self.run):
            bRecv = False
            cNum = self.ser.inWaiting()
            if (cNum > 0):
                #print(cNum)
                data_str = self.ser.read(cNum)
                #print('READ : ', data_str, end='')
                bRecv = True
            if bRecv:
                #print('')
                print('cNum: ', cNum, " >> ", data_str)
                for i in range(len(data_str)):
                    message = ''
                    if i<6:
                        message = self.msg[i]
                    #print('[{}] : {}\t{}'.format(i, data_str[i], message))
                self.extractData(data_str)
            time.sleep(0.01) 

    # Define a function to calculate the Modbus CRC
    def modbus_crc(self, data):
        crc = 0xFFFF
        for i in data:
            crc = crc ^ i
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc

    def decimal2hex(self, dec):
        return hex(dec & (2**16-1))[2:]

    def readInfo(self):
        cmdL = self.readRegister(id=self.left_id)
        cmdR = self.readRegister(id=self.right_id)
        time.sleep(0.1)
        self.ser.write(cmdL)
        time.sleep(0.1)
        self.ser.write(cmdR)
    def setMotorEnable(self, value: bool):
        if value:
            reg_value = 8
        else:
            reg_value = 7
        cmdL = self.cmd(id=self.left_id, register_addr='2031', register_value=reg_value)
        cmdR = self.cmd(id=self.right_id, register_addr='2031', register_value=reg_value)

        for i in range(10):
            self.ser.write(cmdL)
            time.sleep(0.01)
            self.ser.write(cmdR)
            time.sleep(0.01)
    def setMode(self, mode):
        print(mode)
        print(self.modes[mode])
        cmdL = self.cmd(id=self.left_id, register_addr='2032', register_value=self.modes[mode])
        cmdR = self.cmd(id=self.right_id, register_addr='2032', register_value=self.modes[mode])
        for i in range(10):
            self.ser.write(cmdL)
            time.sleep(0.01)
            self.ser.write(cmdR)
            time.sleep(0.01)
        

    def getOperatingMode(self):
        cmd = self.readRegister(id=self.left_id, start_addr='203A', register_num=1)
        #print('getOperatingMode: ', cmd)
        self.ser.write(cmd)
        time.sleep(0.01)
        cmd = self.readRegister(id=self.right_id, start_addr='203A', register_num=1)
        #print('getOperatingMode: ', cmd)
        self.ser.write(cmd)
        time.sleep(0.01)
        
    def readRegister(self, id, function_code='03', start_addr='2024', register_num=11):
        return self.cmd(id, function_code='03', register_addr=start_addr, register_value=register_num)

    def cmd(self, id=2, function_code='06', register_addr='203A', register_value=100):
        hx = self.decimal2hex(register_value)
        res = str(id).zfill(2) + function_code + register_addr + str(hx).zfill(4)
        #print('cmd: ', res)
        res = bytes.fromhex(res)

        # Calculate the Modbus CRC of the data and append it to the end as a bytes object
        crc = self.modbus_crc(res)
        crc_bytes = crc.to_bytes(2, byteorder='little')
        res += crc_bytes
        #print('cmd2: ', res)

        return res

    def forward(self, speed=100):
        #print('speed: ', speed)
        self.move(speed, speed)

    def move(self, spdL=100, spdR=100):
        # Convert the hex string to a bytes object
        cmdL = self.cmd(id=self.left_id, register_value=spdL)
        cmdR = self.cmd(id=self.right_id, register_value=spdR)
        #print(cmdL, ' <<< : >>> ', cmdR)
        time.sleep(0.01)
        self.ser.write(cmdL)
        time.sleep(0.01)
        self.ser.write(cmdR)

    def __del__(self):
        self.setMotorEnable(False)
        self.run = False
        # Close the serial port
        self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

