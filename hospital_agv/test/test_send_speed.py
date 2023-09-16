import serial, time, pygame, sys, threading
import numpy as np

class Controller():
    def __init__(self, port='/dev/ttyUSB0'):
        self.left_id = 2
        self.right_id = 1
        self.run = True
        self.msg = ['Driver Address', 'Function Code', 'Number of byte read', 'High 8 bits of data', 'Low 8 bits of data', 'High CRC', 'Low CRC']
        self.modes = {'AbsolutePosition': 0, 'RelativePosition': 1, 'Velocity': 2, 'Torque': 3}

        # Open serial port at "COM3" with a baud rate of 115200
        self.ser = serial.Serial(port, 
                        baudrate=115200, 
                        bytesize=serial.EIGHTBITS, 
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE
                        )
        
        #self.setMotorEnable()
        
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
            bRecv = False
            cNum = self.ser.inWaiting()
            if (cNum > 0):
                #print(cNum)
                data_str = self.ser.read(cNum)
                #print('READ : ', data_str, end='')
                bRecv = True
            if bRecv:
                #print('')
                #print('cNum: ', cNum)
                for i in range(len(data_str)):
                    message = ''
                    if i<6:
                        message = self.msg[i]
                    #print('[{}] : {}\t{}'.format(i, data_str[i], message))
                self.extractData(data_str)
            time.sleep(0.01) 
    def extractData(self, data):
        if len(data) > 6:
            #print('extractData: ', data)
            if (data[0]==1) and data[1]==3:
                numByte = data[2]
                #print('numByte: ', numByte)
                if len(data) < (numByte+5):
                    return None
                for i in range(3, 3+numByte, 2):
                    if i==15:
                        continue
                    value = self.convert_to_uint16(data[i], data[i+1])
                    if i==3 or i==19 or i==21:
                        value = self.convert_to_int16(data[i], data[i+1])
                    elif i==17:
                        value = self.convert_to_int32(data[i], data[i+1], data[i+2], data[i+3])
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
        #cmd = self.readRegister(id=self.left_id)
        #print('readInfo L: ', cmd)
        #time.sleep(0.1)
        #self.ser.write(cmd)
        cmd = self.readRegister(id=self.right_id)
        #print('readInfo R: ', cmd)
        time.sleep(0.1)
        self.ser.write(cmd)
    def setMotorEnable(self):
        cmdL = self.cmd(id=self.left_id, register_addr='2031', register_value=8)
        cmdR = self.cmd(id=self.right_id, register_addr='2031', register_value=8)

        for i in range(10):
            #self.ser.write(cmdL)
            #time.sleep(0.01)
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
        cmdR = self.cmd(id=self.right_id, register_value=-spdR)
        #print(cmdL, ' <<< : >>> ', cmdR)
        self.ser.write(cmdL)
        time.sleep(0.005)
        self.ser.write(cmdR)

    def __del__(self):
        self.run = False
        # Close the serial port
        self.ser.close()

robot = Controller('/dev/ttyUSB0')

limit=50
spd = 1
mode=0
t=time.time()
t2=time.time()

pygame.init()
display = pygame.display.set_mode((300, 300))

while True:
    if spd > limit or spd < -limit:
        spd = 0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
            print(event.type)

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                spd=0
                robot.forward(0)
                robot.forward(0)
                robot.forward(0)
                break
            elif event.key == pygame.K_a:
                mode=1
            elif event.key == pygame.K_d:
                mode=2
            elif event.key == pygame.K_s:
                mode=0
            elif event.key == pygame.K_UP:
                spd=spd+1
            elif event.key == pygame.K_DOWN:
                spd=spd-1
            elif event.key == pygame.K_0:
                spd=0
            break
    if spd>limit:
        spd=limit
    elif spd<-limit:
        spd=-limit
    if time.time()-t > 1.0:
        print('Current Speed : ' + str(spd))
        #robot.forward(spd)
        cmd = robot.readInfo()
        time.sleep(0.1)
        #cmd = robot.getOperatingMode()
        #print(cmd)
        t=time.time()
    if time.time()-t2 > 0.1:
        if mode == 0:
            #print(spd)
            robot.forward(spd)
        elif mode == 1:
            robot.forward(0)
            time.sleep(0.01)
            robot.move(spd, -spd)
        elif mode == 2:
            robot.forward(0)
            time.sleep(0.01)
            robot.move(-spd, spd)
        t2=time.time()
    time.sleep(0.01)

