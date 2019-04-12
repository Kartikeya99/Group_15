from smbus import SMBus
from time import sleep
from time import time
from RPi import GPIO
from picamera import PiCamera
import boto3
from datetime import datetime
from os import chdir

class ADXL345:
    bus = SMBus(1 if int('9000c10000', 16) >= 4 else 0)
    EARTH_GRAVITY_MS2   = 9.80665
    SCALE_MULTIPLIER    = 0.004
    DATA_FORMAT         = 0x31
    BW_RATE             = 0x2C
    POWER_CTL           = 0x2D
    BW_RATE_1600HZ      = 0x0F
    BW_RATE_800HZ       = 0x0E
    BW_RATE_400HZ       = 0x0D
    BW_RATE_200HZ       = 0x0C
    BW_RATE_100HZ       = 0x0B
    BW_RATE_50HZ        = 0x0A
    BW_RATE_25HZ        = 0x09
    RANGE_2G            = 0x00
    RANGE_4G            = 0x01
    RANGE_8G            = 0x02
    RANGE_16G           = 0x03
    MEASURE             = 0x08
    AXES_DATA           = 0x32
    def __init__(self, address = 0x53): 
        self.address = address
        self.setBandwidthRate(self.BW_RATE_100HZ)
        self.setRange(self.RANGE_2G)
        self.enableMeasurement()

    def enableMeasurement(self):
        self.bus.write_byte_data(self.address, self.POWER_CTL, self.MEASURE)

    def setBandwidthRate(self, rate_flag):
        self.bus.write_byte_data(self.address, self.BW_RATE, rate_flag)

    def setRange(self, range_flag):
        value = self.bus.read_byte_data(self.address, self.DATA_FORMAT)

        value &= ~0x0F;
        value |= range_flag;  
        value |= 0x08;

        self.bus.write_byte_data(self.address, self.DATA_FORMAT, value)
    
    def getAxes(self, gforce = False):
        bytes = self.bus.read_i2c_block_data(self.address, self.AXES_DATA, 6)
        
        x = bytes[0] | (bytes[1] << 8)
        if(x & (1 << 16 - 1)):
            x = x - (1<<16)

        y = bytes[2] | (bytes[3] << 8)
        if(y & (1 << 16 - 1)):
            y = y - (1<<16)

        z = bytes[4] | (bytes[5] << 8)
        if(z & (1 << 16 - 1)):
            z = z - (1<<16)

        x = x * self.SCALE_MULTIPLIER 
        y = y * self.SCALE_MULTIPLIER
        z = z * self.SCALE_MULTIPLIER

        if gforce == False:
            x = x * self.EARTH_GRAVITY_MS2
            y = y * self.EARTH_GRAVITY_MS2
            z = z * self.EARTH_GRAVITY_MS2

        x = round(x, 4)
        y = round(y, 4)
        z = round(z, 4)

        return {"x": x, "y": y, "z": z}


class stepMotor:
    A = None
    B = None
    C = None
    D = None
    def GPIO_PINS(self, x, y, w, z):
        GPIO.setup(x, GPIO.OUT)
        GPIO.setup(y, GPIO.OUT)
        GPIO.setup(w, GPIO.OUT)
        GPIO.setup(z, GPIO.OUT)
        self.A = x
        self.B = y
        self.C = w
        self.D = z

    def GPIO_SETUP(self, a, b, c, d):
        GPIO.output(self.A, a)
        GPIO.output(self.B, b)
        GPIO.output(self.C, c)
        GPIO.output(self.D, d)
        sleep(0.001)

    def right(self, deg):
        full_circle = 510.0
        degree = full_circle/360*deg
        self.GPIO_SETUP(0, 0, 0, 0)
        while degree > 0.0:
            self.GPIO_SETUP(1,0,0,0)
            self.GPIO_SETUP(1,1,0,0)
            self.GPIO_SETUP(0,1,0,0)
            self.GPIO_SETUP(0,1,1,0)
            self.GPIO_SETUP(0,0,1,0)
            self.GPIO_SETUP(0,0,1,1)
            self.GPIO_SETUP(0,0,0,1)
            self.GPIO_SETUP(1,0,0,1)
            degree -= 1
        self.GPIO_SETUP(0,0,0,0)

    def left(self, deg):
        full_circle = 510.0
        degree = full_circle/360*deg
        self.GPIO_SETUP(0,0,0,0)
        while degree > 0.0:
            self.GPIO_SETUP(1,0,0,1)
            self.GPIO_SETUP(0,0,0,1)
            self.GPIO_SETUP(0,0,1,1)
            self.GPIO_SETUP(0,0,1,0)
            self.GPIO_SETUP(0,1,1,0)
            self.GPIO_SETUP(0,1,0,0)
            self.GPIO_SETUP(1,1,0,0)
            self.GPIO_SETUP(1,0,0,0)
            degree -= 1
        self.GPIO_SETUP(0,0,0,0)


################## Project Code ###################

chdir('/home/pi/photos')

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

adxl345 = ADXL345()
motor = stepMotor()
camera = PiCamera()
server = boto3.resource('s3')


motor.GPIO_PINS(12, 16, 20, 21)
location = 'jarvis2'
st_time = time()

print "Start!"
while 1:
    sleep(1)
    gRead = adxl345.getAxes(True)
    if gRead['x'] > 0.5 or gRead['x'] < -0.5:
        print "Threshold Crossed!"
        print "Capturing Images!"
        
        motor.right(45)
        print "Capturing Image 1"
        camera.capture('1.jpg')
        motor.right(90)
        print "Capturing Image 2"
        camera.capture('2.jpg')
        motor.right(90)
        print "Capturing Image 3"
        camera.capture('3.jpg')
        motor.right(90)
        print "Capturing Image 4"
        camera.capture('4.jpg')
        motor.right(45)
        print "Capturing Image 5"
        camera.capture('5.jpg')
        motor.left(90)
        print "Capturing Image 6"
        camera.capture('6.jpg')
        motor.left(90)
        print "Capturing Image 7"
        camera.capture('7.jpg')
        motor.left(90)
        print "Capturing Image 8"
        camera.capture('8.jpg')
        motor.left(90)

        cur_time = datetime.now()
        d = str(cur_time.day)
        m = str(cur_time.month)
        hh = str(cur_time.hour)
        mm = str(cur_time.minute)

        timeStamp = d+'-'+m+'--'+hh+'-'+mm+'--'
        
        print "Uploading Images"
        for i in range(1, 9):
            print "Uploading Image No."+str(i)
            file_name = str(i)+'.jpg'
            save_as = timeStamp+file_name
            server.meta.client.upload_file(file_name, location, save_as)


        
    print "Sensing started!"
    c_time = time()
    if c_time - 60 >= st_time:
        break
