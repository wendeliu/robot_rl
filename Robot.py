import RPi.GPIO as GPIO
from time import sleep
import smbus
import time 
import numpy as np
from SystemAgent import sysAgent

#from IMU import IMUSensor

#

dt=0.02

class Motor():

    def __init__(self,pwm,dirPin,defLevel,en1,spd_pin):
        self.pwm=pwm              # PWM object
        self.dirPin=dirPin        # dir pin Number
        self.defLevel=defLevel    # default voltage level of dir pin # default direction is pointing forward
        #GPIO pin Number
        self.en1=en1
        self.dir=0
        self.pwm.start(0)
        self.spd=0
        self.lastTrig=time.time()
        self.spd_pin = spd_pin

  
        # create feed back callback
    
    def spdCnt(self,channel):
        dt=time.time()-self.lastTrig
        self.lastTrig = time.time() 
        ms=60/dt

    def setSpeed(self,spd):
        if spd>100:
            spd=100
        elif spd<0:
            spd=0    
        
        if self.dir==1:
            spd=100-spd # speed is controlled by difference between dir and sig Pins
                        # 1-1 0-0 -> stop
                        # 1-0 0-1 -> maximum speed
        #print(spd,self.dirPin,'motor')
        self.pwm.ChangeDutyCycle(spd)

        #advanced
        # PID speed control

    def setDir(self,dir):# 1 forward #0 backword
        self.dir=dir
        if dir:
            GPIO.output(self.dirPin, self.defLevel)
        else:
            GPIO.output(self.dirPin, not self.defLevel) 
       
    def output(self,on):
        if on:
            #set enable on
            GPIO.output(self.en1, 1)
        else:
            #set enable off
            GPIO.output(self.en1, 0)

class IMU():

    def __init__(self,argc,argv):
        self.IMU_init(argc,argv)
        return 

    def IMU_init(self,argc,argv):# i2cdev address

        #for debug
        if argc != len(argv):
            print('imu argc argv not match')
            return -1
        #0 i2c bus
        self.bus=argv[0]

        #1 addr
        self.addr=argv[1]

        CHIP_ID=self.bus.read_byte_data(self.addr,0)
        ACC_ID=self.bus.read_byte_data(self.addr,1)
        MAG_ID=self.bus.read_byte_data(self.addr,2)
        GYRO_ID=self.bus.read_byte_data(self.addr,3)
        print('init IMU sensor...\n Sensor info: ')
        print(CHIP_ID,ACC_ID,MAG_ID,GYRO_ID)
        self.setModeOri()
        return 1

    def getTEMP(self):
        temp=self.bus.read_byte_data(self.addr,0x34)#TEMP
        return temp

    def getOri(self):
        
#        hed=int.from_bytes(self.bus.read_byte_data(address,0x1a).to_bytes(1,'little')+(self.bus.read_byte_data(address,0x1b)).to_bytes(1,'little'),'little',signed=True)#EUL_Heading_LSB
        rol=int.from_bytes(self.bus.read_byte_data(self.addr,0x1c).to_bytes(1,'little')+(self.bus.read_byte_data(self.addr,0x1d)).to_bytes(1,'little'),'little',signed=True)#EUL_Roll_LSB
#        pit=int.from_bytes(self.bus.read_byte_data(address,0x1e).to_bytes(1,'little')+(self.bus.read_byte_data(address,0x1f)).to_bytes(1,'little'),'little',signed=True)#EUL_Pitch_LSB
#        if DEBUG:
#            print(hed.to_bytes(2,'little',signed=True))
#            print(rol.to_bytes(2,'little',signed=True))
#            print(pit.to_bytes(2,'little',signed=True))
        #print(rol,'imu')
        return rol/16

    def setModeOri(self):
        # absolute orientation mode
        self.bus.write_byte_data(self.addr,0x3d,0x0c)
        #set unit m/s^2 Dps Deg C WINDOWS -180 - +180
        self.bus.write_byte_data(self.addr,0x3b,0)

class realAgent(sysAgent):

    def sysInit(self):

        GPIO.setwarnings(False)			#disable warnings
        GPIO.setmode(GPIO.BOARD)		#set pin numbering system
        #xs
        MOTOR_1_SIG_PIN = 32				# PWM pin connected to motor1
        MOTOR_1_DIR_PIN = 36
        MOTOR_1_EN_PIN = 15

        MOTOR_2_SIG_PIN = 33				# PWM pin connected to motor2
        MOTOR_2_DIR_PIN = 37
        MOTOR_2_EN_PIN = 16

        #i2c bus pin
        I2C_SDA = 3
        I2C_SCL = 5

        DEF_DIR1=1                           # default voltage level of dir pin
        DEF_DIR2=1                           # default voltage level of dir pin

        #
        
        #set pin mode
        GPIO.setup(MOTOR_1_SIG_PIN,GPIO.OUT)
        GPIO.setup(MOTOR_1_DIR_PIN,GPIO.OUT)
        GPIO.setup(MOTOR_1_EN_PIN,GPIO.OUT)

        GPIO.setup(MOTOR_2_SIG_PIN,GPIO.OUT)
        GPIO.setup(MOTOR_2_DIR_PIN,GPIO.OUT)
        GPIO.setup(MOTOR_2_EN_PIN,GPIO.OUT)

        #GPIO.setup(I2C_SDA,GPIO.OUT)
        #GPIO.setup(I2C_SCL,GPIO.OUT)
        #create i2c bus
        self.I2CBus=smbus.SMBus(1)
        #exit()
        CHIP_ID=self.I2CBus.read_byte_data(0x28,0)
        print(CHIP_ID)
# motor control instance
        
        self.MOTOR_1_CTRL = Motor(GPIO.PWM(MOTOR_1_SIG_PIN,100),MOTOR_1_DIR_PIN,DEF_DIR1,15,0)		#create PWM instance with frequency
        self.MOTOR_2_CTRL = Motor(GPIO.PWM(MOTOR_2_SIG_PIN,100),MOTOR_2_DIR_PIN,DEF_DIR2,16,0)		#create PWM instance with frequency

        # create IMU module
        self.IMU0=IMU(2,[self.I2CBus,0x28])
        #
        # instance init
        self.init=True
        self.speed=0
        self.dir=0
        self.state=[0,0]
        #
    def getSpeed(self):
        return 0

    def getState(self):
        preOri=self.state[0]
        self.state=[self.IMU0.getOri(),(self.IMU0.getOri()-preOri)/dt]
        #speed ori dir
        return self.state 


    
    def setMove(self,direc,speed):# 1 for , 2 back, speed 0-100

        #motor 1
        #setDir
        self.MOTOR_1_CTRL.setDir(direc)
        #setSpeed
        self.MOTOR_1_CTRL.setSpeed(abs(speed))
        self.MOTOR_1_CTRL.output(1)
        #motor 2
        #setDir
        self.MOTOR_2_CTRL.setDir(direc)
        #setSpeed
        self.MOTOR_2_CTRL.setSpeed(abs(speed))
        self.MOTOR_2_CTRL.output(1)
        #advanced
        
        self.speed=speed
        self.dir=direc
        
    def impulse(self,input):

        #parsing cmd
        if input:
            if type(input)==list:# input from keyboard
                if (input[0]=='JOY'):
                    spd=int(input[1])
                    direc = 1 if (spd>0) else 0
                else:
                    spd = 0
                    direc = 1
            #print(spd,direc)
            elif type(input)==int:
                spd=int(input)
                direc = 1 if (spd>0) else 0
        #prase input
            self.setMove(direc,spd)
        else:
            input = 0# no input


        # rotation eliminate


        return 
        #configure i2c
        #self.i2cAddr=0x28
        #self.i2cBus=smbus.SMBus(1)
        #init sensor


