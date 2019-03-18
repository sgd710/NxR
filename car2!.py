import pygame, sys, time, getopt, RTIMU, os.path, math
sys.path.append('.')
from pygame.locals import *
import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from picamera import PiCamera


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
screen = pygame.display.set_mode((400,300))
pygame.display.set_caption('RC CAR')

            ##DRIVING CONTROLS##

GPIO.setup(18,GPIO.OUT) #BACK WHEEL PWM (EN)
GPIO.setup(22,GPIO.OUT) #BACK WHEEL PHASE: LOW:FORWARD | HIGH=BACKWARD
GPIO.setup(17,GPIO.OUT) #FRONT WHEEL PHASE: HIGH=LEFT | LOW=RIGHT
GPIO.setup(19,GPIO.OUT) #FRONT WHEEL PWM (EN)
GPIO.setup(20,GPIO.OUT) #MOTOR CONTROLLER MODE


#MODE: HIGH=PHASE/ENABLE | LOW=IN/IN (DO NOT USE)
GPIO.output(20,GPIO.HIGH)

motor = GPIO.PWM(18,10e6)
GPIO.output(22,GPIO.LOW)
DC = 0

joystick_count = pygame.joystick.get_count()
print("joystick_count")
print(joystick_count)
print("--------------")

numaxes = joystick.get_numaxes()
print("numaxes")
print(numaxes)
print("--------------")

numbuttons = joystick.get_numbuttons()
print("numbuttons")
print(numbuttons)
print("--------------")

            ##CAMERA##
##camera = PiCamera()
##camera.start_preview(fullscreen=False, window = (500, 0, 640, 480))

            ##IMU##
SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
else:
    print("IMU Init Succeeded")

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

roll_collector = []
pitch_collector = []
yaw_collector = []

while True:
    ##CONTROLLER##
    laxis0 = joystick.get_axis(0)
    laxis1 = joystick.get_axis(1)
    raxis0 = joystick.get_axis(2)
    raxis1 = joystick.get_axis(3)
##    up = joystick.get_button(4)
##    right = joystick.get_button(5)
##    down = joystick.get_button(6)
##    left = joystick.get_button(7)
##    L2 = joystick.get_button(8)
##    R2 =joystick.get_button(9)
    L1 =joystick.get_button(10)
    R1 = joystick.get_button(11)
##    triangle = joystick.get_button(12)
##    circle = joystick.get_button(13)
##    cross = joystick.get_button(14)
##    square = joystick.get_button(15)
##    PS = joystick.get_button(16)
    
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
                if raxis1 < 0:
                    GPIO.output(22,GPIO.LOW)
                    motor.start(DC)
                    DC = raxis1*-1*50
                    print("forwards")
                if raxis1 > 0:
                    GPIO.output(22,GPIO.HIGH)
                    motor.start(DC)
                    DC = raxis1*50
                    print("backwards")
                if raxis1 == 0:
                    motor.stop()
                if laxis0 > 0.5:
                    GPIO.output(17,GPIO.LOW)
                    GPIO.output(19,GPIO.HIGH)
                    print("right")
                if laxis0 < -0.5:
                    GPIO.output(17,GPIO.HIGH)
                    GPIO.output(19,GPIO.HIGH)
                    print("left")
                if laxis0 > -0.5 and laxis0 < 0.5:
                    GPIO.output(19,GPIO.LOW)
                if L1 ==1 and R1 == 1:
                    #camera.stop_preview()
                    plt.show()
                    sys.exit()
                    
    ##IMU##
    if imu.IMURead():
        # x, y, z = imu.getFusionData()
        # print("%f %f %f" % (x,y,z))
        data = imu.getIMUData()
        fusionPose = data["fusionPose"]
        print("roll: %f pitch: %f yaw: %f" % (math.degrees(fusionPose[0]), 
            math.degrees(fusionPose[1]), -1*math.degrees(fusionPose[2])))
        time.sleep(poll_interval/100)
        roll_collector.append(math.degrees(fusionPose[0]))
        pitch_collector.append(math.degrees(fusionPose[1]))
        yaw_collector.append(-1*math.degrees(fusionPose[2]))
        np.savetxt('1roll.csv', roll_collector)
        np.savetxt('1pitch.csv', pitch_collector)
        np.savetxt('1yaw.csv', yaw_collector)
        plt.plot([len(pitch_collector)/25.00], [pitch_collector[-1]], 'ro')
        plt.plot([len(pitch_collector)/25.00], [yaw_collector[-1]], 'go')
        plt.plot([len(pitch_collector)/25.00], [roll_collector[-1]], 'bo')
        red_patch = mpatches.Patch(color='red', label='Pitch')
        blue_patch = mpatches.Patch(color='blue', label='Roll')
        green_patch = mpatches.Patch(color='green', label='Yaw')
        plt.legend(handles=[red_patch,blue_patch,green_patch])
        
        
                    

























