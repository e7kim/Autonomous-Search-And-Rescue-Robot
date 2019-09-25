from pixy import BlockArray
from pixy import pixy_init
from pixy import pixy_get_blocks
from pixy import *
import math
import time
import datetime

from ctypes import *
from PiStorms import PiStorms
from mindsensors_i2c import mindsensors_i2c
from mindsensors import ABSIMU
psm = PiStorms()
absimu = ABSIMU()


#print screen
def p(row, text):
    psm.screen.termPrintAt(row, text)

blockSize = 20 # Correct number

pixy_init()

# #200 x 320
# class Mapper(object):
#     def __init__(self, Finder, Driver, cagSensor):
#         self.finder = Finder
#         self.driver = Driver
#         self.cagSensor = cagSensor
#         self.declination = 0
#         self.previousAngle = 0
#         self.currentAngle = 0
#         self.xPlacement = 0
#         self.yPlacement = 0
#         self.originalAngle = self.cagSensor.getAngle()
#         # self.mapData = [][]
#     def getXDisplacement(self):
#         return self.xPlacement

#     def getYDisplacement(self):
#         return self.yPlacement

#     def getDeclination(self):
#         return self.declination

#     def getAngleDelta(self):
#         return self.currentAngle - self.previousAngle

#     def updateMap(self):
#         self.previousAngle = self.currentAngle
#         self.currentAngle = self.cagSensor.getAngle()

#         self.declination = self.currentAngle - self.originalAngle

#         self.driver.update()
#         self.forwardDelta = self.driver.getDeltaPos()

#         self.turnAngle = self.getAngleDelta()


#         self.newXDisplacement = self.forwardDelta * math.cos(self.declination - self.turnAngle)
#         self.newYDisplacement = self.forwardDelta * math.sin(self.declination - self.turnAngle)


#         p(1, "Original Angle" + str(self.originalAngle))
#         p(2, "Current Angle" + str(self.currentAngle))
#         p(6, "ForwardDelta: " + str(self.forwardDelta))


#         # Update x displacement and y displacement
#         self.xPlacement += (self.newXDisplacement / 300)
#         self.yPlacement += (self.newYDisplacement / 300)

#         p(4, "XDisplacement: " + str(self.xPlacement))
#         p(5, "YDisplacement: " + str(self.yPlacement))

#         print "XDisplacement: " + str(self.xPlacement)
#         print "YDisplacement: " + str(self.yPlacement)
        


#         # store it in the array just because it's cool
#         # self.mapData.append([self.xDisplacement, self.yDisplacement]) 

#     def step(self):
#         self.updateMap()
#         self.finder.step()

        

class Finder(object):  

    def __init__(self, driver, SubsumptionPixy):
        self.driver = driver
        self.SubsumptionPixy = SubsumptionPixy

    def turn(self):
        data = self.SubsumptionPixy.findBlocks()

        for block in data:
            # print "X: " + str(block.x)
            # p(1, block.x)
            # print "W: " + str(block.width)
            # p(3, block.width)
            # print "H: " + str(block.height)
            # p(4, block.height)
            if block.signature == 1:
                if block.x < 130: 
                    self.driver.turnLeft()
                    p(3, "RIGHT")
                elif block.x > 190:
                    self.driver.turnRight()
                    p(3, "LEFT")
                else:
                    if block.width < 35 or block.height < 35:
                        print "W: " + str(block.width)
                        print "H: " + str(block.height)
                        self.driver.forward()
                        p(3, "FORWARD")
                    else:
                        self.driver.stop()
                        p(3, "STOP")

    def step(self):
        self.turn()
        self.driver.step()


class Driver(object):
    def __init__(self, leftMotor, rightMotor):
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor

        self.oldPosition = 0
        self.newPosition = 0
        self.deltaPos = 0

        self.motors = 0

    def update(self):
        self.oldPosition = self.newPosition
        self.newPosition = self.leftMotor.pos()
        self.deltaPos = self.newPosition - self.oldPosition

    def getDeltaPos(self):
        return self.deltaPos

    def forward(self):
        self.motors = 2
    def backwards(self):
        self.motors = 1
    def turnRight(self):
        self.motors = 3
    def turnLeft(self):
        self.motors = 4
        print "left"
    def stop(self):
        self.motors = 0
        print "stop"

    def step(self):
        if self.motors == 0:
            self.leftMotor.setSpeed(0)
            self.rightMotor.setSpeed(0)
        elif self.motors == 1:
            print "f"
            self.leftMotor.setSpeedSync(20)
        elif self.motors == 2:
            self.leftMotor.setSpeedSync(-20)
        elif self.motors == 3:
            self.leftMotor.setSpeed(12)
            self.rightMotor.setSpeed(-12)
        elif self.motors == 4:
            self.leftMotor.setSpeed(-12)
            self.rightMotor.setSpeed(12)

class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]

class SubsumptionPixy(object):
    def __init__(self):
        self.blocks = BlockArray(100)
        self.frame = 0

    def findBlocks(self):
        # Wait for blocks #
        count = pixy_get_blocks(100, self.blocks)
        found_blocks = []

        if count > 0:
            # Blocks found #
            print 'frame %3d:' % (self.frame)
            frame = self.frame + 1
            for index in range (0, count):
                block = self.blocks[index]
                found_blocks.append(block)
                # print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
                # print 'sig: {0}, center at ({1}, {2}), height = {3}, width = {4}'.format(block.signature, block.x, block.y, block.height, block.width)
        return found_blocks

class CAG(object):

    def __init__(self, cagSensor):
        self.angle = 0
        self.driver = Driver
        self.calibrate()

    def calibrate(self):
        psm.BAS1.activateCustomSensorI2C()

    def getAngle(self):
        self.compass_heading = absimu.get_heading()
        return self.compass_heading

# def led(k, r, g, b):
#     psm.led(k, r, g, b)

#     Fancy fancy
#     for i in range(0, 255):
#         led(1, i, i / 2, 255 - i)
#         led(2, i / 2, 255 - i, i)
#         time.sleep(0.01)

# cagSensor = CAG(psm.BAS1)
driver = Driver(psm.BBM2, psm.BBM1)
subPixy = SubsumptionPixy()
finder = Finder(driver, subPixy)
# mapper = Mapper(finder, driver, cagSensor)

print "Booting up"

# driver.forward()
# driver.step()

# time.sleep(1)

print "Starting!"

while not psm.isKeyPressed():
    finder.step()

driver.stop()
driver.step()