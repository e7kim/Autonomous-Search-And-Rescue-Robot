from pixy import BlockArray
from pixy import pixy_init
from pixy import pixy_get_blocks
from pixy import *
import math
import time
import datetime
import numpy

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


class Finder(object):  

    def __init__(self, driver, SubsumptionPixy, grabber):
        self.driver = driver
        self.SubsumptionPixy = SubsumptionPixy
        self.grabber = grabber

        self.blockFound = False

    def turn(self):
        data = self.SubsumptionPixy.findBlocks()

        for block in data:
            # print "X: " + str(block.x)
            # p(1, block.x)
            # print "W: " + str(block.width)
            # p(3, block.width)
            # print "H: " + str(block.height)
            # p(4, block.height)
            if block.signature == 1 and self.blockFound == False:
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
                        self.driver.step()
                        p(3, "STOP")
                        self.blockFound = True
                        print self.blockFound
                        self.grabber.Calculate()




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




class Grabber(object):

    def __init__(self, joint1, joint2):
        self.joint1 = joint1
        self.joint2 = joint2

        # self.thetas = numpy.zeros((1, 3))
        # self.lengths = numpy.zeros((1, 3))

        self.matrixR = numpy.zeros((2, 2))
        self.matrixF = numpy.zeros((2, 1))
        self.matrixT = numpy.zeros((2, 1))

        self.theta2 = 0
        self.thetas = [0, 90, self.theta2]
        self.lengths = [0, 24, 33]
        self.turn = False

    def Calculate(self):
        if self.turn == False:
            for i in range(20, -150, -1):
                self.theta2 = i

                self.matrixR[0, 0] = math.cos(math.radians(self.theta2))
                self.matrixR[0, 1] = -math.sin(math.radians(self.theta2))
                self.matrixR[1, 0] = math.sin(math.radians(self.theta2))
                self.matrixR[1, 1] = math.cos(math.radians(self.theta2))
                
                # print self.matrixR

                self.matrixF[0, 0] = self.lengths[2]
                self.matrixF[1, 0] = 0

                # print self.matrixF

                self.matrixT[0, 0] = self.lengths[1]
                self.matrixT[1, 0] = 0

                # print self.matrixT

                self.matrixF = numpy.dot(self.matrixR, self.matrixF) + self.matrixT

                # print self.matrixF

                self.matrixR[0, 0] = math.cos(math.radians(self.thetas[1]))
                self.matrixR[0, 1] = -math.sin(math.radians(self.thetas[1]))
                self.matrixR[1, 0] = math.sin(math.radians(self.thetas[1]))
                self.matrixR[1, 1] = math.cos(math.radians(self.thetas[1]))

                self.matrixF = numpy.dot(self.matrixR, self.matrixF)

                # print i
                # print self.matrixF

                if self.matrixF[0, 0] > 25 and self.matrixF[0, 0] < 30 and self.matrixF[1, 0] > 0 and self.matrixF[1, 0] < 5 and self.turn == False:
                    # self.Grab()
                    print "Theta2 has been found!!!"
                    print self.matrixF
                    print i
                    self.turn = True
                    print self.turn
                    self.Grab(i)
    def Grab(self, degrees):
        self.joint1.runDegs(degrees - 40)
        time.sleep(1)
        self.joint2.setSpeed(-40)
        time.sleep(1)
        self.joint2.setSpeed(-10)
        time.sleep(0.5)
        self.joint1.runDegs(-(degrees + 50))
        time.sleep(1)
        self.Stop()
        # self.joint1.runDegs(0)
        # self.joint2.runDegs(0)

    def Stop(self):
        self.joint1.setSpeed(0)
        self.joint2.setSpeed(0)





grabber = Grabber(psm.BAM1, psm.BAM2)
driver = Driver(psm.BBM2, psm.BBM1)
subPixy = SubsumptionPixy()
finder = Finder(driver, subPixy, grabber)


print "Booting up"

print "Starting!"

while not psm.isKeyPressed():
    finder.step()

driver.stop()
driver.step()