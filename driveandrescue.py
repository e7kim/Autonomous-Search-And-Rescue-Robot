from PiStorms import PiStorms
import time, datetime, math, random, numpy
from pixy import *
from ctypes import *


psm = PiStorms()

SHOULDRUN = True

class Finder(object):

    def __init__(self, camera, driver, grabber):
        self.camera = camera
        self.driver = driver
        self.grabber = grabber

        self.xCenter = 160
        self.targetHeight = 100

    def Step(self):
        red_block = self.camera.findSpecificBlock(1)

        print red_block.signature

        if red_block == None:
            print "The block does not exist."

        else:
            if red_block.x > self.xCenter:
                driver.leftTurn()
            elif red_block.x < self.xCenter:
                driver.rightTurn()
            else:
                if red_block.height < self.targetHeight:
                    self.driver.moveForward()
                elif red_block.height > self.targetHeight:
                    self.driver.moveBackward()
                else:
                    print "At block!"
                    self.driver.Stop()
                    grabber.Calculate()
                    SHOULDRUN = False



# Initialize Pixy Interpreter thread #
pixy_init()

class Blocks(Structure):
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
    count = pixy_get_blocks(100, self.blocks)
    found_blocks = []

    if count > 0:
      # Blocks found #
      # print 'frame %3d:' % (self.frame)
      # print "I see " + str(count) + " blocks!"
      self.frame += 1
      for index in range (0, count):
        block = self.blocks[index]
        if block.height > 10 and block.width > 10:
          found_blocks.append(block)
        # print 'sig: {0}, center at ({1}, {2}), height = {3}, width = {4}'.format (block.signature, block.x, block.y, block.height, block.width)

    return found_blocks


  def findSpecificBlock(self, signature):
    blocks = self.findBlocks()

    for block in blocks:
        print 'sig: {0}, center at ({1}, {2}), height = {3}, width = {4}'.format (block.signature, block.x, block.y, block.height, block.width)



    for block in blocks:
      if block.signature == signature:
        return block

    return None

class Driver(object):

  def __init__(self, lmotor, rmotor):
    self.lmotor = lmotor
    self.rmotor = rmotor

  def moveForward(self):
    # self.lmotor.setSpeedSync(-30)
    self.lmotor.setSpeed(-30)
    self.rmotor.setSpeed(-30)

  def moveBackward(self):
    self.lmotor.setSpeed(30)
    self.rmotor.setSpeed(30)

  def Stop(self):
    self.lmotor.setSpeed(0)
    self.rmotor.setSpeed(0)

  def rightTurn(self):
    self.lmotor.setSpeed(-20)
    self.rmotor.setSpeed(20)

  def leftTurn(self):
    self.lmotor.setSpeed(20)
    self.rmotor.setSpeed(-20)

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
                    self.Grab(i)
                    self.turn = True

    def Grab(self, degrees):
        self.joint1.runDegs(degrees + 55)
        time.sleep(1)
        self.joint2.setSpeed(-10)
        time.sleep(1)
        self.joint2.setSpeed(-5)
        time.sleep(0.5)
        self.joint1.runDegs(-(degrees - 20))
        time.sleep(1)
        self.Stop()

    def Stop(self):
        self.joint1.setSpeed(0)
        self.joint2.setSpeed(0)

        

grabber = Grabber(psm.BAM2, psm.BAM1)
driver = Driver(psm.BBM2, psm.BBM1)
subsumptionPixy = SubsumptionPixy()
finder = Finder(subsumptionPixy, driver, grabber)



while psm.isKeyPressed() == False:
    finder.Step()

driver.Stop()
grabber.Stop()