#test
#test2

from PiStorms import PiStorms
import time, datetime, math, random, numpy
from pixy import *
from ctypes import *
psm = PiStorms()

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
        self.joint1.runDegs(degrees + 55)
        time.sleep(1)
        self.joint2.setSpeed(-10)
        time.sleep(1)
        self.joint2.setSpeed(-5)
        time.sleep(0.5)
        self.joint1.runDegs(-(degrees - 20))
        time.sleep(1)
        self.Stop()
        # self.joint1.runDegs(0)
        # self.joint2.runDegs(0)

    def Stop(self):
        self.joint1.setSpeed(0)
        self.joint2.setSpeed(0)

        

grabber = Grabber(psm.BAM2, psm.BAM1)

while psm.isKeyPressed() == False:
    grabber.Calculate()

grabber.Stop()