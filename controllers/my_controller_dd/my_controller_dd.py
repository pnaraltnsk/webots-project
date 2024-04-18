"""epuck_go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Camera, Motion, Display, RangeFinder, Supervisor, Field
import cv2, numpy as np
import random
import math

TIME_STEP = 64

class Obstacles:
    def __init__(self, positionX, positionY, positionZ, obstacleSize):
        self.x = positionX
        self.y = positionY
        self.z = positionZ
        self.size = obstacleSize

class Nnao(Supervisor):
    def loadMotionFiles(self):
        self.forwards = Motion("../../motions/Forwards.motion")
        #self.sideStepLeft = Motion("../../motions/SideStepLeft.motion")
        #self.sideStepRight = Motion("../../motions/SideStepRight.motion")
        self.turnLeft = Motion("../../motions/TurnLeft60.motion")
        self.turnRight = Motion("../../motions/TurnRight60.motion")
        #self.backwards = Motion("../../motions/Backwards.motion")
        self.handWave = Motion("../../motions/HandWave.motion")

    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    def findAndEnableDevices(self):
        # camera
        self.cameraTop = self.getCamera("CameraTop")
        self.cameraBottom = self.getCamera("CameraBottom")
        self.cameraTop.enable(4 * TIME_STEP)
        self.cameraBottom.enable(4 * TIME_STEP)

        # gps
        self.gps = self.getGPS("gps")
        self.gps.enable(4 * TIME_STEP)

        # rangeFinder
        self.rangeCamera = self.getRangeFinder("range-finder")
        self.rangeCamera.enable(4 * TIME_STEP)

        #pen
        #self.pen = self.getPen("pen")

    def cameraProcess(self):
        image = cv2.imread("img.png")
        imHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(imHSV, np.array([0, 70, 50]),  np.array([10, 255, 255]));
        mask2 = cv2.inRange(imHSV, np.array([170, 70, 50]),  np.array([180, 255, 255]));
        redMask = cv2.bitwise_or(mask1,mask2)

        self.contours, hierarchy = cv2.findContours(redMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image,self.contours, -1, (255,0,0), 1)

        minDepth = 1.0

        for cnt in self.contours:
            self.x, self.y, self.width, self.height = cv2.boundingRect(cnt)

        LeftDepth = float("{:.2f}".format(self.rangeCamera.rangeImageGetDepth(self.rangeCamera.getRangeImage() ,
            self.rangeCamera.getWidth(),44, 48)))

        RightCorner = float("{:.2f}".format(self.rangeCamera.rangeImageGetDepth(self.rangeCamera.getRangeImage() ,
            self.rangeCamera.getWidth(),0, 63)))

        RightDepth = float("{:.2f}".format(self.rangeCamera.rangeImageGetDepth(self.rangeCamera.getRangeImage() ,
            self.rangeCamera.getWidth(),27, 48)))

        #print(RightDepth, LeftDepth)

        if LeftDepth<=0.33 or RightDepth<=0.33:
            self.turnLeft.setLoop(True)
            self.forwards.setLoop(False)
            self.turnLeft.play()
        else:
            self.forwards.setLoop(True)
            self.turnLeft.setLoop(False)
            self.forwards.play()

        #cv2.imshow("result", image)
        cv2.waitKey(10)

    def rangeProcess(self):
        frameRange = cv2.imread("range.png")
        frameRange = cv2.resize(frameRange,(350,250))

        self.distance1 = self.rangeCamera.rangeImageGetDepth(self.rangeCamera.getRangeImage(), self.rangeCamera.getWidth(), 31, 55)
        self.distance2 = self.rangeCamera.rangeImageGetDepth(self.rangeCamera.getRangeImage(), self.rangeCamera.getWidth(), 63, 55)
        self.distance3 = self.rangeCamera.rangeImageGetDepth(self.rangeCamera.getRangeImage(), self.rangeCamera.getWidth(), 15, 55)
        #print(self.rangeCamera.rangeImageGetDepth(self.rangeCamera.getRangeImage(), self.rangeCamera.getWidth(), 32, 63))
        self.walkForward()

        cv2.imshow("Range",frameRange)
        cv2.waitKey(10)

    def walkForward(self):
        x,y,z = self.gps.getValues()
        self.supervisorFieldTrack()
        #print(x," ",y," ",z)
        #print(self.rangeCamera.rangeImageGetDepth(self.rangeCamera.getRangeImage(), self.rangeCamera.getWidth(), int(self.x), int(self.y)))
        if self.distance1 < 0.33 and self.distance1 != 0.0:
            self.angleToObstacle = round(math.atan2(-(obstacle.z - z), -(self.distance1)), 4)
            self.angleToObstacle = round(angleToObstacle * (180 / math.pi), 4)
        else:
            #print("duz")
            self.forwards.setLoop(True)
            self.turnLeft.setLoop(False)
            self.forwards.play()


    def objectDetection(self):
        # camera
        self.cameraBottom.getImage()
        self.cameraBottom.saveImage("img.png",1)

        # range
        self.rangeCamera.getRangeImage()
        self.rangeCamera.saveImage("range.png",1)

        self.cameraProcess()
        #self.rangeProcess()

    #POTENTIAL FIELD ALGO:
    def initializeObstacles(self):
        self.obstacleList = []

        #appending the obstacleList
        self.obstacleList.append( Obstacles(-0.11,0.26,-1.43,0.2))
        self.obstacleList.append( Obstacles(0.65,0.26,2.2,0.2))
        self.obstacleList.append( Obstacles(0.96,0.26,0.12,0.2))
        self.obstacleList.append( Obstacles(1.95,0.26,0.54,0.2))
        self.obstacleList.append( Obstacles(2.68,0.26,-0.45,0.2))
        self.obstacleList.append( Obstacles(0.76,0.26,-2.07,0.2))
        self.obstacleList.append( Obstacles(-0.39,0.26,0.09,0.3))
        self.obstacleList.append( Obstacles(1.38,0.26,-0.94,0.3))
        self.obstacleList.append( Obstacles(0.29,0.26,1.11,0.3))
        self.obstacleList.append( Obstacles(2.21,0.26,-1.79,0.3))
        self.obstacleList.append( Obstacles(2.91,0.26,1.51,0.3))
        self.obstacleList.append( Obstacles(1.38,0.26,1.57,0.3))

    def rangeToObstacle(self):
        self.rangeProcess()


    def calculateObstacles(self):
        self.distance = 0
        self.relativeX = 0
        self.relativeY = 0
        self.angle = 0
        self.degrees = 0
        self.obstacleRadius = 0
        self.bufferZone = 0.15

        self.supervisorFieldTrack()

        x,y,z = self.gps.getValues()
        for obstacle in self.obstacleList:
            self.distance = round(math.sqrt(math.pow((obstacle.x - x),2) + math.pow((obstacle.z - z),2)), 4)
            self.obstacleRadius = round(math.sqrt( math.pow( obstacle.size, 2) +
                                                   math.pow( obstacle.size, 2)), 4)

            if self.distance < (self.obstacleRadius + self.bufferZone):
                angleToObstacle = round(math.atan2(-(obstacle.z - z), -(obstacle.x - x)), 4)
                angleToObstacle = round(angleToObstacle * (180 / math.pi), 4)

                if angleToObstacle < 0:
                    angleToObstacle = (angleToObstacle + 360) % 360

                angleDifference = self.angle - angleToObstacle
                if angleDifference > 0:
                    if angleDifference < 180:
                        self.supervisorFieldSetToRight()
                    else:
                        self.supervisorFieldSetToLeft()
                else:
                    if angleDifference > -180:
                        self.supervisorFieldSetToRight()
                    else:
                        self.supervisorFieldSetToLeft()

    def calculateRoute(self):
        x,y,z = self.gps.getValues()
        self.distance_target = round(math.sqrt(math.pow((self.target_left_x),2) + math.pow((self.target_left_z),2)), 4)

        self.angle_target = round(math.atan2(-(self.target_left_z), self.target_left_x), 4)
        self.degrees_target = round(angle * (180 / math.pi), 4)
        self.degrees_target = -(self.degrees_target)

        if self.degrees_target < 0:
            self.degrees_target = (self.degrees_target + 360) % 360

        self.angle_difference_target = self.angle_target - self.degrees_target

        # if self.angle_difference_target > 0:
        #     if self.angle_difference_target < 180:

    def initializeSupervisor(self):
        self.node = self.getFromDef("Nao")

    def supervisorFieldTrack(self):
        #GET ROTATION FIELDS OF THE ROBOT
        self.rot_field = self.node.getField("rotation")
        self.master_rot_field = self.rot_field.getSFRotation()

        #print(self.master_rot_field)

    def supervisorFieldSetToLeft(self):
        #SET THE VALUES OF ROTATION OF THE ROBOT
        self.rot_pos_left = [-0.86, 0.35, 0.36, 1.72]
        self.rot_field.setSFRotation(self.rot_pos_left)

    def supervisorFieldSetToRight(self):
        #SET THE VALUES OF ROTATION OF THE ROBOT
        self.rot_pos_right = [0.86, 0.35, 0.35, -1.70]
        self.rot_field.setSFRotation(self.rot_pos_right)

    def __init__(self):
        Supervisor.__init__(self)
        self.initializeObstacles()

        # initialize stuff
        self.findAndEnableDevices()
        self.initializeSupervisor()
        self.loadMotionFiles()
        #self.pen.write(True)
        self.handWave.play()

        #initial dummy target positions (x,y,z) based on robot's initial position
        self.target_right_x, self.target_right_y, self.target_right_z = [3.42, 0.294, -2.2]
        self.target_left_x, self.target_left_y, self.target_left_z = [3.39, 0.332, 2.33]
        self.hasTarget = False

    def run(self):
        while robot.step(TIME_STEP) != -1:
            #self.objectDetection()
            #self.calculateGoal()
            self.forwards.play()
            #self.rangeProcess()



# create the Robot instance and run main loop
robot = Nnao()
robot.run()
