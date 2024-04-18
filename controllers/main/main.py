"""epuck_go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Camera, Motion, Display, RangeFinder, Supervisor, Field
import cv2, numpy as np 
import random
import math, vg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
from csv import DictWriter
import transforms3d.axangles as ta
MAXIMUM_NUMBER_OF_COORDINATES = 1000

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

        #distance sensors
        self.distanceSensorRight = self.getDistanceSensor("Sonar/Right")
        self.distanceSensorLeft = self.getDistanceSensor("Sonar/Left")
        self.distanceSensorRight.enable(4 * TIME_STEP)
        self.distanceSensorLeft.enable(4 * TIME_STEP)

        #pen
        #self.pen = self.getPen("pen")

    def sensorPathPlanning(self):
        self.supervisorFieldTrack()
        if self.distanceSensorLeft.getValue() < 0.35:
            self.rotate(50)
            print("saga")
            return True
        elif self.distanceSensorRight.getValue() < 0.35:
            self.rotate(-50)
            print("sola")
            return True
        else:
            self.forwards.play()
            #print("duz")
            return False

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

    def initializeSupervisor(self):
        self.node = self.getFromDef("Nao")
        
    def Rx(self,theta):
      return np.matrix([[ 1, 0           , 0           ],
                       [ 0, math.cos(theta),-math.sin(theta)],
                       [ 0, math.sin(theta), math.cos(theta)]])
      
    def Ry(self,theta):
      return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                       [ 0           , 1, 0           ],
                       [-math.sin(theta), 0, math.cos(theta)]])
      
    def Rz(self,theta):
      return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                       [ math.sin(theta), math.cos(theta) , 0 ],
                       [ 0           , 0            , 1 ]])


    def supervisorFieldTrack(self):
        #GET ROTATION FIELDS OF THE ROBOT
        self.rot_field = self.node.getField("rotation")
        self.master_rot_field =self.rot_field.getSFRotation()
                             
    def rotate(self,rot_degree):
        if math.isnan(rot_degree):
            return 0
            
        #print("rotta",rot_degree)            
        vector = [self.master_rot_field[0], self.master_rot_field[1], self.master_rot_field[2]]    
        vecToMatrix = ta.axangle2mat(vector,self.master_rot_field[3])
        
        dotProduct = np.dot(vecToMatrix, self.Rz(rot_degree))
        direc, angle = ta.mat2axangle(dotProduct)       
        
        self.rot_pos = [direc[0], direc[1], direc[2], angle]  
        self.rot_field.setSFRotation(self.rot_pos)  
        
    def supervisorFieldSetToLeft(self,rad):
        #SET THE VALUES OF ROTATION OF THE ROBOT
        self.rot_pos_left = [0, 0, 1, rad]
        self.rot_field.setSFRotation(self.rot_pos_left)

    def supervisorFieldSetToRight(self):
        #SET THE VALUES OF ROTATION OF THE ROBOT
        self.rot_pos_right = [0, 0, 1, 1.5708]
        self.rot_field.setSFRotation(self.rot_pos_right)
        
            
    def create_Trail(self):
    
        existing_trail = self.getFromDef("TRAIL")
        if existing_trail:
            self.remove(existing_trail)
        
        trail_string=""
        trail_string = trail_string + "DEF TRAIL Shape {\n"
        trail_string = trail_string + "  appearance Appearance {\n"
        trail_string = trail_string + "    material Material {\n"
        trail_string = trail_string + "      diffuseColor 0 1 0\n"
        trail_string = trail_string +  "      emissiveColor 0 1 0\n"
        trail_string = trail_string + "    }\n"
        trail_string = trail_string +  "  }\n"
        trail_string = trail_string + "  geometry DEF TRAIL_LINE_SET IndexedLineSet {\n"
        trail_string = trail_string + "    coord Coordinate {\n"
        trail_string = trail_string + "      point [\n"
        for i in range(MAXIMUM_NUMBER_OF_COORDINATES):
            trail_string = trail_string + "      0 0 0\n"
        trail_string = trail_string + "      ]\n"
        trail_string = trail_string + "    }\n"
        trail_string = trail_string + "    coordIndex [\n"
        for i in range(MAXIMUM_NUMBER_OF_COORDINATES):
            trail_string = trail_string + "      0 0 -1\n"
        trail_string = trail_string + "    ]\n"
        trail_string = trail_string + "  }\n"
        trail_string = trail_string + "}\n"
            
        self.root = self.getRoot()
        root_children_field = self.root.getField("children")
        root_children_field.importMFNodeFromString(-1, trail_string)
        
    def initialize_trail(self):
        self.target_node = self.getFromDef("TARGET")
        
        self.create_Trail()
        
        self.trail_line_set_node =self.getFromDef("TRAIL_LINE_SET")
        self.trail_line_set_field =self.trail_line_set_node.getField("coord")
        self.coordinates_node = self.trail_line_set_field.getSFNode()
        self.point_field = self.coordinates_node.getField("point")
        self.coord_index_field = self.trail_line_set_node.getField("coordIndex")

        self.index = 0         
        self.first_step = True
        
      
    def drawTrail(self):
        target_translation = self.target_node.getPosition()
        #target_translation[2] = round(target_translation[2],1)
        #target_translation[0] = round(target_translation[0],3)
        target_translation[1] = 0.3
        print(target_translation)
        self.point_field.setMFVec3f(self.index, target_translation)

    
        if self.index > 0: 
            self.coord_index_field.setMFInt32(3 * (self.index - 1), self.index - 1)
            self.coord_index_field.setMFInt32(3 * (self.index - 1) + 1, self.index)
        
        elif self.index == 0 and self.first_step == False:
            self.coord_index_field.setMFInt32(3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1), 0)
            self.coord_index_field.setMFInt32(3 * (MAXIMUM_NUMBER_OF_COORDINATES - 1) + 1,
                                       MAXIMUM_NUMBER_OF_COORDINATES - 1)
         
        
        self.coord_index_field.setMFInt32(3 * self.index, self.index)
        self.coord_index_field.setMFInt32(3 * self.index + 1, self.index)

    
        self.first_step = False
        self.index += 1
        self.index = self.index % MAXIMUM_NUMBER_OF_COORDINATES
        
    def calculateGoal(self):
        
        self.supervisorFieldTrack()
        
        if self.sensorPathPlanning() is False:         
            goal1 = 2.34,0.334,-2.16
            goal2 = 2.14,0.334,2.27
            #self.distance = round(math.sqrt(math.pow((goal1[0] - x),2) + math.pow((goal1[2] - z),2)), 4)
            #print("Distance:",self.distance)
            vec2 = np.array(self.gps.getValues())
            relativeX = goal2[0] - vec2[0]
            relativeZ = goal2[2] - vec2[2]
            angle = round(math.atan2(-(relativeX), relativeZ), 4)
            
            degrees = round(angle * (180 / math.pi), 4) 
            degrees = -(degrees)
            
            
            if degrees < 0:
                degrees = (degrees + 360) % 360
            
            robots_degree = -1*(self.master_rot_field[3]* (180 / math.pi))
            #print(robots_degree)
            print(degrees,robots_degree)
            df = goal2[2] - vec2[2]
            angleDifference = robots_degree - degrees 
            
            print(angleDifference)
            # if angleDifference>10:
                # self.rotate(angleDifference)
            # elif angleDifference<-10:
                # self.rotate(angleDifference)
            # else:
                # self.forwards.play()
                   
        
            if angleDifference > 0:
                if angleDifference < 180:
                    self.rotate(-1*(angleDifference))
                    print("eksi rot")
                # else:
                    # self.rotate(angleDifference)
                    # print("arti rot")   
            else:
                if angleDifference > -180:
                    self.rotate(angleDifference)
                    print("arti rot")
                # else:
                    # self.rotate(-1*(angleDifference))
                    # print("eksi rot")
        #self.sensorPathPlanning()        
        #if self.distance<0.1:
        #    self.forwards.stop()
            
    def initializeFile(self):
        self.fieldnames = ["Velocity", "Angular_Velocity", "Time"]
        
        with open("velocity_values.csv", "w") as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
            csv_writer.writeheader()
    
    
    
    def calculateVelocity(self):
        self.velocity = self.node.getVelocity()
        self.linearVel = round(math.sqrt(math.pow(self.velocity[0],2) + math.pow(self.velocity[1],2) +
                                    math.pow(self.velocity[2],2)),4)
                                    
        self.AngularVel = round(math.sqrt(math.pow(self.velocity[3],2) +  math.pow(self.velocity[4],2) +
                                    math.pow(self.velocity[5],2)),4)                            
        
        #print(self.linearVel, self.getTime())
        with open("velocity_values.csv", "a") as csv_file:
            csv_writer =csv.DictWriter(csv_file, fieldnames= self.fieldnames)
            
            info = {
                "Velocity":self.linearVel,
                "Angular_Velocity":self.AngularVel,
                "Time": self.getTime()
            }
            
            csv_writer.writerow(info)
        
     
    
    def __init__(self):
        Supervisor.__init__(self)
        self.fig = plt.figure()
        # initialize stuff
        self.findAndEnableDevices()
        self.initializeSupervisor()
        self.loadMotionFiles()
        
        self.handWave.play()
        
        self.initializeFile()
        self.initialize_trail()
        #initial dummy target positions (x,y,z) based on robot's initial position
        self.target_right_x, self.target_right_y, self.target_right_z = [3.42, 0.294, -2.2]
        self.target_left_x, self.target_left_y, self.target_left_z = [3.39, 0.332, 2.33]
        self.hasTarget = False
        

    def run(self):
        while robot.step(TIME_STEP) != -1:
            #self.forwards.play()
            #self.calculateGoal()
            #self.supervisorFieldTrack()
            #self.calculateVelocity()
            self.drawTrail()
            self.sensorPathPlanning()
            #print(self.gps.getValues())
           


# create the Robot instance and run main loop
robot = Nnao()
robot.run()

