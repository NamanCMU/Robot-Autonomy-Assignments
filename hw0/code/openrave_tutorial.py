#!/usr/bin/env python

PACKAGE_NAME = 'hw0'

# Standard Python Imports
import os
import copy
import time
import numpy as np
import scipy

# OpenRAVE
import openravepy
from openravepy import *
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)


curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata

#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
  if openrave_data_path == '':
      os.putenv('OPENRAVE_DATA', ordata_path_thispack)
  else:
      os.putenv('OPENRAVE_DATA', '%s:%s'%(ordata_path_thispack, openrave_data_path))


class RoboHandler:
  def __init__(self):
    self.env = openravepy.Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('Tutorial Viewer')
    self.env.Load('models/%s.env.xml' %PACKAGE_NAME)
    time.sleep(5) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]
    

  #remove all the time.sleep(0) statements! Those are just there so the code can run before you fill in the functions

  # move in a straight line, depending on which direction the robot is facing
  def move_straight(self, dist):
    #TODO Fill in, remove sleep
    #time.sleep(0)

    with self.env:
	T1 = self.robot.GetTransform() # Get the current Transform
	T2 = openravepy.axisAngleFromRotationMatrix(T1[0:3,0:3]) # Get the current axis angles of the robot
        T1[0,3] = dist*np.cos(T2[2]) # Distance movied in X-direction
        T1[1,3] = dist*np.sin(T2[2]) # Distance moved in Y-direction
        self.robot.SetTransform(T1) # Setting the new transform  
	
 # rotate the robot about the z-axis by the specified angle (in radians)
  def rotate_by(self, ang):
    #TODO Fill in, remove sleep
    #time.sleep(0)

    T = openravepy.matrixFromAxisAngle([0,0,ang]) # Get the transformation matrix
    with self.env:
    	self.robot.SetTransform(np.dot(T,self.robot.GetTransform())) # Applying the new transformation
        
		
  # go to each of the square corners, point towards the center, and snap a photo!
  def go_around_square(self):
    #TODO Fill in

    # set the robot back to the initialize position after
    with self.env:
      self.robot.SetTransform(np.identity(4)); 
      self.rotate_by(np.pi/4)
      self.move_straight(-np.sqrt(2))
      time.sleep(3)
      self.rotate_by(np.pi/2)
      time.sleep(3)
      self.rotate_by(np.pi/2)
      time.sleep(3)
      self.rotate_by(np.pi/2)
        
  # a function to help figure out which DOF indices correspond to which part of HERB
  def figure_out_DOFS(self):
    #TODO Fill in, remove sleep
    #time.sleep(0)

    ### Joints
    with self.env:
	Joint = self.robot.GetJoints()
        print 'Name of all the Joints: [',
        for i in xrange(0,len(Joint)):
	    print Joint[i].GetName(), 
        print ']'
    ###
    
    ### Indices
    print 'Indices of Right Arm: ',list([0,1,2,3])
    print 'Indices of Right Hand: ',list([4,5,6,7,8,9,10,11])
    print 'Indices of Left Arm: ',list([12,13,14,15])
    print 'Indices of Left Hand: ',list([16,17,18,19,20,21,22])
    print 'Indices of Head: ',list([23,24])
    ###

  # put herb in self collision
  def put_in_self_collision(self):
    #TODO Fill in, remove sleep
    #time.sleep(0)
    
    DOFValues = self.robot.GetDOFValues() # Get current DOF values
    DOFValues[1]  = -3 # Modify one of them so that there is self collision
    DOFValues[12]  = 3 # Modify another value
    print 'DOFValues: ', DOFValues
    Indices = xrange(0,len(DOFValues))
    with self.env:
	self.robot.SetDOFValues(DOFValues,Indices,checklimits = False) # Set the new DOF values and set checklimits = False

  # saves an image from above, pointed straight down
  def save_viewer_image_topdown(self, imagename):
    TopDownTrans = np.array([ [0, -1.0, 0, 0], [-1.0, 0, 0, 0], [0, 0, -1.0, 5.0], [0, 0, 0, 1.0] ])
    #seems to work without this line...but its in the tutorial, so I'll keep it here in case
    self.env.GetViewer().SendCommand('SetFiguresInCamera 1') # also shows the figures in the image
    I = self.env.GetViewer().GetCameraImage(640,480,  TopDownTrans,[640,640,320,240])
    scipy.misc.imsave(imagename + '.jpg',I)
      
    
  
if __name__ == '__main__':
  robo = RoboHandler()
  #robo.move_straight(1)
  #robo.rotate_by(-1*np.pi/4)
  #robo.go_around_square() 
  #robo.figure_out_DOFS()
  #robo.put_in_self_collision()
  
  #Uncomment the following to make the script initialize the RoboHandler
  #and drop you into an IPython shell.
  #t = np.array([ [0, -1.0, 0, 0], [-1.0, 0, 0, 0], [0, 0, -1.0, 5.0], [0, 0, 0, 1.0] ])  
  #robo.env.GetViewer().SetCamera(t)

  #import IPython
  #IPython.embed()

  # spin forever
  while True:
    time.sleep(1)
  
  
