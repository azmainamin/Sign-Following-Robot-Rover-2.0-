#!/usr/bin/env python

"""
Sign Following Rover 2.0
This module defines RoverSignFollower class based on Rover 2.0

"""
import time
from rover import Rover20
import time
import sys
import signal
import cv2
import random
import numpy as np
from PIL import Image
from pytesseract import *
import math
from turtle import Turtle
import os


# Avoid button bounce by enforcing lag between button events
MIN_BUTTON_LAG_SEC = 0.5

# Avoid close-to-zero values on axis
MIN_AXIS_ABSVAL    = 0.01
ANGULAR_VELOCITY = 33.5 * 360/ 60  # 33.5 for full battery (>75 %); 22.5 for 60 % battery; 25.0 for less than 60 %.
VELOCITY = 1 #(meter/sec)
TURN_VALUE = 0.25 # proportional value of error (PID Control)


def _signal_handler(signal, frame):
    frame.f_locals['rover'].close()
    sys.exit(0)


class RoverSignFollower(Rover20):
    def __init__(self):
      """Sign follower initiation."""

        # Set up basics for Odometer
        self.state = (0,0,0) #x,y,theta
        self.path = [] #list will contain points of previous positions)
        Rover20.__init__(self)
        self.wname = 'Rover 2.0: Hit ESC to quit'
        self.quit = False
        self.fileIndex = 0
        self.fname = 'filename'


        # Defaults on start-up: lights off, ordinary camera
        self.lightsAreOn = False
        self.stealthIsOn = False

		# Initiate My Turtle graphics to be used as a map. 

        self.pcmfile = open('rover20.pcm', 'w')
        self.myturtle = Turtle()
        self.myturtle.speed(0)
        self.myturtle.pencolor("blue")
        self.nocommand_counter = 0
    
    def turn(self, angle):
    """ 
    Turns a given angle in degrees.
    If angle >0 it turns counter clockwise, if angle < 0 turns clock wise.
    """
        
        timer = angle / ANGULAR_VELOCITY
        time_0 = time.time()
        while time.time() - time_0 < abs(timer):
            if angle > 0:
                self.setTreads(-1,+1)
            else:
                self.setTreads(+1,-1)
        self.setTreads(0,0)
        
		#update odometer
        self.updateState(0,0,angle)
        self.myturtle.left(angle)
        
    def moveForward(self,t, speed = 1):
		  """
		  Moves straight forward a certain time t in seconds. Maximum speed by default.
		  For backward motion set speed = -1
		  """
        time_0 = time.time()
        while time.time() - time_0 < t:
            self.setTreads(speed,speed)
        self.setTreads(0,0)
      
        #update state for odometry
        distance = VELOCITY * t * speed
        x , y, theta = self.state
        x_change = distance * math.cos(math.radians(theta))
        y_change = distance * math.sin(math.radians(theta))
        
        self.updateState(x_change, y_change, 0)
        self.myturtle.forward(t * 300 * speed)

    def dance(self):
    """ Does a simulated dance routine."""
		
		  self.moveForward(0.3,-1)
      time.sleep(0.1) # Needs a small delay in between functions.
        
		  self.turn(45)
      time.sleep(0.1)
        
		  self.turn(-90)
      time.sleep(0.1)
        
		  self.turn(360 + 45)
      time.sleep(0.1)
        
		  self.moveForward(0.3,1)
      os._exit(0)

    def processVideo(self, jpegbytes, timestamp_10msec):
    """ Process video from Rover class. Gets called automatically"""
       
	    self.fname = 'tmp.jpg'
      fd = open(self.fname, 'w')
      fd.write(jpegbytes)
      fd.close()
    
    def getImageName(self):
    """Returns file name for image saved at the moment"""

      return self.fname

    def detect_blue_sign(self, image_name):
    """
    Takes the name of the image as an input.
    It finds the contours bordering BLUE regions and outputs the second biggest contour according to area. Also outputs the centroid of the second biggest contour.
    """
        
		  image=cv2.imread(image_name)
      cv2.imwrite('tmp0.jpg', image)
    
	# Initialize the lower and upper boundary for BLUE.
    
      blueBoundary=[([105, 50, 50], [130, 255, 255])]
    
  # Turn the boundaries into Numpy arrays
    
      for (lower,upper) in blueBoundary:
        lower_np=np.array(lower,dtype="uint8")
        upper_np=np.array(upper,dtype="uint8")
    
    # Mask the image to keep only blue. Blue area becomes white
    
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      mask=cv2.inRange(hsv,lower_np,upper_np)
      output=cv2.bitwise_and(image, image, mask=mask) #Returns only blue areas
      imgray=cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) #Turn blue into gray. Needed for binarizing.
    
    # Find contours bordering the blue areas only.
    
      cnt, h= cv2.findContours(imgray.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
      cnt = sorted(cnt, key=cv2.contourArea, reverse=True) [:-1]
    
    # Draw a rectangle around the inner contour. x,y,w,h are the corner coordinates (pixels).
    
      x,y,w,h=cv2.boundingRect(cnt[1]) 
    
    # Drawing the contours
    
      cv2.drawContours(image,cnt,-1,(128,255,0),2)
    
    # Cropping the inner rectangle and saving it (to be used for OCR).
    
      cropped_image = image[y:y+h,x:x+w]
      cropped_image_name="tmp_cropped.jpg"
      cv2.imwrite(cropped_image_name, cropped_image)

    
      return x,y,w,h, cropped_image_name
    def read_sign(self, sign_name):
      """
      Takes an image name as input and does OCR on it. Outputs the text as string
      """
      image = Image.open(sign_name)
      text = image_to_string(image) 
      text = text.lower().replace(" ","").replace("\n","")# Get rid of all whitespaces
      print("Command found: " + text)
      return text
    def execute_sign(self, cropped_image_name):
      """
		  Takes a image(cropped) as input and executes the command written there
      """
    # Open in PIL and do OCR
    
      command = self.read_sign(cropped_image_name)

    # Compare command and execute accordingly.
    
      if command=="right":
        self.turn(-90)
        time.sleep(1.0)
      elif command=="left":
        self.turn(90)
        time.sleep(1.0)
      elif command=="turnaround" or command == "goback":
        self.turn(180)
        time.sleep(1.0)
      elif command=="dance":
        self.dance()
      elif command=="stop":
        os._exit(0)
      else:
        print("No command detected")
        time.sleep(0.1)
    # If no command detected for more than 3 times due to skewed image, the robot goes back in a randomly chosen angle.
        if self.nocommand_counter < 3: 
          self.nocommand_counter += 1
        else:
          self.nocommand_counter = 0
          self.turn(random.choice([-1,1]) * 15)
          self.moveForward(0.5,-1)        
                
    def compare_centroids(self,image_name,x,y,w,h):
      """ 
      Takes an image name and the x,y,w,h of the sign as input and compares the centre of the image
      and the centre of the sign. Needed to orient and move towards the centre of the sign
      """
      image = cv2.imread(image_name)
      image_x,image_y = image.shape[:2]
      img_centr_x = image_x//2
      x_centroid = (x+w)/2
      gap=img_centr_x-x_centroid
            
      return gap
    def updateState(self,x_change, y_change, theta_change):
      """
      Updates state(x,y,theta) of the device by a certain amount and
      saves previous state in a list (self.path)
      """
      x , y, theta = self.state
      self.path.append(self.state)
      self.state = (x + x_change, y + y_change, theta + theta_change)

#------------------------------MAIN FUNCTION-----------------------------
def main():
  """ 
  Initiates the Rover and wifi signal. Robot moves in autopilot
  looking for a blue sign. Stops if it finds the dance or stop sign
  """
  myrover = RoverSignFollower()
  signal.signal(signal.SIGINT, _signal_handler)
  print("Battery at " + str(myrover.getBatteryPercentage())+"%") #Check battery status

  time.sleep(2)
  sign_counter = 0 
  while True:
    name = myrover.getImageName()
    try:
      x,y,w,h, cropped = myrover.detect_blue_sign(name)
      gap = myrover.compare_centroids('tmp.jpg',x,y,w,h)
      if abs(gap) > 10: # Orient towards the center of the sign and move towards it.
        if gap <0:
          myrover.turn(gap * TURN_VALUE )
        if gap > 0:
          myrover.turn(gap * TURN_VALUE )
      if w*h < 29000:  # Keep moving till the sign is big enough to read the command.
        myrover.moveForward(0.2 , 0.7)
        time.sleep(0.1)
      else:
        myrover.execute_sign(cropped)
        time.sleep(1)
    except:
      print("No blue sign detected") 
			myrover.moveForward(0.2,0.2) # If no blue sign detected, move forward a little and look again.
      sign_counter += 1
      if sign_counter > 10:
        break           

    
if __name__ == '__main__':
    main()

	
    

