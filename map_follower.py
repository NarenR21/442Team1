#!/usr/bin/env python
import rospy
import tf
import math
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
#import statements. A lot of these are unused because we removed some of the IMU and laser functionality. 


class MapFollower(object): # main follower object It's used to control the control loop.
    def __init__(self): #inits object
        # Initialize the node
        rospy.init_node('map_follower')
        
        # Subscribers
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.callback_imu)
        #old Steering and Acceleration errors for calculating D component. 
        self.oldSError =  0 
        self.oldAError = 0
        self.index = 0 #index of waypoint
        self.steerCmd = 0 #steering command number. This has broader scope so we can see it in other functions. 

        
        # Publishers
        gasCmdTopic = rospy.get_param('accelerator_pedal_cmd_topic', 'accelerator_cmd')
        steerCmdTopic = rospy.get_param('steering_cmd_topic', 'steering_cmd')
        self.accPub = rospy.Publisher(gasCmdTopic, Float32, queue_size=1)
        self.steerPub = rospy.Publisher(steerCmdTopic, Float32, queue_size=1)
        
    
        # Current state
        self.current_pose = None  # Updated by AMCL
        self.current_scan = None  # Updated by LaserScan
        #open waypoints
        self.ways = open('/home/nvidia/rallycar_ws/src/maxROStappen/src/scripts/knoy_FinalMap.txt', 'r')
        self.lines = self.ways.readlines() 
        self.lineLength = len(self.lines)
        self.desired_waypoints = [0, 0, 0, 0] #waypoint we're currently going to. 
        #an old version used these variables to save the amcl pose in these variables. This was slow, so now we use the amcl message directlty. These are still used
        # to extact the message into variables as needed later on rather then in the callback.
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
        self.x_pos = 0
        self.y_pos = 0
        #laser scan distances
        self.left_dist = 0
        self.wall_dist = 0
    
        # Control loop rate (10Hz)
        self.rate = rospy.Rate(20)
        
    def callback_imu(self, msg):
        #IMU callback. No longer used with removal of IMU functionality. 
        self.imu_sub = msg.orientation
        #print(str(self.imu_sub))
    def laser_callback(self, msg):
        #Laser callback functionality. No longer used with removal of laser functionality
        # Process laser scan data
        self.current_scan = msg
        self.left_dist = msg.ranges[900]
        self.right_dist = msg.ranges[100]
        
    def odom_callback(self, msg):
        # Process odometry data
        pass
        
    def amcl_callback(self, msg):
        # Update current pose from AMCL
        #gets current position from amcl and saves it.
        self.current_pose = msg
 

    
    def update_waypoint(self, distance_error, steering_error): #update waypoint decects if we're close enough to the next
        #waypoint to set the new desired waypoint to the next one. 
        if((distance_error < 1.25) & (self.index < self.lineLength)): #if we're within 1.25 meters and we haven't maxed out our waypoints
            #the 1.25 is changeable, we selected that as part of our tuning paramaters. It helps us turn earlier in the corner, so as we add speed,
            #it's not a bad idea to increase this number. 
            lines = self.lines[self.index] #get the line at the  index
            self.index+=1 #increment the index for next time. 
            lines = lines.translate({ord('['): None}) #remove the first brace
            lines = lines.translate({ord(']'): None}) #remove the second brace
            lines = lines.split(',') #split based on ",". This builds an array with the three numbers
            self.desired_waypoints[0] = float(lines[0]) #first one is the x value and is saved in the desired waypoints
            self.desired_waypoints[1] = float(lines[1]) #same for the y value
            self.desired_waypoints[2] = float(lines[2]) #and the theta value
    def current_dev(self):
        # Compute steering error based on self.current_pose and self.desired_waypoint
        # Calculate the desired angle to the waypoint
        #This is the drift error. As in how far are we translated from our desired waypoint. 
        #we can't use omni wheels to move in the translation. so we have to get the ackerman steer angle
        # to calculate how far over we need to move to hit our waypoint. 
        x_diff = self.desired_waypoints[0] - self.current_pose.pose.pose.position.x
        y_diff = self.desired_waypoints[1] - self.current_pose.pose.pose.position.y
        #x_diff and y_diff are calculating the delta in x and y to be used in the atan2 function.
        #they're not really important elsewhere, it's just easier to debug when it's seperated like this 
        desired_angle = math.atan2(y_diff, x_diff)
        #atan2 gives us the theta of steering we need given a deviation in x and y. the right triangle formed,
        #then tells us how we need to steer. the 2 in atan2 means that it respects the sign of the arguments. 
        #this makes it easier to get a left or right steer
        desired_angle = desired_angle * 180 / math.pi
        #convert to degrees and return the value. 
        return desired_angle
    
    def current_yaw(self):
        #this gives us our current global yaw. 
        # the above function, current_dev gives us our error in x and y, this gives us our error in theta. Not the actual error, but one of the compents
        #used to calculate it later on. 
        #we make a quaternion with the amcl imputs of our current position. 
        quaternion =(self.current_pose.pose.pose.orientation.x, self.current_pose.pose.pose.orientation.y, self.current_pose.pose.pose.orientation.z, self.current_pose.pose.pose.orientation.w)
        #we then get the euler representation from the quaternion so that it's in thetaX, thetaY, thetaZ.
        #we don't really care about the roll and pitch of the car, but the yaw is crucial. 
        #Yaw is represented by thetaZ so we know that the 3rd value in the eulur representaiton is our yaw.
        euler = tf.transformations.euler_from_quaternion(quaternion)
        #once we have the quaternion, we get the the yaw and convert it to degrees. 
        current_yaw = euler[2] * 180/math.pi
        #return the yaw. 
        return current_yaw
    
    def desired_yaw(self):
        #desired yaw returns the yaw that we want given our waypoint.
        #the third term of the waypoint is the yaw that was defined in that waypoint.
        desired_yaw = self.desired_waypoints[2]
        #get that value
        if(desired_yaw < 0):
            desired_yaw = 360 -  abs(desired_yaw)
        #if it's in the negative angle space, return it to the positive space.
        #return the value. 
        return desired_yaw
    
    def compute_distance_error(self):
        #this distance calculation is used for the update waypoint function. 
        x_diff = self.desired_waypoints[0] - self.current_pose.pose.pose.position.x
        y_diff = self.desired_waypoints[1] - self.current_pose.pose.pose.position.y
        #x and y deltas. 

        distance_error = math.sqrt((x_diff ** 2 ) + (y_diff ** 2))
        #use distance formula to calculate the length of the hypontuse to the waypoint. 
        #return it. 
        return distance_error
    
    def controller(self, distance_error, steering_error):
        #main PD controller
        #constants:
        kiS = 30 #Position steering
        kdS = 10 #derivative steering
        kiA = 70  #positional accelerator
        kdA = 0.0 #deriviative accelerator. 
        #correct the steering error to positive vector scale if it's negative. 
        if(steering_error < -180):
            steering_error = 360 - abs(steering_error)
        #Steering derivative error is old steering error- the current steering error.
        #THis is how much the error has changed. THe desired value for that is 0. 
        #So the overall desierd value is the same as the SDerror
        SDerror = float(self.oldSError) - float(steering_error)
        #the positional steering_error is passed in by the control loop.
        #the newSteering is equal to the positional constant times the steering_error.
        # plus the derivative constant times the derivative error. 
        newSteering = (kiS * steering_error) + (kdS * SDerror)
        
        #The acccelerator follows the exact same logic as above.
        #The derivative constant has been set to 0, because we don't want the current acclerator command
        #to be imfluenced by the previous ones. This has given us much better acceleration time, and corenering abilities
        #as we're not worried on a previous error being large resulting in us being fast. Especially when coming out of a sharp turn. 
        ADerror = self.oldAError - distance_error
        newDistance = (kiA * distance_error) + (kdA * ADerror)

        #this was the really innovative part that we wrote.
        #Our throttle is then remapped to the an exponential curve based on the steering error. 
        #This means that as the steering error jumps up at a turn, we slow down signifacantly. 
        #While still taking into account the distance to the next point.
        #The equation is | 180 / x^2 | plugging this into a graphing tool will show 
        # the overall throttal map shape. 
        throttle = newDistance * (abs(180 / ( steering_error * steering_error)) )
        #the 180 was a tunable constant. if you want the throttle to lower at a lower steering error,
        #this number needs to be reduced. If you want a less dramatic fall off, then the number needs to be icnreased. 
        #the absolute value is purely just to make sure it doesn't go negative. The nature of the equation prevents this too. THis is just a failsafe.         

        self.oldSError = steering_error 
        self.oldAError = distance_error
        #resave the current error values to the global error values to be reused in the next control loop. 
        
        if((throttle < 200) & (throttle > 0)):
           throttle = 200
        if(throttle > 580):
            throttle =580 
        #This is the upper and lower bounds. We restrict it to these values to prevent the 
        #the throttle command from shooting up to infanty when the steering error approaches zero.
        #this values are tuneable. if we want slower turns, we can set the 200 to about 180.
        #if we want faster straightline speed we can set the 580 to 600. 

        self.steerCmd = newSteering #save the steering command to the global steering command. 

        #publish to publishers. 
        self.accPub.publish(Float32(data=throttle))
        self.steerPub.publish(Float32(data=self.steerCmd))

    def control_loop(self): 
        # Main control loop 
        distance_error = 0
        steering_error = 0
        print("waiting on amcl") #wait on amcl to load. 
        while not self.current_pose:
            pass
        print('poses loaded')
        while not rospy.is_shutdown():  #while it's running, 
            
            steer = self.current_yaw() #get the current yaw.
            self.update_waypoint(distance_error, steering_error)  #check if we need to update the new waypoint. 
            # Implement PID control logic here to steer towards the desired waypoint
            desired_steer = self.desired_yaw()#get the desired steer
            drift = self.current_dev() #get the desired drift
            yaw_error = steer - desired_steer  #get the yaw error
            drift_error = drift - desired_steer #get the drift error. 
            steering_error = drift_error - yaw_error #add the errors
            distance_error = self.compute_distance_error()  #get the new distance
            
            self.controller(distance_error, steering_error) #send the errors to the controller. 
            
            # Sleep to maintain the loop rate
            self.rate.sleep()
         
if __name__ == '__main__': #main function
    map_follower = MapFollower() #init a map_follower object
    try: # run the controll loop. 
        map_follower.control_loop()
    except rospy.ROSInterruptException: #check for exceptions 
        pass

