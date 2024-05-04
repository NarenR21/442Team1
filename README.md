# 442Team1

Team Max ROStappen
MFET 442
Naren Ram
Aditya Raj Gupta
Cam Johnson

Assignment.1:

Image in RVIZ with laser scan and dudley map.
![image](https://github.com/NarenR21/442Team1/assets/90937234/00d6544a-9ddf-4275-909b-44098f74aa1f)

Assignment.2:

Waypoint for Dudley in RVIZ:
![image](https://github.com/NarenR21/442Team1/assets/73058520/7541a49b-870c-443b-b205-569a4c53e5d4)

Assignment.3

yaw_error = Theta - desired_theta 
drift_error = arctan((Yd - y) / (Xd- x)) - desired_theta

steering_error = drift_error - yaw_error



Software Description and Explanation:

Launch Files:

For this lab we developed several launch files and python files to succesffuly develop the autonomous race car.  The first file that we created was the map_follower launch file.  This is the main launch file that runs the ROS master and initalizes all of the neccessary nodes the car will rely on.  One of the most important launch files that map_follower.launch runs is the car hardware launch file.  This is the launch file that does the startup of the car and publishes the topics from the car to the ROS Master.  Some of these topics include, lidar scans, IMU data, steering velocity, accelerator velocity.  Once the launch file successfully launches the car hardware launch files it another launch file called map.launch.  The map.launch file finds a map inside of the directory and publishes that to the ROS master.  Then a node is launched called AMCL and this node will determine the position with respect to the map.  This is one of the most important aspects of the car because it localizes the car with respect to the map.  Now the car can determine where it is on the map and if there are any obstacles nearby.

Waypoint.py

Once all of the launch files have successfully initialized every node, the car knows it position in the map.  Now it needs to determine where to go.  To achieve this we use a waypoint system.  We desgined a python script that would record x_position, y_position and angle of points in the map.  When using RViz the user can place waypoints on the map and the python script will publish them and save them to a file.  This can be seen in assignment two.  In assignment two the team needed to develop a series of waypoints that are connected together.  Once the team developed the series of waypoints this file could be use later to run waypoint following.

map_follower.py
The main script used to run the autonomous algorithem was map_follower.py. As the name suggests, map_follower.py takes a text file of waypoints and outputs steering and accelerator commands to control the motion of the car. Our first challange was to establish the location of the car in the map space. This was done through Adaptive Monte Carlo Localization, or amcl. This subroutine utilizes the onboard LIDAR to get a scan of the area around it. This is then run through a monte carlo analysis which is based upon the mean, standard deviation, and other such stastical factors to determine the most likly location and orientation. with this information, we're able to understand where in the map space the car is. Now, we need to decide where the car should be. The waypoint described in waypoints.py are in the textfile that is fed to the map_follower. With this information, we're now able to calculate our theta, and our delta to our desired waypoint from our current location.  The theta is the error in yaw. This is calculated by finding the difference in our current yaw, and our desired yaw. Current yaw is calculated by transforming the orientation from amcl into a euluer rotation and getting the yaw, or Z axis, component. Then we calculate the delta error. This is our drift error and represents how far off in x and y we are from our point. To do this, we calculate another theta that is based off the arc tangent of the difference in x and y from our current location to our desired location. Esentially, how much do we need to steer to go where we want to be going. We calculate all of the above errors in the map context so we're able to add the errors into a total error. With this total error, we pass it along to the PD controller. 
The PD controller does a significant portion of the control theory work. It works of the standard PD equation, input = (kPosition * positionError) + (kDerivative * deriviativeError). Where k is a constant that is definied in the software and the error is defined as the current - desired. For the derivative, we don't want a ton of change, so the desired derivative is 0. 
This control logic is used for the both steering and accelerator inputs. For the steering, we pipe the error described above directly into the equation. The constants, k, were carefully tuned to achieve the optimal preformance. More time and testing, would yield better constants, and changes to the max speed would require retuning as well. However, they're reasonablly accurate for our desired preformance treshold. 
The accelarator command is also derived this way however, instead of a theta error the error is found through the distance to the next waypoint. The distance is found through the hypontuse of the delta in x and the delta in y to our desired waypoint from our current waypoint. As this error gets smaller, we slow down. However, in order to aid in turning, we added a filter onto the previously mentioned PD controller. WE remap the PD controller's output through a k / x ^ 2 equation. Where k is a defined constant that is tuneable, and x is the steering error. This allows us to slow down aggresively when not only the distance small, but also when we need to turn a lot. During the straights, the steering error is less then 15, but as we approach the turn, it sharply increases. The shape of the equation, shown below, demonstrates how this higher steering error will signifanctly slow the car. Through this equation, we can slow down into a turn, rotate through it, and then quickly get back on the power into the next straight section. It's significantly helped our times, and some post-race unoffical testing showed that at highspeeds the car actually powerslides through the turn and is accelerating quite close to the apex. 
This is the esence of our PD controller. The entire loop is run again 10 times a second, allowing us to get very detailed control of the car. 

Graph of the throttle map:

![image](https://github.com/NarenR21/442Team1/assets/42756261/5da75789-f8e9-4232-b39c-f7dd588460c7)



Debugging Errors:

Once we had finished writing the scripts map_follower.py and waypoint.py we needed to test the system.  To start we worked in the Dudley hallway to ensure that the autonomous racecar could go down a straight hallways and follow points.  One of the main issues we encountered was that the AMCL node was not localizing fast enough.  We were expecting the system to localize at a frequency of 20 Hz but in reality it was localizing at a frequency of 1 Hz.  A temporary fix that we used was to rely on the IMU for position and angle.  This was done to test the map_follower.py script and to tune the PID controller.  This system worked reasonably well and did follow waypoints in the straight hallways.  However, when we moved on to the Knoy racetrack we were having significant issues with steering the corners.  We decided to go back to AMCL and figure out why it was running so slow.  The main reason why it was running slow was we were setting too many variables in the AMCL callback.  The read and write speeds of the Nvidia TX2 was lower than we expected so we did not think that would have caused an issue.  Once we fixed the localization issue with the AMCL node we were able to steer and follow waypoints very well.  Another issue we were facing was that the map_follower.py was relying on the angle that was provided by the waypoint and trying to steer to that angle.  The problem is that this only ensures that the racecar is parallel to the waypoint, not actually at the waypoint. To fix this we modified the map_follower.py script to determine the needed angle to be at the designated waypoint.  Now the car was steering towards the waypoint, not just parallel to it.  Finally, our last main issue was updating the waypoints when following the waypoints.  Initially we had the algorithm update to the next waypoint when the car was within 0.2m distance away from the waypoint.  The issue with this is that sometimes the car would never hit some waypoints and would try to turn in a circle to hit the waypoint.  At first, we did not know why the car was steering in circles until we printed the current waypoint it was trying to hit.  To solve this we simply increased the distance it needed to be away to update to the next waypoint.  That being said, because the car was steering earlier and accelerating earlier we had to update the gains on the PID controller to account for this.

Final offical race time: 44 seconds

