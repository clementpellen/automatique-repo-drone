#!/usr/bin/env python

import rospy
import math
from mav_msgs.msg import RollPitchYawrateThrust
from nav_msgs.msg import Odometry

# Global variables
x = 0.0
y = 0.0
z = 0.0

vx = 0.0
vy = 0.0
vz = 0.0

T = 0.0 # thrust
theta = 0.0 # pitch
phi = 0.0 # roll
yaw_rate = 0.0 # rad/s

alpha = 1.0
beta = 2.0

xp = 0.5
yp = 1.0
zp = 1

kp = alpha * beta
kv = alpha + beta
kpsi = 1.0

wanted_orientation = 0.0

mass = 1.544

def callback_odom(data):    
    #process odometry and compute control here
    global x, y, z, vx, vy, vz, T, theta, phi, yaw_rate
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    # velocity
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z
    # orientation
    psi = math.atan2(2*(data.pose.pose.orientation.w*data.pose.pose.orientation.z), 1 - data.pose.pose.orientation.z**2)
    yaw_rate = kpsi * (wanted_orientation - psi)

    # compute ux, uy, uz
    ux = -kp*(x - xp) - kv*(vx)
    uy = -kp*(y - yp) - kv*(vy)
    uz = -kp*(z - zp) - kv*(vz)

    # compute T, theta, phi
    T = mass*(uz + 9.81)
    theta = (mass/T)*(math.cos(psi)*ux + math.sin(psi)*uy)
    phi = (mass/T)*(-math.sin(psi)*ux - math.cos(psi)*uy)





def drone_control():
    global T, theta, phi, yaw_rate
    rospy.init_node('drone_control', anonymous=True)
    pub = rospy.Publisher('/firefly/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=10)
    sub = rospy.Subscriber('/firefly/odometry_sensor1/odometry',Odometry,callback_odom)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = RollPitchYawrateThrust()
        # msg.roll = 0.0	   		# rad	
        # msg.pitch = 0.0		# rad	
        # msg.yaw_rate = 0.0  		# rad/s
        # mass = 1.544
        # msg.thrust.z = mass*(0.0 + 9.81) # m*(g + acc)

        msg.roll = phi
        msg.pitch = theta
        msg.yaw_rate = yaw_rate
        msg.thrust.z = T

        # Compute corrector
        # u = (T/m) * Re3 - ge3
        Re3 = theta
        

        pub.publish(msg)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        drone_control()
    except rospy.ROSInterruptException:
        pass
