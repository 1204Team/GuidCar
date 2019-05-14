#! /usr/bin/env python
from flask import Flask,jsonify

import threading
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

app = Flask(__name__)

position_x=0.0
position_y=0.0


@app.route("/", methods=['GET'])
def hello():
    return "hello world" 


@app.route("/get_position", methods=['GET'])
def get_position():
    result={"position_x":position_x,"position_y":position_y}
    return jsonify(result)


@app.route("/car_test", methods=['GET'])
def car_test():
    #    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 1)
    twist = Twist()                                                                                                                                                                 

    twist.linear.x = 0.21; twist.linear.y = 0; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

    time.sleep(5)

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.56428428648
    pub.publish(twist)

    time.sleep(2.2)
    twist.linear.x = 0.21; twist.linear.y = 0; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    time.sleep(2)
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    return "Succeed!"

def callback(odom):   
    print "we got callback\n"
    print odom.pose.pose.position

    global position_x 
    global position_y 

    position_x=odom.pose.pose.position.x
    position_y=odom.pose.pose.position.y

'''
def ros_init():
    #    rospy.init_node('Web_server')
    #    sub = rospy.Subscriber('odom',Odometry,callback)
    #    while(1):
        server_start()
        #    rospy.spin()
        '''

sub = rospy.Subscriber('odom',Odometry,callback)
print "we got odom\n"
pub = rospy.Publisher('cmd_vel',Twist,queue_size = 1)     
print "we got cmd_vel\n"

if __name__ == "__main__":
    #    ros_init()
    rospy.init_node('Web_server')
    print "we got Web_server\n"
    #    server_start()

    print "we are online"
    #    app.debug = True
    app.run(host='0.0.0.0',port =8080)
    #    app.run()

#    print "aaa\n"



