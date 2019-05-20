#coding=utf-8
#! /usr/bin/env python
from flask import Flask,jsonify

import threading
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

app = Flask(__name__)

Position_x=0.0
Position_y=0.0
Guid_status=False 


'''

地址                      方法      说明                           请求                                             返回

"/"                       GET       连通测试                       无                                               string 
"/get_position"           GET       监听请求返回小车当前位置       无                                               {"status":string,"position_x":float,"position_y":float,"guid_status":bool}
"/send_goods_info"        POST      接收目标商品信息               {"position_x":float,"position_y":float}          {"status":string}
"/car_test"               GET       小车前进并直角转弯测试         无                                               {"status":string}

'''


@app.route("/", methods=['GET'])
def hello():
    return "hello world" 


@app.route("/get_position", methods=['GET'])
def send_position():
    result={"status":"Succeed!","position_x":Position_x,"position_y":Position_y,"guid_status":Guid_status}
    return jsonify(result)


@app.route("/send_goods_info",methods=['POST'])
def get_goods_info():
    global Guid_status
    Guid_status=True 
    result={"status":"Succeed!"}
    return jsonify(result)


@app.route("/car_test", methods=['GET'])
def car_test():
    #    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 1)
    twist = Twist()                                                                                                                                                                 

    twist.linear.x = 0.21; twist.linear.y = 0; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    Pub.publish(twist)

    time.sleep(5)

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.56428428648
    Pub.publish(twist)

    time.sleep(2)
    twist.linear.x = 0.21; twist.linear.y = 0; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    Pub.publish(twist)
    time.sleep(2.05)
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    Pub.publish(twist)
   
    result={"status":"Succeed!"}
   
    return jsonify(result)


#订阅里程记的回调函数
def callback(odom):   
    print "we got callback\n"
    print odom.pose.pose.position

    global Position_x
    global Position_y
    
    Position_x=odom.pose.pose.position.x
    Position_y=odom.pose.pose.position.y



Sub = rospy.Subscriber('odom',Odometry,callback)
print "we got odom\n"
Pub = rospy.Publisher('cmd_vel',Twist,queue_size = 1)     
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



