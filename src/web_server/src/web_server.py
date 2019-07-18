#coding=utf-8
#! /usr/bin/env python

import threading
import time
import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from flask import Flask,jsonify,json,request 

app = Flask(__name__)

Position_x=0.0
Position_y=0.0
Guid_status=False 


'''

地址                      方法      说明                           请求                                                   返回

"/"                       GET       连通测试                       无                                                     string 
"/get_position"           GET       监听请求返回小车当前位置       无                                                     {"status":string,"position_x":float,"position_y":float,"guid_status":bool}
"/send_goods_info"        POST      接收目标商品信息               {"position_x":float,"position_y":float}                {"status":string,"position_x":float,"position_y":float}
"/stop_guid"              GET       停止当前导航                   无                                                     {"status":string}
"/car_test"               GET       小车前进并直角转弯测试         无                                                     {"status":string}
"/controller"             POST      小车控制                       {"x":float,"y":float}                                  {"status":string,"x":float,"y":float}
"/start_following"        POST      获取用户端选框位置             {"x":float,"y:float","width":float,"heigh:float"}      {"status":string}
"/stop_following"         GET       停止人体跟踪                   无                                                     {"status":string}

'''


@app.route("/", methods=['GET'])
def hello():
    return "hello world" 


@app.route("/get_position", methods=['GET'])
def send_position():
    result = {"status":"Succeed!","position_x":Position_x,"position_y":Position_y,"guid_status":Guid_status}
    return jsonify(result)


@app.route("/send_goods_info", methods=['POST'])
def get_goods_info():
    global Guid_status
    Guid_status = True 
    
    data = json.loads(request.get_data())
    target_x = data['position_x']
    target_y = data['position_y']
    
    move_base_client(target_x,target_y)

    result = {"status":"Succeed!","position_x":target_x,"position_y":target_y}
    
    Guid_status = False 

    return jsonify(result)


@app.route("/stop_guid", methods=['GET'])
def stop_guid():
    global Guid_status
    Guid_status = False 
    
    goal = GoalID()     
    CancelGoalPub.publish(goal)

    result = {"status":"Succeed!"}
    return jsonify(result)


@app.route("/car_test", methods=['GET'])
def car_test():
    global Guid_status
    Guid_status = True 
   
    #    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 1)
    twist = Twist()                                                                                                                                                                 

    twist.linear.x = 0.21; twist.linear.y = 0; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    CmdVelPub.publish(twist)

    time.sleep(5)

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.56428428648
    CmdVelPub.publish(twist)

    time.sleep(2)
    twist.linear.x = 0.21; twist.linear.y = 0; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    CmdVelPub.publish(twist)

    time.sleep(2.05)
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    CmdVelPub.publish(twist)

    result = {"status":"Succeed!"}
   
    Guid_status = False 
   
    return jsonify(result)


@app.route("/controller", methods=['POST'])
def controller():
    twist = Twist()                                                                                                                                                                 
    data = json.loads(request.get_data())
    #    print(data)
    x = data["x"]
    y = data["y"]
    
    twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = y
    CmdVelPub.publish(twist)
    
    result = {"status":"Succeed!","x":x,"y":y}
    return jsonify(result)


@app.route("/start_following", methods=['POST'])
def start_following():
    twist = Twist()  

    twist.linear.x = 540; twist.linear.y = 320; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 120; twist.angular.y = 150; twist.angular.z = 0
    OnMousePub.publish(twist)
    
    result = {"status":"Succeed!"}
    return jsonify(result)


@app.route("/stop_following", methods=['GET'])
def stop_following():
    twist = Twist()  

    twist.linear.x = 1; twist.linear.y = 1; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 1; twist.angular.y = 1; twist.angular.z = -1
    OnMousePub.publish(twist)
    
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;                                                                                                                                       
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    CmdVelPub.publish(twist)
    
    result = {"status":"Succeed!"}
    return jsonify(result)


#向movw_base发送目标坐标
def move_base_client(target_x,target_y):
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    rospy.loginfo("Waiting for move_base action server...")
    
    # Wait 60 seconds for the action server to become available
    # 等待move_base服务器建立
    move_base.wait_for_server(rospy.Duration(60))

    rospy.loginfo("Connected to move base server")
    rospy.loginfo("Starting navigation test")

    goal = MoveBaseGoal()
    
    goal.target_pose.header.frame_id = 'odom'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    goal.target_pose.pose.orientation.w = 1.0

    # Send the goal pose to the MoveBaseAction server
    # 把目标位置发送给MoveBaseAction的服务器
    move_base.send_goal(goal)

    # Allow 1 minute to get there
    # 设定1分钟的时间限制
    finished_within_time = move_base.wait_for_result(rospy.Duration(60)) 

 

#订阅里程记的回调函数
def callback(odom):   
    print "we got callback\n"
    print odom.pose.pose.position

    global Position_x
    global Position_y

    Position_x = odom.pose.pose.position.x
    Position_y = odom.pose.pose.position.y



OdomSub = rospy.Subscriber('odom',Odometry,callback)
print "we got odom\n"

CmdVelPub = rospy.Publisher('cmd_vel',Twist,queue_size = 1)     
print "we got cmd_vel\n"

CancelGoalPub = rospy.Publisher('move_base/cancel',GoalID,queue_size = 1)     
print "we got cancel_goal\n"

OnMousePub = rospy.Publisher('OnMouse',Twist,queue_size = 1)     
print "we got OnMouse\n"

if __name__ == "__main__":
    #    ros_init()
    rospy.init_node('Web_server')
    print "we got Web_server\n"
    #    server_start()
    print "we are online"
    #    app.debug = True
    app.run(host = '0.0.0.0',port = 8080)
    #    app.run()

#    print "aaa\n"



