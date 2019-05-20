/******************************************************************
  基于串口通信的ROS小车基础控制器，功能如下：
  1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
  2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
  3.发布里程计主题/odm

  串口通信说明：
  1.写入串口
  （1）内容：左右轮速度，单位为mm/s
  （2）格式：１０字节,[右轮速度４字节][左轮速度４字节][结束符"\r\n"２字节]
  2.读取串口
  （1）内容：小车x,y坐标，方向角，线速度，角速度，单位依次为：mm,mm,rad,mm/s,rad/s
  （2）格式：２１字节，[Ｘ坐标４字节][Ｙ坐标４字节][方向角４字节][线速度４字节][角速度４字节][结束符"\n"１字节]
 *******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//以下为串口通讯需要的头文件
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <inttypes.h>
#include "serial/serial.h"
#include <stdlib.h>
#include "stdbool.h"
#include "string.h"
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sstream>
/****************************************************************************/
using namespace std;
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
//float D = 0.2680859f ;    //两轮间距，单位是m
float D = 0.5728f ;    //两轮间距，单位是m
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
float C = 681.725681f;              //主动轮周长
/****************************************************/
unsigned char data_terminal0=0x0d;  //“/r"字符
unsigned char data_terminal1=0x0a;  //“/n"字符
unsigned char speed_data[10]= {0};  //要发给串口的数据
string rec_buffer;  //串口数据接收变量

/***************************************************/
float wheel_interval= 333.0f;//    272.0f;        //  1.0146
//float wheel_interval=276.089f;    //轴距校正值=原轴距/0.987

float multiplier=9;           //倍频数
float deceleration_ratio=6.63;  //减速比
float wheel_diameter=217;     //轮子直径，单位mm
float pi_1_2=1.570796;          //π/2
float pi=3.141593;              //π
float pi_3_2=4.712389;          //π*3/2
float pi_2_1=6.283186;          //π*2
float dt=0.005;                 //采样时间间隔5ms
float line_number=70.0;       //码盘线数
float oriention_interval=0;  //dt时间内方向变化值

float sin_=0;        //角度计算值
float cos_=0;

float delta_distance=0,delta_oriention=0;   //采样时间间隔内运动的距离

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0;

float oriention_1=0;

//float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;

float odometry_cmd_right,odometry_cmd_left;

//int once=1;
//float odometry_right,odometry_left;
float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;


#define ExPand(a,b) for(i = 0 ; i < b ; i++) strcat(a->num,"0") ; a->dec+=b
typedef struct Real{
    char num[1000] ;
    int dec ;
}Real ;                                                                                                                                                                                                  
Real r1, r2, r3 ;
char s1[1000], s2[1000], ans[1000] ;
//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData //union的作用为实现char数组和float之间的转换
{
    float d;
    unsigned char data[4];
}right_speed_data,left_speed_data,vel_linear,vel_angular;
/************************************************************/


struct Result {
    float position_x;
    float position_y;
    float velocity_angular;
    float vel_linear;
};

float HtoD(char *a) { //16进制转10进制函数
    float num=0;
    int i;
    for(i=0; i<strlen(a); i++) {
        if(*(a+i)>='0'&&*(a+i)<='9') {
            num = num*16 + *(a+i)-'0';
        } else if(*(a+i)>='A'&&*(a+i)<='Z') {
            num = num*16 + *(a+i)-'A'+10;
        } else if(*(a+i)>='a'&&*(a+i)<='z') {
            num = num*16 + *(a+i)-'a'+10;
        } else {
            return -1;//表示输入错误的数
        }
    }
    return num;
}

float sudo(float b) { //速度计算函数
    float c;

    c = b*100/7000;
    b = c*200*pi;
    return b;
}

void ittoa(int i,char* string) {
    int power,j;
    j=i;
    for(power=1; j>=10; j/=10)
        power*=10;
    for(; power>0; power/=10) {
        *string++='0'+i/power;
        i%=power;
    }
    *string='\0';
}

int max(int a, int b){return a>b?a:b;}

void add(char a[], char b[], char c[])
{
    int aa, bb, len1 = strlen(a), len2 = strlen(b), len = max(len1, len2) ;
    int i, j, k, cc = 0 ;
    c[len+1] = '\0' ;
    for (i = len1-1, j =len2-1, k = len ; i >= 0 || j >= 0 ; i--, j--, k--)
    {
        aa = (i<0 ? 0 : a[i]-'0') ;
        bb = (j<0 ? 0 : b[j]-'0') ;
        c[k] = (aa+bb+cc)%10+'0';
        cc = (aa+bb+cc)/10 ;
    }
    if (cc != 0) c[0] = cc+'0' ;
    else strcpy(c, c+1) ;
}

void RealAdd(Real* a, Real* b, Real* c)
{
    int i ;
    if (a->dec < b->dec) {ExPand(a, b->dec-a->dec) ;}
    if (a->dec > b->dec) {ExPand(b, a->dec-b->dec) ;}
    c->dec = a->dec;
    add(a->num, b->num, c->num) ;
}

void Load(char s[], Real* real)
{
    int i, j, c = -1 ;
    for (i = 0, j = 0; s[i] ; i++)
        if (s[i] == '.') c++ ;
        else{
            real->num[j++] = s[i] ;
            if (c != -1) c++ ;
        }
    real->num[j++] = '\0' ;
    real->dec = max(c, 0) ;
    if (real->num[0] == '\0') strcpy (real->num, "0") ;
}

void Set (Real *real, char s[])
{
    int i, j ;
    int len = strlen(real->num) ;
    for (i = 0, j = -1 ; i < len-real->dec ; i++)
    {
        if (real->num[i] != '0' && j==-1) j = 0 ;
        if (j != -1) s[j++] = real->num[i] ;
    }
    if (j<=0) j = 0, s[j++] = '0' ;
    s[j++] = '.' ;
    for (i = len-real->dec ; i < len ; i++)
        s[j++] = real->num[i] ;
    s[j++] = '\0' ;
    j = strlen (s)-1 ;
    while (s[j] == '0') s[j--] = '\0' ;
    if (s[j] == '.') s[j] = '\0' ;
}

float highPrecisionAlgorithm(char s1[],char s2[]){
    //s1+s2=ans
    Load(s1, &r1) ;
    Load(s2, &r2) ;
    RealAdd(&r1,&r2,&r3) ;
    Set (&r3, ans) ;
    int ansSize = sizeof(ans)/sizeof(ans[0]);
    if(ans[0]=='*'){
        for(int i=0;i<=ansSize;i++){
            ans[i]=ans[i+1];
        }
    }
    double f = atof(ans);
    printf("%f\n",f);
    return f;
}

int customRoute(){
    string port("/dev/ttyUSB0");    //小车串口号
    //    string port("/dev/ttyS1");
    unsigned long baud = 115200;    //小车串口波特率
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口
    /*
       speed_data[0]=0xff;
       speed_data[1]=0xfe;
       speed_data[2]=0x15;
       speed_data[3]=0x15;
       speed_data[4]=0x01;
       speed_data[5]=0x01;
       speed_data[6]=0x00;
       speed_data[7]=0x00;
       speed_data[8]=0x00;
       speed_data[9]=0x00;

       my_serial.write(speed_data,10);
       sleep(2);
       */
    speed_data[0]=0xff;
    speed_data[1]=0xfe;
    speed_data[2]=0x15;
    speed_data[3]=0x15;
    speed_data[4]=0x00;
    speed_data[5]=0x01;
    speed_data[6]=0x00;
    speed_data[7]=0x00;
    speed_data[8]=0x00;
    speed_data[9]=0x00;
    my_serial.write(speed_data,10);

    usleep(2000000);
  /*  
    speed_data[0]=0xff;
    speed_data[1]=0xfe;
    speed_data[2]=0x15;
    speed_data[3]=0x15;
    speed_data[4]=0x01;
    speed_data[5]=0x01;
    speed_data[6]=0x00;
    speed_data[7]=0x00;
    speed_data[8]=0x00;
    speed_data[9]=0x00;
    my_serial.write(speed_data,10);

    sleep(2);
    */
    speed_data[0]=0xff;
    speed_data[1]=0xfe;
    speed_data[2]=0x00;
    speed_data[3]=0x00;
    speed_data[4]=0x01;
    speed_data[5]=0x01;
    speed_data[6]=0x00;
    speed_data[7]=0x00;
    speed_data[8]=0x00;
    speed_data[9]=0x00;
    my_serial.write(speed_data,10);
}

void callback(const geometry_msgs::Twist & cmd_input) { //订阅/cmd_vel主题回调函数
    printf("aaa\n");
	string port("/dev/ttyUSB0");    //小车串口号
//    string port("/dev/ttyS1");
	unsigned long baud = 115200;    //小车串口波特率
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //配置串口

    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s
   /*
    if(linear_temp>0){
        customRoute();
        return;
    }
    */
	printf("%f   ",angular_temp);
	printf("%f\n",linear_temp);

	//将转换好的小车速度分量为左右轮速度
	left_speed_data.d = linear_temp - 0.5f*angular_temp*D ;
	right_speed_data.d = linear_temp + 0.5f*angular_temp*D ;

	float left_speed_input_dec = left_speed_data.d*70/C;
	float right_speed_input_dec = right_speed_data.d*70/C;
    

	uint8_t left_speed_input_byte=0x00,right_speed_input_byte=0x00,left_direction_byte=0x00,right_direction_byte=0x00;
	//存入数据到要发布的左右轮速度消息
	left_speed_input_dec*=ratio;   //放大１０００倍，mm/s
	right_speed_input_dec*=ratio;//放大１０００倍，mm/s

    odometry_cmd_right = int(right_speed_input_dec);
    odometry_cmd_left = int(left_speed_input_dec);
    
    if(left_speed_input_dec<0){
        left_speed_input_dec = -left_speed_input_dec;
        left_direction_byte = 0x01;
    }
    if(right_speed_input_dec<0){
        right_speed_input_dec = -right_speed_input_dec;
        right_direction_byte = 0x01;
    }
	/********************************************************/
	printf("%f   ",left_speed_input_dec);    //目标速度
	printf("%f\n",right_speed_input_dec);
	/********************************************************/
    left_speed_input_byte = int(left_speed_input_dec);
    right_speed_input_byte = int(left_speed_input_dec);

    printf("%d   ",left_speed_input_byte);
	printf("%d\n",right_speed_input_byte);

	/*
	    for(int i=0;i<4;i++)    //将左右轮速度存入数组中发送给串口
	    {
	        speed_data[i]=right_speed_data.data[i];
	        speed_data[i+4]=left_speed_data.data[i];
	    }

	    //在写入串口的左右轮速度数据后加入”/r/n“
	    speed_data[8]=data_terminal0;
	    speed_data[9]=data_terminal1;
	*/

	speed_data[0]=0xff;
	speed_data[1]=0xfe;
	speed_data[2]=0x00;
	speed_data[3]=0x00;
    speed_data[4]=0x00;
    speed_data[5]=0x00;
	speed_data[6]=0x00;
	speed_data[7]=0x00;
	speed_data[8]=0x00;
	speed_data[9]=0x00;

	speed_data[2]=left_speed_input_byte;
	speed_data[3]=right_speed_input_byte;
	speed_data[4]=left_direction_byte;
	speed_data[5]=right_direction_byte;
	/*
	    if(angular_temp==0){
	        if(linear_temp==0){
	            speed_data[0]=0xff;
	            speed_data[1]=0xfe;
	            speed_data[2]=0x00;
	            speed_data[3]=0x00;
	            speed_data[4]=0x00;
	            speed_data[5]=0x00;
	            speed_data[6]=0x00;
	            speed_data[7]=0x00;
	            speed_data[8]=0x00;
	            speed_data[9]=0x00;
	        }else if(linear_temp==-0.5){
	            speed_data[0]=0xff;
	            speed_data[1]=0xfe;
	            speed_data[2]=0x10;
	            speed_data[3]=0x10;
	            speed_data[4]=0x00;
	            speed_data[5]=0x00;
	            speed_data[6]=0x00;
	            speed_data[7]=0x00;
	            speed_data[8]=0x00;
	            speed_data[9]=0x00;
	        }else if(linear_temp==0.5){
	            speed_data[0]=0xff;
	            speed_data[1]=0xfe;
	            speed_data[2]=0x10;
	            speed_data[3]=0x10;
	            speed_data[4]=0x01;
	            speed_data[5]=0x01;
	            speed_data[6]=0x00;
	            speed_data[7]=0x00;
	            speed_data[8]=0x00;
	            speed_data[9]=0x00;
	        }
	    }else if(angular_temp==-1){
	        if(linear_temp==0){
	            speed_data[0]=0xff;
	            speed_data[1]=0xfe;
	            speed_data[2]=0x10;
	            speed_data[3]=0x10;
	            speed_data[4]=0x01;
	            speed_data[5]=0x00;
	            speed_data[6]=0x00;
	            speed_data[7]=0x00;
	            speed_data[8]=0x00;
	            speed_data[9]=0x00;
	        }
	    }else if(angular_temp==1){
	        if(linear_temp==0){
	            speed_data[0]=0xff;
	            speed_data[1]=0xfe;
	            speed_data[2]=0x10;
	            speed_data[3]=0x10;
	            speed_data[4]=0x00;
	            speed_data[5]=0x01;
	            speed_data[6]=0x00;
	            speed_data[7]=0x00;
	            speed_data[8]=0x00;
	            speed_data[9]=0x00;
	        }
	    }
	*/

	/*
	    speed_data[0]=0xff;
	    speed_data[1]=0xfe;
	    speed_data[2]=0x10;
	    speed_data[3]=0x10;
	    speed_data[4]=0x00;
	    speed_data[5]=0x00;
	    speed_data[6]=0x00;
	    speed_data[7]=0x00;
	    speed_data[8]=0x00;
	    speed_data[9]=0x00;
	*/

	//写入数据到串口
	my_serial.write(speed_data,10);
}


Result odometry(float right,float left,float odometry_right,float odometry_left) {
    printf("=========================");
	printf("\n right:%.2f\n",right);
	printf("\n left:%.2f\n",left);
	Result ret;
    printf(" odometry_left:%f \n",odometry_left);
    printf(" odometry_right:%f  \n",odometry_right);
    float rAndL = right+left;
    /*
//调用转换函数进行转换
	//float bi,di;
	//float c =12;
	char *right_str = new char[20];
	char *left_str = new char[20];
	//printf("加载十六进制数");
	//cin>>a;
	ittoa(right,right_str);
	ittoa(left,left_str);
	//a[20]= c;
	printf("\n right_str:%s\n",right_str);
	printf("\n left_str:%s\n",left_str);
	float right_Dec= HtoD(right_str);
	float left_Dec= HtoD(left_str);
	if(right_Dec !=-1) {
		printf("\n right_Dec:%.2f\n",right_Dec);//cout<<"输入的数的十进制数为："<<num;
	} else {
		printf("\n right is wrong\n");
	}
	if(left_Dec !=-1) {
		printf("\n left_Dec:%.2f\n",left_Dec);//cout<<"输入的数的十进制数为："<<num;
	} else {
		printf("\n left is wrong\n");
	}
*/

	//转换成功
	//调用计算函数计算速度
	//right = sudo(right);
	//left = sudo(left);
	printf("\n right speed:%.2f\n",right);
	printf("\n left speed:%.2f\n",left);
    //计算成功

	//根据计算出的速度计算里程计
	const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);
    const_angle=const_frame/wheel_interval;



	distance_sum = 0.5f*(right+left);//在很短的时间内，小车行驶的路程为两轮速度和
	distance_diff = right-left;//在很短的时间内，小车行驶的角度为两轮速度差

	//根据左右轮的方向，纠正短时间内，小车行驶的路程和角度量的正负
	if((odometry_right>0)&&(odometry_left>0)) {          //左右均正
		delta_distance = distance_sum;
		delta_oriention = distance_diff;
	} else if((odometry_right<0)&&(odometry_left<0)) {   //左右均负
		delta_distance = -distance_sum;
		delta_oriention = -distance_diff;
	} else if((odometry_right<0)&&(odometry_left>0)) {//左正右负
		delta_distance = -distance_diff;
		delta_oriention = -2.0f*distance_sum;
	} else if((odometry_right>0)&&(odometry_left<0)) {   //左负右正
		delta_distance = distance_diff;
		delta_oriention = 2.0f*distance_sum;
	} else {
		delta_distance=0;
		delta_oriention=0;
	}

    char a[100],b[100];
    oriention_interval = delta_oriention * const_angle;//采样时间内走的角   
    //oriention_interval = (delta_oriention / wheel_interval)/deceleration_ratio;
    
    if(((odometry_right>0)&&(odometry_left>0))||((odometry_right<0)&&(odometry_left<0))){
        oriention_interval = 0;
    }
    
    oriention = oriention + oriention_interval;//计算出里程计方向角
   /*
    printf("\n oriention_interval:%f \n",oriention_interval);
    sprintf(a,"%f",oriention);
    sprintf(b,"%f",oriention_interval*0.5f);
    printf("\n oriention:%f \n",oriention);
    oriention_1 = highPrecisionAlgorithm(a,b);
    */
    oriention_1 = oriention + 0.5f * oriention_interval;//里程计方向角数据位数变化，用于三角函数计算

    printf("\n oriention_1:%f \n",oriention_1);

	sin_ = sin(oriention_1*5.81);//计算出采样时间内y坐标
	cos_ = cos(oriention_1*5.81);//计算出采样时间内x坐标

    printf("\n sin_:%f \n",sin_);
    printf("\n cos_:%f \n",cos_);

//	velocity_linear = delta_distance*const_frame / dt;//计算出里程计线速度
	velocity_linear = rAndL/2;
    velocity_angular = oriention_interval / dt;//计算出里程计角速度
    position_x = position_x + delta_distance * cos_ * const_frame;//计算出里程计x坐标
    position_y = position_y + delta_distance * sin_ * const_frame;//计算出里程计y坐标

	ret.position_x = position_x;
	ret.position_y = position_y;
    ret.vel_linear = velocity_linear;
	ret.velocity_angular = velocity_angular;

	//方向角角度纠正
	if(oriention > pi) {
		oriention -= pi_2_1;
	} else {
		if(oriention < -pi) {
			oriention += pi_2_1;
		}
	}
	printf("\n x:%.2f\n",ret.position_x);
	printf("\n y:%.2f\n",ret.position_y);
	printf("\n linear_speed:%.2f\n",velocity_linear);
	printf("\n angular_speed:%.2f\n",ret.velocity_angular);

	return ret;

}


int main(int argc, char **argv) {

	string port("/dev/ttyUSB0");//小车串口号
	unsigned long baud = 115200;//小车串口波特率
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口

	ros::init(argc, argv, "base_controller");//初始化串口节点
	ros::NodeHandle n;  //定义节点进程句柄

	ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //订阅/cmd_vel主题
	printf("We got /cmd_vel\n");

	ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 20); //定义要发布/odom主题
    ros::Publisher oriention_pub = n.advertise<std_msgs::Float64>("odom1",20);
	printf("We defined /odom\n");
    
	static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
	geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
	
    nav_msgs::Odometry odom;//定义里程计对象

    std_msgs::Float64 ori;

	geometry_msgs::Quaternion odom_quat; //四元数变量
	//定义covariance矩阵，作用为解决文职和速度的不同测量的不确定性
	float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
	                        0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
	                        0,  0,    99999, 0,     0,     0,  // covariance on gps_z
	                        0,  0,    0,     99999, 0,     0,  // large covariance on rot x
	                        0,  0,    0,     0,     99999, 0,  // large covariance on rot y
	                        0,  0,    0,     0,     0,     0.01
	                       };  // large covariance on rot z
	//载入covariance矩阵
	for(int i = 0; i < 36; i++) {
		odom.pose.covariance[i] = covariance[i];;
	}

	ros::Rate loop_rate(10);//设置周期休眠时间
	string rec_buffer_match;
	
		float x = 0.0;
		float y = 0.0;
		float th = 0.0;
	while(ros::ok()) {
//		printf("bbbb\n");
//		rec_buffer =my_serial.readline(12,"\n");    //获取串口发送来的数据
//		printf("%s\n",rec_buffer.c_str());
    	rec_buffer = my_serial.readline(2,"\n");
		while(rec_buffer[0]!=-1&&rec_buffer[1]!=-2) {
			rec_buffer_match = my_serial.readline(1,"\n");
			rec_buffer[0] = rec_buffer[1];
			rec_buffer[1] = rec_buffer_match[0]; 
		}
		rec_buffer =my_serial.readline(10,"\n");    //获取串口发送来的数据
		//实际速度
		
		//打印串口数据
        printf(" ");
		int i;
		for(i=0; i<10; i++) {
			printf("%d  ",rec_buffer[i]);
		}
		printf("\n");
        printf("=========================");
		
//----------------调用函数-----------------------------
		Result res;
		float odometry_right,odometry_left;
        float pos_x,pos_y,linear,angular;
        odometry_right = rec_buffer[2];
        odometry_left = rec_buffer[0];

		res = odometry(odometry_right,odometry_left,odometry_cmd_right,odometry_cmd_left);

        pos_x = res.position_x;

        pos_y = res.position_y;

        linear = res.vel_linear;

        angular = res.velocity_angular;
//-----------------------------------------------------
            //将X，Y坐标，线速度缩小1000倍
/*
            pos_x/=1000; //m

            pos_y/=1000; //m

            linear/=1000; //m/s
*/
            ori.data = oriention_1;

            //里程计的偏航角需要转换成四元数才能发布

            odom_quat = tf::createQuaternionMsgFromYaw(oriention);//将偏航角转换成四元数


            //载入坐标（tf）变换时间戳

            odom_trans.header.stamp = ros::Time::now();

            //发布坐标变换的父子坐标系

            odom_trans.header.frame_id = "odom";     

            odom_trans.child_frame_id = "base_footprint";       

            //tf位置数据：x,y,z,方向

            odom_trans.transform.translation.x = pos_x;

            odom_trans.transform.translation.y = pos_y;

            odom_trans.transform.translation.z = 0.0;

            odom_trans.transform.rotation = odom_quat;        

            //发布tf坐标变化

            odom_broadcaster.sendTransform(odom_trans);



            //载入里程计时间戳

            odom.header.stamp = ros::Time::now(); 

            //里程计的父子坐标系

            odom.header.frame_id = "odom";

            odom.child_frame_id = "base_footprint";       

            //里程计位置数据：x,y,z,方向

            odom.pose.pose.position.x = pos_x;     

            odom.pose.pose.position.y = pos_y;

            odom.pose.pose.position.z = 0.0;

            odom.pose.pose.orientation = odom_quat;       

            //载入线速度和角速度

            odom.twist.twist.linear.x = linear;

            //odom.twist.twist.linear.y = odom_vy;

            odom.twist.twist.angular.z = angular; 

            //发布里程计
            oriention_pub.publish(ori);
            odom_pub.publish(odom);
            ros::spinOnce();//周期执行
            loop_rate.sleep();//周期休眠
            //程序周期性调用

            ros::spinOnce();  //callback函数必须处理所有问题时，才可以用到
    }
    return 0;
}
