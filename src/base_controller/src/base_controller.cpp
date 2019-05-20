/******************************************************************
  ���ڴ���ͨ�ŵ�ROSС���������������������£�
  1.ʵ��ros��������ͨ���̶��ĸ�ʽ�ʹ���ͨ�ţ��Ӷ��ﵽ����С�����ƶ�
  2.������/cmd_vel���⣬ֻҪ������ⷢ����Ϣ������ʵ�ֶԿ���С�����ƶ�
  3.������̼�����/odm

  ����ͨ��˵����
  1.д�봮��
  ��1�����ݣ��������ٶȣ���λΪmm/s
  ��2����ʽ�������ֽ�,[�����ٶȣ��ֽ�][�����ٶȣ��ֽ�][������"\r\n"���ֽ�]
  2.��ȡ����
  ��1�����ݣ�С��x,y���꣬����ǣ����ٶȣ����ٶȣ���λ����Ϊ��mm,mm,rad,mm/s,rad/s
  ��2����ʽ�������ֽڣ�[�����ꣴ�ֽ�][�����ꣴ�ֽ�][����ǣ��ֽ�][���ٶȣ��ֽ�][���ٶȣ��ֽ�][������"\n"���ֽ�]
 *******************************************************************/
#include "ros/ros.h"  //ros��Ҫ��ͷ�ļ�
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//����Ϊ����ͨѶ��Ҫ��ͷ�ļ�
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
float ratio = 1000.0f ;   //ת��ת��������ִ���ٶȵ�������
//float D = 0.2680859f ;    //���ּ�࣬��λ��m
float D = 0.5728f ;    //���ּ�࣬��λ��m
float linear_temp=0,angular_temp=0;//�ݴ�����ٶȺͽ��ٶ�
float C = 681.725681f;              //�������ܳ�
/****************************************************/
unsigned char data_terminal0=0x0d;  //��/r"�ַ�
unsigned char data_terminal1=0x0a;  //��/n"�ַ�
unsigned char speed_data[10]= {0};  //Ҫ�������ڵ�����
string rec_buffer;  //�������ݽ��ձ���

/***************************************************/
float wheel_interval= 333.0f;//    272.0f;        //  1.0146
//float wheel_interval=276.089f;    //���У��ֵ=ԭ���/0.987

float multiplier=9;           //��Ƶ��
float deceleration_ratio=6.63;  //���ٱ�
float wheel_diameter=217;     //����ֱ������λmm
float pi_1_2=1.570796;          //��/2
float pi=3.141593;              //��
float pi_3_2=4.712389;          //��*3/2
float pi_2_1=6.283186;          //��*2
float dt=0.005;                 //����ʱ����5ms
float line_number=70.0;       //��������
float oriention_interval=0;  //dtʱ���ڷ���仯ֵ

float sin_=0;        //�Ƕȼ���ֵ
float cos_=0;

float delta_distance=0,delta_oriention=0;   //����ʱ�������˶��ľ���

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
//���͸���λ�����������ٶȣ���̼Ƶ�����ͷ���
union floatData //union������Ϊʵ��char�����float֮���ת��
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

float HtoD(char *a) { //16����ת10���ƺ���
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
            return -1;//��ʾ����������
        }
    }
    return num;
}

float sudo(float b) { //�ٶȼ��㺯��
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
    string port("/dev/ttyUSB0");    //С�����ں�
    //    string port("/dev/ttyS1");
    unsigned long baud = 115200;    //С�����ڲ�����
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //���ô���
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

void callback(const geometry_msgs::Twist & cmd_input) { //����/cmd_vel����ص�����
    printf("aaa\n");
	string port("/dev/ttyUSB0");    //С�����ں�
//    string port("/dev/ttyS1");
	unsigned long baud = 115200;    //С�����ڲ�����
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); //���ô���

    angular_temp = cmd_input.angular.z ;//��ȡ/cmd_vel�Ľ��ٶ�,rad/s
    linear_temp = cmd_input.linear.x ;//��ȡ/cmd_vel�����ٶ�.m/s
   /*
    if(linear_temp>0){
        customRoute();
        return;
    }
    */
	printf("%f   ",angular_temp);
	printf("%f\n",linear_temp);

	//��ת���õ�С���ٶȷ���Ϊ�������ٶ�
	left_speed_data.d = linear_temp - 0.5f*angular_temp*D ;
	right_speed_data.d = linear_temp + 0.5f*angular_temp*D ;

	float left_speed_input_dec = left_speed_data.d*70/C;
	float right_speed_input_dec = right_speed_data.d*70/C;
    

	uint8_t left_speed_input_byte=0x00,right_speed_input_byte=0x00,left_direction_byte=0x00,right_direction_byte=0x00;
	//�������ݵ�Ҫ�������������ٶ���Ϣ
	left_speed_input_dec*=ratio;   //�Ŵ󣱣���������mm/s
	right_speed_input_dec*=ratio;//�Ŵ󣱣���������mm/s

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
	printf("%f   ",left_speed_input_dec);    //Ŀ���ٶ�
	printf("%f\n",right_speed_input_dec);
	/********************************************************/
    left_speed_input_byte = int(left_speed_input_dec);
    right_speed_input_byte = int(left_speed_input_dec);

    printf("%d   ",left_speed_input_byte);
	printf("%d\n",right_speed_input_byte);

	/*
	    for(int i=0;i<4;i++)    //���������ٶȴ��������з��͸�����
	    {
	        speed_data[i]=right_speed_data.data[i];
	        speed_data[i+4]=left_speed_data.data[i];
	    }

	    //��д�봮�ڵ��������ٶ����ݺ���롱/r/n��
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

	//д�����ݵ�����
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
//����ת����������ת��
	//float bi,di;
	//float c =12;
	char *right_str = new char[20];
	char *left_str = new char[20];
	//printf("����ʮ��������");
	//cin>>a;
	ittoa(right,right_str);
	ittoa(left,left_str);
	//a[20]= c;
	printf("\n right_str:%s\n",right_str);
	printf("\n left_str:%s\n",left_str);
	float right_Dec= HtoD(right_str);
	float left_Dec= HtoD(left_str);
	if(right_Dec !=-1) {
		printf("\n right_Dec:%.2f\n",right_Dec);//cout<<"���������ʮ������Ϊ��"<<num;
	} else {
		printf("\n right is wrong\n");
	}
	if(left_Dec !=-1) {
		printf("\n left_Dec:%.2f\n",left_Dec);//cout<<"���������ʮ������Ϊ��"<<num;
	} else {
		printf("\n left is wrong\n");
	}
*/

	//ת���ɹ�
	//���ü��㺯�������ٶ�
	//right = sudo(right);
	//left = sudo(left);
	printf("\n right speed:%.2f\n",right);
	printf("\n left speed:%.2f\n",left);
    //����ɹ�

	//���ݼ�������ٶȼ�����̼�
	const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);
    const_angle=const_frame/wheel_interval;



	distance_sum = 0.5f*(right+left);//�ں̵ܶ�ʱ���ڣ�С����ʻ��·��Ϊ�����ٶȺ�
	distance_diff = right-left;//�ں̵ܶ�ʱ���ڣ�С����ʻ�ĽǶ�Ϊ�����ٶȲ�

	//���������ֵķ��򣬾�����ʱ���ڣ�С����ʻ��·�̺ͽǶ���������
	if((odometry_right>0)&&(odometry_left>0)) {          //���Ҿ���
		delta_distance = distance_sum;
		delta_oriention = distance_diff;
	} else if((odometry_right<0)&&(odometry_left<0)) {   //���Ҿ���
		delta_distance = -distance_sum;
		delta_oriention = -distance_diff;
	} else if((odometry_right<0)&&(odometry_left>0)) {//�����Ҹ�
		delta_distance = -distance_diff;
		delta_oriention = -2.0f*distance_sum;
	} else if((odometry_right>0)&&(odometry_left<0)) {   //������
		delta_distance = distance_diff;
		delta_oriention = 2.0f*distance_sum;
	} else {
		delta_distance=0;
		delta_oriention=0;
	}

    char a[100],b[100];
    oriention_interval = delta_oriention * const_angle;//����ʱ�����ߵĽ�   
    //oriention_interval = (delta_oriention / wheel_interval)/deceleration_ratio;
    
    if(((odometry_right>0)&&(odometry_left>0))||((odometry_right<0)&&(odometry_left<0))){
        oriention_interval = 0;
    }
    
    oriention = oriention + oriention_interval;//�������̼Ʒ����
   /*
    printf("\n oriention_interval:%f \n",oriention_interval);
    sprintf(a,"%f",oriention);
    sprintf(b,"%f",oriention_interval*0.5f);
    printf("\n oriention:%f \n",oriention);
    oriention_1 = highPrecisionAlgorithm(a,b);
    */
    oriention_1 = oriention + 0.5f * oriention_interval;//��̼Ʒ��������λ���仯���������Ǻ�������

    printf("\n oriention_1:%f \n",oriention_1);

	sin_ = sin(oriention_1*5.81);//���������ʱ����y����
	cos_ = cos(oriention_1*5.81);//���������ʱ����x����

    printf("\n sin_:%f \n",sin_);
    printf("\n cos_:%f \n",cos_);

//	velocity_linear = delta_distance*const_frame / dt;//�������̼����ٶ�
	velocity_linear = rAndL/2;
    velocity_angular = oriention_interval / dt;//�������̼ƽ��ٶ�
    position_x = position_x + delta_distance * cos_ * const_frame;//�������̼�x����
    position_y = position_y + delta_distance * sin_ * const_frame;//�������̼�y����

	ret.position_x = position_x;
	ret.position_y = position_y;
    ret.vel_linear = velocity_linear;
	ret.velocity_angular = velocity_angular;

	//����ǽǶȾ���
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

	string port("/dev/ttyUSB0");//С�����ں�
	unsigned long baud = 115200;//С�����ڲ�����
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//���ô���

	ros::init(argc, argv, "base_controller");//��ʼ�����ڽڵ�
	ros::NodeHandle n;  //����ڵ���̾��

	ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //����/cmd_vel����
	printf("We got /cmd_vel\n");

	ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 20); //����Ҫ����/odom����
    ros::Publisher oriention_pub = n.advertise<std_msgs::Float64>("odom1",20);
	printf("We defined /odom\n");
    
	static tf::TransformBroadcaster odom_broadcaster;//����tf����
	geometry_msgs::TransformStamped odom_trans;//����һ��tf������Ҫʹ�õ�TransformStamped������Ϣ
	
    nav_msgs::Odometry odom;//������̼ƶ���

    std_msgs::Float64 ori;

	geometry_msgs::Quaternion odom_quat; //��Ԫ������
	//����covariance��������Ϊ�����ְ���ٶȵĲ�ͬ�����Ĳ�ȷ����
	float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
	                        0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
	                        0,  0,    99999, 0,     0,     0,  // covariance on gps_z
	                        0,  0,    0,     99999, 0,     0,  // large covariance on rot x
	                        0,  0,    0,     0,     99999, 0,  // large covariance on rot y
	                        0,  0,    0,     0,     0,     0.01
	                       };  // large covariance on rot z
	//����covariance����
	for(int i = 0; i < 36; i++) {
		odom.pose.covariance[i] = covariance[i];;
	}

	ros::Rate loop_rate(10);//������������ʱ��
	string rec_buffer_match;
	
		float x = 0.0;
		float y = 0.0;
		float th = 0.0;
	while(ros::ok()) {
//		printf("bbbb\n");
//		rec_buffer =my_serial.readline(12,"\n");    //��ȡ���ڷ�����������
//		printf("%s\n",rec_buffer.c_str());
    	rec_buffer = my_serial.readline(2,"\n");
		while(rec_buffer[0]!=-1&&rec_buffer[1]!=-2) {
			rec_buffer_match = my_serial.readline(1,"\n");
			rec_buffer[0] = rec_buffer[1];
			rec_buffer[1] = rec_buffer_match[0]; 
		}
		rec_buffer =my_serial.readline(10,"\n");    //��ȡ���ڷ�����������
		//ʵ���ٶ�
		
		//��ӡ��������
        printf(" ");
		int i;
		for(i=0; i<10; i++) {
			printf("%d  ",rec_buffer[i]);
		}
		printf("\n");
        printf("=========================");
		
//----------------���ú���-----------------------------
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
            //��X��Y���꣬���ٶ���С1000��
/*
            pos_x/=1000; //m

            pos_y/=1000; //m

            linear/=1000; //m/s
*/
            ori.data = oriention_1;

            //��̼Ƶ�ƫ������Ҫת������Ԫ�����ܷ���

            odom_quat = tf::createQuaternionMsgFromYaw(oriention);//��ƫ����ת������Ԫ��


            //�������꣨tf���任ʱ���

            odom_trans.header.stamp = ros::Time::now();

            //��������任�ĸ�������ϵ

            odom_trans.header.frame_id = "odom";     

            odom_trans.child_frame_id = "base_footprint";       

            //tfλ�����ݣ�x,y,z,����

            odom_trans.transform.translation.x = pos_x;

            odom_trans.transform.translation.y = pos_y;

            odom_trans.transform.translation.z = 0.0;

            odom_trans.transform.rotation = odom_quat;        

            //����tf����仯

            odom_broadcaster.sendTransform(odom_trans);



            //������̼�ʱ���

            odom.header.stamp = ros::Time::now(); 

            //��̼Ƶĸ�������ϵ

            odom.header.frame_id = "odom";

            odom.child_frame_id = "base_footprint";       

            //��̼�λ�����ݣ�x,y,z,����

            odom.pose.pose.position.x = pos_x;     

            odom.pose.pose.position.y = pos_y;

            odom.pose.pose.position.z = 0.0;

            odom.pose.pose.orientation = odom_quat;       

            //�������ٶȺͽ��ٶ�

            odom.twist.twist.linear.x = linear;

            //odom.twist.twist.linear.y = odom_vy;

            odom.twist.twist.angular.z = angular; 

            //������̼�
            oriention_pub.publish(ori);
            odom_pub.publish(odom);
            ros::spinOnce();//����ִ��
            loop_rate.sleep();//��������
            //���������Ե���

            ros::spinOnce();  //callback�������봦����������ʱ���ſ����õ�
    }
    return 0;
}
