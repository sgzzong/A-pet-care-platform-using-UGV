// node에 관련된 모든 정보
#include "ros/ros.h"       
 
// 메세지 타입 정보를 가지고 있는 헤더
#include "std_msgs/String.h"
#include <sstream>
 
int main( int argc, char **argv )
{
    //-- node 초기화
    ros::init( argc, argv, "example1_a" );
   
    //-- 프로세스 핸들러 추가  
    ros::NodeHandle n;
   
    //-- 마스터한테 토픽 이름과 타입을 전달
    //   message : 토픽 이름
    //   1000    : 버퍼 사이즈
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("message", 1000);
   
    //-- 데이터 전송 주기 10 Hz
    ros::Rate loop_rate(10);
   
   
    while( ros::ok() )
    {
        //-- 메세지 변수 선언 및 값 설정
        std_msgs::String msg;
        std::stringstream ss;
       
        ss << "I am te example1_a node ";
        msg.data = ss.str();
       
        //-- 메세지 전송
        chatter_pub.publish(msg);
       
        ros::spinOnce();
        loop_rate.sleep();
       
        ROS_INFO(" Sleep ");
    }
   
    return 0;
}

