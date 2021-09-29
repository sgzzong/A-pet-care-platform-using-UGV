#include "ros/ros.h"
#include "std_msgs/String.h"
 
 
void chatterCallback( const std_msgs::String::ConstPtr& msg )
{
    //-- print
    ROS_INFO("I heard: [%s]", msg->data.c_str() );
}
 
 
 
int main( int argc, char **argv )
{
    ros::init( argc, argv, "example1_b" );
    ros::NodeHandle n;
   
    ros::Subscriber sub = n.subscribe("message", 1000, chatterCallback);
   
    //-- 라이브러리에서 토픽읽는 처리를 계속 처리
    ros::spin();
   
    return 0;
}