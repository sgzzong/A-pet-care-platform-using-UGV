#!/usr/bin/env python
#-*-coding:utf-8-*-
#해석:  한국어 쓸것이고 python 인터프리터의 경로를 설정하고 스크립트가 실행가능>한지 확인함
import rospy
#해석: python 코드 내에서 ros를 쓰기 위해서 필요한 모듈
from darknet_ros_msgs.msg import BoundingBoxes
#해석: std_msgs.msg 내에 있는 string자료형을  import하기 위해서 이것으로 센서>와 통신을 하기 위한 자료형을 쓴다. 
def callback(msg):
    print(msg.data)
#해석: Topic에서 받아온 메시지를 프린트하기위한 것,메시지를 받을 때마다 callbac>함수가 실행된다
rospy.init_node('control')
#해석: 노드 하나를 생성하고 그 이름을 student라 설정
sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
#해석: 구독자 객체 초기화, 토픽 중에서 my_topic을 받을 것이고 타입은 String임, >받을 때마다 매번 위에 있는 callback함수 실행됨.
rospy.spin()
#무한 반복됨, 토픽하나 도착할 때마다 callback함수를 불러주고 사용자의 노드를 노>드가 셧다운 되기 전까지 종료로 부터 지키는 것이다.