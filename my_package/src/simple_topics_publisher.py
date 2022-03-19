#!/usr/bin/env python
#-*-coding:utf-8-*-
#해석:  한국어 쓸것이고 python 인터프리터의 경로를 설정하고 스크립트가 실행가능>한지 확인함
import rospy
#해석: python 코드 내에서 ros를 사용하기 위해서 써야하는 모듈
from std_msgs.msg import String
#해석: std_msgs.msg 내에 있는 string자료형을 import하여 센서와 통신 위한 자료형으로 쓴다. 우리 과제에서는 초음파를 여러개 썻다.
rospy.init_node('teacher')
#해석: 노드 하나를 생성하고 그 이름을 teacher이라고 설정
pub = rospy.Publisher('my_topic', String)
#해석: 구독자 객체 생성하여 메시지 ‘my_topic’를 String타입으로 보냄
rate = rospy.Rate(2)

#해석: 1초에 2번 루프 돈다, rospy모듈의 Rate 클래스다. 
while not rospy.is_shutdown():
    #해석: ^-C 전까지 무한 반복함
    pub.publish("zz")
    #해석: 메세지를 보낸다. 
    rate.sleep()
    #해석: 전체 길이 1.0초에서 2번 루프도니 루프 한번은 0.5초로 셋팅, 만약 publish time이 0.3초 이면 sleep속도는 0.2초이고 합쳐서 0.5초됨.