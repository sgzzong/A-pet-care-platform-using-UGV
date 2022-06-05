#!/usr/bin/env python
# -*- coding: utf-8- -*-

import rospy
from std_srvs.srv import Empty, EmptyRequest
import time

rospy.init_node("robot_turning_client")
rospy.wait_for_service('/motor_deg_a')
service_client = rospy.ServiceProxy("/motor_deg_a", Empty)

#if 간식주기 버튼을 누른다면
request_srv = EmptyRequest()
result = service_client(request_srv)
print(result)