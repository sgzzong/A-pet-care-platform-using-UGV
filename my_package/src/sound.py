#!/usr/bin/env python
# -*- coding: utf-8- -*-
import roslib
roslib.load_manifest('sound_play')
import rospy
from sound_play.libsoundplay import SoundClient

rospy.init_node('play_sound_file')
#Create a sound client instance
sound_client = SoundClient()
#wait for sound_play node to connect to publishers (otherwise it will miss first published msg)
rospy.sleep(1)
#Method 1: Play Wave file directly from Client
sound = sound_client.waveSound('/home/gilljong/catkin_ws/src/my_package/src/앉아.mp4')
sound.play()