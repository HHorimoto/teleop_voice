#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import math
from geometry_msgs.msg import Twist
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sensor_message_getter import SensorMessageGetter

class TeleopVoice(object):
    def __init__(self, topic_vel='/cmd_vel', topic_voice='/Tablet/voice', linear_vel=0.2, angular_vel=math.radians(10), low_th=0.5, msg_wait=1.0):
        self.pub = rospy.Publisher(topic_vel, Twist, queue_size=10)
        self.voice_msg = SensorMessageGetter(topic_voice, SpeechRecognitionCandidates, msg_wait)
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.low_th = low_th
        self.vel = Twist()
    
    def get_candidates(self, transcript, confidence):
        i = 0
        candidates = []
        for msg in transcript:
            if confidence[i] > self.low_th:
                candidates.append(msg)
            i += 1
        return candidates
        
    def spin(self):
        msg_voice = self.voice_msg.get_msg()
        if msg_voice is not None:
            transcript = msg_voice.transcript
            confidence = msg_voice.confidence
            candidates = self.get_candidates(transcript, confidence)
            if len(candidates) == 0:
                pass
            else:
                for candidate in candidates:
                    candidate = candidate.replace(' ', '')
                    if candidate == "goforward":
                        rospy.loginfo("You said go forward")
                        self.vel.linear.x = self.linear_vel
                        self.vel.angular.z = 0.0
                    elif candidate == "goback":
                        rospy.loginfo("You said go back")
                        self.vel.linear.x = -self.linear_vel
                        self.vel.angular.z = 0.0
                    elif candidate == "turnleft":
                        rospy.loginfo("You said turn left")
                        self.vel.linear.x = 0.0
                        self.vel.angular.z = self.angular_vel
                    elif candidate == "turnright":
                        rospy.loginfo("You said turn right")
                        self.vel.linear.x = 0.0
                        self.vel.angular.z = -self.angular_vel
                    elif candidate == "stop":
                        rospy.loginfo("You said stop")
                        self.vel.linear.x = 0.0
                        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)

def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)
    
    rospy.loginfo(node_name)
    
    # param
    low_th = rospy.get_param("~low_th")
    linear_vel = rospy.get_param("~linear_vel")
    angular_vel = rospy.get_param("~angular_vel")
    topic_vel = rospy.get_param("~topic_vel")
    topic_voice = rospy.get_param("~topic_voice")
    
    rate = rospy.Rate(50)
    node = TeleopVoice(topic_vel=topic_vel, topic_voice=topic_voice, low_th=low_th, linear_vel=linear_vel, angular_vel=angular_vel)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()

if __name__ == '__main__':
    main()    
    