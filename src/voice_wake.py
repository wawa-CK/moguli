#! /usr/bin/env python3
# -*- coding: utf-8-*

import rospy
from std_srvs.srv import SetBool,SetBoolRequest
from rei_voice.srv import REIPlayer,REIPlayerRequest
from std_msgs.msg import String
import time

recordAudio_client = rospy.ServiceProxy("/REIService/RecordAudio",SetBool)
player_client = rospy.ServiceProxy("/REIService/PcmPlayer",REIPlayer)

def playAudio(file):
    playMsg = REIPlayerRequest()
    playMsg.PcmPath = file
    resp = player_client.call(playMsg)
    if resp.success:
        rospy.loginfo(resp.message)
    else:
        rospy.logerr(resp.message)

def voiceState_cb(state):
    rospy.loginfo(f"输入状态：{state}")
    if state.data == "EVENT_WAKEUP":
        recordAudio_client.call(SetBoolRequest(False))
        # 延时模拟人的停顿
        time.sleep(0.2)
        playAudio("./REIAIUI/audio/awake.pcm")
        time.sleep(1)
        recordAudio_client.call(SetBoolRequest(True))

if __name__ == "__main__":
    rospy.init_node("voice_awake_node")
    recordAudio_client.call(SetBoolRequest(True))
    rospy.Subscriber("/REITopic/AIUIState",String,voiceState_cb,queue_size=10)
    rospy.spin()
    