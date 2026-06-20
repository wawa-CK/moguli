#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from face_rec.srv import recognition_results, recognition_resultsRequest
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rei_voice.msg import REIResult
from rei_voice.srv import REITts, REITtsRequest
from std_srvs.srv import SetBool, SetBoolRequest


class ServiceTask1Controller:
    def __init__(self):
        self.target_x = rospy.get_param("~target_x", 2.543)
        self.target_y = rospy.get_param("~target_y", 2.148)
        self.target_z = rospy.get_param("~target_z", 0.0)
        self.target_w = rospy.get_param("~target_w", 1.0)
        self.face_poll_interval = rospy.get_param("~face_poll_interval", 1.0)
        self.command_timeout = rospy.get_param("~command_timeout", 20.0)
        self.face_service = rospy.get_param("~face_service", "/face_recognition_results")
        self.record_service = rospy.get_param("~record_service", "/REIService/RecordAudio")
        self.tts_service = rospy.get_param("~tts_service", "/REIService/voice_tts")
        self.aiui_result_topic = rospy.get_param("~aiui_result_topic", "/REITopic/AIUIResult")
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")

        self.face_client = rospy.ServiceProxy(self.face_service, recognition_results)
        self.record_client = rospy.ServiceProxy(self.record_service, SetBool)
        self.tts_client = rospy.ServiceProxy(self.tts_service, REITts)
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.start_pose = None
        self.latest_pose = None
        self.latest_iat_text = ""
        self.command_event = threading.Event()

        rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber(self.aiui_result_topic, REIResult, self.aiui_result_cb, queue_size=10)

    def pose_cb(self, msg):
        self.latest_pose = msg.pose.pose
        if self.start_pose is None:
            self.start_pose = msg.pose.pose
            rospy.loginfo("已记录初始位姿，任务结束后将返回此位置")

    def aiui_result_cb(self, msg):
        if getattr(msg, "type", "") != "iat":
            return

        text = getattr(msg, "iat", "").strip()
        if not text:
            return

        self.latest_iat_text = text
        rospy.loginfo("识别到语音文本：%s", text)
        self.command_event.set()

    def wait_for_dependencies(self):
        rospy.loginfo("等待依赖服务和动作服务器...")
        rospy.wait_for_service(self.face_service, timeout=20)
        rospy.wait_for_service(self.record_service, timeout=20)
        rospy.wait_for_service(self.tts_service, timeout=20)
        if not self.move_base_client.wait_for_server(rospy.Duration(20)):
            raise rospy.ROSException("move_base 动作服务器未就绪")
        rospy.loginfo("依赖接口已就绪")

    def set_recording(self, enabled):
        try:
            response = self.record_client.call(SetBoolRequest(enabled))
            rospy.loginfo("录音状态切换为 %s: %s", enabled, response.message)
            return response.success
        except rospy.ServiceException as exc:
            rospy.logerr("调用录音服务失败: %s", exc)
            return False

    def speak(self, text):
        rospy.loginfo("播报：%s", text)
        self.set_recording(False)
        try:
            request = REITtsRequest()
            request.text = text
            request.is_play = True
            response = self.tts_client.call(request)
            if not response.success:
                rospy.logerr("TTS 播报失败：%s", response.message)
                return False
            return True
        except rospy.ServiceException as exc:
            rospy.logerr("调用 TTS 服务失败: %s", exc)
            return False

    def detect_face(self):
        try:
            request = recognition_resultsRequest()
            request.mode = 1
            request.str = "/head_camera/image_raw"
            response = self.face_client.call(request)
            if not response.success:
                rospy.logwarn("人脸识别服务返回失败：%s", response.message)
                return []

            labels = []
            for face_data in response.result.face_data:
                labels.append(face_data.header.frame_id)
            return labels
        except rospy.ServiceException as exc:
            rospy.logerr("调用人脸识别服务失败: %s", exc)
            return []

    def wait_for_face(self):
        rate = rospy.Rate(max(1.0 / self.face_poll_interval, 0.2))
        rospy.loginfo("进入待机，轮询人脸识别中...")
        while not rospy.is_shutdown():
            labels = self.detect_face()
            if labels:
                rospy.loginfo("检测到人脸：%s", labels)
                return True
            rate.sleep()
        return False

    def command_matches(self, text):
        normalized = text.replace(" ", "")
        guide_keywords = ("带我去", "参观", "导览")
        place_keywords = ("深圳馆", "深圳")
        return any(word in normalized for word in guide_keywords) and any(
            word in normalized for word in place_keywords
        )

    def wait_for_command(self):
        rospy.loginfo("开始等待导览命令")
        self.latest_iat_text = ""
        self.command_event.clear()
        self.set_recording(True)

        deadline = rospy.Time.now() + rospy.Duration(self.command_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self.command_event.wait(0.2):
                continue

            text = self.latest_iat_text
            self.command_event.clear()
            if self.command_matches(text):
                rospy.loginfo("导览命令匹配成功")
                return True

            rospy.logwarn("语音内容不匹配导览命令：%s", text)
        return False

    def nav_to_pose(self, x, y, z, w, label):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        rospy.loginfo("开始导航到%s: x=%.3f y=%.3f z=%.3f w=%.3f", label, x, y, z, w)
        self.move_base_client.send_goal(goal)
        finished = self.move_base_client.wait_for_result(rospy.Duration(120))
        if not finished:
            self.move_base_client.cancel_goal()
            rospy.logerr("导航到%s超时", label)
            return False

        state = self.move_base_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("成功到达%s", label)
            return True

        rospy.logerr("导航到%s失败，状态码：%s", label, state)
        return False

    def return_to_start(self):
        if self.start_pose is None:
            rospy.logerr("未获取到初始位姿，无法返回出发区")
            return False

        pose = self.start_pose
        return self.nav_to_pose(
            pose.position.x,
            pose.position.y,
            pose.orientation.z,
            pose.orientation.w,
            "出发区",
        )

    def run(self):
        self.wait_for_dependencies()

        if self.start_pose is None:
            rospy.logwarn("暂未收到 /amcl_pose，回航将等待初始位姿")

        if not self.wait_for_face():
            return

        self.speak("你好，欢迎您的到来！有什么需要帮助的吗？")

        if not self.wait_for_command():
            self.speak("我没有听清楚，请您再说一遍。")
            self.set_recording(False)
            return

        self.speak("好的，请跟我来。")

        if not self.nav_to_pose(self.target_x, self.target_y, self.target_z, self.target_w, "深圳馆"):
            self.speak("抱歉，我暂时无法到达深圳馆。")
            self.set_recording(False)
            return

        self.speak(
            "深圳，是广东副省级市、经济特区。毗邻香港，经济发达，创新力强，"
            "有众多世界五百强企业，是粤港澳大湾区中心城市。"
        )
        self.speak("这里就是深圳馆啦，我要继续回去工作啦！")

        if not self.return_to_start():
            self.speak("抱歉，我暂时无法返回出发区。")
            self.set_recording(False)
            return

        self.speak("我已经回到出发区，继续待机。")
        self.set_recording(False)


def main():
    rospy.init_node("service_task1_controller")
    controller = ServiceTask1Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被中断")
    except Exception as exc:
        rospy.logerr("任务执行异常: %s", exc)


if __name__ == "__main__":
    main()
