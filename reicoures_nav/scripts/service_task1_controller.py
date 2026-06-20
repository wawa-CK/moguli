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
from rei_voice.srv import REIPlayer, REIPlayerRequest
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
        self.player_service = rospy.get_param("~player_service", "/REIService/PcmPlayer")
        self.aiui_result_topic = rospy.get_param("~aiui_result_topic", "/REITopic/AIUIResult")
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")

        self.welcome_audio = rospy.get_param("~welcome_audio", "./REIAIUI/audio/tts1.pcm")
        self.guide_ack_audio = rospy.get_param("~guide_ack_audio", "./REIAIUI/audio/tts1.pcm")
        self.intro_audio = rospy.get_param("~intro_audio", "./REIAIUI/audio/tts2.pcm")
        self.return_audio = rospy.get_param("~return_audio", "./REIAIUI/audio/tts3.pcm")
        self.retry_audio = rospy.get_param("~retry_audio", "./REIAIUI/audio/tts1.pcm")
        self.nav_fail_audio = rospy.get_param("~nav_fail_audio", "./REIAIUI/audio/tts1.pcm")
        self.return_fail_audio = rospy.get_param("~return_fail_audio", "./REIAIUI/audio/tts1.pcm")
        self.done_audio = rospy.get_param("~done_audio", "./REIAIUI/audio/tts3.pcm")

        self.face_client = rospy.ServiceProxy(self.face_service, recognition_results)
        self.record_client = rospy.ServiceProxy(self.record_service, SetBool)
        self.player_client = rospy.ServiceProxy(self.player_service, REIPlayer)
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.start_pose = None
        self.latest_iat_text = ""
        self.command_event = threading.Event()

        rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber(self.aiui_result_topic, REIResult, self.aiui_result_cb, queue_size=10)

    def pose_cb(self, msg):
        if self.start_pose is None:
            self.start_pose = msg.pose.pose
            rospy.loginfo("Initial pose recorded for return navigation")

    def aiui_result_cb(self, msg):
        if getattr(msg, "type", "") != "iat":
            return

        text = getattr(msg, "iat", "").strip()
        if not text:
            return

        self.latest_iat_text = text
        rospy.loginfo("Recognized speech: %s", text)
        self.command_event.set()

    def wait_for_dependencies(self):
        rospy.loginfo("Waiting for required services and move_base")
        rospy.wait_for_service(self.face_service, timeout=20)
        rospy.wait_for_service(self.record_service, timeout=20)
        rospy.wait_for_service(self.player_service, timeout=20)
        if not self.move_base_client.wait_for_server(rospy.Duration(20)):
            raise rospy.ROSException("move_base action server is not ready")
        rospy.loginfo("All required interfaces are ready")

    def set_recording(self, enabled):
        try:
            response = self.record_client.call(SetBoolRequest(enabled))
            rospy.loginfo("RecordAudio set to %s: %s", enabled, response.message)
            return response.success
        except rospy.ServiceException as exc:
            rospy.logerr("RecordAudio service call failed: %s", exc)
            return False

    def play_audio(self, audio_path, label):
        rospy.loginfo("Playing %s: %s", label, audio_path)
        self.set_recording(False)
        try:
            request = REIPlayerRequest()
            request.PcmPath = audio_path
            response = self.player_client.call(request)
            if not response.success:
                rospy.logerr("PCM playback failed: %s", response.message)
                return False
            return True
        except rospy.ServiceException as exc:
            rospy.logerr("PcmPlayer service call failed: %s", exc)
            return False

    def detect_face(self):
        try:
            request = recognition_resultsRequest()
            request.mode = 1
            request.str = "/head_camera/image_raw"
            response = self.face_client.call(request)
            if not response.success:
                rospy.logwarn("Face recognition failed: %s", response.message)
                return []

            labels = []
            for face_data in response.result.face_data:
                labels.append(face_data.header.frame_id)
            return labels
        except rospy.ServiceException as exc:
            rospy.logerr("Face recognition service call failed: %s", exc)
            return []

    def wait_for_face(self):
        rate_hz = max(1.0 / self.face_poll_interval, 0.2)
        rate = rospy.Rate(rate_hz)
        rospy.loginfo("Waiting for face detection trigger")
        while not rospy.is_shutdown():
            labels = self.detect_face()
            if labels:
                rospy.loginfo("Detected faces: %s", labels)
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
        rospy.loginfo("Waiting for guide command")
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
                rospy.loginfo("Guide command matched")
                return True

            rospy.logwarn("Speech does not match guide command: %s", text)
        return False

    def nav_to_pose(self, x, y, z, w, label):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        rospy.loginfo(
            "Navigating to %s: x=%.3f y=%.3f z=%.3f w=%.3f",
            label,
            x,
            y,
            z,
            w,
        )
        self.move_base_client.send_goal(goal)
        finished = self.move_base_client.wait_for_result(rospy.Duration(120))
        if not finished:
            self.move_base_client.cancel_goal()
            rospy.logerr("Navigation to %s timed out", label)
            return False

        state = self.move_base_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached %s", label)
            return True

        rospy.logerr("Navigation to %s failed, state=%s", label, state)
        return False

    def return_to_start(self):
        if self.start_pose is None:
            rospy.logerr("No initial pose available for return navigation")
            return False

        pose = self.start_pose
        return self.nav_to_pose(
            pose.position.x,
            pose.position.y,
            pose.orientation.z,
            pose.orientation.w,
            "start_area",
        )

    def run(self):
        self.wait_for_dependencies()

        if self.start_pose is None:
            rospy.logwarn("No /amcl_pose received yet, return navigation will wait for it")

        if not self.wait_for_face():
            return

        self.play_audio(self.welcome_audio, "welcome_audio")

        if not self.wait_for_command():
            self.play_audio(self.retry_audio, "retry_audio")
            self.set_recording(False)
            return

        self.play_audio(self.guide_ack_audio, "guide_ack_audio")

        if not self.nav_to_pose(self.target_x, self.target_y, self.target_z, self.target_w, "shenzhen_hall"):
            self.play_audio(self.nav_fail_audio, "nav_fail_audio")
            self.set_recording(False)
            return

        self.play_audio(self.intro_audio, "intro_audio")
        self.play_audio(self.return_audio, "return_audio")

        if not self.return_to_start():
            self.play_audio(self.return_fail_audio, "return_fail_audio")
            self.set_recording(False)
            return

        self.play_audio(self.done_audio, "done_audio")
        self.set_recording(False)


def main():
    rospy.init_node("service_task1_controller")
    controller = ServiceTask1Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node interrupted")
    except Exception as exc:
        rospy.logerr("Task failed: %s", exc)


if __name__ == "__main__":
    main()
