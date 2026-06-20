#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def nav_to_goal():
    rospy.init_node('move_base_action_client', anonymous=True)

    # 设置目标位置的到达精度（米）和角度精度（弧度）
    # 让机器人在接近目标时就认为成功，避免过度修正摇晃
    rospy.set_param("/move_base/TebLocalPlannerROS/xy_goal_tolerance", 0.10)   # XY容忍 ±10cm
    rospy.set_param("/move_base/TebLocalPlannerROS/yaw_goal_tolerance", 0.10)  # 朝向容忍 ±5.7度

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待 move_base 服务器...")
    client.wait_for_server()
    rospy.loginfo("连接成功！")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # ===== 目标点配置（按你的实际需要修改）=====
    goal.target_pose.pose.position.x = 1.044
    goal.target_pose.pose.position.y = 1.240

    # 设置最终朝向：朝前 (w=1)
    # 四元数 (x, y, z, w) 绕Z轴转0度 = 1.0,0.0,0.0,1.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo(f"发送目标: x={goal.target_pose.pose.position.x}, y={goal.target_pose.pose.position.y}")
    client.send_goal(goal)

    # 等待结果，并定期输出状态
    rate = rospy.Rate(5)  # 5Hz
    while not rospy.is_shutdown():
        state = client.get_state()
        # ActionLib 状态码含义
        # 1 = Active, 3 = Succeeded, 4 = Aborted
        if state == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("导航中...")
        elif state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ 成功到达目标点！")
            break
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.logerr("❌ 导航失败（机器人被卡住/无路径）")
            break
        rate.sleep()

    rospy.loginfo("导航结束")

if __name__ == '__main__':
    try:
        nav_to_goal()
    except rospy.ROSInterruptException:
        rospy.logerr("程序异常退出")