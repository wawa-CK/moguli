#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// 定义 Action 客户端类型
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    // 初始化 ROS 节点
    ros::init(argc, argv, "move_base_nav_client");

    // 创建 Action 客户端（连接 move_base 服务器）
    MoveBaseClient client("move_base", true);

    // 等待服务器连接成功
    ROS_INFO("等待连接 move_base 服务器...");
    client.waitForServer();
    ROS_INFO("连接成功！");

    // 构造导航目标消息
    move_base_msgs::MoveBaseGoal goal;

    // 设置坐标系为 map
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // 设置目标坐标（可修改 x, y）
    goal.target_pose.pose.position.x = 2.543;
    goal.target_pose.pose.position.y = 2.148;

    // 设置朝向（朝前）
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1.0;

    // 发送目标点
    ROS_INFO("发送导航目标...");
    client.sendGoal(goal);

    // 循环监听导航状态
    ros::Rate rate(5);
    while (ros::ok())
    {
        // 获取当前状态
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("当前状态: %s", state.toString().c_str());
        // 判断导航状态
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("导航成功：到达目标点！");
            break;
        }
        else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_ERROR("导航失败：无法到达目标！");
            break;
        }
        rate.sleep();
    }

    ROS_INFO("导航任务结束！");
    return 0;
}
