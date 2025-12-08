#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from action_msgs.msg import GoalStatus, GoalInfo
from unique_identifier_msgs.msg import UUID
from nav_msgs.msg import Goals

# 关键点：导入自定义消息
# ROS 2 将 msg 文件名 'req_pose_msg' 转换为驼峰命名 'ReqPoseMsg'
from loc_msg.msg import ResStartNav

class PoseRequestPublisher(Node):

    def __init__(self):
        super().__init__('pose_request_publisher')
        
        # 创建发布者：话题名为 'req_pose_topic'，队列长度为 10
        self.publisher_ = self.create_publisher(ResStartNav, 'res_nav', 10)
        
        # 创建定时器：每 1 秒发布一次
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # 实例化消息对象
        msg = ResStartNav()
        
        # 填充 status (GoalStatus 类型)
        msg.status = GoalStatus()
        
        # 填充 goal_info
        msg.status.goal_info = GoalInfo()
        msg.status.goal_info.goal_id = UUID()
        msg.status.goal_info.goal_id.uuid = [0] * 16  # 16字节的UUID
        msg.status.goal_info.stamp = Time()
        msg.status.goal_info.stamp.sec = self.i
        msg.status.goal_info.stamp.nanosec = 0
        
        # 设置状态值 (STATUS_EXECUTING = 2)
        msg.status.status = GoalStatus.STATUS_EXECUTING
        
        # 填充 target_poses (Goals 类型)
        msg.target_poses = Goals()
        
        # 填充数值字段
        msg.start_time = float(self.i)
        msg.distance = 10.0
        msg.remain_distance = 10.0 - float(self.i) * 0.5  # 模拟剩余距离减少
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 打印日志
        self.get_logger().info(
            f'Publishing ResStartNav: status={msg.status.status}, '
            f'distance={msg.distance:.1f}, remain={msg.remain_distance:.1f}'
        )
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    
    node = PoseRequestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()