#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# 关键点：导入自定义消息
# ROS 2 将 msg 文件名 'req_pose_msg' 转换为驼峰命名 'ReqPoseMsg'
from loc_msg.msg import ReqPoseMsg

class PoseRequestPublisher(Node):

    def __init__(self):
        super().__init__('pose_request_publisher')
        
        # 创建发布者：话题名为 'req_pose_topic'，队列长度为 10
        self.publisher_ = self.create_publisher(ReqPoseMsg, 'req_pose_topic', 10)
        
        # 创建定时器：每 1 秒发布一次
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # 实例化消息对象
        msg = ReqPoseMsg()
        
        # 填充数据
        msg.frame_id = "map"
        msg.child_frame_id = f"base_link_{self.i}"
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 打印日志
        self.get_logger().info(f'Publishing: frame_id="{msg.frame_id}", child="{msg.child_frame_id}"')
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