import rclpy

import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from yasmin_factory import YasminFactory
from yasmin_msgs.msg import (
    State as StateMsg,
    StateMachine as StateMachineMsg,
    Transition as TransitionMsg,
)
from rclpy.node import Node


class FSMSubscriber(Node):
    def __init__(self):
        super().__init__('fsm_subscriber')
        self.create_subscription(StateMachineMsg, 'fsm_viewer',self.callback, 10)

    def callback(self, StateMachineMsg):
        self.get_logger().info('I heard: "%s"' % StateMachineMsg)

def main(args=None):
    rclpy.init(args=args)

    fsm_subscriber = FSMSubscriber()

    rclpy.spin(fsm_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fsm_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
