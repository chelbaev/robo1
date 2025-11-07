import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from cleaning_task_action.action import CleaningTask

import sys


class CleaningActionClient(Node):
    def __init__(self):
        super().__init__("cleaning_action_client")
        self._action_client = ActionClient(self, CleaningTask, "cleaning_task")
        self._goal_handle = None
        self._result_available = False
        self.get_logger().info("Cleaning Action Client started")

    def send_goal(self, task_type, area_size=0.0, target_x=0.0, target_y=0.0):
        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = area_size
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y

        self.get_logger().info(f"Sending goal: {task_type}, area_size: {area_size}")
        
        self._action_client.wait_for_server()
        self._result_available = False
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            self._result_available = True
            return

        self.get_logger().info("Goal accepted")
        self._goal_handle = goal_handle
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: success={result.success}, "
                             f"cleaned_points={result.cleaned_points}, "
                             f"total_distance={result.total_distance:.2f}")
        self._result_available = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: progress={feedback.progress_percent}%, "
                             f"cleaned={feedback.current_cleaned_points}, "
                             f"pos=({feedback.current_x:.2f}, {feedback.current_y:.2f})")

    def wait_for_result(self):
        while not self._result_available:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    
    action_client = CleaningActionClient()
    
    try:
        action_client.get_logger().info("STARTING TASKS \n")
        inp = input()
        if inp == 'end':
            action_client.get_logger().info("END EXECUTE \n")
        inp = inp.split()
        action_client.get_logger().info(f"Task 1: {inp[0]}")
        if len(inp) == 1:
            action_client.send_goal(inp[0])
        else:
            action_client.send_goal(inp[0], float(inp[1]))
        action_client.wait_for_result()
    except KeyboardInterrupt:
        action_client.get_logger().info("Interrupted by user")
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()