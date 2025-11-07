from names.srv import PartOfName

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(PartOfName, 'push_name', self.push_name)

    def push_name(self, request, response):
        response.full_name = f"{request.first_name} {request.name} {request.last_name}"
        self.get_logger().info('Incoming request\n')             
        self.get_logger().info(f'first name = {request.first_name}\nname = {request.name}\nlast_name = {request.last_name}')

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()