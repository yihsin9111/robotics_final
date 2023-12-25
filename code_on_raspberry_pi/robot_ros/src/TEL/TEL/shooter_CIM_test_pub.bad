import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
import asyncio
import websockets
import functools

async def main_handle(websocket, path, publisher):
    async for message in websockets:
        print(message)

async def serve(publisher):
    print("Opening server...")
    await asyncio.gather(websockets.serve(
        functools.partial(
            main_handle, 
            publisher=publisher), 
        "192.168.0.64", 1484))


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('shooter_CIM_test_pub')
        self.publisher_ = self.create_publisher(Int8, 'Shooter_CIM_switch', 1)
        self.did_run = False
        self.timer = self.create_timer(0.5, self.timer_callback)

    async def timer_callback(self):
        self.did_run = True
        # asyncio.run(serve(self.publisher_))
        await serve(self.publisher_)

async def async_main(args=None):
    rclpy.init(args=args)

    node = MinimalPublisher()

    # await asyncio.gather(
    #     spin_node(minimal_publisher),
    #     serve())
    # await serve()
    while rclpy.ok() and not node.did_run:
        rclpy.spin(node)

    node.timer.cancel()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

def main():
    asyncio.get_event_loop().run_until_complete(async_main())
    asyncio.get_event_loop().close()


if __name__ == '__main__':
    main()
