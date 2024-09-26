import asyncio
from mavsdk import System
from mavsdk.action import ActionError
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class DroneTakeoffService(Node):
    def __init__(self):
        super().__init__('drone_takeoff_service')
        self.srv = self.create_service(Empty, 'drone/takeoff', self.takeoff_callback)

    async def takeoff(self):
        drone = System()
        await drone.connect(system_address="udp://:14550") # Change addr

        print("Waiting for drone...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered with UUID: {state.uuid}")
                break

        try:
            print("Arming drone...")
            await drone.action.arm()

            print("Taking off...")
            await drone.action.takeoff()

            await asyncio.sleep(5)  # Wait for 5 seconds before landing
            print("Landing...")
            await drone.action.land()

        except ActionError as e:
            self.get_logger().error(f"Action failed: {e}")

    def takeoff_callback(self, request, response):
        asyncio.create_task(self.takeoff())
        return response

def main(args=None):
    rclpy.init(args=args)

    drone_takeoff_service = DroneTakeoffService()

    rclpy.spin(drone_takeoff_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
