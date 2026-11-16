import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator

from search_interfaces.action import SearchCubes
from search_interfaces.msg import DetectedCubeArray


WAYPOINTS = [
    (0.0, 3.0),
    (-5.0, 3.0),
    (-5.0, 0.0),
    (-5.0, -5.0),
    (3.0, -5.0),
    (3.0, 0.0),
    (3.0, 4.0),
    (10.0, 4.0),
    (10.0, 0.0),
    (17.0, 0.0),
    (12.0, -5.0),
]


class SearchServer(Node):
    def __init__(self, navigator: BasicNavigator):
        super().__init__('search_server')
        self.navigator = navigator
        self.callback_group = ReentrantCallbackGroup()

        # Przechowywanie wykrytych kostek z topiku
        self.detected_cubes = {}

        # Subskrypcja topika wykrytych kostek
        self.create_subscription(
            DetectedCubeArray,
            '/detected_cubes',
            self._cube_callback,
            10,
            callback_group=self.callback_group,
        )

        # Inicjalizacja serwera akcji
        self.action_server = ActionServer(
            self,
            SearchCubes,
            'search_cubes',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info(f"Serwer akcji gotowy. Trasa ma: {len(WAYPOINTS)} punktow.")

    def _cube_callback(self, msg: DetectedCubeArray):
        for cube in msg.cubes:
            self.detected_cubes[cube.id] = (cube.position.x, cube.position.y, cube.position.z)

    def _navigate_to_pose(self, x: float, y: float, timeout_sec: float = 90.0) -> bool:
        # Przygotowanie celu dojazdu w ukladzie mapy
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.orientation.w = 1.0

        # Wycofanie poprzednich i wyslanie nowego celu do Nav2
        self.navigator.goToPose(target_pose)
        start_time = time.time()

        while not self.navigator.isTaskComplete():
            # Anulowanie jazdy jesli znaleziono komplet 5 kostek
            if len(self.detected_cubes) >= 5:
                self.navigator.cancelTask()
                return True

            # Zabezpieczenie przed zablokowaniem
            if time.time() - start_time > timeout_sec:
                self.navigator.cancelTask()
                return False

            time.sleep(0.2)

        return True

    def _panoramic_spin(self, angle_rad: float = 6.28, time_allowance_sec: int = 14):
        # Pelny obrot robota do skanowania otoczenia kamera
        self.navigator.spin(spin_dist=angle_rad, time_allowance=time_allowance_sec)
        start_time = time.time()

        while not self.navigator.isTaskComplete():
            # Anulowanie obrotu jesli znaleziono komplet 5 kostek
            if len(self.detected_cubes) >= 5:
                self.navigator.cancelTask()
                return

            # Zabezpieczenie przed zablokowaniem
            if time.time() - start_time > time_allowance_sec + 3:
                self.navigator.cancelTask()
                return

            time.sleep(0.2)

    def execute_callback(self, goal_handle):
        feedback = SearchCubes.Feedback()
        result = SearchCubes.Result()

        for step_idx, (wx, wy) in enumerate(WAYPOINTS):
            if len(self.detected_cubes) >= 5:
                break

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                return result

            feedback.found_count = len(self.detected_cubes)
            feedback.total_count = 5
            feedback.current_status = f"Punkt {step_idx + 1}/{len(WAYPOINTS)} ({wx:.1f}, {wy:.1f})"
            goal_handle.publish_feedback(feedback)

            self._navigate_to_pose(wx, wy)

            if len(self.detected_cubes) >= 5:
                break

            self._panoramic_spin()

        sorted_ids = sorted(self.detected_cubes.keys())

        result.success = len(self.detected_cubes) >= 5
        result.ids = sorted_ids
        result.positions = []

        for mid in sorted_ids:
            pos = Point()
            pos.x, pos.y, pos.z = self.detected_cubes[mid]
            result.positions.append(pos)

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    server = SearchServer(navigator)

    # Egzekutor wielowatkowy do wspolbieznej obslugi akcji i subskrypcji
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
