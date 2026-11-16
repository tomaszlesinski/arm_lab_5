import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from search_interfaces.action import SearchCubes


class SearchClient(Node):
    def __init__(self):
        super().__init__('search_client')
        # Inicjalizacja klienta akcji
        self._action_client = ActionClient(self, SearchCubes, 'search_cubes')

    def send_goal(self):
        self.get_logger().info("Oczekiwanie na serwer akcji search_cubes")
        self._action_client.wait_for_server()

        goal_msg = SearchCubes.Goal()
        goal_msg.start_search = True

        # Wyslanie zadania z rejestracja informacji zwrotnej
        self.get_logger().info("Wysylanie celu do serwera")
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Cel odrzucony.")
            return

        # Czeka na koncowy wynik akcji
        self.get_logger().info("Cel potwierdzony, oczekiwanie na wynik")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        # Wyswietlanie biezacego postepu misji w konsoli
        feedback = feedback_msg.feedback
        self.get_logger().info(f"{feedback.current_status} | Znaleziono: {feedback.found_count}/{feedback.total_count}")

    def _result_callback(self, future):
        result = future.result().result
        print("\n" + "=" * 50)
        print(f"Wynik akcji: {'Sukces' if result.success else 'Niepowodzenie'}")
        print(f"Liczba znalezionych kostek: {len(result.ids)}/5")
        for mid, pos in zip(result.ids, result.positions):
            print(f"Kostka ID {mid:2d}:  x = {pos.x:6.2f},  y = {pos.y:6.2f},  z = {pos.z:6.2f}")
        print("=" * 50 + "\n")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = SearchClient()
    client.send_goal()
    rclpy.spin(client)


if __name__ == '__main__':
    main()
