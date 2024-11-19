import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from cg_interfaces.srv import MoveCmd
from collections import deque

class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('reactive_navigator')

        # Cliente para o serviço 'move_command'
        self.command_client = self.create_client(MoveCmd, 'move_command')
        while not self.command_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Waiting for the move_command service to become available...')

        # Requisição para enviar comandos de movimento
        self.move_request = MoveCmd.Request()

        # Rastreamento de locais visitados e posição inicial
        self.explored_positions = set()
        self.current_location = (0, 0)

        # Resposta do serviço de movimentação
        self.service_response = None

    def send_movement_command(self, direction):
        self.move_request.direction = direction
        future = self.command_client.call_async(self.move_request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            self.service_response = future.result()
        except Exception as e:
            self.get_logger().error(f"Error during service call: {e}")
            self.service_response = None

    def explore_environment(self):
        """Explora o ambiente de forma reativa até encontrar o alvo."""
        movement_directions = {
            'up': (0, 1),
            'down': (0, -1),
            'left': (-1, 0),
            'right': (1, 0)
        }

        position_queue = deque([(self.current_location, None)])  # Fila para controle de posições

        while position_queue:
            current_location, movement_direction = position_queue.popleft()
            self.explored_positions.add(current_location)

            if movement_direction:
                self.send_movement_command(movement_direction)

                while self.service_response is None:
                    rclpy.spin_once(self, timeout_sec=0.1)

                response = self.service_response
                self.service_response = None

                if response and tuple(response.target_pos) == tuple(response.robot_pos):
                    self.get_logger().info("Target successfully reached!")
                    return

                if not response or not response.success:
                    self.get_logger().warning(f"Failed to move to {current_location}.")
                    continue

                self.get_logger().info(f"Moved to position {current_location} successfully.")
                self.get_logger().info(f"Robot sensors: left={response.left}, down={response.down}, up={response.up}, right={response.right}")

            for direction, (dx, dy) in movement_directions.items():
                next_position = (current_location[0] + dx, current_location[1] + dy)
                if next_position not in self.explored_positions:
                    position_queue.append((next_position, direction))

        self.get_logger().info("Exploration complete. Target not found.")

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    # Executor para lidar com o nó
    executor = SingleThreadedExecutor()

    # Inicializa o nó de navegação reativa
    navigator = ReactiveNavigator()
    executor.add_node(navigator)

    try:
        # Começa a navegação no ambiente
        navigator.get_logger().info("Starting reactive navigation...")
        navigator.explore_environment()
    finally:
        navigator.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

# Entrada principal do programa
if __name__ == '__main__':
    main()