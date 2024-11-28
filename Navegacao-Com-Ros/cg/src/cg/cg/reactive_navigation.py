import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
import time

class ReactiveNavigation(Node):
    def __init__(self):
        super().__init__('reactive_navigation')
        self.client = self.create_client(MoveCmd, '/move_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /move_command...')
        self.get_logger().info('Navegação reativa inicializada!')
        self.visited = set()  # Set to store visited positions
        self.last_direction = None  # Track the last move to avoid oscillation

    def send_move_command(self, direction):
        """Envia o comando de movimento ao serviço ROS."""
        request = MoveCmd.Request()
        request.direction = direction
        future = self.client.call_async(request)

        # Wait for the future to complete
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error('Falha ao chamar o serviço.')
            return None

    def calculate_distance(self, pos1, pos2):
        """Calcula a distância Manhattan entre duas posições."""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def follow_the_target(self, response):
        """Algoritmo Follow-the-Target."""
        directions = ['left', 'down', 'up', 'right']
        opposite_directions = {'left': 'right', 'right': 'left', 'up': 'down', 'down': 'up'}
        best_direction = None
        min_distance = float('inf')

        for direction, adjacent in zip(directions, [response.left, response.down, response.up, response.right]):
            if adjacent == 'f' or adjacent == 't':  # Direção livre ou alvo
                if direction == opposite_directions.get(self.last_direction):  # Avoid backtracking
                    continue
                new_pos = self.simulate_move(response.robot_pos, direction)
                if tuple(new_pos) in self.visited:  # Avoid already visited positions
                    continue
                distance = self.calculate_distance(new_pos, response.target_pos)
                if distance < min_distance:
                    min_distance = distance
                    best_direction = direction

        return best_direction

    def wall_following(self, response):
        """Algoritmo Wall-Following."""
        directions = ['left', 'down', 'up', 'right']
        opposite_directions = {'left': 'right', 'right': 'left', 'up': 'down', 'down': 'up'}
        adjacent = [response.left, response.down, response.up, response.right]
        wall_follow_order = [3, 1, 0, 2]  # Ordem preferencial: direita, baixo, esquerda, cima

        for idx in wall_follow_order:
            direction = directions[idx]
            if adjacent[idx] == 'f' or adjacent[idx] == 't':  # Livre ou alvo
                if direction == opposite_directions.get(self.last_direction):  # Avoid backtracking
                    continue
                return direction

        return None  # Nenhuma direção disponível (erro)

    def simulate_move(self, pos, direction):
        """Simula a posição futura ao se mover em uma direção."""
        if direction == 'left':
            return [pos[0], pos[1] - 1]
        elif direction == 'down':
            return [pos[0] + 1, pos[1]]
        elif direction == 'up':
            return [pos[0] - 1, pos[1]]
        elif direction == 'right':
            return [pos[0], pos[1] + 1]

    def navigate(self):
        """Função principal que controla o robô."""
        while rclpy.ok():
            response = self.send_move_command('')  # Não move, apenas coleta informações
            if response is None:
                break

            # Adiciona a posição atual à lista de visitados
            self.visited.add(tuple(response.robot_pos))

            # Checar se o robô já chegou no alvo
            if all(response.robot_pos[i] == response.target_pos[i] for i in range(len(response.robot_pos))):
                self.get_logger().info('Alvo alcançado!')
                break

            # Usar Follow-the-Target ou Wall-Following
            move_direction = self.follow_the_target(response)
            if move_direction is None:  # Se Follow-the-Target falhar, use Wall-Following
                move_direction = self.wall_following(response)

            if move_direction:
                self.last_direction = move_direction  # Update last direction
                result = self.send_move_command(move_direction)
                if not result.success:
                    self.get_logger().warning(f'Falha ao mover na direção {move_direction}.')
            else:
                self.get_logger().error('Nenhuma direção válida encontrada. Encerrando.')
                break

def main(args=None):
    """Função principal para inicializar e rodar o nó."""
    rclpy.init(args=args)
    reactive_navigation_node = ReactiveNavigation()

    try:
        reactive_navigation_node.navigate()
    except KeyboardInterrupt:
        reactive_navigation_node.get_logger().info('Execução interrompida pelo usuário.')
    finally:
        reactive_navigation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()