import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, GetMap
import heapq


def reconstruct_path(came_from, current):
    """Reconstruct the path from the A* search."""
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def heuristic(a, b):
    """Manhattan distance heuristic for A*."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search(start, goal, grid):
    """A* algorithm for finding the shortest path."""
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, Down, Left, Right
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] != 'b':
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found


class MapBasedNavigation(Node):
    def __init__(self):
        super().__init__('map_based_navigation')
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.map_client = self.create_client(GetMap, '/get_map')

        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /move_command...')

        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /get_map...')

        self.get_logger().info('Navegação mapeada inicializada!')
    
    def send_move_command(self, direction):
        """Send a move command to the robot."""
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)

        # Non-blocking wait loop to process callbacks
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Comando enviado: {direction}, Resposta: {response}')
            return response
        else:
            self.get_logger().error('Failed to send move command.')
            return None


    def get_map(self):
        """Fetch the map from the /get_map service."""
        request = GetMap.Request()
        future = self.map_client.call_async(request)

        # Non-blocking wait loop to process callbacks
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if future.result() is not None:
            response = future.result()
            occupancy_grid = response.occupancy_grid_flattened
            shape = response.occupancy_grid_shape
            return occupancy_grid, shape
        else:
            self.get_logger().error('Falha ao obter o mapa.')
            return None, None

    def plan_path(self, start, goal, occupancy_grid, shape):
        """Plan the path using A*."""
        rows, cols = shape
        grid = [occupancy_grid[i * cols:(i + 1) * cols] for i in range(rows)]

        # Convert grid strings to A* compatible values
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == 'b':
                    grid[r][c] = 'b'  # Blocked cell
                elif grid[r][c] in ('f', 'r'):  # Treat 'r' as free
                    grid[r][c] = 'f'  # Free cell
                elif grid[r][c] == 't':
                    grid[r][c] = 'f'  # Treat target as free for planning
                else:
                    self.get_logger().error(f"Unexpected cell value at ({r}, {c}): {grid[r][c]}")
                    return None

        # Check if start and goal are valid
        if grid[start[0]][start[1]] == 'b' or grid[goal[0]][goal[1]] == 'b':
            self.get_logger().error('Start or goal is blocked!')
            return None

        path = a_star_search(start, goal, grid)
        if path:
            self.get_logger().info(f'Caminho planejado: {path}')
            return path
        else:
            self.get_logger().error('Nenhum caminho encontrado.')
            return None
        
    def navigate(self, start, goal):
        """Navigate the robot to the target."""
        occupancy_grid, shape = self.get_map()
        if occupancy_grid is None or shape is None:
            return

        rows, cols = shape
        if not (0 <= start[0] < rows and 0 <= start[1] < cols):
            self.get_logger().error('Posição inicial está fora dos limites!')
            return
        if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
            self.get_logger().error('Objetivo está fora dos limites!')
            return

        path = self.plan_path(start, goal, occupancy_grid, shape)
        if path is None:
            return

        direction_map = {
            (-1, 0): 'up',
            (1, 0): 'down',
            (0, -1): 'left',
            (0, 1): 'right'
        }

        for next_pos in path:
            dx = next_pos[0] - start[0]
            dy = next_pos[1] - start[1]
            direction = direction_map[(dx, dy)]

            retry_count = 3
            success = False
            while retry_count > 0 and not success:
                result = self.send_move_command(direction)
                if result and result.success:
                    self.get_logger().info(f'Movimento bem-sucedido para {result.robot_pos}')
                    start = next_pos
                    success = True
                else:
                    self.get_logger().warning(f'Tentativa falhou. Direção: {direction}, Posição atual: {start}')
                    retry_count -= 1

            if not success:
                self.get_logger().error(f'Movimento falhou após {3 - retry_count} tentativas. Recalculando caminho...')
                # Re-plan the path dynamically
                occupancy_grid, shape = self.get_map()
                if occupancy_grid and shape:
                    path = self.plan_path(start, goal, occupancy_grid, shape)
                    if path is None:
                        self.get_logger().error('Re-planning failed. Aborting navigation.')
                        break
                else:
                    self.get_logger().error('Failed to fetch map. Aborting navigation.')
                    break

        self.get_logger().info('Navegação concluída!')


        self.get_logger().info('Navegação concluída!')


def run_map_based_navigation():
    node = MapBasedNavigation()
    try:
        # Example start and goal positions
        start = (2, 2)  # Replace with actual start position
        goal = (17, 17)  # Replace with actual goal position
        node.navigate(start, goal)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
