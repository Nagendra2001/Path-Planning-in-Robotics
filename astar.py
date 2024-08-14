import heapq
class Node:
    def __init__(self, position):
        self.position = position
        self.g_cost = float('inf')  # Cost from start node to this node
        self.h_cost = None           # Heuristic cost (estimated cost from this node to goal)
        self.parent = None

    def __lt__(self, other):
        # Compare nodes based on their f-cost (g-cost + h-cost)
        return (self.g_cost + self.h_cost) < (other.g_cost + other.h_cost)



class Node:
    def __init__(self, position):
        self.position = position
        self.g_cost = float('inf')  # Cost from start node to this node
        self.h_cost = None           # Heuristic cost (estimated cost from this node to goal)
        self.parent = None

    def __lt__(self, other):
        return (self.g_cost + self.h_cost) < (other.g_cost + other.h_cost)

class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.start_node = None
        self.goal_node = None
        self.open_set = []
        self.closed_set = set()

    def heuristic(self, node):
        # Manhatten distance heuristic
        return abs(node.position[0] - self.goal_node.position[0]) + abs(node.position[1] - self.goal_node.position[1])

    def get_neighbors(self, node):
        neighbors = []
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            x, y = node.position[0] + dx, node.position[1] + dy
            if 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0]) and self.grid[x][y] == 0:
                neighbors.append(Node((x, y)))
        return neighbors

    def reconstruct_path(self, current_node):
        path = []
        while current_node is not None:
            path.append(current_node.position)
            current_node = current_node.parent
        return path[::-1]

    def find_path(self, start, goal):
        self.start_node = Node(start)
        self.goal_node = Node(goal)
        self.start_node.g_cost = 0
        self.start_node.h_cost = self.heuristic(self.start_node)
        heapq.heappush(self.open_set, self.start_node)

        while self.open_set:
            current_node = heapq.heappop(self.open_set)

            if current_node == self.goal_node:
                return self.reconstruct_path(current_node)

            self.closed_set.add(current_node)

            for neighbor in self.get_neighbors(current_node):
                if neighbor in self.closed_set:
                    continue

                tentative_g_cost = current_node.g_cost + 1  # Assuming uniform edge cost

                if tentative_g_cost < neighbor.g_cost:
                    neighbor.parent = current_node
                    neighbor.g_cost = tentative_g_cost
                    neighbor.h_cost = self.heuristic(neighbor)
                    heapq.heappush(self.open_set, neighbor)

        return None

# Example usage:
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0]
]

astar = AStar(grid)
start = (0, 0)
goal = (4, 4)
path = astar.find_path(start, goal)
print("Path found:", path)
