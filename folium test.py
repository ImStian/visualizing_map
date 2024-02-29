import folium
import random
import numpy as np

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

def distance(node1, node2):
    return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

def nearest_neighbor(rand_node, nodes):
    return min(nodes, key=lambda node: distance(node, rand_node))

def steer(from_node, to_node, max_distance):
    angle = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = from_node.x + max_distance * np.cos(angle)
    new_y = from_node.y + max_distance * np.sin(angle)
    return Node(new_x, new_y)

def is_obstacle_free(from_node, to_node, grid):
    # Check if the line between from_node and to_node is obstacle-free
    step_size = 1
    num_steps = int(distance(from_node, to_node) / step_size)

    for i in range(num_steps + 1):
        x = int(from_node.x + i * step_size * np.cos(np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)))
        y = int(from_node.y + i * step_size * np.sin(np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)))

        if not (0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] != 'land'):
            return False

    return True

def rrt_star(start, goal, grid, max_iter=500, max_distance=10):
    nodes = [start]

    for _ in range(max_iter):
        rand_node = Node(random.uniform(0, len(grid)), random.uniform(0, len(grid[0])))
        nearest_node = nearest_neighbor(rand_node, nodes)
        new_node = steer(nearest_node, rand_node, max_distance)

        if is_obstacle_free(nearest_node, new_node, grid):
            near_nodes = [node for node in nodes if distance(node, new_node) < max_distance]

            min_cost = nearest_node.cost + distance(nearest_node, new_node)
            best_parent = nearest_node

            for near_node in near_nodes:
                if is_obstacle_free(near_node, new_node, grid) and near_node.cost + distance(near_node, new_node) < min_cost:
                    best_parent = near_node
                    min_cost = near_node.cost + distance(near_node, new_node)

            new_node.cost = min_cost
            new_node.parent = best_parent

            nodes.append(new_node)

    # Trace back from the goal to find the path
    path = []
    current_node = nearest_neighbor(goal, nodes)
    while current_node:
        path.append((current_node.x, current_node.y))
        current_node = current_node.parent

    return path[::-1]

# Example grid with land and water
grid = [
    ['water', 'water', 'water', 'water', 'land'],
    ['water', 'land', 'water', 'water', 'land'],
    ['water', 'land', 'water', 'water', 'land'],
    ['water', 'water', 'water', 'land', 'land'],
    ['land', 'land', 'land', 'land', 'land']
]

start_node = Node(0, 0)
goal_node = Node(4, 4)

path = rrt_star(start_node, goal_node, grid)

if path:
    print("Path found:", path)

    # Create a Folium map
    m = folium.Map(location=[0, 0], zoom_start=10)

    # Mark the start and goal on the map
    folium.Marker(location=[start_node.x, start_node.y], popup='Start').add_to(m)
    folium.Marker(location=[goal_node.x, goal_node.y], popup='Goal').add_to(m)

    # Draw the path on the map
    folium.PolyLine(locations=path, color='blue').add_to(m)

    # Display the map
    m.save('rrt_star_path_map.html')
    print("Map saved as 'rrt_star_path_map.html'")
else:
    print("No path found.")
