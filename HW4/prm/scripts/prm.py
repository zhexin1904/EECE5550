import cv2
import numpy as np
import networkx as nx
from bresenham import bresenham
import pdb

class PRMPlanner:
    def __init__(self, img_path):
        self.img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        _, self.img = cv2.threshold(self.img, 127, 255, cv2.THRESH_BINARY)

    def is_edge_valid(self, start, end):
        start = np.array(start, dtype=int)
        end = np.array(end, dtype=int)

        line = list(bresenham(start[0], start[1], end[0], end[1]))
        line_array = np.array(line)

        if np.any(line_array[:, 0] < 0) or np.any(line_array[:, 0] >= self.img.shape[0]) or \
           np.any(line_array[:, 1] < 0) or np.any(line_array[:, 1] >= self.img.shape[1]) or \
           np.any(self.img[line_array[:, 0], line_array[:, 1]] == 0):
            return False

        return True

    def construct_PRM(self, start, goal, max_distance, num_samples):
        prm_graph = nx.Graph()

        for i in range(2, num_samples + 2):
            random_point = np.random.randint(0, self.img.shape[0]), np.random.randint(0, self.img.shape[1])

            if not np.any(self.img[random_point[0], random_point[1]] == 0):
                random_point = tuple(np.round(random_point).astype(int))
                self.add_VERTEX(random_point, prm_graph, max_distance)

        return prm_graph

    def add_VERTEX(self, point, graph, max_distance):
        graph.add_node(point, pos=point)
        for data in graph.nodes():
            distance = np.linalg.norm(np.array(data) - np.array(point))

            if distance <= max_distance and self.is_edge_valid(data, point):
                graph.add_edge(point, data, weight=distance)

    def draw_PRM_on_map(self, prm_graph, start, goal, path):
        img_with_nodes = self.img.copy()

        pos = {node: data['pos'] for node, data in prm_graph.nodes(data=True)}

        # Draw nodes
        for node, position in pos.items():
            cv2.circle(img_with_nodes, tuple(position[::-1]), 2, (255, 0, 0), -1)  # Swap x and y

        # Draw edges
        for edge in prm_graph.edges():
            start_node, end_node = edge
            start_pos = pos[start_node]
            end_pos = pos[end_node]
            cv2.line(img_with_nodes, tuple(start_pos[::-1]), tuple(end_pos[::-1]), (0, 255, 0), 1)  # Swap x and y

        # Draw start and goal nodes
        cv2.circle(img_with_nodes, tuple(start[::-1]), 10, (0, 0, 255), -1)  # Swap x and y
        cv2.circle(img_with_nodes, tuple(goal[::-1]), 10, (0, 0, 255), -1)  # Swap x and y

        # Draw lines connecting consecutive points in the path
        path_array = np.array(path)
        for i in range(len(path_array) - 1):
            start_point = tuple(path_array[i])
            end_point = tuple(path_array[i + 1])
            cv2.line(img_with_nodes, tuple(start_point[::-1]), tuple(end_point[::-1]), (0, 0, 255), 2)

        cv2.imshow('PRM on Occupancy Map', img_with_nodes)
        cv2.imwrite('output_image.png', img_with_nodes)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def dist(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def main():
    prm_planner = PRMPlanner('/home/jason/EECE5550/hw/HW4/astar/data/occupancy_map.png')

    start_point = (635, 140)
    goal_point = (350, 400)
    max_distance = 75

    prm_graph = prm_planner.construct_PRM(start_point, goal_point, max_distance, num_samples=5000)
    prm_planner.add_VERTEX(tuple(start_point), prm_graph, max_distance)
    prm_planner.add_VERTEX(tuple(goal_point), prm_graph, max_distance)

    path = nx.astar_path(prm_graph, start_point, goal_point, heuristic=dist)
    path_length = sum(dist(path[i], path[i + 1]) for i in range(len(path) - 1))
    print(f'path Length of the Path: {path_length}')

    prm_planner.draw_PRM_on_map(prm_graph, start_point, goal_point, path)

if __name__ == "__main__":
    main()
