import math

def calculate_distance(point1, point2):
    x1, y1 = map(float, point1)
    x2, y2 = map(float, point2)
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance

def calculate_total_distance(path):
    total_distance = 0
    for i in range(len(path) - 1):
        total_distance += calculate_distance(path[i], path[i + 1])
    return total_distance

path_file_path = '/home/jason/EECE5550/hw/HW4/astar/data/path.txt'
with open(path_file_path, 'r') as file:
    lines = file.readlines()

path_points = [line.strip().split() for line in lines]

total_distance = calculate_total_distance(path_points)

print(f'Total distance of the path: {total_distance}')
