import csv
import heapq
from datetime import datetime
start=datetime.now()


class AStar:
    def __init__(self, graph, s_start, s_goal, waypoints, weight_index):
        self.graph = graph
        self.s_start = s_start
        self.s_goal = s_goal
        self.waypoints = waypoints
        self.weight_index = weight_index
        self.OPEN = []
        self.CLOSED = set()
        self.PARENT = {}
        self.g = {}
        self.h = {}
        self.path = []

    def run(self):
        full_path = [self.s_start]
        current_start = self.s_start
        
        # Include waypoints between start and goal
        waypoints = self.waypoints + [self.s_goal]
        for waypoint in waypoints:
            self.s_goal = waypoint
            self.reset_search()
            if not self.find_path(current_start):  # Handle cases where no path is found
                print(f"No path found from {current_start} to {self.s_goal}.")
                return []
            full_path.extend(self.path[1:])  # Avoid duplicate nodes in the path
            current_start = self.s_goal
        
        self.calculate_costs(full_path)
        return full_path

    def reset_search(self):
        self.OPEN = []
        self.CLOSED = set()
        self.PARENT = {}
        self.g = {}
        self.h = {}
        self.path = []

    def find_path(self, current_start):
        # Initialize the starting point
        self.g[current_start] = 0
        self.h[current_start] = self.calculate_heuristic(current_start)
        heapq.heappush(self.OPEN, (self.g[current_start] + self.h[current_start], current_start))

        while self.OPEN:
            current_f, current_node = heapq.heappop(self.OPEN)
            if current_node == self.s_goal:
                self.reconstruct_path()
                return True  # Return True when the goal is reached

            self.CLOSED.add(current_node)
            
            for neighbor, toll, fuel, distance in self.graph.get(current_node, []):
                if neighbor in self.CLOSED:
                    continue

                # Calculate cost based on the selected weight index
                cost = [toll, fuel, distance][self.weight_index]
                new_g = self.g[current_node] + cost

                if neighbor not in self.g or new_g < self.g[neighbor]:
                    self.g[neighbor] = new_g
                    self.PARENT[neighbor] = current_node
                    self.h[neighbor] = self.calculate_heuristic(neighbor)
                    heapq.heappush(self.OPEN, (new_g + self.h[neighbor], neighbor))
        
        return False  # Return False if no path is found

    def reconstruct_path(self):
        current = self.s_goal
        path = []
        while current in self.PARENT:  # Check if current has a parent to prevent infinite loops
            path.append(current)
            current = self.PARENT[current]
        path.append(self.s_start)
        path.reverse()
        self.path = path

    def calculate_costs(self, full_path):
        total_toll, total_fuel, total_distance = 0, 0, 0
        for i in range(len(full_path) - 1):
            for neighbor, toll, fuel, distance in self.graph.get(full_path[i], []):
                if neighbor == full_path[i + 1]:
                    total_toll += toll
                    total_fuel += fuel
                    total_distance += distance
                    break

        print(f"Total Toll Cost: {total_toll}")
        print(f"Total Fuel Cost: {total_fuel}")
        print(f"Total Distance: {total_distance} km")

    def calculate_heuristic(self, node):
        # If coordinates were available, this could use Euclidean or Manhattan distance.
        return 0  # Keeping it as 0 but consider adding a better heuristic.

def read_csv(file_path):
    graph = {}
    with open(file_path, mode='r', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        for row in reader:
            origin, destination = row['origin_city'], row['destination_city']
            toll, fuel, distance = float(row['toll']), float(row['fuel']), float(row['distance_km'])

            if origin not in graph:
                graph[origin] = []
            if destination not in graph:
                graph[destination] = []

            graph[origin].append((destination, toll, fuel, distance))
            graph[destination].append((origin, toll, fuel, distance))  # Assuming bidirectional roads

    return graph

def main():
    file_path = 'cities_nodes_special.csv'
    graph = read_csv(file_path)
    
    city_list = list(graph.keys())
    print("Available cities:")
    for i, city in enumerate(city_list):
        print(f"{i}: {city}")
    
    try:
        start_index = int(input("Enter the number of the origin city: "))
        end_index = int(input("Enter the number of the destination city: "))
        waypoint_indices = input("Enter intermediate city numbers to pass through (comma-separated): ").strip()
        waypoints = [city_list[int(i.strip())] for i in waypoint_indices.split(',') if i.strip().isdigit()]
        
        start, end = city_list[start_index], city_list[end_index]
    except (ValueError, IndexError):
        print("Invalid selection. Please enter numbers corresponding to the available cities.")
        return
    
    criteria = int(input("Choose optimization criteria (0: Toll, 1: Fuel, 2: Distance): "))
    if criteria not in [0, 1, 2]:
        print("Invalid choice.")
        return
    
    

    astar = AStar(graph, start, end, waypoints, criteria)
    path = astar.run()

    if path:
        print(f"Optimal path: {' -> '.join(path)}")
    else:
        print("No path found.")

if __name__ == "__main__":
    main()

print(f"Run time: {datetime.now() - start}")
