import csv
import sys
sys.stdout.reconfigure(encoding='utf-8')

class LRTAStar:
    def __init__(self, graph, s_start, s_goal, waypoints, weight_index):
        self.graph = graph
        self.s_start = s_start
        self.s_goal = s_goal
        self.waypoints = waypoints
        self.weight_index = weight_index
        self.h = {node: 0 for node in self.graph}  # Heuristic values
        self.path = []
        self.all_paths = []  # Store all paths explored
    
    def run(self):
        full_path = []
        current_start = self.s_start
        total_toll, total_fuel, total_distance = 0, 0, 0
        
        for waypoint in self.waypoints + [self.s_goal]:
            self.s_goal = waypoint
            partial_path = self.lrta_star(current_start, self.s_goal)
            if not partial_path:
                return []
            full_path.extend(partial_path[:-1])  # Avoid duplicate nodes in path
            current_start = self.s_goal
        full_path.append(self.s_goal)
        
        # Calculate total costs
        for i in range(len(full_path) - 1):
            for neighbor, toll, fuel, distance in self.graph.get(full_path[i], []):
                if neighbor == full_path[i + 1]:
                    total_toll += toll
                    total_fuel += fuel
                    total_distance += distance
                    break
        
        print("All paths explored:")
        for path in self.all_paths:
            print(" -> ".join(path))
        
        print("\n========================\n")
        print("Best path:")
        print(" -> ".join(full_path))
        print("\n========================\n")
        
        print(f"Total Toll Cost: {total_toll}")
        print(f"Total Fuel Cost: {total_fuel}")
        print(f"Total Distance: {total_distance} km")
        
        return full_path
    
    def lrta_star(self, s_start, s_goal):
        s = s_start
        path = [s]
        visited = set()  # Track visited nodes to prevent loops
        
        while s != s_goal:
            visited.add(s)
            min_cost = float('inf')
            best_next = None
            
            for neighbor, toll, fuel, distance in self.graph.get(s, []):
                if neighbor in visited:
                    continue  # Skip visited nodes to prevent loops
                
                cost = toll if self.weight_index == 0 else fuel if self.weight_index == 1 else distance if self.weight_index == 2 else toll + fuel + distance
                cost += self.h[neighbor]  # f = g + h
                
                if cost < min_cost:
                    min_cost = cost
                    best_next = neighbor
            
            if best_next is None:
                return []  # No path found
            
            self.h[s] = min_cost  # Update heuristic estimate
            self.h[best_next] = min_cost  # Improve heuristic update
            s = best_next
            path.append(s)
        
        self.all_paths.append(path[:])  # Store each attempted path
        return path


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
    
    criteria = int(input("Choose optimization criteria (0: Toll, 1: Fuel, 2: Distance, 3: All three combined): "))
    if criteria not in [0, 1, 2, 3]:
        print("Invalid choice.")
        return
    
    lrta = LRTAStar(graph, start, end, waypoints, criteria)
    path = lrta.run()
    
    if path:
        print(f"Optimal path: {' -> '.join(path)}")
    else:
        print("No path found.")


if __name__ == "__main__":
    main()