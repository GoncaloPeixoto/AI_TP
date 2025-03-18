import csv
import heapq

class DStar:
    def __init__(self, graph, s_start, s_goal, waypoints, weight_index):
        self.graph = graph
        self.s_start = s_start
        self.s_goal = s_goal
        self.waypoints = waypoints
        self.weight_index = weight_index
        self.OPEN = set()
        self.t = {}
        self.PARENT = {}
        self.h = {}
        self.k = {}
        self.path = []
    
    def init(self):
        for node in self.graph:
            self.t[node] = 'NEW'
            self.k[node] = float('inf')
            self.PARENT[node] = None
            self.h[node] = 0
    
    def run(self):
        self.init()
        full_path = []
        current_start = self.s_start
        total_toll = 0
        total_fuel = 0
        total_distance = 0
        
        for waypoint in self.waypoints + [self.s_goal]:
            self.s_goal = waypoint
            self.insert(self.s_goal, 0)
            while self.OPEN:
                self.process_state()
                if self.t.get(current_start, 'NEW') == 'CLOSED':
                    break
            partial_path = self.extract_path(current_start, self.s_goal)
            if not partial_path:
                return []
            full_path.extend(partial_path[:-1])  # Avoid duplicate nodes in path
            current_start = self.s_goal
            self.init()  # Reset for the next segment
        full_path.append(self.s_goal)
        
        # Calculate total costs
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
        
        return full_path
    
    def process_state(self):
        s = self.min_state()
        if s is None:
            return -1
        k_old = self.get_k_min()
        self.delete(s)
        
        for neighbor, toll, fuel, distance in self.graph.get(s, []):
            if self.weight_index == 3:
                cost = toll + fuel + distance
            else:
                cost = [toll, fuel, distance][self.weight_index]
            
            if self.t.get(neighbor, 'NEW') == 'NEW' or self.h.get(neighbor, float('inf')) > self.h.get(s, float('inf')) + cost:
                self.PARENT[neighbor] = s
                self.insert(neighbor, self.h.get(s, float('inf')) + cost)
    
    def extract_path(self, s_start, s_goal):
        path = [s_start]
        s = self.PARENT.get(s_start)
        while s and s != s_goal:
            path.append(s)
            s = self.PARENT.get(s)
        if s:
            path.append(s_goal)
        return path
    
    def min_state(self):
        if not self.OPEN:
            return None
        return min(self.OPEN, key=lambda x: self.k.get(x, float('inf')))
    
    def get_k_min(self):
        if not self.OPEN:
            return -1
        return min(self.k.values())
    
    def insert(self, s, h_new):
        self.h[s] = h_new
        self.k[s] = h_new
        self.t[s] = 'OPEN'
        self.OPEN.add(s)
    
    def delete(self, s):
        if self.t.get(s, 'NEW') == 'OPEN':
            self.t[s] = 'CLOSED'
        self.OPEN.remove(s)

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
    
    dstar = DStar(graph, start, end, waypoints, criteria)
    path = dstar.run()
    
    if path:
        print(f"Optimal path: {' -> '.join(path)}")
    else:
        print("No path found.")

if __name__ == "__main__":
    main()