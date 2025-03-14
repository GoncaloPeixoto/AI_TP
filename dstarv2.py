import csv
import heapq

class DStar:
    def __init__(self, graph, s_start, s_goal, weight_index):
        self.graph = graph
        self.s_start = s_start
        self.s_goal = s_goal
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
        self.insert(self.s_goal, 0)
        while self.OPEN:
            self.process_state()
            if self.t.get(self.s_start, 'NEW') == 'CLOSED':
                break
        self.path = self.extract_path()
        return self.path
    
    def process_state(self):
        s = self.min_state()
        if s is None:
            return -1
        k_old = self.get_k_min()
        self.delete(s)
        
        for neighbor, toll, fuel, distance in self.graph.get(s, []):
            cost = [toll, fuel, distance][self.weight_index]
            if self.t.get(neighbor, 'NEW') == 'NEW' or self.h.get(neighbor, float('inf')) > self.h.get(s, float('inf')) + cost:
                self.PARENT[neighbor] = s
                self.insert(neighbor, self.h.get(s, float('inf')) + cost)
    
    def extract_path(self):
        path = [self.s_start]
        s = self.PARENT.get(self.s_start)
        while s and s != self.s_goal:
            path.append(s)
            s = self.PARENT.get(s)
        if s:
            path.append(self.s_goal)
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
    with open(file_path, mode='r', encoding='utf-8') as file:  # Specify encoding
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
    
    print("Available cities:")
    for city in graph.keys():
        print(city)
    
    start = input("Enter the origin city: ").strip()
    end = input("Enter the destination city: ").strip()
    
    if start not in graph or end not in graph:
        print("Invalid cities entered. Please choose from the list.")
        return
    
    criteria = int(input("Choose optimization criteria (0: Toll, 1: Fuel, 2: Distance): "))
    if criteria not in [0, 1, 2]:
        print("Invalid choice.")
        return
    
    dstar = DStar(graph, start, end, criteria)
    path = dstar.run()
    
    if path:
        print(f"Optimal path: {' -> '.join(path)}")
    else:
        print("No path found.")

if __name__ == "__main__":
    main()
