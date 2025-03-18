import pandas as pd
import heapq
from collections import defaultdict


class LrtAStar:
    def __init__(self, csv_file, start, goal, N, w_toll=1, w_fuel=1, w_distance=1):
        self.graph = self.load_graph_from_csv(csv_file)
        self.s_start, self.s_goal = start.strip(), goal.strip()
        self.N = N
        self.weights = (w_toll, w_fuel, w_distance)
        self.visited = []
        self.path = []
        self.h_table = {}
        self.init_heuristic()

        print("Grafo carregado com os seguintes nós:")
        for node in self.graph:
            print(f"  {node}")  # Debug

        if self.s_start not in self.graph:
            print(f"Erro: A cidade de origem '{self.s_start}' não está no grafo.")
        if self.s_goal not in self.graph:
            print(f"Erro: A cidade de destino '{self.s_goal}' não está no grafo.")

    def load_graph_from_csv(self, csv_file):
        df = pd.read_csv(csv_file)
        graph = defaultdict(list)

        for _, row in df.iterrows():
            origem = str(row['origin_city']).strip()
            destino = str(row['destination_city']).strip()
            toll = float(row['toll'])
            fuel = float(row['fuel'])
            distance = float(row['distance_km'])

            graph[origem].append((destino, toll, fuel, distance))
            graph[destino].append((origem, toll, fuel, distance))

        return graph

    def init_heuristic(self):
        for node in self.graph:
            self.h_table[node] = float("inf")

    def compute_cost(self, toll, fuel, distance):
        w_toll, w_fuel, w_distance = self.weights
        return w_toll * toll + w_fuel * fuel + w_distance * distance

    def lrta_star(self, max_iterations=1000):
        print(f"A iniciar LRTA* de {self.s_start} para {self.s_goal}")

        if self.s_start not in self.graph or self.s_goal not in self.graph:
            print("Erro: Nó inicial ou final não encontrado no grafo.")
            return None

        current = self.s_start
        path = []
        g_values = {current: 0}
        visited = set()
        iterations = 0

        priority_queue = []
        heapq.heappush(priority_queue, (0, current, 0, 0, 0))

        while priority_queue and iterations < max_iterations:
            total_cost, current, total_toll, total_fuel, total_distance = heapq.heappop(priority_queue)
            if current in visited:
                continue

            path.append(current)
            visited.add(current)

            if current == self.s_goal:
                print(
                    f"Path found: {path} with total cost: {total_cost}, toll: {total_toll}, fuel: {total_fuel}, distance: {total_distance}")
                return path, total_cost

            neighbors = self.graph.get(current, [])
            if not neighbors:
                print(f"Erro: Nenhum próximo nó encontrado a partir de {current}. Caminho bloqueado.")
                print(f"Caminho percorrido até agora: {path}")
                return None

            for neighbor, toll, fuel, distance in neighbors:
                if neighbor in visited:
                    continue
                new_cost = self.compute_cost(toll, fuel, distance) + total_cost
                heapq.heappush(priority_queue,
                               (new_cost, neighbor, total_toll + toll, total_fuel + fuel, total_distance + distance))

            iterations += 1

        print("Erro: Máximo de iterações atingido. Loop infinito detectado!")
        print(f"Caminho percorrido até agora: {path}")
        return None



csv_filename = "cities_nodes_special.csv"
start_node = 'Madrid'
goal_node = 'Bucharest'
num_expansions = 250

lrta = LrtAStar(csv_filename, start_node, goal_node, num_expansions, w_toll=1, w_fuel=1,w_distance=1)
lrta.lrta_star()