import csv

class Graph:
    def __init__(self, adjacency_list, heuristics):
        self.adjacency_list = adjacency_list
        self.heuristics = heuristics  # Store heuristic values

    def get_neighbors(self, v):
        return self.adjacency_list.get(v, [])

    def h(self, n):
        """Retrieve the minimum heuristic value for node `n`."""
        return min(self.heuristics.get(n, (0, 0, 0)))  # Default to (0,0,0) if not found

    def a_star_algorithm(self, start_node, stop_node):
        open_list = set([start_node])
        closed_list = set([])
        g = {start_node: 0}  # Travel cost
        h_cost = {start_node: self.h(start_node)}  # Heuristic cost
        parents = {start_node: start_node}

        while open_list:
            n = None
            for v in open_list:
                # Choose the node with the lowest (g + h) cost
                if n is None or (g[v] + h_cost[v]) < (g[n] + h_cost[n]):
                    n = v

            if n is None:
                print('Path does not exist!')
                return None

            if n == stop_node:
                reconst_path = []
                total_travel_cost = g[n]  # Travel cost only

                # Reconstruct the path while accumulating heuristic costs
                total_heuristic_cost = 0
                current = n
                while parents[current] != current:
                    total_heuristic_cost += self.h(current)
                    reconst_path.append(current)
                    current = parents[current]
                reconst_path.append(start_node)
                reconst_path.reverse()

                total_cost = total_travel_cost + total_heuristic_cost  # Combined cost
                print(f'Path found: {reconst_path}')
                print(f'Total travel cost: {total_travel_cost}')
                print(f'Total heuristic cost: {total_heuristic_cost}')
                print(f'Total cost (travel + heuristic): {total_cost}')
                return reconst_path

            for (m, weight) in self.get_neighbors(n):
                new_g = g[n] + weight
                new_h = self.h(m)  # Choose heuristic cost for this node
                total_new_cost = new_g + new_h

                print(f"Node {m}: g={new_g}, h={new_h}, f=g+h={total_new_cost}")  # Print the heuristic cost for each node

                if m not in open_list and m not in closed_list:
                    open_list.add(m)
                    parents[m] = n
                    g[m] = new_g
                    h_cost[m] = new_h  # Store heuristic cost for this node
                elif new_g < g[m]:  # If new path is better, update
                    g[m] = new_g
                    h_cost[m] = new_h
                    parents[m] = n
                    if m in closed_list:
                        closed_list.remove(m)
                        open_list.add(m)

            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None


def load_adjacency_list_from_csv(file_path):
    adjacency_list = {}
    heuristics = {}

    with open(file_path, mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            if len(row) < 5:  # Ensure enough data: Node, Neighbors, Weights, Heuristics
                print(f"Skipping malformed row: {row}")
                continue

            node = row[0]
            neighbors = []
            
            # Extract neighbors and weights
            for i in range(1, len(row) - 3, 2):
                try:
                    neighbor = row[i]
                    weight = int(row[i + 1])
                    neighbors.append((neighbor, weight))
                except (IndexError, ValueError):
                    print(f"Skipping invalid data in row: {row}")

            # Extract heuristic values
            try:
                h1, h2, h3 = map(int, row[-3:])
                heuristics[node] = (h1, h2, h3)
            except ValueError:
                print(f"Skipping invalid heuristics in row: {row}")
                heuristics[node] = (0, 0, 0)  # Default heuristic values

            adjacency_list[node] = neighbors

    return adjacency_list, heuristics

# Load the adjacency list and heuristic values from the CSV file
file_path = "data2.csv"
adjacency_list, heuristics = load_adjacency_list_from_csv(file_path)

# Create a graph instance with heuristic values
graph1 = Graph(adjacency_list, heuristics)

# Run A* algorithm
graph1.a_star_algorithm('A', 'H')
