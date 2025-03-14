import csv

class DStar:
    def __init__(self, s_start, s_goal, grid_size=(10, 10)):
        self.s_start, self.s_goal = s_start, s_goal
        self.x, self.y = grid_size

        self.OPEN = set()
        self.t = dict()
        self.PARENT = dict()
        self.h = dict()  # Store heuristics for target nodes
        self.k = dict()
        self.path = []
        self.visited = set()
        self.count = 0

        self.u_set = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def init(self):
        for node in self.neighbors:
            self.t[node] = 'NEW'
            self.k[node] = float('inf')
            self.PARENT[node] = None
            self.h[node] = self.calculate_heuristic(node, self.s_goal)

    def run(self, s_start, s_end):
        self.init()
        self.insert(s_end, 0)

        while True:
            self.process_state()
            if self.t.get(s_start, 'NEW') == 'CLOSED':
                break

        self.path = self.extract_path(s_start, s_end)
        self.print_path_and_cost(s_start, s_end)

    def print_path_and_cost(self, s_start, s_end):
        total_cost = 0
        total_heuristic = 0
        for i in range(len(self.path) - 1):
            total_cost += self.cost(self.path[i], self.path[i + 1])
            total_heuristic += self.h.get(self.path[i], 0)

        print("Path from start to goal:", self.path)
        print("Total path cost:", total_cost)
        print("Total heuristic cost:", total_heuristic)

    def extract_path(self, s_start, s_end):
        path = [s_start]
        s = self.PARENT.get(s_start)
        while s != s_end and s is not None:
            path.append(s)
            s = self.PARENT.get(s)
        path.append(s_end)
        return path

    def process_state(self):
        s = self.min_state()
        self.visited.add(s)

        if s is None:
            return -1

        k_old = self.get_k_min()
        self.delete(s)

        if k_old < self.h.get(s, float('inf')):
            for s_n in self.get_neighbor(s):
                if self.h.get(s_n, float('inf')) <= k_old and \
                        self.h.get(s, float('inf')) > self.h.get(s_n, float('inf')) + self.cost(s_n, s):
                    self.PARENT[s] = s_n
                    self.h[s] = self.h.get(s_n, float('inf')) + self.cost(s_n, s)

        if k_old == self.h.get(s, float('inf')):
            for s_n in self.get_neighbor(s):
                if self.t.get(s_n, 'NEW') == 'NEW' or \
                        (self.PARENT.get(s_n) == s and self.h.get(s_n, float('inf')) != self.h.get(s, float('inf')) + self.cost(s, s)) or \
                        (self.PARENT.get(s_n) != s and self.h.get(s_n, float('inf')) > self.h.get(s, float('inf')) + self.cost(s, s)):
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h.get(s, float('inf')) + self.cost(s, s))

        else:
            for s_n in self.get_neighbor(s):
                if self.t.get(s_n, 'NEW') == 'NEW' or \
                        (self.PARENT.get(s_n) == s and self.h.get(s_n, float('inf')) != self.h.get(s, float('inf')) + self.cost(s, s_n)):
                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h.get(s, float('inf')) + self.cost(s, s_n))
                else:
                    if self.PARENT.get(s_n) != s and \
                            self.h.get(s_n, float('inf')) > self.h.get(s, float('inf')) + self.cost(s, s_n):
                        self.insert(s, self.h.get(s, float('inf')))

                    else:
                        if self.PARENT.get(s_n) != s and \
                                self.h.get(s, float('inf')) > self.h.get(s_n, float('inf')) + self.cost(s_n, s) and \
                                self.t.get(s_n, 'NEW') == 'CLOSED' and \
                                self.h.get(s_n, float('inf')) > k_old:
                            self.insert(s_n, self.h.get(s_n, float('inf')))

        return self.get_k_min()

    def min_state(self):
        if not self.OPEN:
            return None
        return min(self.OPEN, key=lambda x: self.k.get(x, float('inf')))

    def get_k_min(self):
        if not self.OPEN:
            return -1
        return min([self.k.get(x, float('inf')) for x in self.OPEN])

    def insert(self, s, h_new):
        if self.t.get(s, 'NEW') == 'NEW':
            self.k[s] = h_new
        elif self.t.get(s, 'OPEN') == 'OPEN':
            self.k[s] = min(self.k.get(s, float('inf')), h_new)
        elif self.t.get(s, 'CLOSED') == 'CLOSED':
            self.k[s] = min(self.h.get(s, float('inf')), h_new)

        self.h[s] = h_new
        self.t[s] = 'OPEN'
        self.OPEN.add(s)

    def delete(self, s):
        if self.t.get(s, 'NEW') == 'OPEN':
            self.t[s] = 'CLOSED'
        self.OPEN.remove(s)

    def modify(self, s):
        self.modify_cost(s)

        while True:
            k_min = self.process_state()
            if k_min >= self.h.get(s, float('inf')):
                break

    def modify_cost(self, s):
        if self.t.get(s, 'NEW') == 'CLOSED':
            self.insert(s, self.h.get(self.PARENT.get(s), float('inf')) + self.cost(s, self.PARENT.get(s)))

    def get_neighbor(self, s):
        return self.neighbors.get(s, set())

    def cost(self, s_start, s_goal):
        if self.is_collision(s_start, s_goal):
            return float("inf")
        return self.neighbor_costs.get((s_start, s_goal), 1)

    def is_collision(self, s_start, s_end):
        return False

    def read_csv(self, file_path):
        self.neighbors = {}
        self.neighbor_costs = {}
        self.heuristics = {}  # To store heuristics for each target node
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header row
            for row in reader:
                source, target, weight, *heuristics = row  # Grab all columns after weight
                weight = int(weight)  # Convert weight to integer
                heuristics = [int(h) for h in heuristics]  # Convert all heuristic values to integers

                if source not in self.neighbors:
                    self.neighbors[source] = set()

                self.neighbors[source].add(target)
                self.neighbor_costs[(source, target)] = weight  # Store the cost between the nodes

                # Store all heuristics for the target node
                for i, heuristic in enumerate(heuristics):
                    self.h[(target, i)] = heuristic  # Store heuristics with index to distinguish them

    def calculate_heuristic(self, node, goal):
        node_x, node_y = ord(node) % 10, ord(node) // 10
        goal_x, goal_y = ord(goal) % 10, ord(goal) // 10
        return abs(node_x - goal_x) + abs(node_y - goal_y)


def main():
    # Ask the user for the start and end nodes
    s_start = input("Enter the start node: ").strip()
    s_end = input("Enter the end node: ").strip()

    print(f"Start node: {s_start}, End node: {s_end}")

    grid_size = (10, 10)
    dstar = DStar(s_start=s_start, s_goal=s_end, grid_size=grid_size)
    dstar.read_csv('data1.csv')  # Make sure to update the file name if needed
    dstar.run(s_start=s_start, s_end=s_end)


if __name__ == "__main__":
    main()
