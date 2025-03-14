import heapq
import math

class BidirectionalAStar:
    def __init__(self, graph, s_start, s_goal):
        self.graph = graph
        self.s_start = s_start
        self.s_goal = s_goal
        
        self.OPEN_fore = []  # OPEN set for forward searching
        self.OPEN_back = []  # OPEN set for backward searching
        self.CLOSED_fore = []  # CLOSED set for forward
        self.CLOSED_back = []  # CLOSED set for backward
        self.PARENT_fore = {}  # recorded parent for forward
        self.PARENT_back = {}  # recorded parent for backward
        self.g_fore = {node: math.inf for node in graph}  # cost to come for forward
        self.g_back = {node: math.inf for node in graph}  # cost to come for backward

    def init(self):
        self.g_fore[self.s_start] = 0
        self.g_back[self.s_goal] = 0
        self.PARENT_fore[self.s_start] = None
        self.PARENT_back[self.s_goal] = None
        heapq.heappush(self.OPEN_fore, (self.f_value_fore(self.s_start), self.s_start))
        heapq.heappush(self.OPEN_back, (self.f_value_back(self.s_goal), self.s_goal))

    def searching(self):
        self.init()
        
        while self.OPEN_fore and self.OPEN_back:
            # Forward search step
            _, s_fore = heapq.heappop(self.OPEN_fore)
            if s_fore in self.PARENT_back:
                return self.extract_path(s_fore), self.CLOSED_fore, self.CLOSED_back
            self.CLOSED_fore.append(s_fore)
            self.expand_node(s_fore, self.OPEN_fore, self.PARENT_fore, self.g_fore, self.f_value_fore)
            
            # Backward search step
            _, s_back = heapq.heappop(self.OPEN_back)
            if s_back in self.PARENT_fore:
                return self.extract_path(s_back), self.CLOSED_fore, self.CLOSED_back
            self.CLOSED_back.append(s_back)
            self.expand_node(s_back, self.OPEN_back, self.PARENT_back, self.g_back, self.f_value_back)
        
        return None, self.CLOSED_fore, self.CLOSED_back  # No path found
    
    def expand_node(self, node, open_list, parent, g, f_value):
        for neighbor, cost in self.graph[node]:
            new_cost = g[node] + cost
            if new_cost < g.get(neighbor, math.inf):
                g[neighbor] = new_cost
                parent[neighbor] = node
                heapq.heappush(open_list, (f_value(neighbor), neighbor))
    
    def extract_path(self, s_meet):
        path = []
        node = s_meet
        while node:
            path.append(node)
            node = self.PARENT_fore[node]
        path.reverse()
        node = self.PARENT_back[s_meet]
        while node:
            path.append(node)
            node = self.PARENT_back[node]
        return path
    
    def f_value_fore(self, node):
        return self.g_fore[node] + self.heuristic(node, self.s_goal)
    
    def f_value_back(self, node):
        return self.g_back[node] + self.heuristic(node, self.s_start)
    
    def heuristic(self, node, goal):
        return 0  


def main():
    graph = {
        'A': [('B', 5), ('F', 3)],
        'B': [('A', 5), ('C', 2), ('G', 3)],
        'C': [('B', 2), ('D', 6), ('H', 10)],
        'D': [('C', 6), ('E', 3)],
        'E': [('D', 3), ('F', 8), ('H', 5)],
        'F': [('A', 3), ('E', 8), ('G', 7)],
        'G': [('B', 3), ('F', 7), ('H', 2)],
        'H': [('C', 10), ('E', 5), ('G', 2)],
    }
    s_start = 'A'
    s_goal = 'H'
    
    bastar = BidirectionalAStar(graph, s_start, s_goal)
    path, visited_fore, visited_back = bastar.searching()
    
    if path:
        print("Path found:", " -> ".join(path))
    else:
        print("No path found")
    
    print("Nodes visited in forward search:", " -> ".join(visited_fore))
    print("Nodes visited in backward search:", " -> ".join(visited_back))

if __name__ == '__main__':
    main()
