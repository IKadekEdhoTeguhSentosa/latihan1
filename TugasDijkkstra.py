import heapq
from collections import deque

class Node():
    def __init__(self, state, action=None):
        self.state = state
        self.action = action if action else []
        self.parent = None  

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)

    def get_state(self):
        return self.state

    def get_parent(self):
        return self.parent

    def set_parent(self, node):
        self.parent = node

    def get_action(self):
        return self.action


class Problem():
    def __init__(self, knowledge, start_node, end_node):
        self.knowledge = knowledge
        self.start_node = start_node
        self.end_node = end_node

    def actionFunc(self, node, actiontype):
        if node in self.knowledge:
            ch_nodes = self.knowledge[node]
            action_map = {"turnLeft": 0, "goStraight": 1, "turnRight": 2}
            if len(ch_nodes) > action_map[actiontype]:
                return ch_nodes[action_map[actiontype]][0]  # Mengembalikan node
        return None

    #Depth-First Search Kodingannya
    def DFS(self, node, reached=None, result=None):
        if reached is None:
            reached = set()
        if result is None:
            result = []

        reached.add(node)
        result.append(node)

        if node == self.end_node:
            return result, True

        for action in node.get_action():
            leafnode = self.actionFunc(node, action)
            if leafnode and leafnode not in reached:
                leafnode.set_parent(node)
                result_path, found = self.DFS(leafnode, reached, result)
                if found:
                    return result_path, True

        return result, False

    #Breadth-First Search Kodingannya
    def BFS(self):
        reached = set()
        result = []
        queue = deque([self.start_node])
        reached.add(self.start_node)

        while queue:
            node = queue.popleft()
            result.append(node)

            if node == self.end_node:
                return result, True

            for action in node.get_action():
                leafnode = self.actionFunc(node, action)
                if leafnode and leafnode not in reached:
                    leafnode.set_parent(node)
                    reached.add(leafnode)
                    queue.append(leafnode)

        return result, False

    # Dijkstra Kodingannya
    def Dijkstra(self):
        priority_queue = []
        heapq.heappush(priority_queue, (0, self.start_node))  # (cost, node)
        distances = {node: float('inf') for node in self.knowledge}
        distances[self.start_node] = 0
        parent = {self.start_node: None}

        while priority_queue:
            current_cost, current_node = heapq.heappop(priority_queue)

            if current_node == self.end_node:
                return self.reconstruct_path(parent), True

            for neighbor, cost in self.knowledge.get(current_node, []):
                new_cost = current_cost + cost
                if new_cost < distances[neighbor]:
                    distances[neighbor] = new_cost
                    heapq.heappush(priority_queue, (new_cost, neighbor))
                    parent[neighbor] = current_node

        return [], False

    def reconstruct_path(self, parent):
        path = []
        node = self.end_node
        while node:
            path.append(node)
            node = parent[node]
        path.reverse()
        return path

    def get_ChildNodes(self, method):
        if method == "DFS":
            return self.DFS(self.start_node)
        elif method == "BFS":
            return self.BFS()
        elif method == "Dijkstra":
            return self.Dijkstra()


def main():
    node_A = Node("A", ["turnLeft", "goStraight"])
    node_B = Node("B", ["turnLeft", "goStraight", "turnRight"])
    node_C = Node("C", ["turnLeft", "goStraight"])
    node_D = Node("D")
    node_E = Node("E")
    node_F = Node("F")
    node_G = Node("G")

    knowledge = {
        node_A: [(node_B, 1), (node_C, 4)],
        node_B: [(node_D, 2), (node_E, 5), (node_F, 1)],
        node_C: [(node_F, 3), (node_G, 2)],
        node_D: [], node_E: [], node_F: [], node_G: []
    }

    pathFinding = Problem(knowledge, start_node=node_A, end_node=node_F)
    
    method = "Dijkstra"  # bisa diganti ke DFS, BFS, Dijkstra
    path_list, status = pathFinding.get_ChildNodes(method)

    if status:
        print(f"Jalur ditemukan menggunakan {method}:")
        for pt in path_list:
            print(pt.get_state())
    else:
        print("Target tidak ditemukan")

if __name__ == "__main__":
    main()
