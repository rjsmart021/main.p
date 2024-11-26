# Task 1: Define Graph Representation
class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}

    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight  # For undirected graph

    def get_neighbors(self, vertex):
        return self.vertices.get(vertex, {})

# Example usage
graph = Graph()
graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_edge('A', 'B', 5)
graph.add_edge('B', 'C', 3)
graph.add_edge('A', 'C', 10)

print(graph.vertices)  # {'A': {'B': 5, 'C': 10}, 'B': {'A': 5, 'C': 3}, 'C': {'B': 3, 'A': 10}}


# Task 2
import heapq

def dijkstra(graph, start):
    distances = {vertex: float('inf') for vertex in graph.vertices}
    distances[start] = 0
    priority_queue = [(0, start)]
    previous_nodes = {vertex: None for vertex in graph.vertices}

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        if current_distance > distances[current_vertex]:
            continue

        for neighbor, weight in graph.get_neighbors(current_vertex).items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_vertex
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances, previous_nodes

# Helper function to reconstruct the path
def reconstruct_path(previous_nodes, target):
    path = []
    while target is not None:
        path.insert(0, target)
        target = previous_nodes[target]
    return path

# Task 3: Test the Algorithm Implementation
# Example graph
graph = Graph()
graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_vertex('D')
graph.add_edge('A', 'B', 1)
graph.add_edge('B', 'C', 2)
graph.add_edge('A', 'C', 4)
graph.add_edge('C', 'D', 1)

# Run Dijkstra's algorithm
distances, previous_nodes = dijkstra(graph, 'A')

# Output distances and paths
print("Shortest distances:", distances)  # {'A': 0, 'B': 1, 'C': 3, 'D': 4}
for node in graph.vertices:
    path = reconstruct_path(previous_nodes, node)
    print(f"Path to {node}:", path)  # Example: Path to D: ['A', 'B', 'C', 'D']

# Task 4

# If N is the number of vertices and E is the number of edges
# The time complexity for adding the vertices is O(N) and added the edges is O(E)
    
# Dijktra's Algorithm to set up the distance dictionary it is O(N).
# Each vertex is O(N).
# For the edges and updating the queue it would take O(E log N) 
