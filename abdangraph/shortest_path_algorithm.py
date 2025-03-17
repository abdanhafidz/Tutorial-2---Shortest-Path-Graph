import heapq

def dijkstra(adj:dict, start):
    heap = [(0, start)]
    distances = {node: float('inf') for node in adj}
    distances[start] = 0
    while heap:
        curr_dist, curr_node = heapq.heappop(heap)
        if curr_dist > distances[curr_node]:
            continue
        for connection in adj[curr_node]:
            neighbor, weight = list(connection.items())[0]
            distance = curr_dist + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(heap, (distance, neighbor))
    return distances

import heapq

def a_star(adj: dict, start, goal, heuristic):
    heap = [(0, start)]
    distances = {node: float('inf') for node in adj}
    heuristic_scores = {node: float('inf') for node in adj}
    
    distances[start] = 0
    heuristic_scores[start] = heuristic(start, goal)

    while heap:
        curr_dist, curr_node = heapq.heappop(heap)
        if curr_node == goal:
            return distances, heuristic_scores  # ✅ Return lengkap
        
        for connection in adj[curr_node]:  
            neighbor, weight = list(connection.items())[0]
            new_cost = curr_dist + weight
            if new_cost < distances[neighbor]:
                distances[neighbor] = new_cost
                heuristic_scores[neighbor] = heuristic(neighbor, goal)  # ✅ Simpan heuristic score
                priority = new_cost + heuristic_scores[neighbor]
                heapq.heappush(heap, (priority, neighbor))
    
    return distances, heuristic_scores  # ✅ Return kedua nilai



def bellman_ford(adj, start):
    distances = {node: float('inf') for node in adj}
    distances[start] = 0
    
    for _ in range(len(adj) - 1):
        for node in adj:
            for connection in adj[node]:  
                neighbor, weight = list(connection.items())[0]
                if distances[node] + weight < distances[neighbor]:
                    distances[neighbor] = distances[node] + weight
    
    for node in adj:
        for connection in adj[node]:  
            neighbor, weight = list(connection.items())[0]
            if distances[node] + weight < distances[neighbor]:
                raise ValueError("Graph contains a negative-weight cycle")
    return distances

def floyd_warshall(adj: dict):
    nodes = list(adj.keys())
    
    # Inisialisasi matriks jarak dengan nilai tak hingga
    dist = {i: {j: float('inf') for j in nodes} for i in nodes}
    
    # Jarak ke dirinya sendiri adalah 0
    for node in nodes:
        dist[node][node] = 0
    
    # Inisialisasi jarak langsung berdasarkan adjacency list
    for u in adj:
        for connection in adj[u]:  
            v, w = list(connection.items())[0]
            dist[u][v] = w  # Menetapkan bobot langsung jika ada
    
    # Algoritma Floyd-Warshall
    for k in nodes:
        for i in nodes:
            for j in nodes:
                if dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
    
    return dist
