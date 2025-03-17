def dfs(adj:dict, start, visited=None):
    if visited is None:
        visited = set()
    
    visited.add(start)
    print(start, end=' ')
    
    for connection in adj[start]:
        neighbor = list(connection.keys())[0]
        if  neighbor not in visited:
            dfs(adj, neighbor, visited)
import heapq

def bfs(adj, start):
    visited = set()
    priority_queue = [(0, start)]  # (weight, node)

    while priority_queue:
        weight, node = heapq.heappop(priority_queue)  # Ambil node dengan bobot terkecil
        if node not in visited:
            print(node, end=' ')
            visited.add(node)
            
            for neighbor_dict in adj[node]:  # Akses dictionary dalam list
                for neighbor, w in neighbor_dict.items():  # Ambil tetangga dan bobotnya
                    if neighbor not in visited:
                        heapq.heappush(priority_queue, (w, neighbor))  # Tambahkan ke priority queue

def backtracking(adj, start, goal, path=None, visited=None):
    if path is None:
        path = [start]
    if visited is None:
        visited = set()
    
    if start == goal:
        print(" -> ".join(map(str, path)))
        return
    
    visited.add(start)
    for neighbor in adj[start]:
        if neighbor not in visited:
            backtracking(adj, neighbor, goal, path + [neighbor], visited)
    
    visited.remove(start)