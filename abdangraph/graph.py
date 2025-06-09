

def new_graph(number_of_nodes = 1, nodes_name = None):
    adj = dict()
    if(nodes_name != None):
        for node in nodes_name:
            adj[node] = list()
    else:
        for i in range(1,number_of_nodes+1):
            adj[i] = list()
    return adj

def add_connection(adj, u, v, w, directed = False):
    if(not directed):
        adj[u].append({v: w})
        adj[v].append({u: w})
    else:
        adj[u].append({v: w})






