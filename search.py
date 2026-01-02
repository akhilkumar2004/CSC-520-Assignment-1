"""
search.py

This program will implement graph search algorithms (A* and RBFS) 
to find the optimal path from a start node to a goal node in a given graph from
a .csv file and .txt file. The hueristic used is the Euclidean distance between cities.

"""
import math
import csv 
import sys
from queue import PriorityQueue

def read_graph(filename):
    """
    Reads a graph from the graph.txt file and returns it as a 2D
    list of floats.
    Input: filename (str) - path to the graph file
    Output: graph (list of list of float) - adjacency matrix
    """
    with open(filename, 'r') as f:
        return [list(map(float, line.strip().split())) for line in f]

def read_cities(filename='cities.csv'):
    """
    Reads city names from a CSV file and returns them as a list.
    Input: filename (str) - path to the CSV file
    Output: cities (dict) - dictionary with city names as keys and (x, y)
    """
    cities = {}
    with open(filename, 'r') as f:
        file_reader = csv.reader(f)
        next(file_reader)  # Skip header row
        for row in file_reader:
            name = row[0] # Nodes
            x = float(row[2]) # X coordinate
            y = float(row[3]) # Y coordinate
            cities[name] = (x, y) # Store as tuple (x, y)
    return cities

def euclidean_distance(city1, city2):
    """
    Calculates the Euclidean distance between two cities.
    Input: city1 and city2 are tuples (x, y)
    Output: Euclidean distance (float)
    """
    euclidean_distance = math.sqrt((city1[0] - city2[0]) ** 2 + (city1[1] - city2[1]) ** 2)
    return euclidean_distance

# A* Search Algorithm

def a_star_search(graph, cities, start, goal):
    """
    Implements the A* search algorithm to find the optimal path
    from start to goal in the given graph using the provided heuristic.

    Inputs: graph (list of list of float) - adjacency matrix
            cities (dict) - dictionary with city names as keys and (x, y) coordinates
            start (int) - index of the start node
    
    Outputs: path (list of str) - list of city names in the optimal path
            cost (float) - total cost of the optimal path
            expanded_node_count (int) - number of nodes expanded during the search
    """
    city_names = list(cities.keys()) # List of city names
    front = PriorityQueue() # Priority queue for the frontier
    front.put((0, start)) # (f(n), node)
    parent_map = {start: None} # To reconstruct the path
    g_cost = {start: 0} # Cost from start to the current node
    expanded_node_count = 0

    while not front.empty():
        _, current = front.get() # Get node with lowest f(n) = g(n) + h(n)
        expanded_node_count += 1 # Increment expanded node count

        if current == goal: # Goal found
            break
        
        for neighbor, weight in enumerate(graph[current]):
            if weight > 0:
                tentative_g_cost = g_cost[current] + weight # Cost to neighbor (g(n))
                if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]: # Better path found
                    g_cost[neighbor] = tentative_g_cost # Update g(n)
                    h = euclidean_distance(cities[city_names[neighbor]], cities[city_names[goal]]) #h(n)
                    f = tentative_g_cost + h # f(n) = g(n) + h(n)
                    front.put((f, neighbor)) # Add neighbor to the frontier
                    parent_map[neighbor] = current # Update parent map
    
    path = [] # To store the final path
    node = goal 
    if goal not in g_cost: # Goal not reachable 
        return [], float("inf"), expanded_node_count # Return empty path and infinite cost
    while node is not None:
        path.append(city_names[node]) 
        node = parent_map[node] 
    path.reverse()
    return path, g_cost[goal], expanded_node_count

    

# RBFS Algorithm
def rbfs(graph, cities, start, goal):
    """
    Inputs: graph (list of list of float) - adjacency matrix
            cities (dict) - dictionary with city names as keys and (x, y) coordinates
            start (int) - index of the start node
            goal (int) - index of the goal node
    
    Outputs: path (list of str) - list of city names in the optimal path
    """
    city_names = list(cities.keys()) # List of city names
    states = {"expanded": 0} # To count expanded nodes in a dictionary for mutability

    def helper(node, goal, path, goal_cost, limit):
        """
        Recursive helper function for RBFS.
        Inputs: node (int) - current node index
                goal (int) - goal node index
                path (list of str) - current path
                goal_cost (dict) - cost from start to current node
                limit (float) - current f(n) limit

        Outputs: path (list of str) - optimal path if found, else None
        """
        states["expanded"] += 1 # Increment expanded node count

        path = path + [city_names[node]] # Update path with current node

        if node == goal:
            return path, goal_cost[node]
        
        successors = [] # List to store successors (neighbor, g(n), f(n))
        for neighbor, weight in enumerate(graph[node]): # Explore neighbors
            if weight > 0 and neighbor not in path: # Valid neighbor
                g = goal_cost[node] + weight # Cost to neighbor
                h = euclidean_distance(cities[city_names[neighbor]], cities[city_names[goal]]) # Heuristic
                f = g + h # f(n) = g(n) + h(n)
                successors.append((neighbor, g, f)) # Add to successors list
        
        if not successors: # No successors
            return None, float("inf") # Return failure


        # Explore best successor
        while successors:
            best_neighbor, g_best, f_best = successors[0] #
            if f_best > limit: # f(n) exceeds limit
                return None, f_best
           
            if len(successors) > 1: # More than one successor
                 next_limit = successors[1][2] # f(n) of second best
            else:
                float("inf") # No second best, set to infinity

            goal_cost[best_neighbor] = g_best # Update cost to best neighbor
            result, f_new = helper(best_neighbor, goal, path, goal_cost, min(limit, 
next_limit)) # Recursive call
            successors[0] = (best_neighbor, g_best, f_new) # Update f(n) after recursion
            successors.sort(key=lambda x: x[2]) # Re-sort successors

            # If a solution is found, return it
            if result is not None:
                return result, f_new
            
        return None, float("inf") # All successors explored without success
    
    path, cost = helper(start, goal, [], {start: 0}, float("inf")) # Initial call to helper
    return path, cost, states["expanded"] # Return path, cost, and expanded node count

                
# Main function to run the algorithms
if __name__ == "__main__":
    # Check command line arguments
    if len(sys.argv) != 5:
        print("Usage: python search.py <graph_file> <cities_file> <start_node> <goal_node>")
        sys.exit(1)

    # Read command line arguments
    graph_file = sys.argv[1]
    cities_file = sys.argv[2]
    start_node = sys.argv[3]
    goal_node = sys.argv[4]  
    
    # Read graph and cities
    graph = read_graph(graph_file)
    cities = read_cities(cities_file)
    city_nodes = list(cities.keys())
    start = city_nodes.index(start_node)
    goal = city_nodes.index(goal_node)

    # Run A* Search
    a_star_path, a_star_cost, a_star_expanded = a_star_search(graph, cities, start, goal)
    print("A* Search:")
    print(f"Shortest path: {' -> '.join(a_star_path)}")
    print(f"Shortest path cost: {a_star_cost}")
    print(f"Number of nodes expanded: {a_star_expanded}")

    # Run RBFS
    rbfs_path, rbfs_cost, rbfs_expanded = rbfs(graph, cities, start, goal)
    print("Recursive Best-First Search:")
    print(f"Shortest path: {' -> '.join(rbfs_path)}")
    print(f"Shortest path cost: {rbfs_cost}")
    print(f"Number of nodes expanded: {rbfs_expanded}")