from search import *
import math
import re
import heapq
import itertools

counter = itertools.count()

class RoutingGraph(Graph):
    def __init__(self, map_str):
        self.map = map_str.split('\n')

    def is_goal(self, node):
        """Returns true if the given node is a goal state, false otherwise."""
        row, col, cost = node
        return self.map[row][col] == 'G'

    def starting_nodes(self):
        """Returns a sequence of starting nodes. Often there is only one
        starting node but even then the function returns a sequence
        with one element. It can be implemented as an iterator if
        needed.
        """
        starting_nodes = []
        for row in range(0, len(self.map)):
            for col in range(0, len(self.map[row])):
                cell = self.map[row][col]
                if cell == 'S':
                    starting_nodes.append((row, col, math.inf))
                elif re.match(r'\d', cell):
                    # if the cell is a digit (agent with fuel tank)
                    starting_nodes.append((row, col, int(cell)))
        return starting_nodes

    def outgoing_arcs(self, tail_node):
        """Given a node it returns a sequence of arcs (Arc objects)
        which correspond to the actions that can be taken in that
        state (node)."""
        arcs = []
        row, col, cost = tail_node
        blocking = ['+', '-', '|', 'X']
        if cost > 0:
            # North clear?
            if self.map[row-1][col] not in blocking:
                arcs.append(Arc(tail_node, (row-1, col, cost-1), 'N', 5))
            # East clear?
            if self.map[row][col+1] not in blocking:
                arcs.append(Arc(tail_node, (row, col+1, cost-1), 'E', 5))
            # South clear?
            if self.map[row+1][col] not in blocking:
                arcs.append(Arc(tail_node, (row+1, col, cost-1), 'S', 5))
            # West clear?
            if self.map[row][col-1] not in blocking:
                arcs.append(Arc(tail_node, (row, col-1, cost-1), 'W', 5))

        # Agent on fuel station?
        if self.map[row][col] == 'F' and cost < 9:
            arcs.append(Arc(tail_node, (row, col, 9), "Fuel up", 15))
       
        # Agent on portal?
        if self.map[row][col] == 'P':
            # Search for portals on map iteratively
            for altrow in range(0, len(self.map) - 1):
                for altcol in range(0, len(self.map[row])):
                    if self.map[altrow][altcol] == 'P' and (altrow, altcol) != (row, col):
                        # It is another portal
                        arcs.append(Arc(tail_node, (altrow, altcol, cost), "Teleport to ({}, {})".format(altrow, altcol), 10))

        return arcs

    def estimated_cost_to_goal(self, node):
        """Return the estimated cost to a goal node from the given
        state. This function is usually implemented when there is a
        single goal state. The function is used as a heuristic in
        search. The implementation should make sure that the heuristic
        meets the required criteria for heuristics."""
        if node:
            node_row, node_col, cost = node
            best_manhattan = math.inf
            goal_node_positions = set()

            # Find locations of all goal nodes in map
            for row in range(0, len(self.map) - 1):
                for col in range(0, len(self.map[row])):
                    if self.map[row][col] == 'G':
                        goal_node_positions.add((row, col))

            # Measure best manhattan distance between current node and each goal node
            for goal_x, goal_y in goal_node_positions:

                curr_manhattan = 5 * (abs((goal_x - node_row)) + abs((goal_y - node_col)))
                if curr_manhattan < best_manhattan:
                    best_manhattan = curr_manhattan

            return best_manhattan
        return 0
    

class AStarFrontier():
    def __init__(self, map_graph, paths=[]):
        self.map_graph = map_graph
        self.paths = paths
        # pruning dictionary
        self.visited = set()

    def add(self, path):
        # store the entry as a 3-element list so that I can assign a priority and tie-breaker to each path
        # this helps keep the priority queue stable
        if path[-1].head not in self.visited:
            entry = [sum([hop.cost for hop in path]) + self.map_graph.estimated_cost_to_goal(path[-1].head), next(counter), path]
            heapq.heappush(self.paths, entry)

    def __iter__(self):
        return self

    def __next__(self):
        while self.paths:
            priority, tiebreaker, path = heapq.heappop(self.paths)
            # has head node been expanded?
            if path[-1].head not in self.visited:
                # add head node to expanded set
                self.visited.add(path[-1].head)
                # return path only if end node has not been expanded before
                return path
        raise StopIteration
    

def print_map(map_graph, frontier, solution):
    solution_positions = set()
    expanded_positions = set()
    if solution:
        for node in solution:
            solution_positions.add((node.head[0], node.head[1]))
        
    for node_x, node_y, cost in frontier.visited:
        expanded_positions.add((node_x, node_y))

    old_graph = map_graph.map
    new_graph = []
    reconstructed_map = """"""
    for row in range(0, len(old_graph) - 1):
        new_graph.append([])
        for col in range(0, len(old_graph[row])):
            if (row, col) in solution_positions and old_graph[row][col] == ' ':
                new_graph[-1].append('*')
            elif (row, col) in expanded_positions and old_graph[row][col] == ' ':
                new_graph[-1].append('.')
            else:
                new_graph[-1].append(old_graph[row][col])
            reconstructed_map += new_graph[-1][col]
        reconstructed_map += "\n"
    
    print(reconstructed_map)



### EXAMPLE INPUT ###
map_str = """\
+---------------+
|    G          |
|XXXXXXXXXXXX   |
|           X   |
|  XXXXXX   X   |
|  X S  X   X   |
|  X        X   |
|  XXXXXXXXXX   |
|               |
+---------------+
"""

map_graph = RoutingGraph(map_str)
frontier = AStarFrontier(map_graph)
solution = next(generic_search(map_graph, frontier), None)
print_map(map_graph, frontier, solution)
######################
