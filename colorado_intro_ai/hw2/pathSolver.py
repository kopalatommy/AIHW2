from cmath import cos
from collections import deque
import math
"""
 Class PathSolver

"""

# Create PathSolver Class


class PathSolver:
    """Contains methods to solve multiple path search algorithms"""

    # init for PathSolver Class
    def __init__(self):
        """Create PathSolver"""

    def path(self, previous, s): 
        """
        `previous` is a dictionary chaining together the predecessor state that led to each state

        `s` will be None for the initial state

        otherwise, start from the last state `s` and recursively trace `previous` back to the initial state,
        constructing a list of states visited as we go
        """ 
        
        if s is None:
            return []
        else:
            return self.path(previous, previous[s])+[s]

    def pathcost(self, path, step_costs):
        """add up the step costs along a path, which is assumed to be a list output from the `path` function above"""
        
        cost = 0
        for s in range(len(path)-1):
            cost += step_costs[path[s]][path[s+1]]
        return cost
    

    def breadth_first_search(self,start: tuple, goal, state_graph, return_cost=False):
        """ find a shortest sequence of states from start to the goal """
        print("calliing BFS")
        
        frontier = deque([start]) # doubly-ended queue of states
        previous = {start: None}  # start has no previous state; other states will
        
        # Return on start is goal
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out

        # loop through frontine searching nodes until we find a goal
        while frontier:
            s = frontier.popleft()
            for s2 in state_graph[s]:
                if (s2 not in previous) and (s2 not in frontier):
                    frontier.append(s2)
                    previous[s2] = s
                    if s2 == goal:
                        path_out = self.path(previous, s2)
                        if return_cost: return path_out, self.pathcost(path_out, state_graph)
                        return path_out
        
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []


    def depth_first_search(self, start: tuple, goal, state_graph, return_cost=False):
        """ find a shortest sequence of states from start to the goal """
        print("calliing DFS")
        
        frontier = deque([start]) # doubly-ended queue of states
        previous = {start: None}  # start has no previous state; other states will
        
        # Return on start is goal
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out

        # loop through frontine searching nodes until we find a goal
        while frontier:
            s = frontier.pop()
            for s2 in state_graph[s]:
                if (s2 not in previous) and (s2 not in frontier):
                    frontier.append(s2)
                    previous[s2] = s
                    if s2 == goal:
                        path_out = self.path(previous, s2)
                        if return_cost: return path_out, self.pathcost(path_out, state_graph)
                        return path_out
        
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []

    def uniform_cost_search(self,start: tuple, goal, state_graph, return_cost=False):
        """ find a shortest sequence of states from start to the goal """
        print("calling Uniform")
        
        frontier = deque([(start, 0)]) # doubly-ended queue of states
        previous = {start: None}  # start has no previous state; other states will
        
        # Return on start is goal
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out

        # loop through frontine searching nodes until we find a goal
        while frontier:
            s = frontier.popleft()
            for s2 in state_graph[s[0]]:
                if (s2 not in previous) and (s2 not in frontier):
                    previous[s2] = s[0]
                    cur_path = self.path(previous, s2)
                    cost = self.pathcost(cur_path, state_graph)

                    if s2 == goal:
                        if return_cost: return cur_path, cost
                        return cur_path

                    found = False
                    for i in range(0, len(frontier)):
                        if frontier[i][1] > cost:
                            frontier.insert(i, (s2, cost))
                            found = True
                            break
                    
                    if not found:
                        frontier.append((s2, cost))
                # If in frontier already, check if new cost is less
                elif s2 in frontier:
                    print("Checking")
                    prev = previous[s2]
                    previous[s2] = s[0]
                    cur_path = self.path(previous, s2)
                    cost = self.pathcost(cur_path, state_graph)
                    previous[s2] = prev

                    found = False
                    for i in range(0, len(frontier)):
                        # Find out node
                        if frontier[i][0] == s2 and frontier[i][1] > cost:
                            frontier.remove(i)
                            found = True
                            break
                    if found:
                        found = False
                        previous[s2] = prev
                        for i in range(0, len(frontier)):
                            if frontier[i][1] > cost:
                                frontier.insert(i, (s2, cost))
                                found = True
                                break
                # else:
                #     for i in range(0, len(frontier)):
                #         if frontier[i][0] == s2:
                #             prev = previous[s2]
                #             previous[s2] = s[0]
                #             cur_path = self.path(previous, s2)
                #             cost = self.pathcost(cur_path, state_graph)
                #             previous[s2] = prev

                #             if cost < frontier[i][1]:
                #                 frontier.remove(frontier[i])
                #                 for j in range(0, len(frontier)):
                #                     if frontier[j][1] > cost:
                #                         frontier.insert(j, (s2, cost))
                #                         break
                #             break
        
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []

    def euclidian_heuristic(self, node, goal):
        x = node[0] - goal[0]
        x *= x
        y = node[1] - goal[1]
        y *= y
        return math.sqrt(x + y)

    def a_star_euclidian(self,start: tuple, goal, state_graph, return_cost=False):
        """Problem 2.b: you need to implement this function"""
        print("calling a star euclidian")
        
        costs = {}
        costs[start] = 0

        frontier = deque([(start, 0)]) # doubly-ended queue of states
        previous = {start: None}  # start has no previous state; other states will
        
        # Return on start is goal
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out

        # loop through frontine searching nodes until we find a goal
        while frontier:
            s = frontier.popleft()
            for s2 in state_graph[s[0]]:
                if (s2 not in previous) and (s2 not in frontier):
                    previous[s2] = s[0]
                    cur_path = self.path(previous, s2)

                    cost = costs[s[0]] + self.euclidian_heuristic(s2, goal)
                    costs[s2] = cost

                    if s2 == goal:
                        if return_cost: return cur_path, cost
                        return cur_path

                    found = False
                    for i in range(0, len(frontier)):
                        if frontier[i][1] > cost:
                            frontier.insert(i, (s2, cost))
                            found = True
                            break
                    
                    if not found:
                        frontier.append((s2, cost))
                # If in frontier already, check if new cost is less
                elif s2 in frontier:
                    cost = costs[s[0]] + self.manhattan_heuristic(s2, goal)

                    found = False
                    for i in range(0, len(frontier)):
                        # Find out node
                        if frontier[i][0] == s2 and frontier[i][1] > cost:
                            frontier.remove(i)
                            found = True
                            break
                    if found:
                        found = False
                        previous[s2] = s[0]
                        costs[s2] = cost
                        for i in range(0, len(frontier)):
                            if frontier[i][1] > cost:
                                frontier.insert(i, (s2, cost))
                                found = True
                                break
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []

    def manhattan_heuristic(self, node, goal):
        x = abs(goal[0] - node[0])
        y = abs(goal[1] - node[1])
        return x + y 
    
    def a_star_manhattan(self,start: tuple, goal, state_graph, return_cost=False):
        """Problem 2.b: you need to implement this function"""
        print("calling a star manhattan")
        
        costs = {}
        costs[start] = 0

        frontier = deque([(start, 0)]) # doubly-ended queue of states
        previous = {start: None}  # start has no previous state; other states will
        
        # Return on start is goal
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out

        # loop through frontine searching nodes until we find a goal
        while frontier:
            s = frontier.popleft()
            for s2 in state_graph[s[0]]:
                if (s2 not in previous) and (s2 not in frontier):
                    previous[s2] = s[0]
                    cur_path = self.path(previous, s2)

                    cost = costs[s[0]] + self.manhattan_heuristic(s2, goal)
                    costs[s2] = cost

                    if s2 == goal:
                        if return_cost: return cur_path, cost
                        return cur_path

                    found = False
                    for i in range(0, len(frontier)):
                        if frontier[i][1] > cost:
                            frontier.insert(i, (s2, cost))
                            found = True
                            break
                    
                    if not found:
                        frontier.append((s2, cost))
                # If in frontier already, check if new cost is less
                elif s2 in frontier:
                    cost = costs[s[0]] + self.manhattan_heuristic(s2, goal)

                    found = False
                    for i in range(0, len(frontier)):
                        # Find out node
                        if frontier[i][0] == s2 and frontier[i][1] > cost:
                            frontier.remove(i)
                            found = True
                            break
                    if found:
                        found = False
                        previous[s2] = s[0]
                        costs[s2] = cost
                        for i in range(0, len(frontier)):
                            if frontier[i][1] > cost:
                                frontier.insert(i, (s2, cost))
                                found = True
                                break
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []
