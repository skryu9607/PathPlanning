"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")



import math
import heapq
import env
import plotting

class Astar:
    def __init__(self,s_start,s_goal, heuristic_type):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        # Class Env
        self.Env = env.Env()
        self.candidate = self.Env.motions
        self.obstacles = self.Env.obs_map()

        self.open = [] # priority queue
        self.closed = [] # Visited set
        self.parent = dict() # Parent // Path
        self.g = dict() # cost to come

    def hueristic(self,s):
        heuristic_type = self.heuristic_type
        if heuristic_type == 'manhattan':
            return abs(self.s_goal[0]-s[0]) + abs(self.s_goal[1]-s[1])
        else:
            return math.hypot(self.s_goal[0]-s[0], self.s_goal[1]-s[1])

    def f_value(self,s):
        'f = g + h'
        return self.g[s] + self.hueristic(s)
 
    def get_neighbor(self,s):
        return [(s[0] + u[0], s[1] + u[1]) for u in self.candidate]

    def is_collision(self,s,s_n):
        if s in self.obstacles or s_n in self.obstacles:
            return True
        else:
            return False

    def cost(self,s,s_n):
        if self.is_collision(s,s_n):
            return math.inf
        else:
            return math.hypot(s[0] - s_n[0] , s[1] - s_n[1])

    def extract_path(self,parent):
        path = [self.s_goal]
        s = self.s_goal
        while True:
            s = parent[s]
            path.append(s)
            if s == self.s_start:
                break
        return list(path)

    def searching(self):
        # A star searching
        self.parent[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf

        heapq.heappush(self.open,(self.f_value(self.s_start),self.s_start))

        while self.open:
            _,s = heapq.heappop(self.open)
            self.closed.append(s)

            if s == self.s_goal:
                break
            for s_n in self.get_neighbor(s):

                new_cost = self.g[s] + self.cost(s,s_n)
                
                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:
                    self.g[s_n] = new_cost
                    self.parent[s_n] = s
                    heapq.heappush(self.open,(self.f_value(s_n),s_n))
        return self.extract_path(self.parent), self.closed


def main():
    s_start = (5,5)
    s_goal = (45,25)
    astar = Astar(s_start,s_goal,"manhattan")
    plot = plotting.Plotting(s_start,s_goal)
    path,visited = astar.searching()
    plot.animation(path,visited,"A*")

if __name__ == "__main__":
    main()