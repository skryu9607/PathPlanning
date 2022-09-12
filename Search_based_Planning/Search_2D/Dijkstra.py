
from multiprocessing import heap, set_start_method
import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env

from Search_2D.Astar import AStar


class Dijkstra(AStar):
    def searching(self):
        # Initial state
        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,(0,self.s_start))
        while self.OPEN:
            _,s = heapq.heappop(self.OPEN)
            if s == self.s_goal:
                break
            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s,s_n)
                if s_n not in self.g:
                    self.g[s_n] = math.inf
                if new_cost < self.g[s_n]:
                   self.PARENT[s_n] = s
                   self.g[s_n] = new_cost
                   heapq.heappush(self.OPEN,(self.g[s_n],s_n))
        return self.extract_path(self.PARENT),self.CLOSED

def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    dijkstra = Dijkstra(s_start, s_goal, 'None')
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = dijkstra.searching()
    plot.animation(path, visited, "Dijkstra's")  # animation generate


if __name__ == '__main__':
    main()
'''
    """Dijkstra set the cost as the priority 
    """
    def searching(self):
        """
        Breadth-first Searching.
        :return: path, visited order
        """
        # self.g : cost
        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        # heappush : heap이라는 구조에 push를 하여 값을 밀어넣는 것.
        heapq.heappush(self.OPEN,
                       (0, self.s_start))

        while self.OPEN:
            # heappop : heap에서 가장 작은 값을 제거하고 동시에 그 값을 return하는 것. 
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:
                break
            # get_neighbor :  action_Set에 해당하는 것들에 대해서 값을 불러오는 것. 
            for s_n in self.get_neighbor(s):
                # self.cost : manhatten distance
                # new cost is the summation between current cost and neighbor distance.
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf
                # new cost that is smaller than current best cost should be updated. 
                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    
                    # best first set the heuristics as the priority 
                    heapq.heappush(self.OPEN, (new_cost, s_n))

        return self.extract_path(self.PARENT), self.CLOSED'''
