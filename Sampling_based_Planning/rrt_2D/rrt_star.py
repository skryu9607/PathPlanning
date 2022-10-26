"""
RRT_star
09.28.2022
@author : HUGO RYU

"""
import os
import numpy as np
import math

import plotting, env,utils,queue
# RRT_star : RRT + Least-cost connection and rewiring steps

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class RRT_star:
    def __init__(self, s_start,s_goal,step_len,radius,goal_sample_rate,iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.radius = radius
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.path = []
        'Three import things'
        self.env = env.Env()
        self.plotting = plotting.Plotting(s_start,s_goal)
        self.utils = utils.Utils()

        'Grid World'
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rect = self.env.obs_rectangle
        self.obs_boun = self.env.obs_boundary

    def planning(self):
        for _ in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near,node_rand)
            if _ % 500 == 0:
                print(_)
            if node_new and not self.utils.is_collision(node_near,node_new):
                neighbor_index = self.find_near_neighbor(node_new)
                self.vertex.append(node_new)
                if neighbor_index:
                    self.choose_parent(node_new,neighbor_index)
                    self.rewire(node_new,neighbor_index)
            index = self.search_goal_parent()
            self.path = self.extract_path(self.vertex[index])
            return self.path

    def generate_random_node(self,goal_sample_rate):
        delta = self.utils.delta
        if np.random.random()>goal_sample_rate:
            x_rand = np.random.uniform(self.x_range[0]+delta,self.x_range[1]-delta)
            y_rand = np.random.uniform(self.y_range[0]+delta,self.y_range[1]-delta)
            return Node((x_rand,y_rand))
        return self.s_goal

    def least_cost(self,node_start,node_end):
        dist,_ = self.get_distance_and_angle(node_start,node_end)
        return self.cost(node_start) + dist

    def interpolation(self,node_1,node_2):
        x = [0,0]
        y = [0,0]
        path = [node_2]
        x[0] = node_1.x
        x[1] = node_2.x
        y[0] = node_1.y
        y[1] = node_2.y
        x_cand = [((x[1]-x[0])* i + x[0]) for i in range(self.num)]
        y_cand = [((y[1]-y[0])* i + y[0]) for i in range(self.num)]
        for j in range(self.num):
            node_cand = Node((x_cand[j],y_cand[j]))
            node_cand.parent = node_1
            path.append(node_cand)
        return path

    def cost(node_p):
        node = node_p
        cost = 0.0
        while node.parent:
            cost += math.hypot(node.x - node.parent.x,node.y-node.parent.y)
            node = node.parent
        return cost
    @staticmethod
    def get_distance_and_angle(node_start,node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx,dy),math.atan2(dy,dx)
    
    def new_state(self,node_start,node_end):
        dist,theta = self.get_distance_and_angle(node_start,node_end)

        dist = min(self.step_len,dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
        node_start.y + dist * math.sin(theta)))
        node_new.x = node_start.x + dist * math.cos(theta)
        node_new.y = node_start.y + dist * math.sin(theta)
        node_new.parent = node_start

        return node_new

    def choose_parent(self,node_new, neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_new) for i in neighbor_index]
        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_new.parent = self.vertex[cost_min_index]

    def rewire(self,node_new,neighbor_index):
        for i in neighbor_index:
            node_neighbor = self.vertex[i]

            if self.cost(node_neighbor) > self.get_new_cost(node_new,node_neighbor):
                node_neighbor.parent = node_new

    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.s_goal.x , n.y-self.s_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index 
            if not self.utils.is_collision(self.vertex[i],self.s_goal)]
            return node_index[int(np.argmin(cost_list))]
        return len(self.vertex)-1

    def get_new_cost(self,node_start,node_end):
        dist, _ = self.get_distance_and_angle(node_start,node_end)
        return self.cost(node_start) + dist
    
    def find_near_neighbor(self,node_new):
        n = len(self.vertex) + 1
        r = min(self.radius * math.sqrt((math.log(n)/n)),self.step_len)
        dist_table = [math.hypot(nd.x-node_new.x,nd.y-node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind]<=r and not self.utils.is_collision(node_new,self.vertex[ind])]
        
        return dist_table_index


    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost
    def update_cost(self, parent_node):
        OPEN = queue.QueueFIFO()
        OPEN.put(parent_node)

        while not OPEN.empty():
            node = OPEN.get()

            if len(node.child) == 0:
                continue

        for node_c in node.child:
            node_c.cost = self.get_new_cost(node, node_c)
            OPEN.put(node_c)

    def extract_path(self,node_end):
        # append -> path
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            path.append([node_now.x,node_now.y])
            node_now = node_now.parent
        path.append((node_now.x,node_now.y))
        
        return path    

def main():
    x_start = (2,2)
    x_goal = (10,20)
    #x_goal = (49,24)
    # (self, x_start, x_goal, step_len,goal_sample_rate, search_radius, iter_max):
    rrt_star = RRT_star(x_start, x_goal, 10, 0.10, 20, 10000)
    path = rrt_star.planning()
    if path:
        rrt_star.plotting.animation(rrt_star.vertex,path,"RRT_Star",True)
    else:
        rrt_star.plotting.animation(rrt_star.vertex,path,"RRT_Star",True)
        print("No path Found!")
if __name__ == "__main__":
    main()



    