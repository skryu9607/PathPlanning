"""
RRT_star
09.28.2022
@author : HUGO RYU

"""
import os
import numpy as np
import math

import plotting, env,utils
# RRT_star : RRT + Least-cost connection and rewiring steps

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class RRT_star:
    def __init__(self, s_start,s_goal,step_len,radius,num_candidates,goal_sample_rate,iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.iter_max = iter_max
        self.goal_sample_rate = goal_sample_rate
        self.vertex = [self.s_start]
        self.radius = radius
        self.num = num_candidates
        self.candidate = []
        'Three import thing'
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
            done = True
            while done:
                node_rand = self.generate_random_node(self.goal_sample_rate)
                node_near = self.nearest(self.vertex, node_rand)
                dist,_ = self.get_distance_and_angle(node_rand,node_near)
                if dist < self.radius:
                    done = False
                # node_rand에 대해서 radius r보다 작은 node_near을 찾는다. 
                self.path = self.interpolation(node_near,node_near.parent)
                node_new = self.path[int(np.argmin([self.least_cost(nd,node_rand) for nd in self.path]))]
            if node_new and not self.utils.is_collision(node_near,node_new):
                self.vertex.append(node_new)
                dist,_ = self.get_distance_and_angle(node_new,self.s_goal)

                if dist<=self.step_len and not self.utils.is_collision(node_new,self.s_goal):
                    self.new_state(node_new,self.s_goal)
                    return self.extract_path(node_new)
        
        return None

    def generate_random_node(self,goal_sample_rate):
        delta = self.utils.delta
        if np.random.random()>goal_sample_rate:
            x_rand = np.random.uniform(self.x_range[0]+delta,self.x_range[1])
            y_rand = np.random.uniform(self.y_range[0]+delta,self.y_range[1])
            return Node((x_rand,y_rand))
        return self.s_goal

    def nearest(self,node_list,n):
        return  node_list[int(np.argmin([math.hypot(nd.x-n.x,nd.y-n.y) for nd in node_list]))]

    def least_cost(self,node_start,node_end):
        dist,_ = self.get_distance_and_angle(node_start,node_end)
        return self.cost(node_start) + dist

    def interpolation(self,node_1,node_2):
        x = [0,0]
        y = [0,0]
        path = [node_2]
        x[0] = node_1[0]
        x[1] = node_2[0]
        y[0] = node_1[1]
        y[1] = node_2[1]
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
        
    def get_distance_and_angle(self,node_start,node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx,dy),math.atan2(dy,dx)
    
    def new_state(self,node_start,node_end):
        dist,theta = self.get_distance_and_angle(node_start,node_end)

        dist = min(self.step_len,dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
        node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self,node_end):
        # append -> path
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x,node_now.y))
        
        return path

def main():
    x_start = (2,2)
    x_goal = (49,24)

    rrt_star = RRT_star(x_start,x_goal,0.5,0.05,10000)
    path = rrt_star.planning()
    if path:
        rrt_star.plotting.animation(rrt_star.vertex,path,"RRT_Star",True)
    else:
        print("No path Found!")
if __name__ == "__main__":
    main()



    