import os
import numpy as np
import math

import plotting, env,utils
class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class RRT:
    def __init__(self, s_start,s_goal,step_len,goal_sample_rate,iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.iter_max = iter_max
        self.goal_sample_rate = goal_sample_rate
        
        self.vertex = [self.s_start]

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
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest(self.vertex, node_rand)
            node_new = self.new_state(node_near,node_rand)
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

    rrt = RRT(x_start,x_goal,0.5,0.05,10000)
    path = rrt.planning()
    if path:
        rrt.plotting.animation(rrt.vertex,path,"RRT",True)
    else:
        print("No path Found!")
if __name__ == "__main__":
    main()



    