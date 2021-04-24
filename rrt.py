import numpy as np
import draw_cspace as dc
import matplotlib.pyplot as plt
from graph import Point
from graph import Node
from graph import Edge
from graph import DubinsConfig
import math
from graph import Graph_dubins


class RRt:

    def __init__(self, phi_max = math.pi/2 ,s = 1, rou_min = 0.5, cspace = None, ob_center: list = None, ob_radio: list = None):
        if cspace is None:
            self.cspace = [(-3,3),(-1,1)]
        else:
            self.cspace = [(-3, 3), (-1, 1)]
        self.origin = DubinsConfig(self.cspace[0][0], self.cspace[1][0],-phi_max)
        self.space_width = self.cspace[0][1] - self.cspace[0][0]
        self.space_height = self.cspace[1][1] - self.cspace[1][0]
        self.obs = dict()
        for index, ct in enumerate(ob_center):
            self.obs[ct] = ob_radio[index]
        self.speed = s
        self.turnning_radius = rou_min
        self.phi_max = phi_max
        self.phi_span = phi_max * 2

    def generate_graph(self, qi: 'DubinsConfig', qG: 'DubinsConfig', K, goal_margin = 0, p_thres = 0, collision_margin = 0):

        # fig, ax = plt.subplots()
        # plt.show(ax)
        p_threshold = p_thres
        xi = Node(qi)
        G = Graph_dubins(xi)
        path = []
        for i in range(K):
            x_new = self.random_state(p_threshold, qG)
            #print('new:', x_new)
            d_node, near_node = G.nearest_neighbor(x_new, self.turnning_radius)
            d_edge, intersect, near_edge = G.nearest_edge(x_new)
            # d_edge = float('Inf')
            if d_node <= d_edge:
                node_new = Node(x_new)
                edge_new = Edge(near_node, node_new, self.turnning_radius)
                is_collision, config_colli = self.check_collision(edge_new, collision_margin)
                if is_collision:
                    if config_colli is None:
                        continue
                    else:
                        node_valid = Node(config_colli)

                else:
                    node_valid = node_new
                edge_valid = Edge(near_node, node_valid, self.turnning_radius)
                check_coli, _ = self.check_collision(edge_valid, collision_margin)
                if not check_coli:
                    G.insert_edge(edge_valid)
            else:
                node_new = Node(x_new)
                node_inter = Node(intersect)
                edge_new = Edge(node_inter, node_new, self.turnning_radius)
                is_collision, config_colli = self.check_collision(edge_new, collision_margin)
                if is_collision:
                    if config_colli is None:
                        continue
                    else:
                        node_valid = Node(config_colli)
                else:
                    node_valid = node_new

                edge1 = Edge(near_edge.node0, node_inter, self.turnning_radius)
                edge2 = Edge(node_inter, near_edge.node1, self.turnning_radius)
                edge_valid = Edge(node_inter, node_valid, self.turnning_radius)

                check_valid, _ = self.check_collision(edge_valid, collision_margin)
                check_1, _ = self.check_collision(edge1, collision_margin)
                check_2, _ = self.check_collision(edge2, collision_margin)
                check = (check_valid or check_1 or check_2)
                if not check:
                    G.del_edge(near_edge)
                    G.insert_edge(edge1)
                    G.insert_edge(edge2)
                    G.insert_edge(edge_valid)
                # G.draw(ax)
                # plt.show(ax)
            if node_valid.item.line_distance_point(qG.point) <= goal_margin:
                path = G.getpath(node_new)
                break

        return G, path

    def check_collision(self, edge, collision_margin):
        config_prev = None
        configurations = DubinsConfig.from_list(edge.configurations[1:])
        for config in configurations:
            if self.is_out_boundary(config):
                return True, config_prev
            for ct, r in self.obs.items():
                if config.line_distance_point(ct) - r <= collision_margin:
                    return True, config_prev
            config_prev = config
        return False, None

    def is_out_boundary(self, config:'DubinsConfig'):
        pt = config - self.origin
        w = pt.x
        h = pt.y
        if w > self.space_width or w < 0 or h > self.space_height or h < 0:
            return True
        return False

    def move_step(self, pt1: 'Point', pt2:'Point', ratio) -> 'DubinsConfig':
        x = pt1.x + ratio * (pt2.x - pt1.x)
        y = pt1.y + ratio * (pt2.y - pt1.y)
        return Point(x,y)

    def random_state(self, p_thres, qG):
        p = np.random.random()
        if p < p_thres:
            return qG
        else:
            seed = np.random.rand(3, 1)
            state = self.origin + DubinsConfig(float(seed[0])*self.space_width, float(seed[1]) * self.space_height, float(seed[2])*self.phi_span)
            return state


def generate_circle(center, r, th0=0, th1=np.pi, num=100):
    ob = []
    theta = np.linspace(th0, th1, num)
    for the in theta:
        x = np.cos(the) * r + center[0]
        y = np.sin(the) * r + center[1]
        ob.append((x,y))
    return ob


if __name__ == '__main__':
    s = 1
    phi_max = 1
    rou_min = 0.5

    ob_center = [(0,-1),(0,1)]
    dt = 0.2
    ob_r = [1-dt, 1-dt]
    cspace = [(-3,3),(-1,1)]
    obs = []

    obs.append(generate_circle(ob_center[0], ob_r[0], th0=0, th1=np.pi, num=100))
    obs.append(generate_circle(ob_center[1], ob_r[1], th0=0, th1=-np.pi, num=100))

    obs_ct = Point.point_from_list(ob_center)
    path = []

    rrt = RRt(phi_max=phi_max, s=s, rou_min = rou_min, cspace=cspace, ob_center=obs_ct, ob_radio=ob_r)
    qI = (-2, -0.5, 0)
    qG = (2, -0.5, math.pi/2)
    qi = DubinsConfig(qI[0], qI[1], qI[2])
    qg = DubinsConfig(qG[0], qG[1], qG[2])
    G, path_nd = rrt.generate_graph(qi, qg, K=100, goal_margin = 0, p_thres=0.1)
    for indx in range(len(path_nd) - 1):
        ed = G.edgedict[(path_nd[indx], path_nd[indx+1])]
        config = ed.configurations
        for conf in config:
            path.append(conf)

    fig, ax = plt.subplots()
    #G.draw(ax)
    obstacles = []
    dc.draw(ax, cspace, obs, qI, qG, G, path, title="RRT planning")
    plt.show(ax)
