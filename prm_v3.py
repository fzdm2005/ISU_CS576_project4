import numpy as np
import draw_cspace as dc
import matplotlib.pyplot as plt
import heapq
import ForwardSearch as fs
from graph import Point
from graph import Node
from graph import Edge
from graph import Graph_dubins
from graph import DubinsConfig
import math



class Graph:
    def __init__(self, qi):
        self.qi = qi
        self.nodedict = dict()
        self.edgedict = dict()
        self.node_num = 0
        self.edge_num = 0
        self.insert_node(qi)

    def insert_edge(self, ed: 'Edge'):
        if ed.node1.parent is not None:
            #print('node has already connected, operation canceled')
            return False
        if self.edgedict.get((ed.node0, ed.node1)):
            return False
        else:
            ed.node0.add_child(ed.node1)
            ed.node1.set_parent(ed.node0)
            self.edgedict[(ed.node0, ed.node1)] = ed
            self.edge_num += 1
            self.insert_node(ed.node0)
            self.insert_node(ed.node1)
            return True

    def del_edge(self, ed: 'Edge'):
        if self.edgedict.get((ed.node0, ed.node1)):
            self.edgedict.pop((ed.node0, ed.node1))
            ed.node1.set_parent(None)
            self.edge_num -= 1
        else:
            print('Edge does not exist')

    def insert_node(self, nd: 'Node'):
        if self.nodedict.get(nd):
            return
        else:
            self.nodedict[nd] = nd.children
            self.node_num += 1

    def del_node(self, nd: 'Node'):
        children = nd.children
        parent = nd.parent
        if self.edgedict.get((parent, nd)):
            self.edgedict.pop((parent, nd))
        for cld in children:
            if self.edgedict.get((nd, cld)):
                self.edgedict.pop((nd, cld))
        self.nodedict.pop(nd)

    def getpath(self, des: Node):  # get path from the last node to the initial node
        path = []
        cur = des
        while cur is not None:
            path.append(cur)
            cur = cur.parent
        return path

    def nearest_neighbor(self, state: 'Point'):
        dist = float('inf')
        node_near = None
        for nd, _ in self.nodedict.items():
            d = state.distance(nd.coordinate)
            if d < dist:
                dist = d
                node_near = nd
        return dist, node_near

    def nearest_k_neighbor(self, state: 'Point',k):
        node_near = []
        h = []
        i = k
        for nd, _ in self.nodedict.items():
            d = state.distance(nd.coordinate)
            heapq.heappush(h, (d, nd))

        while i > 0 and len(h):
            node = heapq.heappop(h)[1]
            node_near.append(node)
            i -= 1
        return node_near

    def nearest_edge(self, state: 'Point'):
        dist = float('inf')
        intersect = None
        near_ed = None
        for key, ed in self.edgedict.items():
            d, pt = ed.distance_point(state)
            if d < dist and pt is not None:
                dist = d
                intersect = pt
                near_ed = ed
        return dist, intersect, near_ed

    def is_edge(self,ed:Edge):
        if self.edgedict.get((ed.node0,ed.node1)) or self.edgedict.get((ed.node1,ed.node0)):
            return True
        else:
            return False

    def is_node(self, nd):
        if self.nodedict.get(nd):
            return True
        else:
            return False


    def draw(self, ax):
        for key, ed in self.edgedict.items():
            ed.draw(ax)


class PRM:

    def __init__(self, phi_max = math.pi/2 ,s = 1, rou_min = 0.5, N=1000, K=15, cspace=None, ob_center: list = None, ob_radio: list = None):
        if cspace is None:
            self.cspace = [(-3,3),(-1,1)]
        else:
            self.cspace = [(-3, 3), (-1, 1)]
        self.origin = DubinsConfig(self.cspace[0][0], self.cspace[1][0], -phi_max)
        self.space_width = self.cspace[0][1] - self.cspace[0][0]
        self.space_height = self.cspace[1][1] - self.cspace[1][0]
        self.obs = dict()
        self.N = N
        self.K = K
        for index, ct in enumerate(ob_center):
            self.obs[ct] = ob_radio[index]
        self.speed = s
        self.turnning_radius = rou_min
        self.phi_max = phi_max
        self.phi_span = phi_max * 2

    def generate_graph(self, qi: 'DubinsConfig', qG: 'DubinsConfig', goal_margin = 0, collision_margin = 0):
        K = self.K
        xi = Node(qi)
        G = Graph_dubins(xi)
        i = 0
        target_flag = 0
        achieved = 0
        istarget = 0
        while i < self.N and not achieved:
            # if i == self.N-1 or target_flag == 1:
            # target_flag = 0
            if target_flag == 1:
                alpha = qG
                istarget = 1
                target_flag = 0
            else:
                alpha = self.random_state()
                istarget = 0
            if self.check_collision_point(alpha, collision_margin):
                continue
            alpha_nd = Node(alpha)
            if G.is_node(alpha_nd):
                print('Node skip')
                continue
            neighbor = G.nearest_k_neighbor(alpha, K, self.turnning_radius)
            for node in neighbor:
                ed = Edge(node, alpha_nd, self.turnning_radius)
                if not G.is_edge(ed) and not self.check_collision_edge(ed) and ed.get_length():
                    flag = G.insert_edge(ed)
                    if flag:
                        i += 1
                        if istarget:
                            achieved = 1
                            break
                        if not i % 50:
                            target_flag = 1
                        break
        return G

    def check_collision_edge(self, edge, collision_margin = 0):
        configurations = DubinsConfig.from_list(edge.configurations[1:])
        for config in configurations:
            if self.is_out_boundary(config):
                return True
            for ct, r in self.obs.items():
                if config.line_distance_point(ct) - r <= collision_margin:
                    return True
        return False

    def check_collision_point(self, config: 'DubinsConfig', collision_margin):
        pt = config.point
        for ct, r in self.obs.items():
            if pt.distance(ct) - r <= collision_margin:
                return True
            elif pt.distance(ct) - r <= collision_margin:
                return True
        return False

    def is_out_boundary(self, config:'DubinsConfig'):
        pt = config.point - self.origin
        w = pt.x
        h = pt.y
        if w > self.space_width or w < 0 or h > self.space_height or h < 0:
            return True
        return False

    def random_state(self)->DubinsConfig:
        seed = np.random.rand(3, 1)
        state = self.origin + DubinsConfig(float(seed[0]) * self.space_width, float(seed[1]) * self.space_height,
                                           float(seed[2]) * self.phi_span)
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

    qI = (-2, -0.5, 0)
    qG = (2, -0.5, math.pi/2)
    qi = DubinsConfig(qI[0], qI[1], qI[2])
    qg = DubinsConfig(qG[0], qG[1], qG[2])

    prm = PRM(phi_max=phi_max, s=s, rou_min = rou_min, cspace= cspace, N=1000, K=15, ob_center = obs_ct, ob_radio = ob_r)
    G = prm.generate_graph(qi, qg)
    srch = fs.Search(G)
    xi = G.qi
    visited_edge, visited_node, path_nd, flag = srch.fsearch(xi, qg, 'bfs')
    print(flag)
    for indx in range(len(path_nd) - 1):
        ed = G.edgedict[(path_nd[indx], path_nd[indx+1])]
        config = ed.configurations
        for conf in config:
            path.append(conf)

    fig, ax = plt.subplots()
    #G.draw(ax)

    dc.draw(ax, cspace, obs, qI, qG, G, path, title="PRM planning")
    for ed in visited_edge:
        ed.draw(ax, col= 'slategray')
    plt.show(ax)

