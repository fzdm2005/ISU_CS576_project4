import heapq
import numpy as np
import dubins
import math
import matplotlib.pyplot as plt
from typing import List


class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    @staticmethod
    def point_from_list(l):
        pt_list = []
        for tp in l:
            pt = Point(tp[0], tp[1])
            pt_list.append(pt)
        return pt_list

    def distance(self, other):
        v = other - self
        return np.linalg.norm(v.to_array())

    def norm(self):
        return self.distance(Point(0, 0))

    def __sub__(self, other):
        """
        reload sub operation, return vector pointing from self to other point
        :param other: other point
        :return: vector pointing from self to other point
        """
        return Point(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def to_array(self):
        return np.array([self.x, self.y])

    def to_tuple(self):
        t = (self.x, self.y)
        return t

    def __str__(self):
        return "[{},{}]".format(self.x, self.y)


class DubinsConfig:
    def __init__(self, x, y, phi):
        self.x = x
        self.y = y
        self.phi = phi
        self.point = Point(self.x,self.y)

    @staticmethod
    def from_list(l):
        pt_list = []
        for tp in l:
            pt = DubinsConfig(tp[0], tp[1], tp[2])
            pt_list.append(pt)
        return pt_list

    @staticmethod
    def from_tuple(tp):
        if tp is None:
            return tp
        pt = DubinsConfig(tp[0], tp[1], tp[2])
        return pt

    @staticmethod
    def shortest_path(q0:'DubinsConfig', q1:'DubinsConfig', turning_radius, step = 0.1):
        conf0 = q0.to_tuple()
        conf1 = q1.to_tuple()
        path = dubins.shortest_path(conf0, conf1, turning_radius)
        configurations,_ = path.sample_many(step)
        index = 0
        for index, config in enumerate(configurations):
            conf = DubinsConfig.from_tuple(config)
            if conf.is_close(q1):
                break
        configurations_dubin = DubinsConfig.from_list(configurations[0:index + 1])
        configurations_dubin.append(q1)
        length = DubinsConfig.path_length(configurations_dubin)
        conf_rst = configurations[0:index+1]
        conf_rst.append(conf1)
        return conf_rst, length

    @staticmethod
    def path_length(configurations: List['DubinsConfig']):
        length = 0
        for i in range(len(configurations)-1):
            conf0 = configurations[i]
            conf1 = configurations[i + 1]
            length = length + conf0.norm(conf1)

        return length

    def distance(self, other:'DubinsConfig', turning_radius):
        path, d = DubinsConfig.shortest_path(self, other, turning_radius)
        return d

    def line_distance(self, other:'DubinsConfig'):
        return self.point.distance(other.point)

    def line_distance_point(self, other: 'Point'):
        return self.point.distance(other)

    def __sub__(self, other:'DubinsConfig'):
        """
        reload sub operation, return vector pointing from self to other point
        :param other: other point
        :return: vector pointing from self to other point
        """
        return DubinsConfig(self.x - other.x, self.y - other.y, self.phi - other.phi)

    def __add__(self, other:'DubinsConfig'):
        return DubinsConfig(self.x + other.x, self.y + other.y, self.phi + other.phi)

    def to_array(self):
        return np.array([self.x, self.y, self.phi])

    def to_tuple(self):
        t = (self.x, self.y, self.phi)
        return t

    def to_list(self):
        t = [self.x, self.y, self.phi]
        return t

    def norm(self, other:'DubinsConfig'):
        vec = (other - self).to_list()
        vec[2] = min([abs(vec[2]), abs(vec[2]+math.pi*2), abs(vec[2]-math.pi*2)])
        n = np.linalg.norm(vec)
        return n

    def is_close(self, other:'DubinsConfig', margin_distance = 0.01, margin_heading = 0.04):
        distance =self.point.distance(other.point)
        heading_diff = abs(self.phi - other.phi)
        if distance <= margin_distance and heading_diff <= margin_heading:
            return True
        return False



    def __str__(self):
        return "[{},{},{}]".format(self.x, self.y, self.phi)


class Node:
    def __init__(self, item, parent: 'Node' = None):
        self.item = item
        self.parent = parent
        self.children = set()

    def set_parent(self, parent: 'Node'):
        self.parent = parent

    def add_child(self, child: 'Node'):
        self.children.add(child)


class Edge:
    def __init__(self, node0: 'Node', node1: 'Node', turning_radius, step=0.1):
        self.node0 = node0
        self.node1 = node1
        self.turning_radius = turning_radius
        self.step = step
        self.configurations, self.length = self.get_path(turning_radius, step)

    def get_path(self, turning_radius = None, step = None):
        if step is None:
            step = self.step
        if turning_radius is None:
            turning_radius = self.turning_radius
        conf0 = self.node0.item
        conf1 = self.node1.item
        configurations, length = DubinsConfig.shortest_path(conf0, conf1, turning_radius)
        # configurations, _ = path.sample_many(step)
        # configurations.append(conf1)
        return configurations, length

    def get_length(self):
        return self.length

    def distance_point(self, conf:'DubinsConfig'):
        configurations = self.configurations
        intersect = None
        dist = float('inf')
        for cf in configurations:
            path, d = DubinsConfig.shortest_path(DubinsConfig.from_tuple(cf), conf, self.turning_radius)
            if d <= dist:
                dist=d
                intersect = cf
        return dist, DubinsConfig.from_tuple(intersect)

    def draw(self, ax, col = 'black'):
        configurations = self.configurations
        if len(configurations) == 0:
            return
        x = []
        y = []
        orient = []
        for config in configurations:
            x.append(config[0])
            y.append(config[1])
            orient.append(math.degrees(config[2]))

        ax.plot(x, y, color = col)
        ax.plot(x[0], y[0], marker=(3, 0, orient[0] - 90),markersize=12,color=col)
        ax.plot(x[-1], y[-1], marker=(3, 0, orient[-1] - 90),markersize=12,color=col)

    def __str__(self):
        conf0 = self.node0.item.to_tuple()
        conf1 = self.node1.item.to_tuple()
        return "[{}{}]".format(conf0,conf1)


class Graph_dubins:
    def __init__(self, qi):
        self.qi = qi
        self.nodedict = dict()
        self.edgedict = dict()
        self.node_num = 0
        self.edge_num = 0
        self.insert_node(qi)

    def insert_edge(self, ed: 'Edge'):
        if ed.node1.parent is not None:
            print('node has already connected, operation canceled')
            return False
        if self.edgedict.get((ed.node0, ed.node1)):
            return False
        if ed.node0 == ed.node1:
            return False
        else:
            ed.node0.add_child(ed.node1)
            ed.node1.set_parent(ed.node0)
            self.edgedict[(ed.node0, ed.node1)] = ed
            self.edge_num += 1
            self.insert_node(ed.node0)
            self.insert_node(ed.node1)
        # print(ed)
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

    def getpath(self, des: Node):  # get path from the initial node to the goal node
        path = []
        cur = des
        while cur is not None:
            path.append(cur)
            cur = cur.parent
        path.reverse()
        return path

    def nearest_neighbor(self, state: 'DubinsConfig', turning_radius):
        dist = float('inf')
        node_near = None
        for nd, _ in self.nodedict.items():
            d = state.distance(nd.item, turning_radius=turning_radius)
            if d < dist:
                dist = d
                node_near = nd
        return dist, node_near

    def nearest_k_neighbor(self, state: 'DubinsConfig', k, turning_radius):
        node_near = []
        h = []
        i = k
        for nd, _ in self.nodedict.items():
            d = state.distance(nd.item, turning_radius=turning_radius)
            heapq.heappush(h, (d, nd))

        while i > 0 and len(h):
            node = heapq.heappop(h)[1]
            node_near.append(node)
            i -= 1
        return node_near

    def nearest_edge(self, state:'DubinsConfig'):
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
        if self.edgedict.get((ed.node0, ed.node1)):
            return True
        else:
            return False

    def is_node(self,nd:Node):
        if self.nodedict.get(nd):
            return True
        else:
            return False

    def draw(self, ax):
        for key, ed in self.edgedict.items():
            ed.draw(ax)



if __name__ == '__main__':
    x0 = 0
    y0 = 0
    theta0 = 0

    x1 = 10
    y1 = 10
    theta1 = math.pi

    q0 = DubinsConfig(x0, y0, theta0)
    q1 = DubinsConfig(x1, y1, theta1)
    q2 = (5, 5, 0)
    turning_radius = 1.0
    step_size = 0.1
    n0 = Node(q0)
    n1 = Node(q1)
    ed = Edge(n0,n1,turning_radius)
    fig, ax = plt.subplots()
    ed.draw(ax)
    plt.show(ax)

    l = ed.get_length()
    g = Graph_dubins()
    g.insert_edge(ed)
    d,inter,e = g.nearest_edge(q2)
    print(d)

