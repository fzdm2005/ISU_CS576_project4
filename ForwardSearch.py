import json
import heapq
# import draw_grid as dg
import matplotlib.pyplot as plt
import sys
from graph import Point
from graph import Node
from graph import DubinsConfig


class Queue:  # FIFO queue by list
    def __init__(self):
        self.entries = []  # Items in the queue
        self.length = 0  # length of the queue

    def enqueue(self, item):  # get the first element
        self.entries.append(item)  # add a item into the queue
        self.length = self.length + 1  # increase length

    def dequeue(self):
        if self.length >= 1:
            dequeued = self.entries[0]  # get item
            self.entries = self.entries[1:]  # replace list with the rest
            self.length = self.length - 1  # decrease length
        else:
            dequeued = None
        return dequeued

    def pop(self):
        if self.length >= 1:
            popped = self.entries[-1]  # get item
            self.entries.pop()
            self.length = self.length - 1  # decrease length
        else:
            popped = None
        return popped

    def peek(self):
        if self.length > 0:
            return self.entries[0]  # return the first element
        else:
            return None

    def isempty(self):
        if self.length > 0:
            return False
        else:
            return True

    #def print(self):
        #print(self.entries)


class QueueBFS(Queue):
    def __init__(self):
        super().__init__()
        self.parents = {}

    def insert(self, x, parent):  # insert node and parent
        if not self.isDiscovered(x):
            self.enqueue(x)
            self.parents[x] = parent

    def isDiscovered(self, x):
        if x in self.parents:
            return True
        else:
            return False

    def resolve(self, x, parent):
        pass

    def getpath(self, des):  # get path from the last node to the initial node
        path = []
        cur = des
        while cur is not None:
            path.append(cur)
            cur = self.parents[cur]
        return path


class QueueDFS(Queue):
    def __init__(self):
        super().__init__()
        self.parents = {}

    def insert(self, x, parent):
        if not self.isDiscovered(x):
            self.enqueue(x)
            self.parents[x] = parent

    def dequeue(self):  # get the last item in the queue
        return self.pop()

    def isDiscovered(self, x):
        if x in self.parents:
            return True
        else:
            return False

    def resolve(self, x, parent):
        pass

    def getpath(self, des):  # get path from the last node to the initial node
        path = []
        cur = des
        while cur is not None:
            path.append(cur)
            cur = self.parents[cur]
        return path


class QueueAstar(Queue):
    def __init__(self,XG):
        super().__init__()
        self.parents = {}
        self.Gset = {}  # dict{loc:G value}
        self.Fset = {}  # dict{loc:F value}
        self.openlist = set()  # set{loc}
        self.closelist = set() # set{loc}
        self.h = [] # heap(f,(loc_i, loc_j)) to sort f value
        self.xg = XG  # list of goal

    def getHvalue(self, x):  # get the minimum estimated distance from x to goals
        minH = float('inf')
        for ds in self.xg:
            H = abs(x[0] - ds[0]) + abs(x[1] - ds[1])
            if H < minH:
                minH = H
        return minH

    def insert(self, x, parent):  # put x into openlist, put (f value,x) in to the heap, put g f value into Gset, Fset
        if parent is None:
            g = 0
        else:
            g = self.Gset[parent] + 1
        f = self.getHvalue(x) + g
        self.Gset[x] = g
        self.Fset[x] = f
        heapq.heappush(self.h, (f, x))
        self.openlist.add(x)
        self.parents[x] = parent

    def dequeue(self):
        x = heapq.heappop(self.h)[1]  # get minimum f value and the node
        self.closelist.add(x)
        return x

    def isDiscovered(self, x):  # check if x is closed
        if x in self.openlist:
            return True
        else:
            return False

    def resolve(self, x, parent):  # if x is in open list when visited, update F value if g value is smaller
        if x in self.openlist:
            g = self.Gset[parent] + 1
            g_cur = self.Gset[x]
            if g < g_cur:
                self.Gset[x] = g
                f = self.Fset[x] - g_cur + g
                self.Fset[x] = f
                heapq.heappush(self.h, (f, x))
                self.parents[x] = parent

    def getpath(self, des):
        path = []
        cur = des
        while cur is not None:
            path.append(cur)
            cur = self.parents[cur]
        return path

    def isempty(self):  # check if openlist is empty
        return not self.h


class Search:
    def __init__(self, g):
        self.G = g

    def fsearch(self, xI, XG, alg): # forward search
        visited = [xI]
        visited_edge = []
        path = []
        if alg == "bfs":
            Q = QueueBFS()
            Q.insert(xI,None)
        if alg == "dfs":
            Q = QueueDFS()
            Q.insert(xI, None)
        if alg == "astar":
            Q = QueueAstar(XG)
            Q.insert(xI, None)
        while not Q.isempty():
            x = Q.dequeue()
            if self.is_achieved(x, XG):
                path = Q.getpath(x)
                path.reverse()
                return visited_edge, visited, path, True  # achieve goal
            for des in x.children:
                if des is not None:
                    if not Q.isDiscovered(des):
                        Q.insert(des, x)
                        visit_ed = self.G.edgedict[(x,des)]
                        visited_edge.append(visit_ed)
                        visited.append(des)
                    else:
                        Q.resolve(des, x)
        path.reverse()
        return visited_edge, visited, path, False  # cannot achieve goal, return path, visited, False flag

    def is_achieved(self, x: Node, xG: DubinsConfig, margin = 0):
        config = x.item
        if xG.is_close(config):
            return True
        return False


# def main():
#     U = [(0, 1), (0, -1), (-1, 0), (1, 0)]  # action list [up, down, left, right]
#     XG = []
#     filename = 'project1_desc.json'
#     method = 'astar'
#     with open(filename, 'r') as f:
#         data = json.load(f)
#     G = data['G']
#     xI = tuple(data['xI'])
#     for xg in data['XG']:  # convert list to tuple
#         XG.append(tuple(xg))  # list of goal cell
#     sch = Search(G,U)
#     visited, path, flag = sch.fsearch(xI,XG,method)
#     rst = {"visited": visited,
#            "path": path}
#     print(flag)
#     # b = json.dumps(rst)
#     # f = open(savename, 'w')
#     # f.write(b)
#     print("Visited:", visited)
#     print("Path:", path)
#
#     fig, ax = plt.subplots()  # plot the result
#     dg.draw(ax, G, path, visited)
#     plt.pause(5)
#
# if __name__ == "__main__":
#     main()
