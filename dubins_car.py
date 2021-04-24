import dubins
import matplotlib.pyplot as plt
from graph import Edge, DubinsConfig
from graph import Point
from graph import Node
from rrt import RRt



x0 = 10
y0 = 10
theta0 = 0

x1 = 10
y1 = 9.9
theta1 = 0

q0 = (-1.2199559242110303, -0.8651945309735829, 6.074952869120775)
q1 = (-0.7607923756239869, -0.7272899164210449, 0.7917675619411886)
q0=(-2,-0.5,0)
q1=(1.195952357338463,0.26702924459657296,0.9756583154337937)
q2 = (-0.9676497498599093,-0.9790110699545767,4.2723201582020955)
turning_radius = 0.5
step_size = 0.01

# path = dubins.shortest_path(q0, q1, turning_radius)
# e = path.path_length()
# configurations, d = path.sample_many(step_size)
conf0 = DubinsConfig.from_tuple(q0)
conf1 = DubinsConfig.from_tuple(q1)
conf, length = DubinsConfig.shortest_path(conf0, conf1, turning_radius, step_size)

nd0 = Node(conf0)
nd1 = Node(conf1)
ed = Edge(nd0,nd1,turning_radius)

s = 1
phi_max = 1
rou_min = 0.5

ob_center = [(0,-1),(0,1)]
dt = 0.2
ob_r = [1-dt, 1-dt]
cspace = [(-3,3),(-1,1)]
obs = []


obs_ct = Point.point_from_list(ob_center)
path = []

rrt = RRt(phi_max=phi_max, s=s, rou_min = rou_min, cspace=cspace, ob_center=obs_ct, ob_radio=ob_r)

flag, inter = rrt.check_collision(ed, 0)
print(flag)
print(inter)


fig, ax = plt.subplots()

ax = ed.draw(ax)
plt.show(ax)
a = 1